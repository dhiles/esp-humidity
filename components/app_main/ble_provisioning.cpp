#ifdef __cplusplus
extern "C"
{
#endif

#include "ble_provisioning.h"

#define DEVICE_NAME "ESP32-PROV"

// NVS Keys
#define NVS_NAMESPACE "wifi_creds"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASS "pass"

// BLE UUID Strings (Matching the Flutter App)
#define PROV_SERVICE_UUID_STR "0000AAB0-218F-4424-8A05-F417242F7B0B"
#define SSID_CHAR_UUID_STR "0000AAB1-218F-4424-8A05-F417242F7B0B"
#define PASS_CHAR_UUID_STR "0000AAB2-218F-4424-8A05-F417242F7B0B"
#define REBOOT_CHAR_UUID_STR "0000AAB3-218F-4424-8A05-F417242F7B0B"

#define PROVISIONING_TIMEOUT_MS 30000  // Reduced to 30s for faster error detection (was 300000)

static const char *TAG = "BLE_PROV";

// Semaphore for signaling provisioning completion
SemaphoreHandle_t provisioning_sem = NULL;

// PROV_SERVICE_UUID (0000AAB0-218F-4424-8A05-F417242F7B0B)
static const ble_uuid128_t PROV_SERVICE_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB0, 0xAA, 0x00, 0x00}};

// SSID_CHAR_UUID (0000AAB1-218F-4424-8A05-F417242F7B0B)
static const ble_uuid128_t SSID_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB1, 0xAA, 0x00, 0x00}};

// PASS_CHAR_UUID (0000AAB2-218F-4424-8A05-F417242F7B0B)
static const ble_uuid128_t PASS_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB2, 0xAA, 0x00, 0x00}};

// REBOOT_CHAR_UUID (0000AAB3-218F-4424-8A05-F417242F7B0B)
static const ble_uuid128_t REBOOT_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB3, 0xAA, 0x00, 0x00}};

// GATT attribute handles
static uint16_t prov_conn_handle;
static uint16_t ssid_handle;
static uint16_t pass_handle;
static uint16_t reboot_handle;

// Forward declaration for the write callback (needed before array reference)
static int ble_provisioning_write_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Array of attributes (GATT server structure)
static const struct ble_gatt_chr_def provisioning_chars[] = {
    {
        // SSID Characteristic (Write property)
        .uuid = (const ble_uuid_t *)&SSID_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &ssid_handle,
    },
    {
        // Password Characteristic (Write property)
        .uuid = (const ble_uuid_t *)&PASS_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &pass_handle,
    },
    {
        // Reboot Characteristic (Write property - Trigger)
        .uuid = (const ble_uuid_t *)&REBOOT_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &reboot_handle,
    },
    {0}, // Stop marker
};

// Array of services (Provisioning Service)
static const struct ble_gatt_svc_def provisioning_service[] = {
    {
        // Provisioning Service (AAB0)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&PROV_SERVICE_UUID.u,
        .characteristics = provisioning_chars,
    },
    {0}, // Stop marker
};

// Global to hold the received credentials
static char received_ssid[33] = {0};
static char received_pass[64] = {0};

// Task handle for BLE host task (for cleanup on timeout)
static TaskHandle_t ble_host_handle = NULL;

/**
 * @brief Registers the GATT service definitions.
 */
static void gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Count the number of attributes to pre-allocate space
    rc = ble_gatts_count_cfg(provisioning_service);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error counting GATT attributes; rc=%d", rc);
        return;
    }

    // Register the custom provisioning service
    rc = ble_gatts_add_svcs(provisioning_service);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to register GATT service; rc=%d", rc);
    }
}

/**
 * @brief Builds the advertising payload (flags and device name)
 */
static void ble_prov_advertise(void)
{
    // Configure the advertising data
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Flags: LE general discoverable, BR/EDR not supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set the device name
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);

    // REMOVED: Set the custom service UUID in the advertising data (causes 31-byte overflow)
    // fields.uuids128 = &PROV_SERVICE_UUID;
    // fields.num_uuids128 = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        // REMOVED: xSemaphoreGive(provisioning_sem);  // Let it timeout on error (no fake success)
        return;
    }

    // Begin advertising
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Undirected connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General discoverable

    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC, // Use public address type
        NULL,                // direct_addr (not used for undirected adv)
        BLE_HS_FOREVER,      // duration
        &adv_params,         // adv_params
        NULL,                // event callback (optional)
        NULL                 // callback context
    );

    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
        // REMOVED: xSemaphoreGive(provisioning_sem);  // Let it timeout on error (no fake success)
        return;
    }
    ESP_LOGI(TAG, "BLE Provisioning Advertising Started as '%s'", DEVICE_NAME);
}

/**
 * @brief Host synchronization callback: register services and start advertising.
 */
static void ble_hs_sync_cb(void)
{
    int rc;

    // Register GATT services (now safe after host sync)
    gatt_svr_init();

    // Set device name
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting device name; rc=%d", rc);
        // REMOVED: xSemaphoreGive(provisioning_sem);  // Let it timeout on error (no fake success)
        return;
    }

    // Start advertising
    ble_prov_advertise();
}

/**
 * @brief The GATT write callback. Handles data received from the Flutter app.
 */
static int ble_provisioning_write_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    prov_conn_handle = conn_handle;

    // Check which characteristic was written to
    if (attr_handle == ssid_handle)
    {
        // SSID Characteristic
        size_t len = ctxt->om->om_len;
        len = len < sizeof(received_ssid) - 1 ? len : sizeof(received_ssid) - 1;
        memcpy(received_ssid, ctxt->om->om_data, len);
        received_ssid[len] = '\0';
        ESP_LOGI(TAG, "Received SSID: %s", received_ssid);
    }
    else if (attr_handle == pass_handle)
    {
        // Password Characteristic
        size_t len = ctxt->om->om_len;
        len = len < sizeof(received_pass) - 1 ? len : sizeof(received_pass) - 1;
        memcpy(received_pass, ctxt->om->om_data, len);
        received_pass[len] = '\0';
        ESP_LOGI(TAG, "Received PASS: [HIDDEN]");
    }
    else if (attr_handle == reboot_handle)
    {
        // Reboot/Finalize Characteristic
        ESP_LOGI(TAG, "Reboot command received. Finalizing provisioning.");
        // This is the trigger to save and signal completion
        nvs_save_credentials(received_ssid, received_pass);
        ESP_LOGI(TAG, "Credentials saved. Signaling completion and stopping BLE...");
        // Signal the main task that provisioning is done
        if (provisioning_sem != NULL)
        {
            xSemaphoreGive(provisioning_sem);
        }
        // Stop the NimBLE host to unblock the run loop
        nimble_port_stop();
        // Do not reboot here; let the task handle it after the loop exits
    }

    return 0; // Success
}

/**
 * @brief The function executed when the BLE Host is ready.
 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");

    // This loop prevents the task from exiting
    nimble_port_run();

    ESP_LOGI(TAG, "BLE Host Task completed (stopped). Rebooting in 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

/**
 * @brief Initializes and runs the NimBLE host task.
 */
void ble_provisioning_init(void)
{
    // No credentials found, start BLE provisioning
    ESP_LOGI(TAG, "No credentials found. Starting BLE Provisioning.");

    // Reset globals
    ble_host_handle = NULL;
    provisioning_sem = NULL;
    memset(received_ssid, 0, sizeof(received_ssid));
    memset(received_pass, 0, sizeof(received_pass));

    // Create the semaphore for signaling completion/failure
    provisioning_sem = xSemaphoreCreateBinary();
    if (provisioning_sem == NULL)
    {
        ESP_LOGE(TAG, "Failed to create provisioning semaphore");
        return;
    }

    // Initialize NimBLE host controller and stack
    nimble_port_init();

    // Set the sync callback to register services and advertise
    ble_hs_cfg.sync_cb = ble_hs_sync_cb;

    // Start the BLE host task
    // We use a dedicated FreeRTOS task to run the NimBLE stack
    xTaskCreate(ble_host_task, "ble_host", 8192, NULL, 5, &ble_host_handle);

    // Wait for the provisioning to complete (or fail/timeout) - blocks the caller (app_main)
    BaseType_t sem_result = xSemaphoreTake(provisioning_sem, pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS));
    if (sem_result == pdTRUE)
    {
        ESP_LOGI(TAG, "Provisioning completed successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Provisioning timed out after %d ms", PROVISIONING_TIMEOUT_MS);
        // Cleanup on timeout/failure
        nimble_port_stop();
        if (ble_host_handle != NULL)
        {
            vTaskDelete(ble_host_handle);
            ble_host_handle = NULL;
        }
    }

    // Clean up semaphore after use
    vSemaphoreDelete(provisioning_sem);
    provisioning_sem = NULL;
}

#ifdef __cplusplus
    // End the extern "C" block
}
#endif