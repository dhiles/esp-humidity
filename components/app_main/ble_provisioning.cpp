#ifdef __cplusplus
extern "C"
{
#endif

#include "ble_provisioning.h"
#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// Note: You must ensure 'ble_provisioning.h' defines the necessary function
// prototypes (like nvs_save_credentials) and includes the necessary ESP-IDF headers.
// For demonstration, we assume nvs_save_credentials is defined elsewhere.
// Example: void nvs_save_credentials(const char *ssid, const char *pass);

#define DEVICE_NAME "ESP32-PROV"

// NVS Keys (Assumed to be defined in main or nvs file)
#define NVS_NAMESPACE "wifi_creds"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASS "pass"

// BLE UUID Strings (Matching the Flutter App)
// Service UUID (AAB0)
#define PROV_SERVICE_UUID_STR "0000AAB0-218F-4424-8A05-F417242F7B0B"
// Characteristics UUIDs (AAB1, AAB2, AAB3)
#define SSID_CHAR_UUID_STR "0000AAB1-218F-4424-8A05-F417242F7B0B"
#define PASS_CHAR_UUID_STR "0000AAB2-218F-4424-8A05-F417242F7B0B"
#define REBOOT_CHAR_UUID_STR "0000AAB3-218F-4424-8A05-F417242F7B0B"

#define PROVISIONING_TIMEOUT_MS 300000  // 300s timeout

static const char *TAG = "BLE_PROV";

// --- Global NimBLE UUIDs ---
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
// --- End Global NimBLE UUIDs ---

// Global variables
SemaphoreHandle_t provisioning_sem = NULL;
static TaskHandle_t ble_host_handle = NULL;
static char received_ssid[33] = {0};
static char received_pass[64] = {0};

// GATT attribute handles
static uint16_t ssid_handle;
static uint16_t pass_handle;
static uint16_t reboot_handle;

// Forward declarations
static int ble_provisioning_write_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/**
 * @brief Definition of the custom provisioning GATT service and its characteristics.
 */
static const struct ble_gatt_chr_def provisioning_chars[] = {
    {
        // SSID Characteristic (AAB1)
        .uuid = (const ble_uuid_t *)&SSID_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &ssid_handle,
    },
    {
        // Password Characteristic (AAB2)
        .uuid = (const ble_uuid_t *)&PASS_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &pass_handle,
    },
    {
        // Reboot Characteristic (AAB3) - Write property to trigger final action
        .uuid = (const ble_uuid_t *)&REBOOT_CHAR_UUID.u,
        .access_cb = ble_provisioning_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &reboot_handle,
    },
    {0}, // Stop marker
};

static const struct ble_gatt_svc_def provisioning_service[] = {
    {
        // Provisioning Service (AAB0)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&PROV_SERVICE_UUID.u,
        .characteristics = provisioning_chars,
    },
    {0}, // Stop marker
};

/**
 * @brief Registers the GATT service definitions.
 */
static void gatt_svr_init(void)
{
    int rc;

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register the custom provisioning service
    rc = ble_gatts_count_cfg(provisioning_service);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(provisioning_service);
    }

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to register GATT service; rc=%d", rc);
    }
}

/**
 * @brief Builds and sets the Scan Response payload.
 * * CRUCIAL for making the device discoverable by UUID on iOS/Android.
 * Places the full device name and the 128-bit Service UUID in the response packet.
 */
static void ble_prov_set_scan_rsp_data(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // 1. Set the service UUIDs in the scan response
    fields.uuids128 = (ble_uuid128_t *)&PROV_SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1; // Mark the list of UUIDs as complete

    // 2. Set the complete device name in the scan response fields
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response data; rc=%d", rc);
    }
}

/**
 * @brief Builds the advertising payload and starts advertising.
 */
/**
 * @brief Builds the advertising payload and starts advertising.
 */
/**
 * @brief Builds the advertising payload and starts advertising.
 */
static void ble_prov_advertise(void)
{
    // --- Advertisement Packet Data (AD) ---
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Flags: LE general discoverable, BR/EDR not supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // Omit the device name from the main advertisement packet to save space for the UUID
    // (Full name is in Scan Response; UUID enables filtering)
    fields.name_len = 0;

    // Include the service UUID in the advertisement data for iOS/Android filtering
    fields.uuids128 = (ble_uuid128_t *)&PROV_SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }
    
    // --- Scan Response Packet Data (SRD) ---
    // This sets the Service UUID and the full name in the SRD.
    ble_prov_set_scan_rsp_data();

    // --- Start Advertising ---
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

    gatt_svr_init();

    // Set device name
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting device name; rc=%d", rc);
        return;
    }

    ble_prov_advertise();
}

/**
 * @brief The GATT write callback. Handles data received from the Flutter app.
 */
static int ble_provisioning_write_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
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
        
        // --- NOTE: You must replace this with your actual NVS save function ---
        // For demonstration, we assume a function like this exists:
        // nvs_save_credentials(received_ssid, received_pass);
        ESP_LOGW(TAG, "!! Placeholder: Call your NVS save function here !!");
        // ----------------------------------------------------------------------
        
        ESP_LOGI(TAG, "Credentials processed. Signaling completion and stopping BLE...");
        
        // Signal the main task that provisioning is done
        if (provisioning_sem != NULL)
        {
            xSemaphoreGive(provisioning_sem);
        }
        // Stop the NimBLE host to unblock the run loop
        nimble_port_stop();
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
 * @brief Initializes and runs the NimBLE host task, blocking until provisioning completes or times out.
 */
void ble_provisioning_init(void)
{
    ESP_LOGI(TAG, "Starting BLE Provisioning.");

    ble_host_handle = NULL;
    provisioning_sem = NULL;
    memset(received_ssid, 0, sizeof(received_ssid));
    memset(received_pass, 0, sizeof(received_pass));

    provisioning_sem = xSemaphoreCreateBinary();
    if (provisioning_sem == NULL)
    {
        ESP_LOGE(TAG, "Failed to create provisioning semaphore");
        return;
    }

    nimble_port_init();

    ble_hs_cfg.sync_cb = ble_hs_sync_cb;

    // Start the BLE host task
    xTaskCreate(ble_host_task, "ble_host", 8192, NULL, 5, &ble_host_handle);

    // Wait for provisioning to complete (signaled by xSemaphoreGive in the write callback)
    BaseType_t sem_result = xSemaphoreTake(provisioning_sem, pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS));
    if (sem_result == pdTRUE)
    {
        ESP_LOGI(TAG, "Provisioning completed successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Provisioning timed out after %d ms. Attempting cleanup...", PROVISIONING_TIMEOUT_MS);
        
        // Clean up on timeout/failure
        nimble_port_stop();
        if (ble_host_handle != NULL)
        {
            vTaskDelete(ble_host_handle);
            ble_host_handle = NULL;
        }
    }

    vSemaphoreDelete(provisioning_sem);
    provisioning_sem = NULL;
}

#ifdef __cplusplus
}
#endif
