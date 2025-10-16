
#ifdef __cplusplus
extern "C"
{
#endif

#include "ble_provisioning.h"

    // --- Configuration Constants ---
    static const char *TAG = "BLE_PROV";
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

    // Global to hold the received credentials
    static char received_ssid[33] = {0};
    static char received_pass[64] = {0};

    // GATT attribute handles
    static uint16_t prov_conn_handle;
    static uint16_t ssid_handle;
    static uint16_t pass_handle;
    static uint16_t reboot_handle;

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
            // This is the trigger to save and reboot the device
            nvs_save_credentials();
            ESP_LOGI(TAG, "Credentials saved. Rebooting in 3 seconds...");
            // Stop BLE here to prevent crashes during reboot
            nimble_port_stop();
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        }

        return 0; // Success
    }

    // Array of attributes (GATT server structure)
    static const struct ble_gatt_chr_def provisioning_chars[] = {
        {
            // SSID Characteristic (Write property)
            // FIX: Casting to (const ble_uuid_t *) which is the complete type for the base union.
            .uuid = (const ble_uuid_t *)&SSID_CHAR_UUID.u,
            .access_cb = ble_provisioning_write_cb,
            .flags = BLE_GATT_CHR_F_WRITE,
            .val_handle = &ssid_handle,
        },
        {
            // Password Characteristic (Write property)
            // FIX: Casting to (const ble_uuid_t *)
            .uuid = (const ble_uuid_t *)&PASS_CHAR_UUID.u,
            .access_cb = ble_provisioning_write_cb,
            .flags = BLE_GATT_CHR_F_WRITE,
            .val_handle = &pass_handle,
        },
        {
            // Reboot Characteristic (Write property - Trigger)
            // FIX: Casting to (const ble_uuid_t *)
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
            // FIX: Casting to (const ble_uuid_t *)
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
        ble_svc_gap_init();
        ble_svc_gatt_init();

        // Register the custom provisioning service
        int rc = ble_gatts_add_svcs(provisioning_service);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Failed to register GATT service; rc=%d", rc);
        }
    }

    // ----------------------------------------------------------------------
    // 4. BLE Advertising Functions
    // ----------------------------------------------------------------------

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

        // Set the custom service UUID in the advertising data
        // FIX: Correctly set the pointer to the static ble_uuid128_t structure
        fields.uuids128 = (ble_uuid128_t *)&PROV_SERVICE_UUID;
        fields.num_uuids128 = 1;

        int rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
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
            return;
        }
        ESP_LOGI(TAG, "BLE Provisioning Advertising Started as '%s'", DEVICE_NAME);
    }

    /**
     * @brief The function executed when the BLE Host is ready.
     */
    static void ble_host_task(void *param)
    {
        ESP_LOGI(TAG, "BLE Host Task Started");

        // This loop prevents the task from exiting
        nimble_port_run();
    }

    /**
     * @brief Initializes and runs the NimBLE host task.
     */
    void ble_provisioning_init(void)
    {
        ESP_LOGI(TAG, "Initializing BLE Provisioning...");

        // NVS initialization must happen before Wi-Fi or BLE
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        // Load credentials and check if we should connect to Wi-Fi
        wifi_config_t wifi_config = {0};
        if (nvs_load_credentials(&wifi_config) == ESP_OK)
        {
            ESP_LOGI(TAG, "Credentials found. Connecting to Wi-Fi...");
            // Start Wi-Fi connection with loaded config
           // start_wifi_connection(&wifi_config);
        }
        else
        {
            // No credentials found, start BLE provisioning
            ESP_LOGI(TAG, "No credentials found. Starting BLE Provisioning.");

            // Initialize NimBLE host controller and stack
            nimble_port_init();

            // Configure GAP and GATT services
            ble_svc_gap_device_name_set(DEVICE_NAME);
            gatt_svr_init();

            // Register the host synchronization callback
            // This callback starts advertising once the NimBLE stack is fully initialized
            ble_hs_cfg.sync_cb = ble_prov_advertise;

            // Start the BLE host task
            // We use a dedicated FreeRTOS task to run the NimBLE stack indefinitely
            xTaskCreate(ble_host_task, "ble_host", 4096, NULL, 1, NULL);
        }
    }
#ifdef __cplusplus
    // End the extern "C" block
}
#endif
