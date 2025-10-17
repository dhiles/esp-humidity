#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
// #include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_psram.h"
#include <esp_sleep.h>
#include "sdkconfig.h"
// #include "oled.h"

#include "espnow_basic_config.h"
#include "nvs_json.h"
#include "mywifi.h"
#include "ble_provisioning.h"
#include "mymqtt.h"
#include "myntp.h"
#include "constants.h"
#include "webserver.h"

#include "bme280mgr.h"

static const char *TAG = "MAIN";

static void check_psram(void)
{
    if (esp_psram_is_initialized())
    {
        size_t psram_size = esp_psram_get_size();
        size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "PSRAM initialized: Total size = %u bytes, Free = %u bytes", psram_size, psram_free);
    }
    else
    {
        ESP_LOGE(TAG, "PSRAM not initialized!");
    }
}

// =================================================================
// APP ENTRY POINT
// =================================================================
extern "C" void app_main(void)
{
    // Initialize all NVS JSON objects
    esp_err_t err = init_nvs_json_all();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS JSON: %s", esp_err_to_name(err));
        return;
    }
    check_psram();
    MyWiFi::global_init();

    // Initial Wi-Fi connection attempt
    esp_err_t wifi_ret = MyWiFi::connectWithBackoff("");
    while (wifi_ret != ESP_OK)
    {
        if (wifi_ret == ESP_ERR_NO_WIFI_CREDENTIALS)
        {
            ESP_LOGI(TAG, "No Wi-Fi credentials found. Starting BLE provisioning...");
            ble_provisioning_init(); // Blocks until provisioned, timed out, or failed

            // After provisioning, reload and retry connection immediately
            wifi_ret = MyWiFi::connectWithBackoff(""); // Use "" to trigger fresh load
            if (wifi_ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Wi-Fi connected successfully after provisioning");
                break; // Exit loop on success
            }
            // If provisioning succeeded but connect still fails (unlikely, but e.g., bad creds), fall through to generic failure handling
        }

        // Generic failure: creds present but connection failed (bad network/creds)
        ESP_LOGE(TAG, "Wi-Fi connection failed (error: %s). Performing full shutdown and retry.",
                 esp_err_to_name(wifi_ret));
        // If connectWithBackoff fails, it already calls MyWiFi::full_deinitialize()
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Retry the connection
        wifi_ret = MyWiFi::connectWithBackoff("");
    }

    MyNTP::initialize();
    MyNTP::syncTime();

    humidity_start();
    start_webserver();
}