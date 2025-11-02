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
#include <esp_sleep.h>
#include "sdkconfig.h"
// #include "oled.h"

#include "espnow_basic_config.h"
#include "nvs_json.h"
#include "mysystem.h"
#include "mywifi.h"
#include "mymqtt.h"
#include "myntp.h"
#include "constants.h"
#include "webserver.h"

#include "myled.h"

static const char *TAG = "MAIN";
#define BLINK_DELAY_MS 1000

// =================================================================
// APP ENTRY POINT
// =================================================================
extern "C" void app_main(void)
{
    esp_log_level_set("MYLED", ESP_LOG_NONE);

    // Initialize all NVS JSON objects
    esp_err_t err = init_nvs_json_all();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS JSON: %s", esp_err_to_name(err));
        return;
    }
    MySystem::check_psram();

    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
    led_set_state(RED_LED, true);

    MyWiFi::global_init();

    // Automatic WiFi start: Tries STA if creds exist, else starts AP for provisioning
    esp_err_t wifi_ret = MyWiFi::startWiFiAuto();
    if (wifi_ret == WIFI_MODE_STA_SUCCESS)
    {
        led_set_state(GREEN_LED, true);
    }
    else if (wifi_ret == WIFI_MODE_AP_SUCCESS)
    {
        led_start_flashing(GREEN_LED, 500);
    }
    else if (wifi_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start WiFi (STA or AP). Halting.");
        return;
    }

    // If Wi-Fi started successfully (STA or AP mode), proceed with services
    // In AP mode, webserver provides /provision endpoint for credential input + reboot to STA
  //  humidity_start();
    start_webserver(); // Start webserver (handles /readings, /provision, etc.)

    // If in STA mode, initialize NTP (skipped in AP for offline provisioning)
    if (MyWiFi::s_sta_netif != nullptr && MyWiFi::isConnected())
    {
        MyNTP::initialize();
        MyNTP::syncTime();
    }

    // NEW: Handle provisioning completion in AP mode (wait for provisioning signal)
    if (wifi_ret == WIFI_MODE_AP_SUCCESS) {
        ESP_LOGI(TAG, "In AP mode: Waiting for provisioning signal...");
        if (provisioning_sem != NULL && xSemaphoreTake(provisioning_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Provisioning signaled: Credentials applied, rebooting to STA mode.");
            // Optional: Stop webserver/BLE tasks here if needed for clean shutdown
            // stop_webserver(web_server);
            // ble_provisioning_deinit(); // If you have a deinit function
            vTaskDelay(pdMS_TO_TICKS(1000)); // Brief delay for logs to flush
            esp_restart();
        } else {
            ESP_LOGW(TAG, "Provisioning semaphore wait failed or timed out.");
        }
    }

    // Main task now idles; webserver and sensor tasks run in background
    // Optional: Add timeout in AP mode to auto-reboot if no provisioning (e.g., after 10min)
    while (1)
    {
        //  led_set_state(RED_LED, true);
        //  vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);

        //  led_set_state(RED_LED, false);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Example: If you have a connected BLE device, send a status message
        // send_message_notification(some_conn_handle, "WiFi Status: Connected");
    }
}
