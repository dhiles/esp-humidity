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

void notify_demo_task(void *param)
{
    int counter = 0;
    while (1)
    { // Run until disconnect (handled in gap_cb)

        char msg[50];
        snprintf(msg, sizeof(msg), "notify_demo_task #%d: Hello from ESP32!", ++counter);

        // Use the safe queue wrapper which now uses the global conn_handle internally
        send_notification_safe(msg);
        //  ESP_LOGI(TAG, msg);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Every 5s
    }
    vTaskDelete(NULL);
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

    // Start BLE provisioning service persistently (non-blocking)
  //  ble_provisioning_init(false); // Modified to take bool blocking param; starts task without waiting

    // Initial Wi-Fi connection attempt
    //   esp_err_t wifi_ret = MyWiFi::connectWithBackoff("");
    esp_err_t wifi_ret = ESP_ERR_NO_WIFI_CREDENTIALS;
    if (wifi_ret == ESP_ERR_NO_WIFI_CREDENTIALS)
    {
        ESP_LOGI(TAG, "No Wi-Fi credentials found. Waiting for BLE provisioning...");
        wifi_ret = MyWiFi::startAPWithBackoff();
/*
        // Block and wait for provisioning to complete (up to timeout)
        BaseType_t sem_result = xSemaphoreTake(provisioning_sem, pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS));
        if (sem_result == pdTRUE)
        {
            ESP_LOGI(TAG, "Provisioning completed. Retrying Wi-Fi connection...");
            wifi_ret = MyWiFi::connectWithBackoff(""); // Reload fresh credentials
        }
        else
        {
            ESP_LOGE(TAG, "Provisioning timed out. Continuing without Wi-Fi (BLE still active for future connections).");
            wifi_ret = ESP_FAIL; // Keep as failure to skip further steps if desired
        } */
    }

    // If Wi-Fi connected successfully (or skip if failed), proceed
    if (wifi_ret == ESP_OK)
    {
      //  MyNTP::initialize();
      //  MyNTP::syncTime();

        humidity_start();
        start_webserver();
    }
    else
    {
        ESP_LOGW(TAG, "Skipping NTP/webserver due to Wi-Fi failure. BLE remains available for provisioning/notifications.");
    }

    //  xTaskCreate(notify_demo_task, "notify_demo", 8192, NULL, 3, &notify_demo_task_handle);

    // Main task now idles; BLE host task runs persistently in background
    // You can add periodic tasks here if needed, e.g., to send notifications via BLE
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Idle loop
        // Example: If you have a connected BLE device, send a status message
        // send_message_notification(some_conn_handle, "WiFi Status: Connected");
    }
}