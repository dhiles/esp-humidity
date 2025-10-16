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
    while (MyWiFi::connectWithBackoff("") != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect WiFi. Full shutdown and retry.");
        // If connectWithBackoff fails, it already calls MyWiFi::full_deinitialize()
        vTaskDelay(pdMS_TO_TICKS(10000));
        //        heap_trace_stop();
        ESP_LOGE(TAG, "Dumping heap trace on WiFi connection failure:");
        //        heap_trace_dump();
    }
    MyNTP::initialize();
    MyNTP::syncTime();

    humidity_start();
    start_webserver();
}