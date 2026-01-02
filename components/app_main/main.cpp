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
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_psram.h"
#include <esp_sleep.h>
#include "sdkconfig.h"
#include "espnow_basic_config.h"
#include "nvs_json.h"
#include "mywifi.h"
#include "ble_provisioning.h"
#include "mymqtt.h"
#include "myntp.h"
#include "constants.h"
#include "webserver.h"
#include "work.h"
#include "myled.h"
#include "mysystem.h"
#include "custom_webserver.h"
#include "loop.h"

static const char *TAG = "MAIN";
RTC_DATA_ATTR static int boot_count = 0;


void mqtt_command_handler(const char *topic, const char *payload)
{
    if (!topic || !payload)
        return;

    ESP_LOGI("MQTT_CMD", "Received: %s => %s", topic, payload);

    const char *prefix = "acreage/well/cmd/";
    if (strncmp(topic, prefix, strlen(prefix)) != 0)
    {
        return; // Not a command topic
    }

    const char *cmd = topic + strlen(prefix);

    if (strcmp(cmd, "mode") == 0)
    {
        if (strcmp(payload, "always_on") == 0)
        {
            ESP_LOGI("MQTT_CMD", "Mode set to ALWAYS_ON");
            MySystem::set_device_mode(DEVICE_MODE_ALWAYS_ON);
            // No reboot needed if already in awake mode
        }
        else if (strcmp(payload, "deep_sleep") == 0)
        {
            ESP_LOGI("MQTT_CMD", "Mode set to DEEP_SLEEP - rebooting to apply");
            MySystem::set_device_mode(DEVICE_MODE_DEEP_SLEEP);

            vTaskDelay(pdMS_TO_TICKS(1000)); // Let logs flush, allow clean MQTT disconnect
            esp_restart();
        }
    }
    else if (strcmp(cmd, "restart") == 0)
    {
        ESP_LOGI("MQTT_CMD", "Restart requested");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

// =================================================================
// APP ENTRY POINT
// =================================================================
extern "C" void app_main(void)
{
    ++boot_count;
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    ESP_LOGI(TAG, "Permanent 8-char Device ID: %s", MySystem::get_device_id());
    esp_log_level_set("MYLED", ESP_LOG_NONE);
    // esp_log_level_set("BME280_OLED", ESP_LOG_NONE);
    esp_log_level_set("CUSTOM_WEBSERVER", ESP_LOG_NONE);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash re-initialized for boot");
    // Early boot logs to confirm full reset
    esp_reset_reason_t reset_reason = esp_reset_reason();
    esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "FULL BOOT - Reset reason: %d | Wake cause: %d (4=TIMER)", reset_reason, wake_cause);

    // Initialize all NVS JSON objects
    esp_err_t err = init_nvs_json_all();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS JSON: %s", esp_err_to_name(err));
        return;
    }
    MySystem::check_psram();

    device_mode_t current_mode = MySystem::get_device_mode();

    if (current_mode == DEVICE_MODE_DEEP_SLEEP)
    {
        ESP_LOGI(TAG, "=== STARTING IN DEEP SLEEP MODE ===");
        sleep_loop_params_t deep_params = {
            .sleep_duration = 900, // 15 minutes
            .mqtt_uri = MQTTConfig::URI,
            .work_impl = &WorkImplementation::getInstance()};

        xTaskCreate(
            deep_sleep_loop_task,
            "deep_sleep_task",
            16384, // Adjust stack if needed
            &deep_params,
            5,
            NULL);
    }
    else
    {
        ESP_LOGI(TAG, "=== STARTING IN AWAKE MODE ===");

        init_led(GREEN_LED);
        init_led(RED_LED);
        init_led(BLUE_LED);
        led_set_state(RED_LED, true); // app started indicator

        MyWiFi::global_init();

        esp_err_t wifi_ret = MyWiFi::startWiFiAuto();
        if (wifi_ret == WIFI_MODE_STA_SUCCESS)
        {
            led_set_state(GREEN_LED, true);
            MyWiFi::start_mdns();
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

        ble_provisioning_init(false);
        start_work_task();
        start_custom_webserver();

        // === MQTT Setup in Awake Mode ===
        if (MyWiFi::s_sta_netif != nullptr && MyWiFi::isConnected())
        {
            MyNTP::initialize();
            MyNTP::syncTime();

            if (MyMQTT::connect(MQTTConfig::URI) == ESP_OK)
            {
                ESP_LOGI(TAG, "Connected to MQTT in awake mode");

                // Register the command handler so we can receive mode changes
                MyMQTT::registerMessageHandler(mqtt_command_handler);

                // Subscribe to all command topics
                MyMQTT::subscribe("acreage/well/cmd/#", 1);

                ESP_LOGI(TAG, "MQTT command handler registered - ready for remote commands");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to connect to MQTT in awake mode");
                MyMQTT::shutdown();
            }
        }

        // Handle provisioning in AP mode
        if (wifi_ret == WIFI_MODE_AP_SUCCESS)
        {
            ESP_LOGI(TAG, "In AP mode: Waiting for provisioning signal...");
            if (provisioning_sem != NULL && xSemaphoreTake(provisioning_sem, portMAX_DELAY) == pdTRUE)
            {
                ESP_LOGI(TAG, "Provisioning complete - rebooting to apply STA credentials");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }

        // Main loop - just idle, everything runs in background tasks
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
