#include <stdlib.h>
#include <string.h>
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
#include "mywifi.h"
#include "mymqtt.h"
#include "myntp.h"
#include "constants.h"
#include "loop.h"
#include "work_interface.h"

#include <stdio.h>
#include "bme280mgr.h"

extern "C"
{
    void app_main(void);
}

static const char *TAG = "MAIN";
/*
// Structure to pass parameters to the task
typedef struct
{
    uint32_t sleep_duration;
    const char *mqtt_uri;
    WorkInterface *work_impl;
} sleep_loop_params_t;

class WorkImplementation : public WorkInterface
{
public:
    void init_work() override
    {
        ESP_LOGI(TAG, "init_work");
    }

    void do_work() override
    {
        ESP_LOGI(TAG, "Performing work...");
}

void end_work() override
{
    ESP_LOGI(TAG, "Ending work...");
}

void deinit_work() override
{
    ESP_LOGI(TAG, "Deinitializing work...");
    // deinit_cam();
}
}
;

// Task function to run sleep_loop
static void sleep_loop_task(void *pvParameters)
{
    sleep_loop_params_t *params = (sleep_loop_params_t *)pvParameters;
    ESP_LOGI(TAG, "Starting sleep_loop task...");

    // Call sleep_loop with the provided parameters
    sleep_loop(params->sleep_duration, params->mqtt_uri, *(params->work_impl));

    // If sleep_loop returns, delete the task (optional, depending on your use case)
    ESP_LOGI(TAG, "sleep_loop task completed, deleting task...");
    vTaskDelete(NULL);
}
    */
/*
void app_main(void)
{
    static WorkImplementation work_impl;

    // Initialize NVS (required for Wi-Fi and other components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Dynamically allocate parameters for the task to avoid dangling pointer
    sleep_loop_params_t *params = (sleep_loop_params_t *)malloc(sizeof(sleep_loop_params_t));
    if (params == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for task parameters");
        return;
    }

    // Initialize the parameters
    params->sleep_duration = 30;
    params->mqtt_uri = MQTTConfig::URI;
    params->work_impl = &work_impl;

    // Create the sleep_loop task
    BaseType_t task_created = xTaskCreate(
        sleep_loop_task,   // Task function
        "sleep_loop_task", // Task name
        4096,              // Stack size (adjust as needed)
        params,            // Task parameters (dynamically allocated)
        5,                 // Task priority
        NULL               // Task handle (NULL if not needed)
    );

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sleep_loop task");
        free(params); // Clean up on failure
        return;
    }

    ESP_LOGI(TAG, "sleep_loop task created successfully");
}
    */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting...");

    humidity_start();

    while (1)
    {
        float humidity = humidity_read();
        ESP_LOGI(TAG, "Humidity: %.2f %%RH", humidity);
        //  char timestamp[32];
        //  MyNTP::getTimestamp(timestamp, sizeof(timestamp));
        //  char msg[64];
        //  snprintf(msg, sizeof(msg), "{\"temperature\":%.2f,\"timestamp\":\"%s\"}", temperature, timestamp);
        //  MyMQTT::publish("diymalls1/well", msg);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}