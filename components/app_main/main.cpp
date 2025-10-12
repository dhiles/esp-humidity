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
/*
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion_for_all(bus));

        for (int i = 0; i < ds18b20_device_num; i++)
        {
            float temperature;
            ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
            ESP_LOGI(TAG, "Sample %d: DS18B20[%d] = %.2f °C", i, i, temperature);
            char timestamp[32];
            MyNTP::getTimestamp(timestamp, sizeof(timestamp));
            char msg[64];
            snprintf(msg, sizeof(msg), "{\"temperature\":%.2f,\"timestamp\":\"%s\"}", temperature, timestamp);
            MyMQTT::publish("diymalls1/well", msg);
        } */
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
};

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
/*
extern "C" void app_main1(void)
{
    ESP_LOGI(TAG, "Starting DS18B20 example");

    // Install 1-Wire bus using RMT driver
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
        .flags = {
            .en_pull_up = true, // Enable internal pull-up if no external
        }};
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // For ROM command + 8-byte ROM + device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    // Search for DS18B20 devices
    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Searching for devices...");
    do
    {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        {
            ds18b20_config_t ds_cfg = {}; // Fixed: No space in "ds18b20_config_t"
            onewire_device_address_t address;
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK)
            {
                ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
                ESP_LOGI(TAG, "Found DS18B20[%d], address: %016llX", ds18b20_device_num, address);
                ds18b20_device_num++;
            }
            else
            {
                ESP_LOGI(TAG, "Found unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Found %d DS18B20 device(s)", ds18b20_device_num);

    if (ds18b20_device_num == 0)
    {
        ESP_LOGE(TAG, "No DS18B20 devices found!");
        // onewire_del_bus(bus);
        return;
    }

    // Main loop: Trigger conversion and read temperature
    int sample_count = 0;
    while (1)
    {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion_for_all(bus));

        for (int i = 0; i < ds18b20_device_num; i++)
        {
            float temperature;
            ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
            ESP_LOGI(TAG, "Sample %d: DS18B20[%d] = %.2f °C", ++sample_count, i, temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_SAMPLE_PERIOD_MS));
    }

    // Cleanup (unreachable in infinite loop; uncomment after updating component if needed)
} */