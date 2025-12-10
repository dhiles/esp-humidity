// components/work/work.cpp
#include "work.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <esp_log.h>
#include "sht4xmgr.h"

static constexpr const char *TAG = "WORK";

// ====================================================================
// Singleton static instance
// ====================================================================
WorkImplementation* WorkImplementation::instance = nullptr;

// ====================================================================
// Constructor (private)
// ====================================================================
WorkImplementation::WorkImplementation()
{
    memset(msg, 0, sizeof(msg));
    humidity = 0.0f;
    temperature = 0.0f;
    sampleCount = 0;
}

// ====================================================================
// Singleton accessor
// ====================================================================
WorkImplementation& WorkImplementation::getInstance()
{
    // Thread-safe in C++11+ (static local init is thread-safe)
    if (instance == nullptr) {
        instance = new WorkImplementation();
    }
    return *instance;
}

// ====================================================================
// Interface method implementations
// ====================================================================

void WorkImplementation::init_work()
{
    ESP_LOGI(TAG, "Initializing SHT4x-based work task...");

    char timestamp[40] = {0};
    MyNTP::getTimestamp(timestamp, sizeof(timestamp));

    // Initial dummy message (invalid values)
    snprintf(msg, sizeof(msg),
             "{\"temp\":-127.00,\"humidity\":-127.00,\"timestamp\":\"%s\"}",
             timestamp);

    // Initialize the SHT4x sensor (I2C bus + soft reset + test read)
    sht4x_init();

    ESP_LOGI(TAG, "Work initialization complete. Sampling every %d ms", WORK_SAMPLE_PERIOD_MS);
}

void WorkImplementation::do_work()
{
    ESP_LOGD(TAG, "do_work() - sample %lu", sampleCount);

    float temp = -127.0f;
    float rh   = -127.0f;

    esp_err_t ret = sht4x_measure_high_precision(&temp, &rh);

    if (ret == ESP_OK) {
        temperature = temp;
        humidity    = rh;

        ESP_LOGI(TAG, "SHT4x [Sample %lu] → %.2f °C | %.2f %%RH",
                 sampleCount, temperature, humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read SHT4x: %s (0x%x)", esp_err_to_name(ret), ret);
        temperature = -127.0f;
        humidity    = -127.0f;
    }

    // Get current timestamp
    char timestamp[40] = {0};
    MyNTP::getTimestamp(timestamp, sizeof(timestamp));

    // Build final JSON message
  //  snprintf(msg, sizeof(msg),
  //           "{\"temp\":%.2f,\"humidity\":%.2f,\"timestamp\":\"%s\"}",
  //           temperature, humidity, timestamp);

    // Build final JSON message
    snprintf(msg, sizeof(msg),
             "{\"humidity\":%.2f}",
             humidity);

             // Send via BLE (your existing safe function)
    send_notification_safe(msg);

    // Optional: MQTT publish
    // MyMQTT::publish("home/bedroom/sensor", msg);

    sampleCount++;
}

void WorkImplementation::end_work()
{
    ESP_LOGI(TAG, "end_work() called");
}

void WorkImplementation::deinit_work()
{
    ESP_LOGI(TAG, "Deinitializing work...");
    memset(msg, 0, sizeof(msg));
    humidity = 0.0f;
    temperature = 0.0f;
    sampleCount = 0;
}

const char* WorkImplementation::getMessage()
{
    return msg;
}

// Optional convenience getters (defined here, not inline in header)
float WorkImplementation::getHumidity() const
{
    return humidity;
}

float WorkImplementation::getTemperature() const
{
    return temperature;
}

uint32_t WorkImplementation::getSampleCount() const
{
    return sampleCount;
}

void WorkImplementation::logE(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    esp_log_writev(ESP_LOG_ERROR, TAG, format, args);
    va_end(args);
}

// ====================================================================
// FreeRTOS task wrapper
// ====================================================================
static void work_task_function(void *pvParameters)
{
    (void)pvParameters;

    auto& work = WorkImplementation::getInstance();
    work.init_work();

    while (true) {
        work.do_work();
        vTaskDelay(pdMS_TO_TICKS(WORK_SAMPLE_PERIOD_MS));
    }

    // Never reached
    work.deinit_work();
    vTaskDelete(NULL);
}

// ====================================================================
// Public task starter
// ====================================================================
void start_work_task()
{
    BaseType_t result = xTaskCreate(
        work_task_function,
        "work_task",
        8192,
        nullptr,
        5,
        nullptr
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "Work task started successfully (SHT3x mode)");
    } else {
        ESP_LOGE(TAG, "FAILED to create work task!");
    }
}

const char* get_current_work_message()
{
    return WorkImplementation::getInstance().getMessage();
}