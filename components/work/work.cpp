#include "work.h"
#include "bme280mgr.h"

static constexpr const char *TAG = "WORK";

// Initialize the static instance pointer
WorkImplementation* WorkImplementation::instance = nullptr;

WorkImplementation::WorkImplementation()
{
    memset(msg, 0, sizeof(msg));
    humidity = 0.0f;
    sampleCount = 0;
}

WorkImplementation& WorkImplementation::getInstance() {
    if (instance == nullptr) {
        instance = new WorkImplementation();
    }
    return *instance;
}

void WorkImplementation::init_work()
{
    ESP_LOGI(TAG, "Starting work..");
    char timestamp[32];
    MyNTP::getTimestamp(timestamp, sizeof(timestamp));
    snprintf(msg, sizeof(msg), "{\"humidity\":%.2f,\"timestamp\":\"%s\"}", -127.0, timestamp);
    humidity_init();
}

void WorkImplementation::do_work()
{
    ESP_LOGI(TAG, "Performing work...");
    float current_humidity;
    int8_t result = getHumidityReading(&current_humidity);
    if (result == SUCCESS)
    {
        ESP_LOGI(TAG, "Got humidity: %.2f%%", current_humidity);
        humidity = current_humidity;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read humidity");
        humidity = -127.0f;
    }

    ESP_LOGI(TAG, "Sample %d: = %.2f %%RH", sampleCount, humidity);
    char timestamp[32];
    MyNTP::getTimestamp(timestamp, sizeof(timestamp));

    snprintf(msg, sizeof(msg), "{\"humidity\":%.2f,\"timestamp\":\"%s\"}", humidity, timestamp);
    send_notification_safe(msg);
    //MyMQTT::publish("diymalls1/well", msg);
    
    sampleCount++;
}

void WorkImplementation::end_work()
{
    ESP_LOGI(TAG, "Ending work...");
}

void WorkImplementation::deinit_work()
{
    ESP_LOGI(TAG, "Deinitializing work...");
    memset(msg, 0, sizeof(msg));
    humidity = 0.0f;
    sampleCount = 0;
}

const char *WorkImplementation::getMessage()
{
    return msg;
}

void WorkImplementation::logE(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    esp_log_writev(ESP_LOG_ERROR, TAG, format, args);
    va_end(args);
}

// Static task function
static void work_task_function(void *pvParameters)
{
    WorkImplementation::getInstance().init_work();
    while (1)
    {
        WorkImplementation::getInstance().do_work();
        vTaskDelay(pdMS_TO_TICKS(WORK_SAMPLE_PERIOD_MS));
    }
    WorkImplementation::getInstance().deinit_work();
    vTaskDelete(NULL);
}

void start_work_task()
{
    BaseType_t result = xTaskCreate(
        work_task_function,
        "work_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS)
    {
        ESP_LOGE("WORK", "Failed to create work task");
    }
    else
    {
        ESP_LOGI("WORK", "Work task started successfully");
    }
}

const char *get_current_work_message()
{
    return WorkImplementation::getInstance().getMessage();
}