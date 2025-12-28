#include "loop.h"
#include "mywifi.h"
#include "myntp.h"
#include "mymqtt.h" // Assuming mymqtt.h exists
#include <esp_sleep.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h" // Include for heap tracing functions
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_provisioning.h"
#include "esp_bt.h" // For BLE sleep APIs
#include "esp_pm.h"
// #include "esp_clk.h"

static const char *TAG = "LOOP";

extern SemaphoreHandle_t provisioning_sem; // From BLE code

// The s_heap_trace_record buffer might not be directly used by heap_trace_start()
// in your IDF version, but we keep it declared in case it's internally linked
// or for future compatibility.
// static heap_trace_record_t s_heap_trace_record[2048]; // Keep this line as a placeholder

void sleep_loop1(int sleep_seconds, const char *uri, WorkInterface &work)
{
    // Initial heap information (once at the start)
    //   size_t initial_heap_free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    //   size_t initial_heap_free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    //   ESP_LOGI(TAG, "Initial free heap (internal): %u bytes", initial_heap_free_internal);
    //   ESP_LOGI(TAG, "Initial free heap (PSRAM): %u bytes", initial_heap_free_psram);
    MyWiFi::global_init();
    work.init_work();

    while (true)
    {
        // Get heap info at the start of each loop iteration
        size_t heap_free_internal_start = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        size_t heap_free_psram_start = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        ESP_LOGI(TAG, "Loop Start - Free Heap (Internal): %u, (PSRAM): %u",
                 heap_free_internal_start, heap_free_psram_start);

        // --- Heap Tracing Setup for this Cycle ---
        // We're now relying on heap_trace_start() to handle its own buffer
        // initialization if CONFIG_HEAP_TRACING_ALL is enabled.
        //      heap_trace_start(HEAP_TRACE_LEAKS);
        // Add a small delay after starting trace to ensure it's fully active
        //      vTaskDelay(pdMS_TO_TICKS(1));

        // --- Wi-Fi Connection Attempt ---
        if (MyWiFi::connectWithBackoff(uri) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to connect WiFi. Full shutdown and retry.");
            // If connectWithBackoff fails, it already calls MyWiFi::full_deinitialize()
            vTaskDelay(pdMS_TO_TICKS(10000));
            //        heap_trace_stop();
            ESP_LOGE(TAG, "Dumping heap trace on WiFi connection failure:");
            //        heap_trace_dump();
            continue;
        }

        if (!MyNTP::isTimeSynced())
        {
            MyNTP::initialize();
            MyNTP::syncTime();
        }

        // --- MQTT Connection Attempt ---
        if (MyMQTT::connect(uri) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to connect MQTT. Disconnecting WiFi and retrying.");
            MyMQTT::shutdown();            // Shutdown MQTT
            MyWiFi::disconnect_and_stop(); // Disconnect Wi-Fi but keep driver initialized
            vTaskDelay(pdMS_TO_TICKS(10000));
            //         heap_trace_stop();
            ESP_LOGE(TAG, "Dumping heap trace on MQTT connection failure:");
            //         heap_trace_dump();
            continue;
        }
        //   init_filetransfer();
        ESP_LOGI(TAG, "System fully connected - beginning operations");
        work.do_work(); // Perform the main work
        work.end_work();
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay after work

        // --- MQTT Shutdown (always disconnect MQTT when done) ---
        MyMQTT::shutdown();

        // --- Wi-Fi Disconnect and Stop (but keep driver initialized) ---
        // This puts Wi-Fi into a low-power state, ready for quick resume.
        MyWiFi::disconnect_and_stop();
        /*
                // --- Stop Heap Tracing ---
                heap_trace_stop();

                // Get heap info at the end of each loop iteration
                size_t heap_free_internal_end = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                size_t heap_free_psram_end = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                ESP_LOGI(TAG, "Loop End   - Free Heap (Internal): %u, (PSRAM): %u",
                         heap_free_internal_end, heap_free_psram_end);

                // --- Leak Detection and Reporting ---
                if (heap_free_internal_end < heap_free_internal_start)
                {
                    int leaked_bytes = heap_free_internal_start - heap_free_internal_end;
                    ESP_LOGW(TAG, "Potential Internal RAM Leak: %d bytes lost this cycle.", leaked_bytes);
                    ESP_LOGW(TAG, "Dumping heap trace for potential internal RAM leaks:");
                    heap_trace_dump();
                }
                if (heap_free_psram_end < heap_free_psram_start)
                {
                    int leaked_bytes = heap_free_psram_start - heap_free_psram_end;
                    ESP_LOGW(TAG, "Potential PSRAM Leak: %d bytes lost this cycle.", leaked_bytes);
                    ESP_LOGW(TAG, "Dumping heap trace for potential PSRAM leaks:");
                    heap_trace_dump();
                }
        */
        // --- Enter Light Sleep ---
        ESP_LOGI(TAG, "Entering light sleep for %d seconds with Wi-Fi in modem sleep...", sleep_seconds);
        esp_sleep_enable_timer_wakeup(sleep_seconds * 1000000);
        esp_light_sleep_start(); // Wi-Fi modem should enter modem sleep here
        ESP_LOGI(TAG, "Woke from light sleep");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Stabilization delay after waking
    }
}

void sleep_loop(int sleep_seconds, const char *uri, WorkInterface &work)
{
    work.init_work();

    ESP_LOGI(TAG, "Starting BLE-only mode with Modem-sleep + %d-second sensor interval", sleep_seconds);

    ble_provisioning_init(false);

    // Optional: calibrate slow clock (helps BLE timing accuracy)
    #ifdef CONFIG_ESP_HW_SUPPORT_ENABLED
        uint32_t calib = esp_clk_slowclk_cal_get();
        ESP_LOGI(TAG, "RTC slow clock calibrated: %lu (~%lu Hz)", calib, (calib * 1000000ULL) / 16384ULL);
    #endif

    esp_bt_sleep_enable();
    ESP_LOGI(TAG, "BLE Modem-sleep enabled → always connectable, CPU sleeps between events");

    // Try full automatic light sleep (usually fails with BLE → expected)
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,
        .light_sleep_enable = true
    };
    if (esp_pm_configure(&pm_config) != ESP_OK) {
        ESP_LOGW(TAG, "Full light sleep not available with BLE → using modem-sleep + idle (still excellent power)");
    } else {
        ESP_LOGI(TAG, "Full automatic light sleep enabled!");
    }

    ESP_LOGI(TAG, "Setup complete. Sensor reads every %d seconds. BLE always on.", sleep_seconds);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        // === DO YOUR WORK (sensor read, send notification, etc.) ===
        work.do_work();      // BME280 read + BLE notify
        work.end_work();

        // Check provisioning
        if (provisioning_sem && xSemaphoreTake(provisioning_sem, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Provisioning done → restart");
            esp_restart();
        }

        ESP_LOGI(TAG, "Work done. Sleeping CPU for %d seconds (BLE stays active!)", sleep_seconds);

        // === THIS IS WHERE REAL SLEEP HAPPENS ===
        // CPU enters light sleep automatically during this delay
        // BLE controller wakes it instantly if phone connects or sends data
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(sleep_seconds * 1000));
    }
}

// Task function to run sleep_loop
void sleep_loop_task(void *pvParameters)
{
    sleep_loop_params_t *params = (sleep_loop_params_t *)pvParameters;
    ESP_LOGI(TAG, "Starting sleep_loop task...");

    // Call sleep_loop with the provided parameters
    sleep_loop(params->sleep_duration, params->mqtt_uri, *(params->work_impl));

    // If sleep_loop returns, delete the task (optional, depending on your use case)
    ESP_LOGI(TAG, "sleep_loop task completed, deleting task...");
    vTaskDelete(NULL);
}
