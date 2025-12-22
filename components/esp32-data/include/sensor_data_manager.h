#ifndef SENSOR_DATA_MANAGER_H
#define SENSOR_DATA_MANAGER_H

#include "sensor_data_circular_buffer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cassert>

// Logging tag
static const char *TAG2 = "SENSOR_DATA_MANAGER";

// Singleton class managing the PSRAM circular buffer for SensorData
class SensorDataManager {
public:
    // Get the singleton instance (Meyers' singleton - thread-safe in C++11+)
    static SensorDataManager& getInstance() {
        static SensorDataManager instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    SensorDataManager(const SensorDataManager&) = delete;
    SensorDataManager& operator=(const SensorDataManager&) = delete;

    /**
     * @brief Initialize the circular buffer with the given capacity
     * @param max_rows Number of SensorData entries to store
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init(size_t max_rows) {
        if (initialized_) {
            ESP_LOGW(TAG2, "SensorDataManager already initialized");
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t err = buffer_.init(max_rows);
        if (err != ESP_OK) {
            ESP_LOGE(TAG2, "Failed to initialize circular buffer");
            return err;
        }

        initialized_ = true;
        ESP_LOGI(TAG2, "SensorDataManager initialized with %u entries capacity", max_rows);
        return ESP_OK;
    }

    /**
     * @brief Add a new SensorData entry (delegates to buffer)
     */
    bool add(const SensorData& data) {
        return buffer_.add(data);
    }

    /**
     * @brief In-place construction of SensorData (delegates to buffer)
     */
    template<typename... Args>
    bool emplace(Args&&... args) {
        return buffer_.emplace(std::forward<Args>(args)...);
    }

    /**
     * @brief Get the most recent SensorData
     */
    const SensorData* getLatest() const {
        return buffer_.getLatest();
    }

    /**
     * @brief Get the oldest SensorData
     */
    const SensorData* getOldest() const {
        return buffer_.getOldest();
    }

    /**
     * @brief Get SensorData at relative index (0 = oldest, size()-1 = newest)
     */
    const SensorData* getAt(size_t index) const {
        return buffer_.getAt(index);
    }

    /**
     * @brief Get current number of stored readings
     */
    size_t size() const {
        return buffer_.size();
    }

    /**
     * @brief Check if buffer is full
     */
    bool isFull() const {
        return buffer_.isFull();
    }

    /**
     * @brief Check if buffer is empty
     */
    bool isEmpty() const {
        return buffer_.isEmpty();
    }

    /**
     * @brief Clear the buffer (resets count, head, tail)
     */
    void clear() {
        buffer_.clear();
    }

    /**
     * @brief Free the PSRAM memory (only call when shutting down)
     */
    void free() {
        buffer_.free();
        initialized_ = false;
    }

    // Optional: Get direct access to the underlying buffer if needed
    SensorDataCircularBuffer& getBuffer() {
        return buffer_;
    }

private:
    // Private constructor
    SensorDataManager() : initialized_(false) {}

    // The actual circular buffer
    SensorDataCircularBuffer buffer_;
    bool initialized_;
};

#endif // SENSOR_DATA_MANAGER_H