#ifndef SENSOR_DATA_CIRCULAR_BUFFER_H
#define SENSOR_DATA_CIRCULAR_BUFFER_H

#include "sensor_data.h"  // Your header
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Logging tag
static const char *TAG1 = "SENSOR_CB";

// Circular buffer for SensorData objects in PSRAM
class SensorDataCircularBuffer {
public:
    SensorDataCircularBuffer() : buffer_(nullptr), capacity_(0), count_(0), head_(0), tail_(0) {}
    ~SensorDataCircularBuffer() { free(); }

    size_t capacity() const { return capacity_; }
    
    /**
     * @brief Initialize the circular buffer in PSRAM
     * @param max_rows Maximum number of SensorData objects to store
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init(size_t max_rows) {
        if (max_rows == 0) {
            ESP_LOGE(TAG1, "Capacity cannot be zero");
            return ESP_ERR_INVALID_ARG;
        }

        if (buffer_ != nullptr) {
            ESP_LOGW(TAG1, "Buffer already initialized");
            return ESP_ERR_INVALID_STATE;
        }

        size_t bytes_needed = max_rows * sizeof(SensorData);
        buffer_ = static_cast<SensorData*>(heap_caps_malloc(bytes_needed, MALLOC_CAP_SPIRAM));
        if (buffer_ == nullptr) {
            ESP_LOGE(TAG1, "Failed to allocate %u bytes in PSRAM", bytes_needed);
            return ESP_ERR_NO_MEM;
        }

        capacity_ = max_rows;
        count_    = 0;
        head_     = 0;
        tail_     = 0;

        // Call default constructor on each element (important!)
        for (size_t i = 0; i < capacity_; ++i) {
            new (&buffer_[i]) SensorData();  // placement new
        }

        ESP_LOGI(TAG1, "Circular buffer initialized: %u rows (%.1f KB in PSRAM)",
                 max_rows, bytes_needed / 1024.0f);

        return ESP_OK;
    }

    /**
     * @brief Add a new SensorData entry (overwrites oldest if full)
     * @param data The SensorData to add
     * @return true if added successfully
     */
    bool add(const SensorData& data) {
        if (buffer_ == nullptr) {
            ESP_LOGE(TAG1, "Buffer not initialized");
            return false;
        }

        // Overwrite the next slot
        buffer_[head_] = data;  // Copy assignment

        head_ = (head_ + 1) % capacity_;

        if (count_ < capacity_) {
            ++count_;
        } else {
            tail_ = (tail_ + 1) % capacity_;  // Move tail forward (discard oldest)
        }

        return true;
    }

    /**
     * @brief Add a new SensorData by constructing it in-place
     */
    template<typename... Args>
    bool emplace(Args&&... args) {
        if (buffer_ == nullptr) {
            ESP_LOGE(TAG1, "Buffer not initialized");
            return false;
        }

        // Construct in-place at head
        new (&buffer_[head_]) SensorData(std::forward<Args>(args)...);

        head_ = (head_ + 1) % capacity_;

        if (count_ < capacity_) {
            ++count_;
        } else {
            tail_ = (tail_ + 1) % capacity_;
        }

        return true;
    }

    /**
     * @brief Get the most recent (newest) SensorData
     * @return Pointer to latest or nullptr if empty
     */
    const SensorData* getLatest() const {
        if (count_ == 0 || buffer_ == nullptr) {
            return nullptr;
        }
        size_t latest_idx = (head_ == 0) ? (capacity_ - 1) : (head_ - 1);
        return &buffer_[latest_idx];
    }

    /**
     * @brief Get the oldest SensorData
     * @return Pointer to oldest or nullptr if empty
     */
    const SensorData* getOldest() const {
        if (count_ == 0 || buffer_ == nullptr) {
            return nullptr;
        }
        return &buffer_[tail_];
    }

    /**
     * @brief Get SensorData at relative index (0 = oldest, count-1 = newest)
     */
    const SensorData* getAt(size_t index) const {
        if (index >= count_ || buffer_ == nullptr) {
            return nullptr;
        }
        size_t pos = (tail_ + index) % capacity_;
        return &buffer_[pos];
    }

    /**
     * @brief Get current number of stored readings
     */
    size_t size() const { return count_; }

    /**
     * @brief Check if buffer is full
     */
    bool isFull() const { return count_ == capacity_; }

    /**
     * @brief Check if buffer is empty
     */
    bool isEmpty() const { return count_ == 0; }

    /**
     * @brief Clear the buffer (resets count, head, tail)
     */
    void clear() {
        count_ = 0;
        head_ = 0;
        tail_ = 0;
    }

    /**
     * @brief Free the allocated PSRAM memory
     */
    void free() {
        if (buffer_) {
            for (size_t i = 0; i < capacity_; ++i) {
                buffer_[i].~SensorData();  // Explicit destructor call
            }
            heap_caps_free(buffer_);
            buffer_ = nullptr;
            capacity_ = 0;
            count_ = 0;
            head_ = 0;
            tail_ = 0;
            ESP_LOGI(TAG1, "Circular buffer freed");
        }
    }

    // Optional: thread safety (uncomment if needed)
    // private:
    //     SemaphoreHandle_t mutex_ = xSemaphoreCreateMutex();

private:
    SensorData* buffer_;    // PSRAM allocated array
    size_t      capacity_;  // max number of items
    size_t      count_;     // current number of items
    size_t      head_;      // next write position
    size_t      tail_;      // oldest item position
};

#endif // SENSOR_DATA_CIRCULAR_BUFFER_H