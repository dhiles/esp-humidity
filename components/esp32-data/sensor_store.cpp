#include "sensor_store.h"
#include <algorithm>
#include <cstring>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <limits>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "cJSON.h"

#define TAG "SENSOR_STORE"

// Private implementation class
class SensorStore::Impl
{
public:
    struct CircularBuffer
    {
        SensorData *buffer = nullptr;
        size_t capacity = 0;
        size_t head = 0;
        size_t tail = 0;
        size_t count = 0;
        size_t overwrites = 0;
        bool is_full = false;
        SemaphoreHandle_t mutex = nullptr;

        ~CircularBuffer()
        {
            cleanup();
        }

        void cleanup()
        {
            if (buffer)
            {
                delete[] buffer;
                buffer = nullptr;
            }

            if (mutex)
            {
                vSemaphoreDelete(mutex);
                mutex = nullptr;
            }

            capacity = 0;
            head = tail = count = overwrites = 0;
            is_full = false;
        }

        size_t getSizeInBytes() const
        {
            return capacity * sizeof(SensorData);
        }

        bool isInitialized() const
        {
            return buffer != nullptr && mutex != nullptr;
        }
    };

    CircularBuffer buffer_;
};

// SensorStore implementation
SensorStore::SensorStore() : pimpl_(std::make_unique<Impl>()) {}

SensorStore::~SensorStore()
{
    deinit();
}

SensorStore &SensorStore::getInstance()
{
    static SensorStore instance;
    return instance;
}

esp_err_t SensorStore::init(const Config &config, const Callbacks &callbacks)
{
    if (initialized_)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (config.max_entries == 0)
    {
        ESP_LOGE(TAG, "Invalid configuration: max_entries cannot be 0");
        return ESP_ERR_INVALID_ARG;
    }

    config_ = config;
    callbacks_ = callbacks;

    // Set reasonable limits
    if (config_.max_entries > 100000)
    {
        ESP_LOGW(TAG, "Limiting max_entries from %zu to 100000 for safety", config_.max_entries);
        config_.max_entries = 100000;
    }

    // Initialize buffer
    esp_err_t ret = initBuffer();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize buffer: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized_ = true;

    ESP_LOGI(TAG, "Sensor storage initialized successfully");
    ESP_LOGI(TAG, "Capacity: %zu entries", config_.max_entries);
    ESP_LOGI(TAG, "Overwrite oldest: %s", config_.overwrite_oldest ? "yes" : "no");
    ESP_LOGI(TAG, "Memory allocated: %zu bytes", pimpl_->buffer_.getSizeInBytes());

    return ESP_OK;
}

void SensorStore::deinit()
{
    if (!initialized_)
    {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing sensor storage...");

    // Cleanup buffer
    cleanupBuffer();

    initialized_ = false;

    ESP_LOGI(TAG, "Sensor storage deinitialized");
}

esp_err_t SensorStore::addData(const SensorData &data)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!data.validate())
    {
        ESP_LOGW(TAG, "Invalid sensor data rejected");
        if (callbacks_.on_error)
        {
            callbacks_.on_error(ESP_ERR_INVALID_ARG);
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (data.isEmpty())
    {
        ESP_LOGW(TAG, "Empty sensor data rejected");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = addToBuffer(data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add data to buffer: %s", esp_err_to_name(ret));
        if (callbacks_.on_error)
        {
            callbacks_.on_error(ret);
        }
        return ret;
    }

    // Call callback if set
    if (callbacks_.on_data_added)
    {
        callbacks_.on_data_added(data);
    }

    return ESP_OK;
}

esp_err_t SensorStore::getLatest(SensorData &data)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getLatest");
        return ESP_ERR_TIMEOUT;
    }

    if (pimpl_->buffer_.count == 0)
    {
        xSemaphoreGive(pimpl_->buffer_.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    size_t index = (pimpl_->buffer_.head == 0) ? pimpl_->buffer_.capacity - 1 : pimpl_->buffer_.head - 1;
    data = pimpl_->buffer_.buffer[index];

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return ESP_OK;
}

esp_err_t SensorStore::getByIndex(size_t index, SensorData &data)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getByIndex");
        return ESP_ERR_TIMEOUT;
    }

    if (index >= pimpl_->buffer_.count)
    {
        xSemaphoreGive(pimpl_->buffer_.mutex);
        ESP_LOGW(TAG, "Index %zu out of bounds (count=%zu)", index, pimpl_->buffer_.count);
        return ESP_ERR_INVALID_ARG;
    }

    size_t buffer_index = (pimpl_->buffer_.tail + index) % pimpl_->buffer_.capacity;
    data = pimpl_->buffer_.buffer[buffer_index];

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return ESP_OK;
}

std::vector<SensorData> SensorStore::getLastN(size_t n)
{
    std::vector<SensorData> result;

    if (!initialized_)
    {
        ESP_LOGE(TAG, "Store not initialized");
        return result;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getLastN");
        return result;
    }

    size_t available = pimpl_->buffer_.count;
    if (n > available)
    {
        n = available;
    }

    if (n == 0)
    {
        xSemaphoreGive(pimpl_->buffer_.mutex);
        return result;
    }

    result.reserve(n);

    // Calculate starting index
    size_t start_index;
    if (pimpl_->buffer_.head >= n)
    {
        start_index = pimpl_->buffer_.head - n;
    }
    else
    {
        start_index = pimpl_->buffer_.capacity - (n - pimpl_->buffer_.head);
    }

    // Copy data in correct chronological order
    for (size_t i = 0; i < n; i++)
    {
        size_t buffer_index = (start_index + i) % pimpl_->buffer_.capacity;
        result.push_back(pimpl_->buffer_.buffer[buffer_index]);
    }

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return result;
}

std::vector<SensorData> SensorStore::getRange(const DataRange &range)
{
    std::vector<SensorData> result;

    if (!initialized_)
    {
        ESP_LOGE(TAG, "Store not initialized");
        return result;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getRange");
        return result;
    }

    // If buffer is empty, return empty result
    if (pimpl_->buffer_.count == 0)
    {
        xSemaphoreGive(pimpl_->buffer_.mutex);
        return result;
    }

    // Start from tail and iterate through all entries
    size_t current_index = pimpl_->buffer_.tail;
    size_t entries_checked = 0;

    while (entries_checked < pimpl_->buffer_.count)
    {
        const SensorData &current = pimpl_->buffer_.buffer[current_index];

        if (range.contains(current))
        {
            result.push_back(current);
        }

        current_index = (current_index + 1) % pimpl_->buffer_.capacity;
        entries_checked++;
    }

    xSemaphoreGive(pimpl_->buffer_.mutex);

    // Sort by timestamp (oldest first)
    std::sort(result.begin(), result.end(),
              [](const SensorData &a, const SensorData &b)
              {
                  return a.getTimestamp() < b.getTimestamp();
              });

    return result;
}

std::vector<SensorData> SensorStore::getByDay(uint32_t day_timestamp)
{
    // Calculate start and end of day
    time_t time_val = day_timestamp;
    struct tm time_info;

    // Use localtime_r instead of gmtime for portability
    if (localtime_r(&time_val, &time_info) == nullptr)
    {
        ESP_LOGE(TAG, "Invalid timestamp for getByDay: %lu", day_timestamp);
        return {};
    }

    // Set to beginning of day
    time_info.tm_hour = 0;
    time_info.tm_min = 0;
    time_info.tm_sec = 0;

    // Convert back to timestamp
    time_t start_of_day_time = mktime(&time_info);
    if (start_of_day_time == -1)
    {
        ESP_LOGE(TAG, "Failed to calculate start of day");
        return {};
    }

    uint32_t start_of_day = static_cast<uint32_t>(start_of_day_time);

    // Calculate end of day (23:59:59)
    uint32_t end_of_day = start_of_day + 86399;

    DataRange range(start_of_day, end_of_day, 0, true);
    return getRange(range);
}

SensorStore::Stats SensorStore::getStats()
{
    Stats stats;

    if (!initialized_)
    {
        return stats;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getStats");
        return stats;
    }

    stats.capacity = pimpl_->buffer_.capacity;
    stats.count = pimpl_->buffer_.count;
    stats.used_memory = pimpl_->buffer_.count * sizeof(SensorData);
    stats.free_memory = (pimpl_->buffer_.capacity - pimpl_->buffer_.count) * sizeof(SensorData);
    stats.overwrites = pimpl_->buffer_.overwrites;
    stats.is_full = pimpl_->buffer_.is_full;

    if (pimpl_->buffer_.count > 0)
    {
        stats.oldest_timestamp = pimpl_->buffer_.buffer[pimpl_->buffer_.tail].getTimestamp();

        size_t latest_index = (pimpl_->buffer_.head == 0) ? pimpl_->buffer_.capacity - 1 : pimpl_->buffer_.head - 1;
        stats.newest_timestamp = pimpl_->buffer_.buffer[latest_index].getTimestamp();
    }

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return stats;
}

size_t SensorStore::getCount()
{
    if (!initialized_)
    {
        return 0;
    }

    size_t count = 0;

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for getCount");
        return 0;
    }

    count = pimpl_->buffer_.count;
    xSemaphoreGive(pimpl_->buffer_.mutex);

    return count;
}

bool SensorStore::isFull()
{
    if (!initialized_)
    {
        return false;
    }

    bool is_full = false;

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Timeout acquiring mutex for isFull");
        return false;
    }

    is_full = pimpl_->buffer_.is_full;
    xSemaphoreGive(pimpl_->buffer_.mutex);

    return is_full;
}

esp_err_t SensorStore::clear()
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    // Clear the buffer by resetting indices
    pimpl_->buffer_.head = 0;
    pimpl_->buffer_.tail = 0;
    pimpl_->buffer_.count = 0;
    pimpl_->buffer_.overwrites = 0;
    pimpl_->buffer_.is_full = false;

    // Clear the actual data
    memset(pimpl_->buffer_.buffer, 0, pimpl_->buffer_.capacity * sizeof(SensorData));

    xSemaphoreGive(pimpl_->buffer_.mutex);

    ESP_LOGI(TAG, "All data cleared");

    // Call callback if set
    if (callbacks_.on_clear)
    {
        callbacks_.on_clear();
    }

    return ESP_OK;
}

esp_err_t SensorStore::removeOlderThan(uint32_t timestamp, size_t *count)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (count)
    {
        *count = 0;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (pimpl_->buffer_.count == 0)
    {
        xSemaphoreGive(pimpl_->buffer_.mutex);
        return ESP_OK;
    }

    // Find first entry not older than timestamp
    size_t new_tail = pimpl_->buffer_.tail;
    size_t entries_checked = 0;
    size_t removed_count = 0;

    while (entries_checked < pimpl_->buffer_.count)
    {
        if (pimpl_->buffer_.buffer[new_tail].getTimestamp() >= timestamp)
        {
            break;
        }

        new_tail = (new_tail + 1) % pimpl_->buffer_.capacity;
        removed_count++;
        entries_checked++;
    }

    // Update buffer state
    if (removed_count > 0)
    {
        pimpl_->buffer_.tail = new_tail;
        pimpl_->buffer_.count -= removed_count;
        pimpl_->buffer_.is_full = (pimpl_->buffer_.count == pimpl_->buffer_.capacity);

        // Clear the removed entries
        for (size_t i = 0; i < removed_count; i++)
        {
            size_t clear_index = (pimpl_->buffer_.tail - removed_count + i + pimpl_->buffer_.capacity) % pimpl_->buffer_.capacity;
            memset(&pimpl_->buffer_.buffer[clear_index], 0, sizeof(SensorData));
        }
    }

    xSemaphoreGive(pimpl_->buffer_.mutex);

    if (count)
    {
        *count = removed_count;
    }

    if (removed_count > 0)
    {
        ESP_LOGI(TAG, "Removed %zu entries older than %lu", removed_count, timestamp);
    }

    return ESP_OK;
}

std::string SensorStore::exportData(ExportFormat format)
{
    if (!initialized_)
    {
        ESP_LOGE(TAG, "Store not initialized");
        return "";
    }

    // Get all data
    std::vector<SensorData> data = getLastN(pimpl_->buffer_.capacity);

    if (data.empty())
    {
        ESP_LOGI(TAG, "No data to export");
        return "";
    }

    std::string result;

    switch (format)
    {
    case ExportFormat::CSV:
    {
        ESP_LOGI(TAG, "Exporting %zu entries as CSV", data.size());
        result = exportToCsv(data);
        break;
    }

    case ExportFormat::JSON:
    {
        ESP_LOGI(TAG, "Exporting %zu entries as JSON", data.size());
        result = exportToJson(data);
        break;
    }

    case ExportFormat::BINARY:
    {
        ESP_LOGI(TAG, "Exporting %zu entries as binary", data.size());
        result = exportToBinary(data);
        break;
    }
    }

    ESP_LOGI(TAG, "Export completed: %zu bytes", result.size());
    return result;
}

SensorStore::MemoryInfo SensorStore::getMemoryInfo()
{
    MemoryInfo info;

    if (!initialized_)
    {
        return info;
    }

    if (xSemaphoreTake(pimpl_->buffer_.mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return info;
    }

    info.total = pimpl_->buffer_.capacity * sizeof(SensorData);
    info.used = pimpl_->buffer_.count * sizeof(SensorData);
    info.free = info.total - info.used;

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return info;
}

std::vector<uint8_t> SensorStore::getSensorIds()
{
    std::vector<uint8_t> ids;

    if (!initialized_)
    {
        return ids;
    }

    std::vector<SensorData> all_data = getLastN(pimpl_->buffer_.capacity);

    for (const auto &data : all_data)
    {
        uint8_t sensor_id = data.getSensorId();
        if (std::find(ids.begin(), ids.end(), sensor_id) == ids.end())
        {
            ids.push_back(sensor_id);
        }
    }

    // Sort for consistency
    std::sort(ids.begin(), ids.end());

    return ids;
}

std::vector<SensorType> SensorStore::getSensorTypes(uint8_t sensor_id)
{
    std::vector<SensorType> types;

    if (!initialized_)
    {
        return types;
    }

    std::vector<SensorData> all_data = getLastN(pimpl_->buffer_.capacity);

    for (const auto &data : all_data)
    {
        if (sensor_id != 0 && data.getSensorId() != sensor_id)
        {
            continue;
        }

        for (uint8_t i = 0; i < data.getValueCount(); i++)
        {
            const SensorValue *value = data.getValueByIndex(i);
            if (value)
            {
                if (std::find(types.begin(), types.end(), value->type) == types.end())
                {
                    types.push_back(value->type);
                }
            }
        }
    }

    return types;
}

SensorStore::TypeStats SensorStore::getStatsByType(SensorType type, uint8_t sensor_id)
{
    TypeStats stats;
    stats.type = type;

    if (!initialized_)
    {
        return stats;
    }

    DataRange range(0, UINT32_MAX, sensor_id, sensor_id == 0);
    range.addType(type);

    std::vector<SensorData> data = getRange(range);

    if (data.empty())
    {
        return stats;
    }

    stats.count = data.size();
    stats.min_value = std::numeric_limits<float>::max();
    stats.max_value = std::numeric_limits<float>::lowest();
    stats.avg_value = 0.0f;
    stats.std_dev = 0.0f;
    stats.last_timestamp = 0;

    float sum = 0.0f;
    std::vector<float> values;

    for (const auto &entry : data)
    {
        float value = 0.0f;
        if (entry.getValue(type, value))
        {
            stats.min_value = std::min(stats.min_value, value);
            stats.max_value = std::max(stats.max_value, value);
            sum += value;
            values.push_back(value);

            if (entry.getTimestamp() > stats.last_timestamp)
            {
                stats.last_timestamp = entry.getTimestamp();
            }
        }
    }

    if (!values.empty())
    {
        stats.count = values.size();
        stats.avg_value = sum / values.size();

        // Calculate standard deviation - use std::sqrt
        float variance_sum = 0.0f;
        for (float value : values)
        {
            float diff = value - stats.avg_value;
            variance_sum += diff * diff;
        }
        stats.std_dev = std::sqrt(variance_sum / values.size());
    }
    else
    {
        stats.count = 0;
        stats.min_value = 0.0f;
        stats.max_value = 0.0f;
        stats.avg_value = 0.0f;
    }

    return stats;
}

// Private methods implementation
esp_err_t SensorStore::initBuffer()
{
    size_t capacity = config_.max_entries;

    ESP_LOGI(TAG, "Initializing buffer with capacity: %zu", capacity);

    // Simple allocation without exception handling
    pimpl_->buffer_.buffer = new (std::nothrow) SensorData[capacity];
    if (!pimpl_->buffer_.buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate buffer: out of memory");
        return ESP_ERR_NO_MEM;
    }

    // Initialize buffer with zeros
    memset(pimpl_->buffer_.buffer, 0, capacity * sizeof(SensorData));
    pimpl_->buffer_.capacity = capacity;
    pimpl_->buffer_.head = 0;
    pimpl_->buffer_.tail = 0;
    pimpl_->buffer_.count = 0;
    pimpl_->buffer_.overwrites = 0;
    pimpl_->buffer_.is_full = false;

    // Create mutex
    pimpl_->buffer_.mutex = xSemaphoreCreateMutex();
    if (pimpl_->buffer_.mutex == nullptr)
    {
        delete[] pimpl_->buffer_.buffer;
        pimpl_->buffer_.buffer = nullptr;
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Buffer initialized successfully: %zu bytes",
             capacity * sizeof(SensorData));

    return ESP_OK;
}

void SensorStore::cleanupBuffer()
{
    pimpl_->buffer_.cleanup();
}

esp_err_t SensorStore::addToBuffer(const SensorData &data)
{
    if (xSemaphoreTake(pimpl_->buffer_.mutex, portMAX_DELAY) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    // Store data
    pimpl_->buffer_.buffer[pimpl_->buffer_.head] = data;

    // Handle buffer full condition
    if (pimpl_->buffer_.is_full)
    {
        pimpl_->buffer_.tail = (pimpl_->buffer_.tail + 1) % pimpl_->buffer_.capacity;
        pimpl_->buffer_.overwrites++;

        if (config_.overwrite_oldest)
        {
            // Call storage full callback
            if (callbacks_.on_storage_full)
            {
                xSemaphoreGive(pimpl_->buffer_.mutex);
                callbacks_.on_storage_full();
                xSemaphoreTake(pimpl_->buffer_.mutex, portMAX_DELAY);
            }
        }
        else
        {
            // Don't overwrite, just move head back
            pimpl_->buffer_.head = (pimpl_->buffer_.head == 0) ? pimpl_->buffer_.capacity - 1 : pimpl_->buffer_.head - 1;
            xSemaphoreGive(pimpl_->buffer_.mutex);
            return ESP_ERR_NO_MEM;
        }
    }

    // Update indices
    pimpl_->buffer_.head = (pimpl_->buffer_.head + 1) % pimpl_->buffer_.capacity;
    pimpl_->buffer_.is_full = pimpl_->buffer_.head == pimpl_->buffer_.tail;

    if (!pimpl_->buffer_.is_full)
    {
        pimpl_->buffer_.count++;
    }

    xSemaphoreGive(pimpl_->buffer_.mutex);

    return ESP_OK;
}

// Export helper methods
std::string SensorStore::exportToCsv(const std::vector<SensorData> &data)
{
    if (data.empty())
    {
        return "";
    }

    std::ostringstream ss;

    // Collect all unique sensor types in the dataset
    std::vector<SensorType> all_types;
    for (const auto &entry : data)
    {
        for (uint8_t i = 0; i < entry.getValueCount(); i++)
        {
            const SensorValue *value = entry.getValueByIndex(i);
            if (value)
            {
                SensorType type = value->type;
                if (std::find(all_types.begin(), all_types.end(), type) == all_types.end())
                {
                    all_types.push_back(type);
                }
            }
        }
    }

    // Sort types for consistent column order
    std::sort(all_types.begin(), all_types.end());

    // CSV header
    ss << "timestamp,datetime,sensor_id,value_count";
    for (const auto &type : all_types)
    {
        std::string type_name = SensorTypeUtils::toString(type);
        ss << "," << type_name << "_value," << type_name << "_quality";
    }
    ss << "\n";

    // Data rows
    for (const auto &entry : data)
    {
        // Format datetime
        time_t time_val = entry.getTimestamp();
        struct tm *time_info = localtime(&time_val);
        char datetime_str[64];
        if (time_info)
        {
            strftime(datetime_str, sizeof(datetime_str), "%Y-%m-%d %H:%M:%S", time_info);
        }
        else
        {
            strcpy(datetime_str, "N/A");
        }

        ss << entry.getTimestamp() << ","
           << datetime_str << ","
           << static_cast<int>(entry.getSensorId()) << ","
           << static_cast<int>(entry.getValueCount());

        // Add values for each type
        for (const auto &type : all_types)
        {
            float value = 0.0f;
            uint8_t quality = 0;
            bool has_value = false;

            for (uint8_t i = 0; i < entry.getValueCount(); i++)
            {
                const SensorValue *sensor_value = entry.getValueByIndex(i);
                if (sensor_value && sensor_value->type == type)
                {
                    value = sensor_value->value;
                    quality = sensor_value->quality;
                    has_value = true;
                    break;
                }
            }

            if (has_value)
            {
                ss << "," << value << "," << static_cast<int>(quality);
            }
            else
            {
                ss << ",,"; // Empty columns for missing values
            }
        }

        ss << "\n";
    }

    return ss.str();
}

std::string SensorStore::exportToJson(const std::vector<SensorData> &data)
{
    if (data.empty())
    {
        return "[]";
    }

    cJSON *root = cJSON_CreateArray();
    if (!root)
    {
        return "[]";
    }

    for (const auto &entry : data)
    {
        std::string entry_json_str = entry.toJson();
        cJSON *entry_json = cJSON_Parse(entry_json_str.c_str());
        if (entry_json)
        {
            cJSON_AddItemToArray(root, entry_json);
        }
    }

    char *json_str = cJSON_PrintUnformatted(root);
    std::string result(json_str ? json_str : "[]");

    if (json_str)
    {
        free(json_str);
    }
    cJSON_Delete(root);

    return result;
}

// Fixed exportToBinary method
std::string SensorStore::exportToBinary(const std::vector<SensorData>& data) {
    if (data.empty()) {
        return "";
    }
    
    // Create header
    struct BinaryHeader {
        uint32_t magic = 0x53454E53; // "SENS" in hex
        uint32_t version = 1;
        uint32_t entry_count = 0;
        uint32_t total_size = 0;
        uint64_t export_time = 0;
    } header;
    
    header.entry_count = data.size();
    header.total_size = sizeof(BinaryHeader) + (data.size() * sizeof(SensorData));
    header.export_time = esp_timer_get_time() / 1000000ULL;
    
    // Create binary data
    std::string result;
    result.resize(header.total_size);
    
    char* result_data = (char*)result.data(); // Get mutable pointer
    memcpy(result_data, &header, sizeof(BinaryHeader));
    
    // Copy data
    char* dest = result_data + sizeof(BinaryHeader);
    for (const auto& entry : data) {
        memcpy(dest, &entry, sizeof(SensorData));
        dest += sizeof(SensorData);
    }
    
    return result;
}

// Stats toJson implementation
std::string SensorStore::Stats::toJson() const
{
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        return "{}";
    }

    cJSON_AddNumberToObject(root, "capacity", capacity);
    cJSON_AddNumberToObject(root, "count", count);
    cJSON_AddNumberToObject(root, "used_memory", used_memory);
    cJSON_AddNumberToObject(root, "free_memory", free_memory);
    cJSON_AddNumberToObject(root, "overwrites", overwrites);
    cJSON_AddNumberToObject(root, "oldest_timestamp", oldest_timestamp);
    cJSON_AddNumberToObject(root, "newest_timestamp", newest_timestamp);
    cJSON_AddBoolToObject(root, "is_full", is_full);
    cJSON_AddNumberToObject(root, "usage_percentage", getUsagePercentage());

    // Format timestamps as human-readable
    if (oldest_timestamp > 0)
    {
        time_t time_val = oldest_timestamp;
        struct tm *time_info = localtime(&time_val);
        char buffer[64];
        if (time_info && strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info))
        {
            cJSON_AddStringToObject(root, "oldest_datetime", buffer);
        }
    }

    if (newest_timestamp > 0)
    {
        time_t time_val = newest_timestamp;
        struct tm *time_info = localtime(&time_val);
        char buffer[64];
        if (time_info && strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", time_info))
        {
            cJSON_AddStringToObject(root, "newest_datetime", buffer);
        }
    }

    char *json_str = cJSON_PrintUnformatted(root);
    std::string result(json_str ? json_str : "{}");

    if (json_str)
    {
        free(json_str);
    }
    cJSON_Delete(root);

    return result;
}

float SensorStore::Stats::getUsagePercentage() const
{
    return capacity > 0 ? (static_cast<float>(count) / capacity) * 100.0f : 0.0f;
}