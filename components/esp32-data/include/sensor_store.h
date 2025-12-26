#ifndef SENSOR_STORE_HPP
#define SENSOR_STORE_HPP

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <cmath>
#include "sensor_data.h"

class SensorStore {
public:
    /**
     * @brief Storage configuration
     */
    struct Config {
        size_t max_entries = 1000;       ///< Maximum number of entries
        bool overwrite_oldest = true;    ///< Overwrite oldest when full
        
        Config() = default;
    };
    
    /**
     * @brief Storage statistics
     */
    struct Stats {
        size_t capacity = 0;
        size_t count = 0;
        size_t used_memory = 0;
        size_t free_memory = 0;
        size_t overwrites = 0;
        uint32_t oldest_timestamp = 0;
        uint32_t newest_timestamp = 0;
        bool is_full = false;
        
        Stats() = default;
        std::string toJson() const;
        float getUsagePercentage() const;
    };
    
    /**
     * @brief Type statistics
     */
    struct TypeStats {
        SensorType type = SensorType::UNKNOWN;
        size_t count = 0;
        float min_value = 0.0f;
        float max_value = 0.0f;
        float avg_value = 0.0f;
        float std_dev = 0.0f;
        uint32_t last_timestamp = 0;
        
        TypeStats() = default;
        TypeStats(SensorType t) : type(t) {}
    };
    
    /**
     * @brief Storage callbacks
     */
    struct Callbacks {
        std::function<void(const SensorData&)> on_data_added;
        std::function<void()> on_storage_full;
        std::function<void()> on_clear;
        std::function<void(esp_err_t)> on_error;
        
        Callbacks() = default;
    };
    
    /**
     * @brief Memory usage information
     */
    struct MemoryInfo {
        size_t total = 0;
        size_t used = 0;
        size_t free = 0;
        
        float getUsagePercentage() const {
            return total > 0 ? (static_cast<float>(used) / total) * 100.0f : 0.0f;
        }
    };
    
    static SensorStore& getInstance();
    
    esp_err_t init(const Config& config, const Callbacks& callbacks = Callbacks());
    void deinit();
    bool isInitialized() const { return initialized_; }
    
    esp_err_t addData(const SensorData& data);
    esp_err_t getLatest(SensorData& data);
    esp_err_t getByIndex(size_t index, SensorData& data);
    std::vector<SensorData> getLastN(size_t n);
    std::vector<SensorData> getRange(const DataRange& range);
    std::vector<SensorData> getByDay(uint32_t day_timestamp);
    
    Stats getStats();
    size_t getCount();
    bool isFull();
    esp_err_t clear();
    esp_err_t removeOlderThan(uint32_t timestamp, size_t* count = nullptr);
    
    std::string exportData(ExportFormat format);
    MemoryInfo getMemoryInfo();
    
    std::vector<uint8_t> getSensorIds();
    std::vector<SensorType> getSensorTypes(uint8_t sensor_id = 0);
    TypeStats getStatsByType(SensorType type, uint8_t sensor_id = 0);
    
    SensorStore(const SensorStore&) = delete;
    SensorStore& operator=(const SensorStore&) = delete;
    
private:
    SensorStore();
    ~SensorStore();
    
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    bool initialized_ = false;
    Config config_;
    Callbacks callbacks_;
    
    esp_err_t initBuffer();
    void cleanupBuffer();
    esp_err_t addToBuffer(const SensorData& data);
    
    std::string exportToCsv(const std::vector<SensorData>& data);
    std::string exportToJson(const std::vector<SensorData>& data);
    std::string exportToBinary(const std::vector<SensorData>& data);
};

#endif