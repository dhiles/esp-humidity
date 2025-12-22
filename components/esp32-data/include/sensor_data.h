#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <array>
#include <functional>
#include <algorithm>
#include "esp_err.h"
#include "esp_log.h"
#include "cJSON.h"


// Maximum number of sensor values per reading
#ifndef MAX_SENSOR_VALUES
#define MAX_SENSOR_VALUES 16
#endif

/**
 * @brief Sensor value types
 */
enum class SensorType : uint8_t {
    TEMPERATURE,        ///< Temperature in Celsius
    HUMIDITY,           ///< Relative humidity in percentage
    PRESSURE,           ///< Atmospheric pressure in hPa
    VOLTAGE,            ///< Voltage in volts
    CURRENT,            ///< Current in amperes
    POWER,              ///< Power in watts
    LIGHT,              ///< Light intensity in lux
    CO2,                ///< CO2 concentration in ppm
    TVOC,               ///< Total volatile organic compounds in ppb
    PM1_0,              ///< Particulate matter 1.0μm in μg/m³
    PM2_5,              ///< Particulate matter 2.5μm in μg/m³
    PM10,               ///< Particulate matter 10μm in μg/m³
    NOISE,              ///< Noise level in dB
    MOTION,             ///< Motion detection (0/1)
    DOOR_STATUS,        ///< Door status (0=closed, 1=open)
    BATTERY_LEVEL,      ///< Battery level in percentage
    RSSI,               ///< WiFi signal strength in dBm
    LATITUDE,           ///< GPS latitude
    LONGITUDE,          ///< GPS longitude
    ALTITUDE,           ///< Altitude in meters
    SPEED,              ///< Speed in m/s
    CUSTOM_1,           ///< Custom sensor type 1
    CUSTOM_2,           ///< Custom sensor type 2
    CUSTOM_3,           ///< Custom sensor type 3
    UNKNOWN = 255       ///< Unknown sensor type
};

/**
 * @brief Sensor value with type and data
 */
struct SensorValue {
    SensorType type;
    float value;
    uint8_t quality;    ///< Data quality (0-100), 100 = best
    
    SensorValue() : type(SensorType::UNKNOWN), value(0.0f), quality(100) {}
    SensorValue(SensorType t, float v, uint8_t q = 100) : type(t), value(v), quality(q) {}
    
    bool isValid() const { return type != SensorType::UNKNOWN; }
    std::string toString() const;
    std::string getTypeString() const;
    std::string getUnit() const;
    float getMinRange() const;
    float getMaxRange() const;
    bool isInRange() const;
};

/**
 * @brief Main sensor data structure with flexible values
 */
class SensorData {
private:
    uint32_t timestamp_;                 ///< Unix timestamp
    uint8_t sensor_id_;                  ///< Sensor identifier
    std::array<SensorValue, MAX_SENSOR_VALUES> values_;
    uint8_t value_count_;                ///< Actual number of values stored
    uint16_t checksum_;                  ///< Data integrity checksum
    
public:
    /**
     * @brief Default constructor
     */
    SensorData();
    
    /**
     * @brief Constructor with timestamp and sensor ID
     */
    SensorData(uint32_t timestamp, uint8_t sensor_id = 0);
    
    /**
     * @brief Create sensor data with current timestamp
     */
    static SensorData create(uint8_t sensor_id = 0);
    
    // Getters
    uint32_t getTimestamp() const { return timestamp_; }
    uint8_t getSensorId() const { return sensor_id_; }
    uint8_t getValueCount() const { return value_count_; }
    uint16_t getChecksum() const { return checksum_; }
    const std::array<SensorValue, MAX_SENSOR_VALUES>& getValues() const { return values_; }
    
    // Setters
    void setTimestamp(uint32_t timestamp) { timestamp_ = timestamp; }
    void setSensorId(uint8_t sensor_id) { sensor_id_ = sensor_id; }
    
    /**
     * @brief Add a sensor value
     * @return true if added successfully, false if array is full
     */
    bool addValue(SensorType type, float value, uint8_t quality = 100);
    bool addValue(const SensorValue& value);
    
    /**
     * @brief Remove a sensor value by type
     * @return true if removed, false if not found
     */
    bool removeValue(SensorType type);
    
    /**
     * @brief Check if a sensor type exists in the data
     */
    bool hasValue(SensorType type) const;
    
    /**
     * @brief Get sensor value by type
     * @param[out] value Output value
     * @return true if found, false otherwise
     */
    bool getValue(SensorType type, float& value) const;
    
    /**
     * @brief Get sensor value by index
     */
    const SensorValue* getValueByIndex(uint8_t index) const;
    
    /**
     * @brief Get sensor value by type
     */
    const SensorValue* getValueByType(SensorType type) const;
    
    /**
     * @brief Get all values of a specific type (for arrays/multiple sensors)
     */
    std::vector<const SensorValue*> getValuesByType(SensorType type) const;
    
    /**
     * @brief Clear all sensor values
     */
    void clearValues();
    
    /**
     * @brief Calculate checksum for sensor data
     */
    uint16_t calculateChecksum() const;
    
    /**
     * @brief Validate sensor data using checksum and value ranges
     */
    bool validate() const;
    
    /**
     * @brief Update checksum (call after modifying data)
     */
    void updateChecksum();
    
    /**
     * @brief Format sensor data as JSON string
     */
    std::string toJson() const;
    
    /**
     * @brief Format sensor data as CSV line
     */
    std::string toCsv() const;
    
    /**
     * @brief Parse sensor data from JSON
     */
    static SensorData fromJson(const std::string& json);
    
    /**
     * @brief Get formatted date/time string
     */
    std::string getFormattedTime() const;
    
    /**
     * @brief Get the number of bytes used by this data
     */
    size_t getSize() const;
    
    /**
     * @brief Check if data is empty (no values)
     */
    bool isEmpty() const { return value_count_ == 0; }
    
    /**
     * @brief Merge another sensor data into this one
     * @param other Other sensor data to merge
     * @param overwrite If true, overwrite existing values with same type
     */
    void merge(const SensorData& other, bool overwrite = true);
    
    /**
     * @brief Get a subset of sensor data with only specific types
     */
    SensorData filter(const std::vector<SensorType>& types) const;
    
    /**
     * @brief Equality operator
     */
    bool operator==(const SensorData& other) const;
    
    /**
     * @brief Inequality operator
     */
    bool operator!=(const SensorData& other) const;
    
    /**
     * @brief Get memory usage information
     */
    static void getMemoryInfo(size_t* total, size_t* used, size_t* free);
    
private:
    uint8_t findValueIndex(SensorType type) const;
};

/**
 * @brief Sensor data statistics for a specific type
 */
struct SensorStats {
    SensorType type;
    uint32_t timestamp;
    float min_value;
    float max_value;
    float avg_value;
    float std_dev;
    uint32_t sample_count;
    
    SensorStats(SensorType t = SensorType::UNKNOWN);
    std::string toJson() const;
    bool isValid() const { return type != SensorType::UNKNOWN; }
};

/**
 * @brief Data range for queries
 */
struct DataRange {
    uint32_t start_time;
    uint32_t end_time;
    uint8_t sensor_id;
    std::vector<SensorType> types;  ///< Empty vector means all types
    bool include_all_sensors;        ///< If true, ignore sensor_id
    
    DataRange(uint32_t start = 0, uint32_t end = UINT32_MAX,
              uint8_t sensor_id = 0, bool include_all = true);
    
    void addType(SensorType type) { types.push_back(type); }
    void clearTypes() { types.clear(); }
    bool containsType(SensorType type) const;
    bool contains(const SensorData& data) const;
};

/**
 * @brief Export format types
 */
enum class ExportFormat {
    JSON,
    CSV,
    BINARY
};

/**
 * @brief Utility functions for sensor types
 */
namespace SensorTypeUtils {
    std::string toString(SensorType type);
    SensorType fromString(const std::string& str);
    std::string getUnit(SensorType type);
    float getMinRange(SensorType type);
    float getMaxRange(SensorType type);
    bool isValidRange(SensorType type, float value);
    uint8_t getRecommendedQuality(SensorType type, float value);
    bool isNumeric(SensorType type);
    bool isBoolean(SensorType type);
    bool isLocation(SensorType type);
    bool isEnvironmental(SensorType type);
    bool isElectrical(SensorType type);
    bool isAirQuality(SensorType type);
    SensorType getPrimaryType(const SensorData& data);
}

#endif /* SENSOR_DATA_H */