#include "sensor_data.h"
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cstring> // for memcpy

// Helper: simple FNV-1a hash for checksum
static uint16_t fnv1a_16(const uint8_t* data, size_t len) {
    uint16_t hash = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= 0x0101; // 257 (FNV prime for 16-bit)
    }
    return hash;
}

std::string SensorValue::toString() const {
    std::stringstream ss;
    ss << getTypeString() << ": " << value << " " << getUnit() 
       << " (quality: " << (int)quality << "%)";
    return ss.str();
}

std::string SensorValue::getTypeString() const {
    return SensorTypeUtils::toString(type);
}

std::string SensorValue::getUnit() const {
    return SensorTypeUtils::getUnit(type);
}

float SensorValue::getMinRange() const {
    return SensorTypeUtils::getMinRange(type);
}

float SensorValue::getMaxRange() const {
    return SensorTypeUtils::getMaxRange(type);
}

bool SensorValue::isInRange() const {
    return SensorTypeUtils::isValidRange(type, value);
}

// SensorData implementation
SensorData::SensorData() 
    : timestamp_(0), sensor_id_(0), value_count_(0), checksum_(0) {
    std::fill(values_.begin(), values_.end(), SensorValue());
}

SensorData::SensorData(uint32_t timestamp, uint8_t sensor_id)
    : timestamp_(timestamp), sensor_id_(sensor_id), value_count_(0), checksum_(0) {
    std::fill(values_.begin(), values_.end(), SensorValue());
}

SensorData SensorData::create(uint8_t sensor_id) {
    return SensorData(0, sensor_id); // timestamp will be set later
}

bool SensorData::addValue(SensorType type, float value, uint8_t quality) {
    if (value_count_ >= MAX_SENSOR_VALUES) {
        return false;
    }
    values_[value_count_++] = SensorValue(type, value, quality);
    updateChecksum();
    return true;
}

bool SensorData::addValue(const SensorValue& value) {
    if (value_count_ >= MAX_SENSOR_VALUES) {
        return false;
    }
    values_[value_count_++] = value;
    updateChecksum();
    return true;
}

bool SensorData::removeValue(SensorType type) {
    uint8_t idx = findValueIndex(type);
    if (idx == 255) {
        return false;
    }
    // Shift remaining values left
    for (uint8_t i = idx; i < value_count_ - 1; ++i) {
        values_[i] = values_[i + 1];
    }
    value_count_--;
    updateChecksum();
    return true;
}

bool SensorData::hasValue(SensorType type) const {
    return findValueIndex(type) != 255;
}

bool SensorData::getValue(SensorType type, float& value) const {
    uint8_t idx = findValueIndex(type);
    if (idx == 255) return false;
    value = values_[idx].value;
    return true;
}

const SensorValue* SensorData::getValueByIndex(uint8_t index) const {
    if (index >= value_count_) return nullptr;
    return &values_[index];
}

const SensorValue* SensorData::getValueByType(SensorType type) const {
    uint8_t idx = findValueIndex(type);
    return (idx != 255) ? &values_[idx] : nullptr;
}

std::vector<const SensorValue*> SensorData::getValuesByType(SensorType type) const {
    std::vector<const SensorValue*> result;
    for (uint8_t i = 0; i < value_count_; ++i) {
        if (values_[i].type == type) {
            result.push_back(&values_[i]);
        }
    }
    return result;
}

void SensorData::clearValues() {
    value_count_ = 0;
    updateChecksum();
}

uint16_t SensorData::calculateChecksum() const {
    uint16_t sum = timestamp_ ^ sensor_id_ ^ value_count_;
    for (uint8_t i = 0; i < value_count_; ++i) {
        const auto& v = values_[i];
        uint8_t bytes[sizeof(float) + 1 + 1]; // type + value + quality
        memcpy(bytes, &v.type, 1);
        memcpy(bytes + 1, &v.value, sizeof(float));
        memcpy(bytes + 1 + sizeof(float), &v.quality, 1);
        sum ^= fnv1a_16(bytes, sizeof(bytes));
    }
    return sum;
}

bool SensorData::validate() const {
    return calculateChecksum() == checksum_;
}

void SensorData::updateChecksum() {
    checksum_ = calculateChecksum();
}

uint8_t SensorData::findValueIndex(SensorType type) const {
    for (uint8_t i = 0; i < value_count_; ++i) {
        if (values_[i].type == type) return i;
    }
    return 255;
}

// SensorStats implementation
SensorStats::SensorStats(SensorType t) : type(t), timestamp(0), 
    min_value(0), max_value(0), avg_value(0), std_dev(0), sample_count(0) {}

// DataRange implementation
DataRange::DataRange(uint32_t start, uint32_t end, uint8_t id, bool all) 
    : start_time(start), end_time(end), sensor_id(id), include_all_sensors(all) {}

bool DataRange::containsType(SensorType type) const {
    return include_all_sensors || std::find(types.begin(), types.end(), type) != types.end();
}

bool DataRange::contains(const SensorData& data) const {
    return (data.getTimestamp() >= start_time && data.getTimestamp() <= end_time) &&
           (include_all_sensors || sensor_id == data.getSensorId());
}

// SensorTypeUtils (minimal implementation — expand as needed)
namespace SensorTypeUtils {
    std::string toString(SensorType type) {
        switch (type) {
            case SensorType::TEMPERATURE: return "Temperature";
            case SensorType::HUMIDITY:    return "Humidity";
            case SensorType::PRESSURE:    return "Pressure";
            case SensorType::VOLTAGE:     return "Voltage";
            case SensorType::CURRENT:     return "Current";
            // ... add others
            default: return "Unknown";
        }
    }

    std::string getUnit(SensorType type) {
        switch (type) {
            case SensorType::TEMPERATURE: return "°C";
            case SensorType::HUMIDITY:    return "%";
            case SensorType::PRESSURE:    return "hPa";
            case SensorType::VOLTAGE:     return "V";
            case SensorType::CURRENT:     return "A";
            default: return "";
        }
    }

    float getMinRange(SensorType type) {
        switch (type) {
            case SensorType::TEMPERATURE: return -40.0f;
            case SensorType::HUMIDITY:    return 0.0f;
            case SensorType::PRESSURE:    return 300.0f;
            default: return 0.0f;
        }
    }

    float getMaxRange(SensorType type) {
        switch (type) {
            case SensorType::TEMPERATURE: return 125.0f;
            case SensorType::HUMIDITY:    return 100.0f;
            case SensorType::PRESSURE:    return 1100.0f;
            default: return 10000.0f;
        }
    }

    bool isValidRange(SensorType type, float value) {
        return value >= getMinRange(type) && value <= getMaxRange(type);
    }
}