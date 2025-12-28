// sensor_type_utils.cpp
#include "sensor_type_utils.h"
#include <cstring>
#include <algorithm>

namespace SensorTypeUtils {

std::string toString(SensorType type) {
    switch (type) {
        case SensorType::UNKNOWN:       return "unknown";
        case SensorType::TEMPERATURE:   return "temperature";
        case SensorType::HUMIDITY:      return "humidity";
        case SensorType::PRESSURE:      return "pressure";
        case SensorType::LIGHT:         return "light";
        case SensorType::CO2:           return "co2";
        case SensorType::VOC:           return "voc";
        case SensorType::PM25:          return "pm25";
        case SensorType::PM10:          return "pm10";
        case SensorType::BATTERY_LEVEL: return "battery_level";
        case SensorType::SOIL_MOISTURE: return "soil_moisture";
        default:                        return "unknown";
    }
}

SensorType fromString(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "unknown" || lower.empty())            return SensorType::UNKNOWN;
    if (lower == "temperature" || lower == "temp")      return SensorType::TEMPERATURE;
    if (lower == "humidity" || lower == "hum")          return SensorType::HUMIDITY;
    if (lower == "pressure" || lower == "press")        return SensorType::PRESSURE;
    if (lower == "light" || lower == "lux")             return SensorType::LIGHT;
    if (lower == "co2")                                 return SensorType::CO2;
    if (lower == "voc")                                 return SensorType::VOC;
    if (lower == "pm25")                                return SensorType::PM25;
    if (lower == "pm10")                                return SensorType::PM10;
    if (lower == "battery_level" || lower == "battery") return SensorType::BATTERY_LEVEL;
    if (lower == "soil_moisture" || lower == "soil")    return SensorType::SOIL_MOISTURE;

    // Fallback
    return SensorType::UNKNOWN;
}

} // namespace SensorTypeUtils