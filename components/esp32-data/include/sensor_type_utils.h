// sensor_type_utils.h
#ifndef SENSOR_TYPE_UTILS_H
#define SENSOR_TYPE_UTILS_H

#include <string>
#include "sensor_data.h"  // SensorType is defined here

namespace SensorTypeUtils {

std::string toString(SensorType type);
SensorType fromString(const std::string& str);

} // namespace SensorTypeUtils

#endif // SENSOR_TYPE_UTILS_H