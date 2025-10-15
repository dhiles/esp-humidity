#ifndef BME280MGR_H
#define BME280MGR_H

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// No #include "driver/i2c.h" -- handled by i2c_bus component

#include "bme280.h"  // Espressif component
#include "bme280mgr.h"  // Your header (now cleaned)

void humidity_start(void);
float humidity_read(void);

#endif