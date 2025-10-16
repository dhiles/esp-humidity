#ifndef BME280MGR_H
#define BME280MGR_H

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "bme280.h"  // Espressif component

extern struct bme280_data comp_data;

void humidity_start(void);
void humidity_reader_task(void*);
void i2c_scanner(void);
void i2c_master_init(void);

#endif