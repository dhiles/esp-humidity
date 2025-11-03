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
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#define SUCCESS 0

extern struct bme280_data comp_data;

void humidity_init(void);
int8_t getHumidityReading(float *humidity_value);
//void humidity_reader_task(void*);
void i2c_scanner(void);
void i2c_master_init(void);

#endif