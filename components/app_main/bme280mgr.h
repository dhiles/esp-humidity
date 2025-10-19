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

extern struct bme280_data comp_data;

void humidity_start(void);
void humidity_reader_task(void*);
esp_err_t i2c_scanner(void);
void i2c_master_init(void);

#endif