#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bme280.h" // Bosch driver v3

#define TAG "BME280_HUMIDITY"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 100000 // 100 kHz for stability
#define BME280_I2C_ADDR 0x77 // Default per Waveshare manual

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

// Bosch-required types (if not defined in bme280.h)
typedef int8_t s8;
typedef uint8_t u8;
typedef uint16_t u16;
typedef int32_t s32;
typedef uint32_t u32;
#define SUCCESS 0
#define BME280_INIT_VALUE -1
#define SAMPLE_COUNT UINT8_C(50)

static struct bme280_dev bme280_dev;

#define BLINK_GPIO GPIO_NUM_8

void blink_task(void *pvParameters) {
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);
        ESP_LOGI(TAG, "blink on %d", BLINK_GPIO);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        ESP_LOGI(TAG, "blink off %d", BLINK_GPIO);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// I2C bus init
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized");
}

// Bosch I2C write (v3 API: reg_addr, data, len, intf_ptr)
s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr) {
    (void)intf_ptr;
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    if (len > 0) {
        i2c_master_write(cmd, (u8 *)data, len, true);
    }
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(espRc));
    }
    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

// Bosch I2C read (v3 API: reg_addr, data, len, intf_ptr)
s8 BME280_I2C_bus_read(u8 reg_addr, u8 *data, u32 len, void *intf_ptr) {
    (void)intf_ptr;
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // 1. Send Register Address (Write Phase)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // 2. REPEATED START
    i2c_master_start(cmd);
    
    // 3. Send Slave Address + READ bit
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    // 4. Read Data
    if (len > 0) {
        // Read all but the last byte with ACK
        if (len > 1) {
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        }
        // Read the last byte with NACK
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    }
    
    // 5. Stop
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        // Log the failure, but return the Bosch error code type
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(espRc));
    }
    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

// Bosch delay (v3 API)
void BME280_delay_usec(u32 usec, void *intf_ptr) {
    (void)intf_ptr;
    vTaskDelay(pdMS_TO_TICKS(usec / 1000));
}

// Simple I2C scanner (probes CHIPID 0xD0 for BME280 ID 0x60)
void i2c_scanner(void) {
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    bool found = false;
    for (u8 addr = 0x08; addr <= 0x77; addr++) {
        u8 test_data = 0x00;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0xD0, true);  // CHIPID reg
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &test_data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (res == ESP_OK) {
            if (test_data == 0x60) {
                ESP_LOGI(TAG, "BME280 confirmed at address 0x%02X (CHIPID=0x%02X)", addr, test_data);
                found = true;
            } else {
                ESP_LOGI(TAG, "Device found at address 0x%02X (ID=0x%02X)", addr, test_data);
                found = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!found) {
        ESP_LOGE(TAG, "No I2C devices found—check wiring/power/pull-ups!");
    }
}

// Task for reading humidity (adapted from example)
void humidity_reader_task(void *ignore) {
    int8_t rslt;
    uint32_t period;
    struct bme280_settings settings;
    struct bme280_data comp_data;
    int8_t idx = 0;
    uint8_t status_reg;

    // Get current settings before modifying
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    if (rslt != SUCCESS) {
        ESP_LOGE(TAG, "bme280_get_sensor_settings failed: %d", rslt);
        vTaskDelete(NULL);
        return;
    }

    // Configuring the over-sampling rate, filter coefficient and standby time
    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280_dev);
    if (rslt != SUCCESS) {
        ESP_LOGE(TAG, "bme280_set_sensor_settings failed: %d", rslt);
        vTaskDelete(NULL);
        return;
    }

    // Always set the power mode after setting the configuration
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    if (rslt != SUCCESS) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode failed: %d", rslt);
        vTaskDelete(NULL);
        return;
    }

    // Calculate measurement time in microseconds
    rslt = bme280_cal_meas_delay(&period, &settings);
    if (rslt != SUCCESS) {
        ESP_LOGE(TAG, "bme280_cal_meas_delay failed: %d", rslt);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Humidity calculation (Data displayed are compensated values)");
    ESP_LOGI(TAG, "Measurement time : %lu us", (long unsigned int)period);

    while (idx < SAMPLE_COUNT) {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &bme280_dev);
        if (rslt != SUCCESS) {
            ESP_LOGE(TAG, "bme280_get_regs failed: %d", rslt);
            continue;
        }

        if (status_reg & BME280_STATUS_MEAS_DONE) {
            // Measurement time delay given to read sample
            bme280_dev.delay_us(period, bme280_dev.intf_ptr);

            // Read compensated data
            rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, &bme280_dev);
            if (rslt != SUCCESS) {
                ESP_LOGE(TAG, "bme280_get_sensor_data failed: %d", rslt);
                continue;
            }

#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

#ifdef BME280_DOUBLE_ENABLE
            ESP_LOGI(TAG, "Humidity[%d]:   %lf %%RH", idx, comp_data.humidity);
#else
            ESP_LOGI(TAG, "Humidity[%d]:   %lu %%RH", idx, (long unsigned int)comp_data.humidity);
#endif
            idx++;
        }
        // Optional: small delay to avoid busy-waiting (not in example, but good for ESP tasks)
        // vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "Humidity sampling complete");
    vTaskDelete(NULL);
}

void app_main(void) {
 //    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Starting BME280 with Bosch driver v3");

    i2c_master_init();

    // Simple scan first
    i2c_scanner();

    // Setup dev struct
    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    bme280_dev.intf_ptr = NULL;

    // Init
    s32 com_rslt = bme280_init(&bme280_dev);
    if (com_rslt != SUCCESS) {
        ESP_LOGE(TAG, "Init failed: %d (check address/wiring)", com_rslt);
        return;
    }
    ESP_LOGI(TAG, "BME280 initialized at 0x%02X", BME280_I2C_ADDR);

    xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
}