#include "sht4xmgr.h"
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "SHT40";

// ====================================================================
// SHT40 Configuration (uses GPIO 1 & 2 as before)
// ====================================================================
#define SHT4X_SDA_IO           GPIO_NUM_2
#define SHT4X_SCL_IO           GPIO_NUM_1
#define SHT4X_I2C_ADDR         0x44        // Fixed address for SHT40

// ====================================================================
// SHT4x Commands (from Sensirion SHT4x datasheet)
// ====================================================================
#define SHT4X_CMD_MEASURE_HIGH     0xFD    // High precision (best accuracy)
#define SHT4X_CMD_MEASURE_MED      0xF6    // Medium precision
#define SHT4X_CMD_MEASURE_LOW      0xE0    // Lowest precision (fastest)
#define SHT4X_CMD_SOFT_RESET       0x94
#define SHT4X_CMD_READ_SERIAL      0x89    // Optional: read serial number
#define SHT4X_CMD_HEATER_HIGH      0x39    // Not available on all models
#define SHT4X_CMD_HEATER_MED       0x32
#define SHT4X_CMD_HEATER_LOW       0x2F

// Timing (datasheet section 4.6)
#define SHT4X_MEAS_DELAY_HIGH_MS  10       // Max 8.2 ms → use 10 ms safe
#define SHT4X_MEAS_DELAY_MED_MS    6
#define SHT4X_MEAS_DELAY_LOW_MS    2

// Global handles
static i2c_master_bus_handle_t   i2c_bus_handle   = NULL;
static i2c_master_dev_handle_t   sht4x_dev_handle  = NULL;

// ====================================================================
// CRC8 for SHT4x (same as SHT3x: poly 0x31, init 0xFF)
// ====================================================================
static uint8_t sht4x_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool sht4x_check_crc(uint16_t value, uint8_t crc)
{
    uint8_t bytes[2] = {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return sht4x_crc8(bytes, 2) == crc;
}

// ====================================================================
// I2C Bus Initialization
// ====================================================================
static void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = SHT4X_SDA_IO,
        .scl_io_num        = SHT4X_SCL_IO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags              = { .enable_internal_pullup = true }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C master bus initialized (SDA:%d SCL:%d)", SHT4X_SDA_IO, SHT4X_SCL_IO);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT4X_I2C_ADDR,
        .scl_speed_hz    = 100000,                 // 100 kHz is safe and recommended
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &sht4x_dev_handle));
    ESP_LOGI(TAG, "SHT40 device handle created (0x%02X)", SHT4X_I2C_ADDR);
}

// ====================================================================
// Soft Reset
// ====================================================================
static esp_err_t sht4x_soft_reset(void)
{
    uint8_t cmd = SHT4X_CMD_SOFT_RESET;
    esp_err_t ret = i2c_master_transmit(sht4x_dev_handle, &cmd, 1, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Reset completes in <1 ms
        ESP_LOGI(TAG, "SHT40 soft reset sent");
    }
    return ret;
}

// ====================================================================
// High Precision Measurement (recommended)
// ====================================================================
esp_err_t sht4x_measure_high_precision(float *temperature, float *humidity)
{
    if (!sht4x_dev_handle) {
        ESP_LOGE(TAG, "SHT40 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = SHT4X_CMD_MEASURE_HIGH;
    uint8_t buffer[6];

    // Send measurement command
    esp_err_t ret = i2c_master_transmit(sht4x_dev_handle, &cmd, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measure command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for conversion (max 8.2 ms)
    vTaskDelay(pdMS_TO_TICKS(SHT4X_MEAS_DELAY_HIGH_MS));

    // Read 6 bytes: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC
    ret = i2c_master_receive(sht4x_dev_handle, buffer, 6, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(ret));
        return ret;
    }

    uint16_t t_raw = (buffer[0] << 8) | buffer[1];
    uint8_t  t_crc = buffer[2];
    uint16_t h_raw = (buffer[3] << 8) | buffer[4];
    uint8_t  h_crc = buffer[5];

    // Verify CRCs
    if (!sht4x_check_crc(t_raw, t_crc)) {
        ESP_LOGE(TAG, "Temperature CRC fail (raw=0x%04X, crc=0x%02X)", t_raw, t_crc);
        return ESP_ERR_INVALID_CRC;
    }
    if (!sht4x_check_crc(h_raw, h_crc)) {
        ESP_LOGE(TAG, "Humidity CRC fail (raw=0x%04X, crc=0x%02X)", h_raw, h_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Convert using official SHT4x formulas (identical to SHT3x)
    *temperature = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *humidity    = 100.0f * ((float)h_raw / 65535.0f);

    // Clamp humidity
    if (*humidity < 0.0f)   *humidity = 0.0f;
    if (*humidity > 100.0f) *humidity = 100.0f;

    return ESP_OK;
}

// ====================================================================
// I2C Scanner (helpful for debugging)
// ====================================================================
void sht4x_i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;
    for (int addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(10)) == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X %s", addr, (addr == SHT4X_I2C_ADDR) ? "<-- SHT40" : "");
            found++;
        }
    }
    ESP_LOGI(TAG, "Scan complete. %d device(s) found.", found);
}

// ====================================================================
// Public Initialization Function
// ====================================================================
void sht4x_init(void)
{
    ESP_LOGI(TAG, "Initializing SHT40 sensor on SDA:%d SCL:%d", SHT4X_SDA_IO, SHT4X_SCL_IO);

    i2c_master_init();

    if (!i2c_bus_handle || !sht4x_dev_handle) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus or device");
        return;
    }

    // Optional: scan bus
   // sht4x_i2c_scan();

    // Soft reset
    if (sht4x_soft_reset() != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed — check wiring/power!");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Test first reading
    float temp, hum;
    if (sht4x_measure_high_precision(&temp, &hum) == ESP_OK) {
        ESP_LOGI(TAG, "SHT40 initialized successfully!");
        ESP_LOGI(TAG, "First reading → Temperature: %.2f °C, Humidity: %.2f %%RH", temp, hum);
    } else {
        ESP_LOGE(TAG, "SHT40 communication failed — double-check wiring and pull-ups!");
    }
}

// Optional: wrapper for medium/low precision if you want faster reads
esp_err_t sht4x_measure_medium_precision(float *t, float *h) {
    // Same as above but use SHT4X_CMD_MEASURE_MED and shorter delay
    // ... implement if needed
    return ESP_OK;
}