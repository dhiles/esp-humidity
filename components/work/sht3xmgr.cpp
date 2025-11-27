// components/work/sht3xmgr.cpp
#include "sht3xmgr.h"
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "SHT3X";

// ====================================================================
// I2C Configuration (shared bus with OLED)
// ====================================================================
#define SHARED_SDA_IO       GPIO_NUM_7
#define SHARED_SCL_IO       GPIO_NUM_8
#define SHT3X_I2C_ADDR      0x44

// SHT3x Commands (single shot, high repeatability)
#define SHT3X_CMD_MEAS_HIGHREP   0x2400
#define SHT3X_CMD_SOFT_RESET     0x30A2

// Global handles
i2c_master_bus_handle_t   i2c_bus_handle     = nullptr;
i2c_master_dev_handle_t   sht3x_dev_handle   = nullptr;

// ====================================================================
// CRC8 for SHT3x (polynomial 0x31, init 0xFF)
// ====================================================================
static uint8_t sht3x_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

static bool sht3x_check_crc(uint16_t value, uint8_t crc)
{
    // Fixed: explicit cast to uint8_t — no narrowing warning
    uint8_t bytes[2] = {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return sht3x_crc8(bytes, 2) == crc;
}

// ====================================================================
// I2C bus initialization (shared with OLED)
// ====================================================================
void i2c_master_init_single_bus(void)
{
    // Enable internal pull-ups via GPIO (safe)
    ESP_ERROR_CHECK(gpio_set_pull_mode(SHARED_SDA_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SHARED_SCL_IO, GPIO_PULLUP_ONLY));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port           = I2C_NUM_0,
        .sda_io_num         = SHARED_SDA_IO,
        .scl_io_num         = SHARED_SCL_IO,
        .clk_source         = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt  = 7,
        .flags              = { .enable_internal_pullup = true }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle));
    ESP_LOGI(TAG, "Shared I2C bus initialized (SDA:%d, SCL:%d)", SHARED_SDA_IO, SHARED_SCL_IO);

    // Add SHT3x device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT3X_I2C_ADDR,
        .scl_speed_hz    = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &sht3x_dev_handle));
    ESP_LOGI(TAG, "SHT3x device added (addr 0x%02X)", SHT3X_I2C_ADDR);
}

// ====================================================================
// Soft reset
// ====================================================================
esp_err_t sht3x_soft_reset(void)
{
    uint8_t cmd[2] = { 0x30, 0xA2 };
    return i2c_master_transmit(sht3x_dev_handle, cmd, 2, pdMS_TO_TICKS(1000));
}

// ====================================================================
// Main reading function
// ====================================================================
esp_err_t get_sht3x_reading(float *temperature, float *humidity)
{
    if (!sht3x_dev_handle) {
        ESP_LOGE(TAG, "SHT3x device not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd[2] = { 0x24, 0x00 };  // High repeatability, no clock stretching
    uint8_t data[6];

    esp_err_t ret = i2c_master_transmit(sht3x_dev_handle, cmd, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(20));  // Max 15 ms needed

    ret = i2c_master_receive(sht3x_dev_handle, data, 6, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;

    uint16_t t_raw = (data[0] << 8) | data[1];
    uint8_t  t_crc = data[2];
    uint16_t h_raw = (data[3] << 8) | data[4];
    uint8_t  h_crc = data[5];

    if (!sht3x_check_crc(t_raw, t_crc) || !sht3x_check_crc(h_raw, h_crc)) {
        ESP_LOGE(TAG, "SHT3x CRC error!");
        return ESP_ERR_INVALID_CRC;
    }

    *temperature = -45.0f + 175.0f * (t_raw / 65535.0f);
    *humidity    = 100.0f * (h_raw / 65535.0f);

    // Clamp humidity to 0–100%
    if (*humidity < 0.0f)   *humidity = 0.0f;
    if (*humidity > 100.0f) *humidity = 100.0f;

    return ESP_OK;
}

// ====================================================================
// Public init function (called from work.cpp)
// ====================================================================
void sht3x_init(void)
{
    ESP_LOGI(TAG, "Initializing SHT3x sensor...");
    i2c_master_init_single_bus();

    // Soft reset + small delay
    sht3x_soft_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Test first reading
    float t, h;
    if (get_sht3x_reading(&t, &h) == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x ready! First reading: %.2f°C, %.2f%%RH", t, h);
    } else {
        ESP_LOGE(TAG, "SHT3x communication failed — check wiring!");
    }
}