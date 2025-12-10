#include "sht3xmgr.h"
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "SHT3X";

// ====================================================================
// I2C Configuration - Using clean GPIOs 7/8
// ====================================================================
#define SHT3X_SDA_IO         GPIO_NUM_1
#define SHT3X_SCL_IO         GPIO_NUM_2  
#define SHT3X_I2C_ADDR       0x44

// SHT3x Commands (Section 4.3)
#define SHT3X_CMD_MEAS_HIGHREP_STRETCH  0x2C06  // Clock stretching enabled
#define SHT3X_CMD_MEAS_HIGHREP          0x2400  // No clock stretching
#define SHT3X_CMD_SOFT_RESET            0x30A2
#define SHT3X_CMD_HEATER_ENABLE         0x306D
#define SHT3X_CMD_HEATER_DISABLE        0x3066
#define SHT3X_CMD_READ_STATUS           0xF32D
#define SHT3X_CMD_CLEAR_STATUS          0x3041

// Measurement timing (Section 2.4)
#define SHT3X_MEASUREMENT_DELAY_MS      15      // Max for high repeatability

// Global handles
i2c_master_bus_handle_t   i2c_bus_handle     = nullptr;
i2c_master_dev_handle_t   sht3x_dev_handle   = nullptr;

void test_basic_i2c_comm(void) {
    ESP_LOGI(TAG, "=== BASIC I2C COMMUNICATION TEST ===");
    
    // Try to probe the SHT3x device directly
    esp_err_t probe_ret = i2c_master_probe(i2c_bus_handle, SHT3X_I2C_ADDR, pdMS_TO_TICKS(100));
    
    if (probe_ret == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x device detected at 0x%02X", SHT3X_I2C_ADDR);
    } else {
        ESP_LOGE(TAG, "SHT3x device NOT detected at 0x%02X: %s", SHT3X_I2C_ADDR, esp_err_to_name(probe_ret));
    }
    
    // Also try alternative address
    esp_err_t alt_probe_ret = i2c_master_probe(i2c_bus_handle, 0x45, pdMS_TO_TICKS(100));
    if (alt_probe_ret == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x device detected at alternative address 0x45");
    }
}

// ====================================================================
// CRC8 for SHT3x (Section 4.4 - Polynomial 0x31, init 0xFF)
// ====================================================================
static uint8_t sht3x_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static bool sht3x_check_crc(uint16_t value, uint8_t crc)
{
    uint8_t bytes[2] = {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return sht3x_crc8(bytes, 2) == crc;
}

// ====================================================================
// I2C bus initialization
// ====================================================================
void i2c_master_init_single_bus(void)
{
    // Configure I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port           = I2C_NUM_0,
        .sda_io_num         = SHT3X_SDA_IO,
        .scl_io_num         = SHT3X_SCL_IO,
        .clk_source         = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt  = 7,
        .flags              = { .enable_internal_pullup = true }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized (SDA:%d, SCL:%d)", SHT3X_SDA_IO, SHT3X_SCL_IO);

    // Add SHT3x device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT3X_I2C_ADDR,
        .scl_speed_hz    = 100000  // Standard mode (Section 4.2)
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &sht3x_dev_handle));
    ESP_LOGI(TAG, "SHT3x device added (addr 0x%02X)", SHT3X_I2C_ADDR);
}

// ====================================================================
// Soft reset (Section 4.8)
// ====================================================================
esp_err_t sht3x_soft_reset(void)
{
    uint8_t cmd[2] = { 0x30, 0xA2 };  // Soft reset command
    esp_err_t ret = i2c_master_transmit(sht3x_dev_handle, cmd, 2, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Reset time < 1ms (Section 4.8)
    }
    return ret;
}

// ====================================================================
// Read status register (Section 4.9)
// ====================================================================
esp_err_t sht3x_read_status(uint16_t *status)
{
    uint8_t cmd[2] = { 0xF3, 0x2D };  // Read status register
    uint8_t data[3];
    
    esp_err_t ret = i2c_master_transmit(sht3x_dev_handle, cmd, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;
    
    ret = i2c_master_receive(sht3x_dev_handle, data, 3, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) return ret;
    
    if (!sht3x_check_crc((data[0] << 8) | data[1], data[2])) {
        return ESP_ERR_INVALID_CRC;
    }
    
    *status = (data[0] << 8) | data[1];
    return ESP_OK;
}

// ====================================================================
// Main reading function (Section 4.5 - Single Shot Mode)
// ====================================================================
esp_err_t get_sht3x_reading(float *temperature, float *humidity)
{
    if (!sht3x_dev_handle) {
        ESP_LOGE(TAG, "SHT3x device not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    // Single shot, high repeatability, no clock stretching (Section 4.3)
    uint8_t cmd[2] = { 0x24, 0x00 };
    uint8_t data[6];

    // Send measurement command
    esp_err_t ret = i2c_master_transmit(sht3x_dev_handle, cmd, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Measurement command failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for measurement to complete (Section 2.4)
    vTaskDelay(pdMS_TO_TICKS(SHT3X_MEASUREMENT_DELAY_MS));

    // Read measurement data (6 bytes: T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC)
    ret = i2c_master_receive(sht3x_dev_handle, data, 6, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Data read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Parse temperature (Section 4.5)
    uint16_t t_raw = (data[0] << 8) | data[1];
    uint8_t  t_crc = data[2];
    
    // Parse humidity (Section 4.5)  
    uint16_t h_raw = (data[3] << 8) | data[4];
    uint8_t  h_crc = data[5];

    // Verify CRC (Section 4.4)
    if (!sht3x_check_crc(t_raw, t_crc)) {
        ESP_LOGE(TAG, "Temperature CRC error!");
        return ESP_ERR_INVALID_CRC;
    }
    
    if (!sht3x_check_crc(h_raw, h_crc)) {
        ESP_LOGE(TAG, "Humidity CRC error!");
        return ESP_ERR_INVALID_CRC;
    }

    // Convert to physical values (Section 2.5)
    *temperature = -45.0f + 175.0f * (t_raw / 65535.0f);
    *humidity    = 100.0f * (h_raw / 65535.0f);

    // Clamp humidity to valid range (0-100%)
    if (*humidity < 0.0f)   *humidity = 0.0f;
    if (*humidity > 100.0f) *humidity = 100.0f;

    return ESP_OK;
}

// ====================================================================
// Public init function
// ====================================================================
void sht3x_init1(void)
{
    ESP_LOGI(TAG, "Initializing SHT3x sensor...");
    i2c_master_init_single_bus();

    // Soft reset
    if (sht3x_soft_reset() != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed!");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow sensor to stabilize

    // Optional: Read status register to verify communication
    uint16_t status;
    if (sht3x_read_status(&status) == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x status: 0x%04X", status);
    }

    // Test first reading
    float t, h;
    if (get_sht3x_reading(&t, &h) == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x ready! First reading: %.2f°C, %.2f%%RH", t, h);
    } else {
        ESP_LOGE(TAG, "SHT3x communication failed — check wiring!");
        
        // Debug: Try to scan I2C bus
        ESP_LOGI(TAG, "Scanning for I2C devices...");
        // Add I2C scanner here if needed
    }
}

void i2c_scan_devices(void) {
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    
    int found_count = 0;
    
    for (int addr = 1; addr < 127; addr++) {
        // Use i2c_master_probe to check if device exists
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, addr, pdMS_TO_TICKS(50));
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", addr);
            found_count++;
        }
    }
    
    if (found_count == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", found_count);
    }
}

void sht3x_init(void)
{
    ESP_LOGI(TAG, "Initializing SHT3x sensor...");
    
    // Test GPIO functionality first
    ESP_LOGI(TAG, "Testing GPIO configuration...");
    ESP_LOGI(TAG, "SDA: GPIO %d, SCL: GPIO %d", SHT3X_SDA_IO, SHT3X_SCL_IO);
    
    // Initialize I2C bus
    i2c_master_init_single_bus();
    
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus handle is NULL - bus initialization failed!");
        return;
    }
    
    if (!sht3x_dev_handle) {
        ESP_LOGE(TAG, "SHT3x device handle is NULL - device addition failed!");
        return;
    }
    
    ESP_LOGI(TAG, "I2C bus and device handles created successfully");
    
    // Simple I2C scan to test communication
    ESP_LOGI(TAG, "Performing I2C bus scan...");
    i2c_scan_devices();
    
    // Try soft reset with detailed error reporting
    ESP_LOGI(TAG, "Attempting soft reset...");
    esp_err_t reset_ret = sht3x_soft_reset();
    
    if (reset_ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s (0x%x)", esp_err_to_name(reset_ret), reset_ret);
        
        // Check specific error types
        if (reset_ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Timeout - check SCL line and pull-up resistors");
        } else if (reset_ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Invalid state - I2C bus not properly initialized");
        } else if (reset_ret == ESP_FAIL) {
            ESP_LOGE(TAG, "General failure - check wiring and power");
        }
        
        // Test basic I2C communication
        ESP_LOGI(TAG, "Testing basic I2C communication...");
        test_basic_i2c_comm();
        return;
    }
    
    ESP_LOGI(TAG, "Soft reset successful");
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Test communication with a simple read
    float t, h;
    ESP_LOGI(TAG, "Attempting first measurement...");
    esp_err_t read_ret = get_sht3x_reading(&t, &h);
    
    if (read_ret == ESP_OK) {
        ESP_LOGI(TAG, "SHT3x ready! First reading: %.2f°C, %.2f%%RH", t, h);
    } else {
        ESP_LOGE(TAG, "First measurement failed: %s (0x%x)", esp_err_to_name(read_ret), read_ret);
    }
}