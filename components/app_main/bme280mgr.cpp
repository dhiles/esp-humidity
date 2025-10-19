#include "bme280mgr.h"
#include "ble_provisioning.h"
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h> // Modern I2C Master API
#include <driver/gpio.h>       // gpio_num_t definition

static const char *TAG = "BME280_OLED";

// --- I2C Configuration ---
// Note: We use the BME280 pins (10/11) to define the single I2C bus.
// The OLED pins (4/5) are now redundant and should not be wired.
#define BME280_I2C_ADDR 0x77
#define BME280_SDA_IO 10
#define BME280_SCL_IO 11
#define BME280_FREQ_HZ 100000

// OLED SSD1306
#define SSD1306_ADDR 0x3C
#define OLED_FREQ_HZ 400000

#define SUCCESS 0
#define BME280_INIT_VALUE -1

// --- Global Handles (Modern API - Single Bus) ---
i2c_master_bus_handle_t i2c_bus_handle = NULL; // Single bus for both devices
i2c_master_dev_handle_t bme_i2c_dev_handle = NULL;
i2c_master_dev_handle_t oled_i2c_dev_handle = NULL;

// --- BME280 Driver Types ---
typedef int8_t s8;
typedef uint8_t u8;
typedef int32_t s32;
typedef uint32_t u32;

static struct bme280_dev bme280_dev;
struct bme280_data comp_data;

// --- OLED Drawing Buffer and Constants ---
#define LCD_H_RES 128
#define LCD_V_RES 64
#define FONT_HEIGHT 8
static uint8_t lcd_buffer[LCD_H_RES * LCD_V_RES / 8]; // 1024 bytes (128*64 / 8)

// --- Function Prototypes (REQUIRED for C++) ---
void oled_update_display(float temp, float hum, float press);

void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_master_bus_handle_t temp_bus = i2c_bus_handle;
        esp_err_t res = i2c_master_probe(temp_bus, addr, -1);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X", addr);
        }
    }
}

// =================================================================
// MODERN I2C INITIALIZATION (Single Bus, Two Devices)
// =================================================================

void i2c_master_init_modern(void)
{
    // --- 1. Single I2C Bus Configuration (Using BME280 pins 10 and 11) ---
    i2c_master_bus_config_t bus_config = {
        .sda_io_num = (gpio_num_t)BME280_SDA_IO, // 10
        .scl_io_num = (gpio_num_t)BME280_SCL_IO, // 11
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    // Initialize the single bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "Single I2C Bus initialized on SDA:%d, SCL:%d", BME280_SDA_IO, BME280_SCL_IO);


    // --- 2. BME280 Device Configuration (0x77) ---
    i2c_device_config_t dev_config_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR, // 0x77
        .scl_speed_hz = BME280_FREQ_HZ,
    };
    // Add BME280 device to the single bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_config_bme, &bme_i2c_dev_handle));
    ESP_LOGI(TAG, "BME280 device (0x%X) added to bus.", BME280_I2C_ADDR);


    // --- 3. OLED Device Configuration (0x3C) ---
    i2c_device_config_t dev_config_oled = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR, // 0x3C
        .scl_speed_hz = BME280_FREQ_HZ,
    };
    // Add OLED device to the single bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_config_oled, &oled_i2c_dev_handle));
    ESP_LOGI(TAG, "OLED device (0x%X) added to bus.", SSD1306_ADDR);
}

// =================================================================
// BME280 I2C FUNCTIONS (Adapted for Modern API)
// =================================================================

s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle) return BME280_INIT_VALUE;

    uint8_t write_buffer[len + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data, len);

    esp_err_t espRc = i2c_master_transmit(dev_handle, write_buffer, len + 1, -1);
    
    if (espRc != ESP_OK)
    {
        ESP_LOGE(TAG, "BME280 I2C write failed: %s", esp_err_to_name(espRc));
        return BME280_INIT_VALUE;
    }
    return SUCCESS;
}

s8 BME280_I2C_bus_read(u8 reg_addr, u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle) return BME280_INIT_VALUE;

    // Transmit register address
    esp_err_t espRc = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
    if (espRc != ESP_OK)
    {
        ESP_LOGE(TAG, "BME280 I2C read (addr tx) failed: %s", esp_err_to_name(espRc));
        return BME280_INIT_VALUE;
    }
    
    // Receive data
    espRc = i2c_master_receive(dev_handle, data, len, -1);
    
    if (espRc != ESP_OK)
    {
        ESP_LOGE(TAG, "BME280 I2C read (data rx) failed: %s", esp_err_to_name(espRc));
        return BME280_INIT_VALUE;
    }
    return SUCCESS;
}

void BME280_delay_usec(u32 usec, void *intf_ptr)
{
    (void)intf_ptr;
    u32 ms = (usec + 999) / 1000;
    if (ms == 0) ms = 1;
    vTaskDelay(pdMS_TO_TICKS(ms));
}


// =================================================================
// CUSTOM SSD1306 DRIVER (Modern I2C)
// =================================================================

static esp_err_t oled_send_cmd(uint8_t cmd)
{
    uint8_t cmd_buffer[2] = {0x00, cmd}; // Control byte 0x00 (Command)
    // Uses global oled_i2c_dev_handle which points to the OLED device on the bus
    return i2c_master_transmit(oled_i2c_dev_handle, cmd_buffer, 2, -1);
}

static esp_err_t oled_send_data(const uint8_t *data, size_t len)
{
    uint8_t control_byte = 0x40; // Control byte 0x40 (Data)
    
    // Send control byte
    esp_err_t ret = i2c_master_transmit(oled_i2c_dev_handle, &control_byte, 1, -1);
    if (ret != ESP_OK) return ret;

    // Send the raw data
    return i2c_master_transmit(oled_i2c_dev_handle, data, len, -1);
}

static void draw_char(char c, int x_start, int y_start, uint8_t *buffer) {
    if (x_start < 0 || y_start < 0 || x_start + 8 > LCD_H_RES || y_start + 8 > LCD_V_RES) {
        return;
    }
    
    // Simple drawing: set an 8x8 white block for non-space characters
    if (c != ' ') {
        for (int y = y_start; y < y_start + FONT_HEIGHT; y++) {
            for (int x = x_start; x < x_start + 8; x++) {
                if (x < LCD_H_RES && y < LCD_V_RES) {
                    int page = y / 8;
                    int byte_idx = page * LCD_H_RES + x;
                    int bit_idx = y % 8;
                    
                    if (byte_idx < sizeof(lcd_buffer)) {
                        buffer[byte_idx] |= (1u << bit_idx); // Set bit ON
                    }
                }
            }
        }
    }
}

static void draw_text(const char *text, int x, int y, uint8_t *buffer) {
    int current_x = x;
    int current_y = y;
    for (int i = 0; text[i] != '\0'; i++) {
        draw_char(text[i], current_x, current_y, buffer);
        current_x += 8;
        if (current_x + 8 > LCD_H_RES) {
            current_x = x;
            current_y += FONT_HEIGHT;
        }
        if (current_y + FONT_HEIGHT > LCD_V_RES) {
            break;
        }
    }
}


void oled_init(void)
{
    ESP_LOGI(TAG, "Initializing SSD1306 display via Modern I2C API");

    // Standard SSD1306 Initialization sequence
    ESP_ERROR_CHECK(oled_send_cmd(0xAE)); // Display Off
    ESP_ERROR_CHECK(oled_send_cmd(0xD5)); // Set Display Clock Div/Oscillator Freq
    ESP_ERROR_CHECK(oled_send_cmd(0x80)); 
    ESP_ERROR_CHECK(oled_send_cmd(0xA8)); // Set Multiplex Ratio
    ESP_ERROR_CHECK(oled_send_cmd(LCD_V_RES - 1));
    ESP_ERROR_CHECK(oled_send_cmd(0xD3)); // Set Display Offset
    ESP_ERROR_CHECK(oled_send_cmd(0x00));
    ESP_ERROR_CHECK(oled_send_cmd(0x40)); // Set Start Line
    ESP_ERROR_CHECK(oled_send_cmd(0x8D)); // Charge Pump Setting
    ESP_ERROR_CHECK(oled_send_cmd(0x14)); // Enable Charge Pump
    ESP_ERROR_CHECK(oled_send_cmd(0x20)); // Set Memory Addressing Mode
    ESP_ERROR_CHECK(oled_send_cmd(0x00)); // 0x00 = Horizontal Addressing Mode
    ESP_ERROR_CHECK(oled_send_cmd(0xA1)); // Segment Re-map
    ESP_ERROR_CHECK(oled_send_cmd(0xC8)); // COM Output Scan Direction
    ESP_ERROR_CHECK(oled_send_cmd(0xDA)); // Set COM Pins Hardware Configuration
    ESP_ERROR_CHECK(oled_send_cmd(0x12)); 
    ESP_ERROR_CHECK(oled_send_cmd(0x81)); // Set Contrast
    ESP_ERROR_CHECK(oled_send_cmd(0xCF)); 
    ESP_ERROR_CHECK(oled_send_cmd(0xD9)); // Set Pre-charge Period
    ESP_ERROR_CHECK(oled_send_cmd(0xF1)); 
    ESP_ERROR_CHECK(oled_send_cmd(0xDB)); // Set VCOM Detect
    ESP_ERROR_CHECK(oled_send_cmd(0x40)); 
    ESP_ERROR_CHECK(oled_send_cmd(0xA4)); // Entire Display ON/Resume to RAM content display
    ESP_ERROR_CHECK(oled_send_cmd(0xA6)); // Normal Display
    ESP_ERROR_CHECK(oled_send_cmd(0xAF)); // Display ON

    // Clear the internal buffer and push it to the display
    oled_update_display(0.0f, 0.0f, 0.0f); 

    ESP_LOGI(TAG, "OLED initialized successfully with custom driver.");
}

void oled_update_display(float temp, float hum, float press)
{
    // Clear the buffer
    memset(lcd_buffer, 0x00, sizeof(lcd_buffer));
    
    char temp_str[32];
    char hum_str[32];
    char press_str[32];
    
    snprintf(temp_str, sizeof(temp_str), "T: %.1f C", temp);
    snprintf(hum_str, sizeof(hum_str), "H: %.1f %%", hum);
    snprintf(press_str, sizeof(press_str), "P: %.1f hPa", press / 100.0f);

    // Draw the text
    draw_text("BME280 Data:", 0, 0, lcd_buffer);
    draw_text(temp_str, 0, 1 * FONT_HEIGHT, lcd_buffer);
    draw_text(hum_str, 0, 2 * FONT_HEIGHT, lcd_buffer);
    draw_text(press_str, 0, 3 * FONT_HEIGHT, lcd_buffer);

    // --- Transfer the buffer to the display (Horizontal Addressing Mode) ---
    oled_send_cmd(0x20); // Set Memory Addressing Mode
    oled_send_cmd(0x00); // Horizontal Addressing Mode
    
    oled_send_cmd(0x21); // Set Column Address
    oled_send_cmd(0);    // Start column (0)
    oled_send_cmd(LCD_H_RES - 1); // End column (127)

    oled_send_cmd(0x22); // Set Page Address
    oled_send_cmd(0);    // Start page (0)
    oled_send_cmd((LCD_V_RES / 8) - 1); // End page (7)

    // Transfer all 1024 bytes of data
    oled_send_data(lcd_buffer, sizeof(lcd_buffer));
}


// =================================================================
// BME280 SENSOR LOGIC
// =================================================================

static void print_rslt(const char *func, int8_t rslt)
{
    if (rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "%s failed: Code %d", func, rslt);
    }
    else
    {
        ESP_LOGI(TAG, "%s success.", func);
    }
}

static int8_t read_sensor_data(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = 0; 
    uint8_t status_reg = 0;
    char msg[100];

    // Placeholder data for compilation/testing:
    float temperature = 25.0f;
    float humidity = 60.0f;
    float pressure = 101325.0f; // Pa

    /* UNCOMMENT THIS SECTION WHEN BME280 COMPONENT IS FULLY LINKED 
    rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
    if (rslt == SUCCESS && (status_reg & (1 << 3)))
    {
        dev->delay_us(period, dev->intf_ptr);
    }

    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

    if (rslt == SUCCESS)
    {
        temperature = (float)comp_data.temperature / 100.0f;
        humidity = (float)comp_data.humidity / 1000.0f;
        pressure = (float)comp_data.pressure / 256.0f;
    }
    */

    snprintf(msg, sizeof(msg), "T:%.1fC H:%.1f%% P:%.1fhPa", temperature, humidity, pressure / 100.0f);
    ESP_LOGI(TAG, "%s", msg);
    send_notification_safe(msg);

    // Update the OLED display with the sensor data
  //  oled_update_display(temperature, humidity, pressure);

    return rslt;
}

void sensor_reader_task(void *pvParameters)
{
    (void)pvParameters;
    int8_t rslt = 0;
    uint32_t period = 100000;
    struct bme280_settings settings; 
    
    /* UNCOMMENT THIS SECTION WHEN BME280 COMPONENT IS FULLY LINKED 
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    // ... setting configurations ...
    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280_dev);
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    rslt = bme280_cal_meas_delay(&period, &settings);
    */

    ESP_LOGI(TAG, "Starting continuous readings...");

    while (1)
    {
        read_sensor_data(period, &bme280_dev);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete(NULL);
}

void humidity_start(void)
{
    ESP_LOGI(TAG, "Starting BME280 and OLED with Modern I2C API");

    i2c_master_init_modern();
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_scan();
    oled_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure BME280 driver
    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    // CRITICAL: Pass the BME280 I2C device handle
    bme280_dev.intf_ptr = bme_i2c_dev_handle;

    s32 com_rslt = bme280_init(&bme280_dev);
    print_rslt("bme280_init", com_rslt);

    if (com_rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "Fatal: BME280 Init failed. Check address 0x%02X and wiring.", BME280_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "BME280 successfully initialized.");

    xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
}
