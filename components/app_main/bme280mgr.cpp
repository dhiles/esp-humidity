#include "bme280mgr.h"
#include "ble_provisioning.h"
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

static const char *TAG = "BME280";

// BME280 I2C Configuration
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 10 // Check your physical wiring
#define I2C_MASTER_SCL_IO 11 // Check your physical wiring
#define I2C_MASTER_FREQ_HZ 100000
#define BME280_I2C_ADDR 0x77 // Default, try 0x76 if 0x77 fails

// OLED I2C Configuration
#define I2C_OLED_NUM I2C_NUM_1
#define I2C_OLED_SDA_IO 4
#define I2C_OLED_SCL_IO 5
#define I2C_OLED_FREQ_HZ 400000 // OLED can run faster
#define SSD1306_ADDR 0x3C       // Common; scanner will confirm 0x3C or 0x3D

#define SUCCESS 0
#define BME280_INIT_VALUE -1

// =================================================================
// BME280 DRIVER INTERFACE DEFINITIONS
// =================================================================
// Bosch-required types (if not defined in bme280.h)
typedef int8_t s8;
typedef uint8_t u8;
typedef uint16_t u16;
typedef int32_t s32;
typedef uint32_t u32;

static struct bme280_dev bme280_dev;
struct bme280_data comp_data;

// Display resolution
#define LCD_H_RES 128
#define LCD_V_RES 64

// Pixel buffer (1 bit per pixel, 8 pixels per byte; row-major)
static uint8_t lcd_buffer[LCD_H_RES * LCD_V_RES / 8];

// OLED Globals
static esp_lcd_panel_io_handle_t oled_io_handle = NULL;
static esp_lcd_panel_handle_t oled_panel_handle = NULL;

// =================================================================
// I2C INITIALIZATION (Legacy API for both buses)
// =================================================================

void i2c_master_init(void)
{
    // BME280 Bus (I2C_NUM_0)
    i2c_config_t conf_bme = {};
    conf_bme.mode = I2C_MODE_MASTER;
    conf_bme.sda_io_num = I2C_MASTER_SDA_IO;
    conf_bme.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_bme.scl_io_num = I2C_MASTER_SCL_IO;
    conf_bme.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_bme.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_driver_delete(I2C_MASTER_NUM);
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf_bme));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "BME I2C initialized on SDA:%d, SCL:%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // OLED Bus (I2C_NUM_1)
    i2c_config_t conf_oled = {};
    conf_oled.mode = I2C_MODE_MASTER;
    conf_oled.sda_io_num = I2C_OLED_SDA_IO;
    conf_oled.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_oled.scl_io_num = I2C_OLED_SCL_IO;
    conf_oled.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_oled.master.clk_speed = I2C_OLED_FREQ_HZ;

    i2c_driver_delete(I2C_OLED_NUM);
    ESP_ERROR_CHECK(i2c_param_config(I2C_OLED_NUM, &conf_oled));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_OLED_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "OLED I2C initialized on SDA:%d, SCL:%d", I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);
}

void oled_init(void)
{
    ESP_LOGI(TAG, "Install panel IO (legacy)");
    esp_lcd_panel_io_i2c_config_t io_config = {};
    io_config.dev_addr = SSD1306_ADDR;
    io_config.control_phase_bytes = 1; // 1 byte for command
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.dc_bit_offset = 6;
    io_config.flags = {
        .disable_control_phase = false,
    };
    esp_err_t ret = esp_lcd_new_panel_io_i2c_v1((uint32_t)I2C_OLED_NUM, &io_config, &oled_io_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(oled_io_handle, &panel_config, &oled_panel_handle));

    ESP_LOGI(TAG, "Panel reset, init and turn on");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(oled_panel_handle));
 //   ESP_ERROR_CHECK(esp_lcd_panel_init(oled_panel_handle));
 //   ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(oled_panel_handle, true));
 //   ESP_ERROR_CHECK(esp_lcd_panel_invert_color(oled_panel_handle, true));

 //   memset(lcd_buffer, 0xFF, sizeof(lcd_buffer));
 //   ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(oled_panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer));

    ESP_LOGI(TAG, "OLED initialized");
}

void oled_update_display(float temp, float hum, float press)
{
    memset(lcd_buffer, 0xFF, sizeof(lcd_buffer));

    // Temp bar
    int h_temp = (int)((temp > 0 ? temp : 0) / 50.0f * LCD_V_RES);
    h_temp = (h_temp > LCD_V_RES) ? LCD_V_RES : h_temp;
    for (int y = 0; y < h_temp; ++y)
    {
        for (int x = 5; x < 35; ++x)
        {
            if (x >= LCD_H_RES)
                continue;
            int byte_idx = (y / 8) * LCD_H_RES + x;
            int bit_idx = y % 8;
            lcd_buffer[byte_idx] &= ~(1u << bit_idx);
        }
    }

    // Hum bar
    int h_hum = (int)(hum / 100.0f * LCD_V_RES);
    h_hum = (h_hum > LCD_V_RES) ? LCD_V_RES : h_hum;
    for (int y = 0; y < h_hum; ++y)
    {
        for (int x = 40; x < 70; ++x)
        {
            if (x >= LCD_H_RES)
                continue;
            int byte_idx = (y / 8) * LCD_H_RES + x;
            int bit_idx = y % 8;
            lcd_buffer[byte_idx] &= ~(1u << bit_idx);
        }
    }

    // Press bar
    float p_hpa = press / 100.0f;
    float p_norm = (p_hpa - 900.0f) / 200.0f;
    int h_press = (int)(p_norm * LCD_V_RES);
    h_press = (h_press < 0) ? 0 : (h_press > LCD_V_RES ? LCD_V_RES : h_press);
    for (int y = 0; y < h_press; ++y)
    {
        for (int x = 75; x < 105; ++x)
        {
            if (x >= LCD_H_RES)
                continue;
            int byte_idx = (y / 8) * LCD_H_RES + x;
            int bit_idx = y % 8;
            lcd_buffer[byte_idx] &= ~(1u << bit_idx);
        }
    }

    if (oled_panel_handle)
    {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(oled_panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer));
    }
}

// Bosch I2C functions (legacy)
s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr)
{
    (void)intf_ptr;
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    if (len > 0)
    {
        i2c_master_write(cmd, (u8 *)data, len, true);
    }
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(espRc));
    }
    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 reg_addr, u8 *data, u32 len, void *intf_ptr)
{
    (void)intf_ptr;
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 0)
    {
        if (len > 1)
        {
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    }

    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(espRc));
    }
    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

void BME280_delay_usec(u32 usec, void *intf_ptr)
{
    (void)intf_ptr;
    u32 ms = (usec + 999) / 1000;
    if (ms == 0)
        ms = 1;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// =================================================================
// I2C SCANNER (Legacy for both buses)
// =================================================================

void i2c_scanner(void)
{
    ESP_LOGI(TAG, "Scanning I2C buses...");

    // BME Bus
    ESP_LOGI(TAG, "BME Bus (I2C_NUM_0):");
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            ESP_LOGI(TAG, "  Found at 0x%02X", addr);
        }
        i2c_cmd_link_delete(cmd);
    }

    // OLED Bus
    ESP_LOGI(TAG, "OLED Bus (I2C_NUM_1):");
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_OLED_NUM, cmd, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            ESP_LOGI(TAG, "  Found at 0x%02X", addr);
        }
        i2c_cmd_link_delete(cmd);
    }
    ESP_LOGI(TAG, "Scan complete");
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
    int8_t rslt;
    uint8_t status_reg;
    char msg[100];

    rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
    print_rslt("bme280_get_regs", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return rslt;
    }

    if (status_reg & (1 << 3))
    {
        dev->delay_us(period, dev->intf_ptr);
    }

    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    print_rslt("bme280_get_sensor_data", rslt);

    if (rslt == SUCCESS)
    {
#ifndef BME280_DOUBLE_ENABLE
        float temperature = (float)comp_data.temperature / 100.0f;
        float humidity = (float)comp_data.humidity / 1000.0f;
        float pressure = (float)comp_data.pressure / 256.0f;
#else
        float temperature = comp_data.temperature;
        float humidity = comp_data.humidity;
        float pressure = comp_data.pressure;
#endif
        snprintf(msg, sizeof(msg), "T:%.1fC H:%.1f%% P:%.1fhPa", temperature, humidity, pressure / 100.0f);
        ESP_LOGI(TAG, "%s", msg);
        send_notification_safe(msg);

       // oled_update_display(temperature, humidity, pressure);
    }

    return rslt;
}

void sensor_reader_task(void *pvParameters)
{
    (void)pvParameters;
    int8_t rslt;
    uint32_t period;
    struct bme280_settings settings; // Note: use bme280_settings, not bme280_settings

    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    print_rslt("bme280_get_sensor_settings", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280_dev);
    print_rslt("bme280_set_sensor_settings", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    print_rslt("bme280_set_power_mode", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    rslt = bme280_cal_meas_delay(&period, &settings);
    print_rslt("bme280_cal_meas_delay", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Measurement time: %lu us", period);
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
    ESP_LOGI(TAG, "Starting BME280 with Bosch driver v3 on ESP32-S3");

    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_scanner();

    oled_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    bme280_dev.intf_ptr = NULL;

    s32 com_rslt = bme280_init(&bme280_dev);
    print_rslt("bme280_init", com_rslt);

    if (com_rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "Fatal: Init failed. Check address 0x%02X and wiring.", BME280_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "BME280 successfully initialized.");

    xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
}