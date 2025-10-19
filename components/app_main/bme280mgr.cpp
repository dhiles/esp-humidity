#include "bme280mgr.h"
#include "ble_provisioning.h"
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h> // Modern I2C Master API
#include <driver/gpio.h>

static const char *TAG = "BME280_OLED";

// --- I2C Configuration ---
// Single Shared Bus Pins (Only use the pins where devices were found)
#define SHARED_SDA_IO 10
#define SHARED_SCL_IO 11

// BME280 Device Settings
#define BME280_I2C_ADDR 0x77
#define BME280_FREQ_HZ 100000

// OLED SSD1306 Device Settings
#define SSD1306_ADDR 0x3C
#define OLED_FREQ_HZ 400000 

#define SUCCESS 0
#define BME280_INIT_VALUE -1

// --- Global Handles (Modern API) ---
i2c_master_bus_handle_t i2c_bus_single = NULL;
i2c_master_dev_handle_t bme_i2c_dev_handle = NULL;
i2c_master_dev_handle_t oled_i2c_dev_handle = NULL;

// --- BME280 Driver Types (assuming these are provided by bme280.h) ---
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

// --- Function Prototypes ---
void oled_update_display(float temp, float hum, float press);


// =================================================================
// I2C SCAN FUNCTION (Modern API)
// =================================================================

/**
 * @brief Scans the I2C bus for devices by checking for ACKs across all 7-bit addresses.
 * This uses the i2c_master_probe function, the modern API's dedicated way to check
 * if a device exists and acknowledges its address.
 */
static void i2c_scan_master_bus(i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGI(TAG, "Scanning I2C bus for 7-bit addresses (0x01 to 0x77)...");
    int count = 0;
    
    // Iterate through all possible 7-bit addresses
    for (int addr = 0x01; addr < 0x78; addr++) {
        // i2c_master_probe attempts a simple address write/read transaction.
        // If the device exists and acknowledges its address, ESP_OK is returned.
        // Timeout increased slightly to be more robust during probing.
        esp_err_t ret = i2c_master_probe(
            bus_handle,         // The bus handle
            (uint8_t)addr,      // The address to probe
            pdMS_TO_TICKS(50)   // Timeout increased to 50ms
        );

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  -> Found device at 0x%02X", addr);
            count++;
        }
    }
    ESP_LOGI(TAG, "I2C Scan finished. %d device(s) found.", count);
}


// =================================================================
// MODERN I2C INITIALIZATION (Strict Single Bus on 10/11)
// =================================================================

void i2c_master_init_single_bus(void)
{
    // FIX FOR COMPILATION ERROR: The 'sda_pullup_en' field is missing in this ESP-IDF version.
    // We enable internal pull-ups using the standard GPIO function instead, 
    // which must be done BEFORE creating the I2C bus.
    ESP_LOGI(TAG, "Setting internal pull-ups on SDA(%d) and SCL(%d) via GPIO functions.", SHARED_SDA_IO, SHARED_SCL_IO);
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SDA_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SCL_IO, GPIO_PULLUP_ONLY));


    // --- 1. Single Shared Bus Configuration (Pins 10/11) ---
    i2c_master_bus_config_t bus_config_single = {
        .sda_io_num = (gpio_num_t)SHARED_SDA_IO,
        .scl_io_num = (gpio_num_t)SHARED_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, 
        // .sda_pullup_en and .scl_pullup_en are REMOVED here to fix the compile error.
    };
    // Initialize the shared bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_single, &i2c_bus_single));
    ESP_LOGI(TAG, "MODERN I2C Bus initialized on SDA:%d, SCL:%d for BME280 and OLED.", SHARED_SDA_IO, SHARED_SCL_IO);


    // --- 2. BME280 Device Configuration (0x77) ---
    i2c_device_config_t dev_config_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME280_FREQ_HZ,
    };
    // Add BME280 device to the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_bme, &bme_i2c_dev_handle));
    ESP_LOGI(TAG, "Device BME280 (0x%02X) added.", BME280_I2C_ADDR);


    // --- 3. OLED Device Configuration (0x3C) ---
    i2c_device_config_t dev_config_oled = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR,
        .scl_speed_hz = OLED_FREQ_HZ,
    };
    // Add OLED device to the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_oled, &oled_i2c_dev_handle));
    ESP_LOGI(TAG, "Device OLED (0x%02X) added.", SSD1306_ADDR);
}

// =================================================================
// BME280 I2C FUNCTIONS (Adapted for Modern API)
// =================================================================
// (BME280_I2C_bus_write, BME280_I2C_bus_read, BME280_delay_usec remain unchanged)

s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle) return BME280_INIT_VALUE;

    uint8_t write_buffer[len + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data, len);

    // Transmit register address + data
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

    // Transmit register address (write phase)
    esp_err_t espRc = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
    if (espRc != ESP_OK)
    {
        ESP_LOGE(TAG, "BME280 I2C read (addr tx) failed: %s", esp_err_to_name(espRc));
        return BME280_INIT_VALUE;
    }
    
    // Receive data (read phase)
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
// CUSTOM SSD1306 DRIVER (Modern API)
// =================================================================
// (oled_send_cmd, oled_send_data, draw_char, draw_text, oled_init, oled_update_display remain unchanged)

/**
 * @brief Send an SSD1306 command (Fixed to ensure protocol compliance)
 */
static esp_err_t oled_send_cmd(uint8_t cmd)
{
    // Sequence: [START] [ADDR+W] [ACK] [0x00] [ACK] [CMD] [ACK] [STOP]
    uint8_t cmd_buffer[2] = {0x00, cmd}; // Control byte 0x00 (Command)
    
    esp_err_t ret = i2c_master_transmit(oled_i2c_dev_handle, cmd_buffer, 2, -1);
    
    return ret;
}

/**
 * @brief Send display data (GDDRAM)
 */
static esp_err_t oled_send_data(const uint8_t *data, size_t len)
{
    // Sequence: [START] [ADDR+W] [ACK] [0x40] [ACK] [DATA...] [ACK] [STOP]
    if (len == 0) return ESP_OK;

    // Allocate buffer for Control Byte (1 byte) + Data (len bytes)
    uint8_t *transfer_buffer = (uint8_t *)malloc(len + 1);
    if (transfer_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for OLED transfer buffer.");
        return ESP_ERR_NO_MEM;
    }

    transfer_buffer[0] = 0x40; // Control byte 0x40 (Data)
    memcpy(&transfer_buffer[1], data, len);

    // Transmit the control byte and data in a single transaction
    esp_err_t ret = i2c_master_transmit(oled_i2c_dev_handle, transfer_buffer, len + 1, -1);

    free(transfer_buffer);
    return ret;
}

/**
 * @brief Draws a single 8x8 character using simple font logic
 */
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

/**
 * @brief Draws text using the placeholder font
 */
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
    ESP_LOGI(TAG, "Initializing SSD1306 display via custom Modern I2C driver.");

    // Standard SSD1306 Initialization sequence
    oled_send_cmd(0xAE); // Display Off
    oled_send_cmd(0xD5); // Set Display Clock Div/Oscillator Freq (0x80)
    oled_send_cmd(0x80); 
    oled_send_cmd(0xA8); // Set Multiplex Ratio (0x3F for 64)
    oled_send_cmd(LCD_V_RES - 1); 
    oled_send_cmd(0xD3); // Set Display Offset (0x00)
    oled_send_cmd(0x00);
    oled_send_cmd(0x40); // Set Start Line (0x00)
    oled_send_cmd(0x8D); // Charge Pump Setting (0x14 - enable)
    oled_send_cmd(0x14); 
    oled_send_cmd(0x20); // Set Memory Addressing Mode (0x02 - Page Addressing)
    oled_send_cmd(0x02); 
    oled_send_cmd(0xA1); // Segment Re-map
    oled_send_cmd(0xC8); // COM Output Scan Direction
    oled_send_cmd(0xDA); // Set COM Pins Hardware Configuration (0x12)
    oled_send_cmd(0x12); 
    oled_send_cmd(0x81); // Set Contrast (0xCF)
    oled_send_cmd(0xCF); 
    oled_send_cmd(0xD9); // Set Pre-charge Period (0xF1)
    oled_send_cmd(0xF1); 
    oled_send_cmd(0xDB); // Set VCOM Detect (0x40)
    oled_send_cmd(0x40); 
    oled_send_cmd(0xA4); // Entire Display ON/Resume to RAM content display
    oled_send_cmd(0xA6); // Normal Display
    oled_send_cmd(0xAF); // Display ON

    // Clear the internal buffer and push it to the display
    memset(lcd_buffer, 0x00, sizeof(lcd_buffer));
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
    
    // Convert Pa to hPa for readability
    snprintf(temp_str, sizeof(temp_str), "T: %.1f C", temp);
    snprintf(hum_str, sizeof(hum_str), "H: %.1f %%", hum);
    snprintf(press_str, sizeof(press_str), "P: %.1f hPa", press / 100.0f);

    // Draw the text
    draw_text("BME280 Data:", 0, 0, lcd_buffer);
    draw_text(temp_str, 0, 1 * FONT_HEIGHT, lcd_buffer);
    draw_text(hum_str, 0, 2 * FONT_HEIGHT, lcd_buffer);
    draw_text(press_str, 0, 3 * FONT_HEIGHT, lcd_buffer);

    // --- Transfer the buffer to the display (Page Addressing Mode) ---

    for (uint8_t page = 0; page < LCD_V_RES / 8; ++page) {
        // Set page start address (0xB0 | page), then column start (0x00 and 0x10 for 0,0)
        oled_send_cmd(0xB0 | page); // Set start page address (page 0-7)
        oled_send_cmd(0x00);        // Set low column address (0)
        oled_send_cmd(0x10);        // Set high column address (0)
        
        // Transfer 128 bytes (one page) of data
        oled_send_data(lcd_buffer + (page * LCD_H_RES), LCD_H_RES);
    }
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
    int8_t rslt = BME280_INIT_VALUE;
    
    // Use the Bosch driver to read the data
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

    if (rslt == SUCCESS)
    {
        // Convert and format the data
        float temperature = (float)comp_data.temperature / 100.0f;
        float humidity = (float)comp_data.humidity / 1000.0f;
        float pressure = (float)comp_data.pressure / 256.0f; // Pa

        char msg[100];
        // Log the final user-friendly data
        snprintf(msg, sizeof(msg), "T:%.1fC H:%.1f%% P:%.1fhPa", temperature, humidity, pressure / 100.0f);
        ESP_LOGI(TAG, "%s", msg);
        send_notification_safe(msg);

        // Update the OLED display with the sensor data
        oled_update_display(temperature, humidity, pressure);
    } else {
        ESP_LOGE(TAG, "Failed to read BME280 data.");
    }

    return rslt;
}

void sensor_reader_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t period = 100000;
    
    // BME280 setup logic goes here...

    ESP_LOGI(TAG, "Starting continuous readings...");

    while (1)
    {
        // Wait for the measurement delay time. Using the wrapper function name.
        BME280_delay_usec(period, bme280_dev.intf_ptr); 

        // Read the data and update display/notifications
        read_sensor_data(period, &bme280_dev);
        
        // Wait for the next reading (5 seconds)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete(NULL);
}

void humidity_start(void)
{
    ESP_LOGI(TAG, "Starting BME280 and OLED on single, shared I2C Bus (SDA 10 / SCL 11).");

    // 1. Initialize the single shared I2C bus and devices using the Modern API
    i2c_master_init_single_bus();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Run the scan to confirm the addresses
  //  i2c_scan_master_bus(i2c_bus_single);


    // 2. Initialize the OLED
    oled_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Configure BME280 driver
    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    // CRITICAL: Pass the BME280 device handle
    bme280_dev.intf_ptr = bme_i2c_dev_handle;

    s32 com_rslt = bme280_init(&bme280_dev);
    print_rslt("bme280_init", com_rslt);

    if (com_rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "BME280 Init failed. Cannot start sensor reading task.");
    } else {
        // Set the sensor configuration only if initialization was successful
        // (Assuming the bme280_set_sensor_settings and bme280_set_power_mode calls are handled elsewhere 
        // or that the library defaults are good enough for the task to start reading)
        ESP_LOGI(TAG, "BME280 driver successfully initialized. Starting task.");
        xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
    }
}
