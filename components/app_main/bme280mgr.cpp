#include "bme280mgr.h"
#include "ble_provisioning.h"
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h" // For xSemaphoreCreateMutex, xSemaphoreTake, xSemaphoreGive
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h> // Modern I2C Master API
#include <driver/gpio.h>
#include <u8g2.h>
#include "esp_rom_sys.h"

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
#define SAMPLE_COUNT UINT8_C(50)

// --- Global Handles (Modern API) ---
i2c_master_bus_handle_t i2c_bus_single = NULL;
i2c_master_dev_handle_t bme_i2c_dev_handle = NULL;
// i2c_master_dev_handle_t oled_i2c_dev_handle = NULL;

// --- BME280 Driver Types (assuming these are provided by bme280.h) ---
typedef int8_t s8;
typedef uint8_t u8;
typedef int32_t s32;
typedef uint32_t u32;

static struct bme280_dev bme280_dev;
struct bme280_data comp_data;

static SemaphoreHandle_t i2c_mutex = NULL; // FreeRTOS semaphore handle
/*
// --- OLED Drawing Buffer and Constants ---
#define LCD_H_RES 128
#define LCD_V_RES 64
#define FONT_HEIGHT 8
static uint8_t lcd_buffer[LCD_H_RES * LCD_V_RES / 8]; // 1024 bytes (128*64 / 8)

// --- Function Prototypes ---
void oled_update_display(float temp, float hum, float press); */

// =================================================================
// MODERN I2C INITIALIZATION (Strict Single Bus on 10/11)
// =================================================================
/*
void i2c_master_init_single_bus1(void)
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

    // In your init function (e.g., app_main or setup):
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = (gpio_num_t)SHARED_SDA_IO; // Your SDA pin
    u8g2_esp32_hal.bus.i2c.scl = (gpio_num_t)SHARED_SCL_IO; // Your SCL pin
    u8g2_esp32_hal_init(u8g2_esp32_hal);                    // Initializes I2C with new driver

    // Example for SSD1306 OLED (adjust for your display):
    u8g2_t u8g2;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // Wake up display

    // Set I2C address (shifted left by 1, e.g., 0x3C -> 0x78):
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
}
*/
static i2c_master_dev_handle_t oled_i2c_dev_handle; // For SSD1306

static uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t tx_buf[257];
    static size_t tx_len = 0;
    static uint8_t control_byte = 0x00; // Default command
    static TickType_t timeout = pdMS_TO_TICKS(100);

    // Debug: Log msg sequence (comment out after testing)
    // ESP_LOGI("U8G2_CB", "Msg: %d, arg_int: %d", msg, arg_int);

    switch (msg)
    {
    case U8X8_MSG_BYTE_SET_DC:
        control_byte = (*(uint8_t *)arg_ptr == 0) ? 0x00 : 0x40;
        // ESP_LOGI("U8G2_CB", "SET_DC: control=0x%02X", control_byte);
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        tx_len = 0;
        break;
    case U8X8_MSG_BYTE_SEND:
        // Prefix control if not yet (first SEND)
        if (tx_len == 0)
        {
            tx_buf[0] = control_byte;
            tx_len = 1;
        }
        // Append bytes
        if (tx_len + arg_int <= sizeof(tx_buf))
        {
            memcpy(tx_buf + tx_len, arg_ptr, arg_int);
            tx_len += arg_int;
        }
        else
        {
            ESP_LOGE("U8G2", "Buffer overflow (%d bytes)", tx_len + arg_int);
            return 0;
        }
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        if (tx_len > 0)
        {
            esp_err_t ret;
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)))
            { // Take mutex (timeout 10ms to avoid deadlock)
                ret = i2c_master_transmit(oled_i2c_dev_handle, tx_buf, tx_len, timeout);
                xSemaphoreGive(i2c_mutex); // Always give back
            }
            else
            {
                ESP_LOGE("U8G2", "Failed to acquire I2C mutex");
                ret = ESP_ERR_TIMEOUT;
            }
            if (ret != ESP_OK)
            {
                ESP_LOGE("U8G2", "Transmit failed: %s (len=%d, control=0x%02X)", esp_err_to_name(ret), tx_len, control_byte);
            }
            tx_len = 0;
            return (ret == ESP_OK) ? 1 : 0;
        }
        break;
    default:
        return 0;
    }
    return 1;
}

// Custom GPIO and delay callback (dummy reset for SSD1306, as no dedicated reset pin)
static uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        // No GPIO init needed (pull-ups already set)
        break;
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        break;
    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(arg_int * 10UL); // Fixed: Use esp_rom_delay_us instead of ets_delay_us
        break;
    case U8X8_MSG_DELAY_100NANO:
        esp_rom_delay_us(arg_int / 10UL); // Also fixed here for consistency
        break;
    case U8X8_MSG_GPIO_RESET:
        // Dummy: SSD1306 often has no reset pin; ignore or toggle if wired (e.g., to GPIO_NUM_X)
        // gpio_set_level((gpio_num_t)U8G2_ESP32_HAL_UNDEFINED, arg_int ? 0 : 1);  // If defined
        break;
    default:
        // Ignore other GPIO messages (e.g., CS, DC not used for I2C)
        break;
    }
    return 1;
}

esp_err_t i2c_scanner(void)
{
    ESP_LOGI(TAG, "--- I2C Scanner Starting ---");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(i2c_bus_single, &dev_cfg, &dev_handle);
        if (ret == ESP_OK)
        {
            uint8_t dummy[1] = {0x00}; // Probe read
            ret = i2c_master_receive(dev_handle, dummy, 1, pdMS_TO_TICKS(100));
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            }
            i2c_master_bus_rm_device(dev_handle); // Clean up
        }
    }
    ESP_LOGI(TAG, "--- I2C Scanner Done ---");
    return ESP_OK;
}

void i2c_master_init_single_bus(void)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex(); // Create the mutex (returns NULL if fails)
        if (i2c_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            return; // Or handle error
        }
    }
    // FIX FOR COMPILATION ERROR: The 'sda_pullup_en' field is missing in this ESP-IDF version.
    // We enable internal pull-ups using the standard GPIO function instead,
    // which must be done BEFORE creating the I2C bus.
    ESP_LOGI(TAG, "Setting internal pull-ups on SDA(%d) and SCL(%d) via GPIO functions.", SHARED_SDA_IO, SHARED_SCL_IO);
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SDA_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SCL_IO, GPIO_PULLUP_ONLY));

    // --- 1. Single Shared Bus Configuration (Pins 10/11) ---
    i2c_master_bus_config_t bus_config_single = {
        // Fixed: Declare struct here (local scope)
        .sda_io_num = (gpio_num_t)SHARED_SDA_IO,
        .scl_io_num = (gpio_num_t)SHARED_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true, // Reinforce internal pull-ups
        },
    };
    // Initialize the shared bus

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_single, &i2c_bus_single));
    ESP_LOGI(TAG, "MODERN I2C Bus initialized on SDA:%d, SCL:%d for BME280 and OLED.", SHARED_SDA_IO, SHARED_SCL_IO);

    // --- 2. BME280 Device Configuration (0x77) ---
    i2c_device_config_t dev_config_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME280_FREQ_HZ, // Assume 100kHz; confirm in your defines
    };
    // Add BME280 device to the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_bme, &bme_i2c_dev_handle));
    ESP_LOGI(TAG, "Device BME280 (0x%02X) added.", BME280_I2C_ADDR);

    // --- 3. SSD1306 OLED Device Configuration (0x3C) ---
    i2c_device_config_t dev_config_oled = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_oled, &oled_i2c_dev_handle));
    ESP_LOGI(TAG, "Device SSD1306 OLED (0x%02X) added.", 0x3C);

    // Raw test: Send display off command to verify handle (0x00 control + 0xAE cmd)
    uint8_t test_cmd[] = {0x00, 0xAE};
    esp_err_t test_ret = i2c_master_transmit(oled_i2c_dev_handle, test_cmd, sizeof(test_cmd), pdMS_TO_TICKS(100));
    if (test_ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Raw OLED test transmit OK");
    }
    else
    {
        ESP_LOGE(TAG, "Raw OLED test failed: %s", esp_err_to_name(test_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Short settle delay

    // SSD1306 OLED setup:
    u8g2_t u8g2;
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C); // Moved: Set before init
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                           u8g2_esp32_i2c_byte_cb,
                                           u8g2_esp32_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Post-init settle

    // Test draw:
  //  u8g2_ClearBuffer(&u8g2);
  //  u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
  //  u8g2_DrawStr(&u8g2, 0, 15, "XXXXX");
  //  u8g2_SendBuffer(&u8g2);

    // Test 1: Full screen fill (white box – should light up entire display)
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawBox(&u8g2, 0, 0, 128, 64); // x=0, y=0, width=128, height=64
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(10000)); // View for 2s

    // Test 2: Frame/Outline (border – tests lines without fill)
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawFrame(&u8g2, 0, 0, 128, 64); // Border around full screen
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Test 3: Horizontal/Vertical Lines (grid – tests partial draws)
    u8g2_ClearBuffer(&u8g2);
    for (int x = 0; x < 128; x += 10)
        u8g2_DrawHLine(&u8g2, x, 20, 10); // Horizontal lines
    for (int y = 0; y < 64; y += 10)
        u8g2_DrawVLine(&u8g2, 60, y, 10); // Vertical lines
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI(TAG, "OLED test draw sent");
}

void i2c_master_init_single_bus1(void)
{
    // FIX FOR COMPILATION ERROR: The 'sda_pullup_en' field is missing in this ESP-IDF version.
    // We enable internal pull-ups using the standard GPIO function instead,
    // which must be done BEFORE creating the I2C bus.
    ESP_LOGI(TAG, "Setting internal pull-ups on SDA(%d) and SCL(%d) via GPIO functions.", SHARED_SDA_IO, SHARED_SCL_IO);
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SDA_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SCL_IO, GPIO_PULLUP_ONLY));

    // --- 1. Single Shared Bus Configuration (Pins 10/11) ---
    i2c_master_bus_config_t bus_config_single = {
        // Fixed: Declare struct here (local scope)
        .sda_io_num = (gpio_num_t)SHARED_SDA_IO,
        .scl_io_num = (gpio_num_t)SHARED_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true, // Reinforce internal pull-ups
        },
    };
    // Initialize the shared bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_single, &i2c_bus_single));
    ESP_LOGI(TAG, "MODERN I2C Bus initialized on SDA:%d, SCL:%d for BME280 and OLED.", SHARED_SDA_IO, SHARED_SCL_IO);

    // Optional: Run scanner to detect devices (uncomment for diagnosis)
    // i2c_scanner();

    // --- 2. BME280 Device Configuration (0x77) ---
    i2c_device_config_t dev_config_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME280_FREQ_HZ, // Assume 100kHz; confirm in your defines
    };
    // Add BME280 device to the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_bme, &bme_i2c_dev_handle));
    ESP_LOGI(TAG, "Device BME280 (0x%02X) added.", BME280_I2C_ADDR);

    // --- 3. SSD1306 OLED Device Configuration (0x3C) ---
    i2c_device_config_t dev_config_oled = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C, // Confirmed by scanner
        .scl_speed_hz = 100000, // 100kHz for reliability (matches scanner)
    };
    // Add OLED device to the shared bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_oled, &oled_i2c_dev_handle));
    ESP_LOGI(TAG, "Device SSD1306 OLED (0x%02X) added.", 0x3C);

    // SSD1306 OLED setup (use corrected CB):
    u8g2_t u8g2;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                           u8g2_esp32_i2c_byte_cb,        // Now handles control prefix
                                           u8g2_esp32_gpio_and_delay_cb); // Unchanged
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // Wake up display

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C); // Matches handle (unshifted)

    // Short delay post-init
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test draw (should now succeed):
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 0, 15, "Hello ESP32!");
    u8g2_SendBuffer(&u8g2);
    ESP_LOGI(TAG, "OLED test draw sent");
}

// =================================================================
// BME280 I2C FUNCTIONS (Adapted for Modern API)
// =================================================================
// (BME280_I2C_bus_write, BME280_I2C_bus_read, BME280_delay_usec remain unchanged)

s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle)
        return BME280_INIT_VALUE;

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
    if (!dev_handle)
        return BME280_INIT_VALUE;

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
    if (ms == 0)
        ms = 1;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*
// =================================================================
// CUSTOM SSD1306 DRIVER (Modern API)
// =================================================================
// (oled_send_cmd, oled_send_data, draw_char, draw_text, oled_init, oled_update_display remain unchanged)

static esp_err_t oled_send_cmd(uint8_t cmd)
{
    // Sequence: [START] [ADDR+W] [ACK] [0x00] [ACK] [CMD] [ACK] [STOP]
    uint8_t cmd_buffer[2] = {0x00, cmd}; // Control byte 0x00 (Command)

    esp_err_t ret = i2c_master_transmit(oled_i2c_dev_handle, cmd_buffer, 2, -1);

    return ret;
}


static esp_err_t oled_send_data(const uint8_t *data, size_t len)
{
    // Sequence: [START] [ADDR+W] [ACK] [0x40] [ACK] [DATA...] [ACK] [STOP]
    if (len == 0)
        return ESP_OK;

    // Allocate buffer for Control Byte (1 byte) + Data (len bytes)
    uint8_t *transfer_buffer = (uint8_t *)malloc(len + 1);
    if (transfer_buffer == NULL)
    {
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


static void draw_char(char c, int x_start, int y_start, uint8_t *buffer)
{
    if (x_start < 0 || y_start < 0 || x_start + 8 > LCD_H_RES || y_start + 8 > LCD_V_RES)
    {
        return;
    }

    // Only attempt to draw actual character glyphs (e.g., if you had font data)
    // For this simple placeholder, let's just draw a solid 8x8 block for non-space characters
    // The key is that we SET bits for white, and ensure the background is cleared (all 0s).
    if (c != ' ')
    {
        for (int y = y_start; y < y_start + FONT_HEIGHT; y++)
        {
            for (int x = x_start; x < x_start + 8; x++)
            {
                if (x < LCD_H_RES && y < LCD_V_RES)
                {
                    int page = y / 8;
                    int byte_idx = page * LCD_H_RES + x;
                    int bit_idx = y % 8;

                    if (byte_idx < sizeof(lcd_buffer))
                    {
                        buffer[byte_idx] |= (1u << bit_idx); // Set bit ON (white pixel)
                    }
                }
            }
        }
    }
    // If you had actual font data, this is where you'd iterate through it
    // For example:
    // const uint8_t *font_glyph = get_font_glyph(c);
    // if (font_glyph) {
    //     for (int col = 0; col < 8; col++) {
    //         for (int row = 0; row < 8; row++) {
    //             if ((font_glyph[col] >> row) & 0x01) { // Check if pixel is part of the glyph
    //                 // Set the corresponding bit in lcd_buffer
    //             }
    //         }
    //     }
    // }
}


static void draw_text(const char *text, int x, int y, uint8_t *buffer)
{
    int current_x = x;
    int current_y = y;
    for (int i = 0; text[i] != '\0'; i++)
    {
        draw_char(text[i], current_x, current_y, buffer);
        current_x += 8;
        if (current_x + 8 > LCD_H_RES)
        {
            current_x = x;
            current_y += FONT_HEIGHT;
        }
        if (current_y + FONT_HEIGHT > LCD_V_RES)
        {
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
    oled_send_cmd(0xA6); // Normal Display (!!! This is the key for white on black)
    // IMPORTANT: 0xA7 is Inverse Display (black on white)
    oled_send_cmd(0xAF); // Display ON

    // Clear the internal buffer and push it to the display
    memset(lcd_buffer, 0x00, sizeof(lcd_buffer));
    oled_update_display(0.0f, 0.0f, 0.0f);

    ESP_LOGI(TAG, "OLED initialized successfully with custom driver.");
}

void oled_update_display(float temp, float hum, float press)
{
    // Clear the buffer to all black (all bits 0) before drawing
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

    for (uint8_t page = 0; page < LCD_V_RES / 8; ++page)
    {
        // Set page start address (0xB0 | page), then column start (0x00 and 0x10 for 0,0)
        oled_send_cmd(0xB0 | page); // Set start page address (page 0-7)
        oled_send_cmd(0x00);        // Set low column address (0)
        oled_send_cmd(0x10);        // Set high column address (0)

        // Transfer 128 bytes (one page) of data
        oled_send_data(lcd_buffer + (page * LCD_H_RES), LCD_H_RES);
    }
}
*/

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

/*
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
*/

// =================================================================
// BME280 SENSOR LOGIC (Adapted from your example code)
// =================================================================

static int8_t get_humidity(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = SUCCESS;
    int8_t idx = 0;
    uint8_t status_reg;

    while (idx < SAMPLE_COUNT)
    {
        // Read status register to check if measurement is done
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        print_rslt("bme280_get_regs", rslt);
        if (rslt != SUCCESS)
        {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying on hard error
            continue;
        }

        // Check the 'measuring' bit (bit 3). If 0, measurement is complete.
        if (!(status_reg & BME280_STATUS_MEAS_DONE))
        {
            // If the measurement isn't done, wait for the calculated time
            dev->delay_us(period, dev->intf_ptr);
        }

        /* Read compensated data */
        rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, dev);
        print_rslt("bme280_get_sensor_data", rslt);

        if (rslt == SUCCESS)
        {
// The scaling is needed when BME280_DOUBLE_ENABLE is NOT defined
// because the compensated value is returned in 1000*RH% in integer mode.
#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

            char msg[50];

#ifdef BME280_DOUBLE_ENABLE
            ESP_LOGI(TAG, "Humidity[%d]:   %lf %%RH", idx, comp_data.humidity);
            snprintf(msg, sizeof(msg), "Humidity[%d]:   %lf %%RH", idx, comp_data.humidity);

#else
            ESP_LOGI(TAG, "Humidity[%d]:   %lu %%RH", idx, (long unsigned int)comp_data.humidity);
#endif

            // Use the safe queue wrapper which now uses the global conn_handle internally
            send_notification_safe(msg);

            // idx++;
        }

        // Add a small delay for good measure in a FreeRTOS loop
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    return rslt;
}

// Task wrapper for the main sensor loop
void humidity_reader_task(void *)
{
    int8_t rslt;
    uint32_t period;
    struct bme280_settings settings;

    // Get current settings before modifying
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    print_rslt("bme280_get_sensor_settings", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    /* Configuring the over-sampling rate, filter coefficient and standby time */
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

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    print_rslt("bme280_set_power_mode", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    print_rslt("bme280_cal_meas_delay", rslt);
    if (rslt != SUCCESS)
    {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Measurement time : %lu us", (long unsigned int)period);
    ESP_LOGI(TAG, "Starting %d humidity samples...", SAMPLE_COUNT);

    get_humidity(period, &bme280_dev);

    ESP_LOGI(TAG, "Humidity sampling complete");
    vTaskDelete(NULL);
}

void humidity_start(void)
{
    ESP_LOGI(TAG, "Starting BME280 and OLED on single, shared I2C Bus (SDA 10 / SCL 11).");

    // 1. Initialize the single shared I2C bus and devices using the Modern API
    i2c_master_init_single_bus();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Initialize the OLED
    //  oled_init();
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
    }
    else
    {
        // Set the sensor configuration only if initialization was successful
        // (Assuming the bme280_set_sensor_settings and bme280_set_power_mode calls are handled elsewhere
        // or that the library defaults are good enough for the task to start reading)
        ESP_LOGI(TAG, "BME280 driver successfully initialized. Starting task.");
        xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
    }
}
