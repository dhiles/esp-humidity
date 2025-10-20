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

// --- CRITICAL FIX: Include the HAL header that was previously causing the linkage issue ---
// The u8g2.h already includes u8x8.h, but if you need other HAL specific functions, you'd include it.
// Assuming linkage is fixed in the u8g2_esp32_hal.h file as discussed.

static const char *TAG = "BME280_OLED";

// --- I2C Configuration ---
#define SHARED_SDA_IO 10
#define SHARED_SCL_IO 11
#define BME280_I2C_ADDR 0x77
#define BME280_FREQ_HZ 100000
#define SSD1306_ADDR 0x3C
#define OLED_FREQ_HZ 400000

#define SUCCESS 0
#define BME280_INIT_VALUE -1
#define SAMPLE_COUNT UINT8_C(50)

// --- U8G2 & I2C Globals ---
i2c_master_bus_handle_t i2c_bus_single = NULL;
i2c_master_dev_handle_t bme_i2c_dev_handle = NULL;
static i2c_master_dev_handle_t oled_i2c_dev_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL; 
static u8g2_t u8g2; 

// CRITICAL FIX: Buffer size for full 128x64 display (1024 bytes) + 1 control byte
#define MAX_OLED_TRANSFER_SIZE (1024 + 1) 
static uint8_t tx_buf[MAX_OLED_TRANSFER_SIZE]; 

// --- BME280 Driver Types (assuming these are provided by bme280.h) ---
typedef int8_t s8;
typedef uint8_t u8;
typedef int32_t s32;
typedef uint32_t u32;

static struct bme280_dev bme280_dev;
struct bme280_data comp_data;

// --- Function Prototypes ---
void oled_update_display(float temp, float hum, float press);


// =================================================================
// U8G2 CALLBACKS (Bridging U8G2 to Modern I2C API)
// =================================================================

static uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static size_t tx_len = 0;
    static uint8_t control_byte = 0x00; 
    static TickType_t timeout = pdMS_TO_TICKS(100);

    switch (msg)
    {
    case U8X8_MSG_BYTE_SET_DC:
        // Set the control byte based on Data/Command (0x40 for data, 0x00 for command)
        control_byte = (*(uint8_t *)arg_ptr == 0) ? 0x00 : 0x40;
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        tx_len = 0;
        break;
    case U8X8_MSG_BYTE_SEND:
        // Prefix control byte if this is the first byte of the transaction
        if (tx_len == 0)
        {
            tx_buf[0] = control_byte;
            tx_len = 1;
        }
        
        // CRITICAL: Check buffer size against the new max size
        if (tx_len + arg_int <= MAX_OLED_TRANSFER_SIZE)
        {
            memcpy(tx_buf + tx_len, arg_ptr, arg_int);
            tx_len += arg_int;
        }
        else
        {
            // This should no longer happen with MAX_OLED_TRANSFER_SIZE = 1025
            ESP_LOGE(TAG, "U8G2 Buffer overflow (%d bytes attempted)", tx_len + arg_int);
            return 0;
        }
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        if (tx_len > 0)
        {
            esp_err_t ret = ESP_OK;
            // Use mutex for I2C access
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)))
            { 
                ret = i2c_master_transmit(oled_i2c_dev_handle, tx_buf, tx_len, timeout);
                xSemaphoreGive(i2c_mutex); 
            }
            else
            {
                ESP_LOGE(TAG, "Failed to acquire I2C mutex for U8G2 transfer");
                ret = ESP_ERR_TIMEOUT;
            }
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "U8G2 Transmit failed: %s (len=%d, control=0x%02X)", 
                         esp_err_to_name(ret), tx_len, control_byte);
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

static uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        break;
    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(arg_int * 10UL); 
        break;
    case U8X8_MSG_DELAY_100NANO:
        // Use us delay for consistency
        esp_rom_delay_us(arg_int / 10UL); 
        break;
    case U8X8_MSG_GPIO_RESET:
        // SSD1306 often has no dedicated reset pin, ignore this if unused
        break;
    default:
        break;
    }
    return 1;
}

// =================================================================
// MODERN I2C INITIALIZATION 
// =================================================================

void i2c_master_init_single_bus(void)
{
    if (i2c_mutex == NULL)
    {
        i2c_mutex = xSemaphoreCreateMutex(); 
        if (i2c_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            return; 
        }
    }
    
    ESP_LOGI(TAG, "Setting internal pull-ups on SDA(%d) and SCL(%d) via GPIO functions.", SHARED_SDA_IO, SHARED_SCL_IO);
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SDA_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SHARED_SCL_IO, GPIO_PULLUP_ONLY));

    // --- 1. Single Shared Bus Configuration (Pins 10/11) ---
    i2c_master_bus_config_t bus_config_single = {
        .sda_io_num = (gpio_num_t)SHARED_SDA_IO,
        .scl_io_num = (gpio_num_t)SHARED_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true, 
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_single, &i2c_bus_single));
    ESP_LOGI(TAG, "MODERN I2C Bus initialized on SDA:%d, SCL:%d.", SHARED_SDA_IO, SHARED_SCL_IO);

    // --- 2. BME280 Device Configuration (0x77) ---
    i2c_device_config_t dev_config_bme = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME280_FREQ_HZ, 
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_bme, &bme_i2c_dev_handle));
    ESP_LOGI(TAG, "Device BME280 (0x%02X) added.", BME280_I2C_ADDR);

    // --- 3. SSD1306 OLED Device Configuration (0x3C) ---
    i2c_device_config_t dev_config_oled = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR, 
        .scl_speed_hz = OLED_FREQ_HZ, 
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_single, &dev_config_oled, &oled_i2c_dev_handle));
    ESP_LOGI(TAG, "Device SSD1306 OLED (0x%02X) added.", SSD1306_ADDR);
}

// =================================================================
// OLED U8G2 FUNCTIONS
// =================================================================

/**
 * @brief Initializes the U8G2 object using the custom I2C callbacks.
 */
void oled_init_u8g2(void)
{
    ESP_LOGI(TAG, "Initializing SSD1306 display via U8G2.");
    
    // Setup the U8G2 object for SSD1306 128x64 I2C display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( 
        &u8g2, 
        U8G2_R0, 
        u8g2_esp32_i2c_byte_cb, 
        u8g2_esp32_gpio_and_delay_cb); 

    // Set the I2C Address (U8G2 expects 7-bit unshifted address for U8X8_MSG_BYTE_INIT)
    // NOTE: This must be done on the u8x8 internal structure, but we rely on the handle for transfers.
    // Setting it here ensures U8G2's internal state is correct for non-transfer messages.
    u8x8_SetI2CAddress(&u8g2.u8x8, SSD1306_ADDR);

    // Initialize and power up the display
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); 
    u8g2_ClearDisplay(&u8g2);    
    
    ESP_LOGI(TAG, "U8G2 initialized successfully.");

    // TEST DRAW: Should now be successful with the larger buffer
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 0, 15, "I2C Works!");
    u8g2_DrawBox(&u8g2, 0, 0, 128, 64); // Draw a full frame box to test the 1024 byte transfer
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Hold for 5s
    u8g2_ClearDisplay(&u8g2); // Clear screen
}

/**
 * @brief Updates the OLED display buffer with BME280 data and sends it to the screen.
 */
void oled_update_display(float temp, float hum, float press)
{
    // Ensure mutex is taken before accessing the I2C device via U8G2
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)))
    {
        u8g2_ClearBuffer(&u8g2);
        
        char temp_str[32];
        char hum_str[32];
        char press_str[32];
        
        // Convert Pa to hPa for readability
        snprintf(temp_str, sizeof(temp_str), "T: %.1f C", temp);
        snprintf(hum_str, sizeof(hum_str), "H: %.1f %%", hum);
        snprintf(press_str, sizeof(press_str), "P: %.1f hPa", press / 100.0f);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr); 
        
        u8g2_DrawStr(&u8g2, 0, 12, "BME280 Monitor");
        
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tr); 
        
        u8g2_DrawStr(&u8g2, 5, 28, temp_str);
        u8g2_DrawStr(&u8g2, 5, 42, hum_str);
        u8g2_DrawStr(&u8g2, 5, 56, press_str);

        u8g2_SendBuffer(&u8g2);
        xSemaphoreGive(i2c_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to update OLED: Mutex timeout.");
    }
}


// =================================================================
// BME280 I2C FUNCTIONS (Using Modern API Handle)
// =================================================================

s8 BME280_I2C_bus_write(u8 reg_addr, const u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle)
        return BME280_INIT_VALUE;

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)))
    {
        uint8_t write_buffer[len + 1];
        write_buffer[0] = reg_addr;
        memcpy(&write_buffer[1], data, len);

        esp_err_t espRc = i2c_master_transmit(dev_handle, write_buffer, len + 1, -1);
        
        xSemaphoreGive(i2c_mutex);

        if (espRc != ESP_OK)
        {
            ESP_LOGE(TAG, "BME280 I2C write failed: %s", esp_err_to_name(espRc));
            return BME280_INIT_VALUE;
        }
        return SUCCESS;
    }
    ESP_LOGE(TAG, "BME280 I2C write failed: Mutex timeout.");
    return BME280_INIT_VALUE;
}

s8 BME280_I2C_bus_read(u8 reg_addr, u8 *data, u32 len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    if (!dev_handle)
        return BME280_INIT_VALUE;
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)))
    {
        // Transmit register address (write phase)
        esp_err_t espRc = i2c_master_transmit(dev_handle, &reg_addr, 1, -1);
        if (espRc == ESP_OK)
        {
            // Receive data (read phase)
            espRc = i2c_master_receive(dev_handle, data, len, -1);
        }

        xSemaphoreGive(i2c_mutex);

        if (espRc != ESP_OK)
        {
            ESP_LOGE(TAG, "BME280 I2C read failed: %s", esp_err_to_name(espRc));
            return BME280_INIT_VALUE;
        }
        return SUCCESS;
    }
    ESP_LOGE(TAG, "BME280 I2C read failed: Mutex timeout.");
    return BME280_INIT_VALUE;
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

/**
 * @brief Continuous task to read BME280 data and update the OLED display.
 */
void humidity_reader_task(void *)
{
    int8_t rslt;
    uint32_t period;
    struct bme280_settings settings;

    // --- 1. Sensor Configuration ---
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    print_rslt("bme280_get_sensor_settings", rslt);
    if (rslt != SUCCESS) { goto task_cleanup; }

    settings.filter = BME280_FILTER_COEFF_2;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.standby_time = BME280_STANDBY_TIME_500_MS; 

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280_dev);
    print_rslt("bme280_set_sensor_settings", rslt);
    if (rslt != SUCCESS) { goto task_cleanup; }

    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    print_rslt("bme280_set_power_mode", rslt);
    if (rslt != SUCCESS) { goto task_cleanup; }

    rslt = bme280_cal_meas_delay(&period, &settings);
    print_rslt("bme280_cal_meas_delay", rslt);
    if (rslt != SUCCESS) { goto task_cleanup; }

    ESP_LOGI(TAG, "Minimum measurement time : %lu us. Starting monitoring loop...", (long unsigned int)period);

    // --- 2. Main Monitoring Loop ---
    while (1)
    {
        bme280_dev.delay_us(period, bme280_dev.intf_ptr);

        // Read compensated data for Temperature, Pressure, and Humidity
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_dev);

        if (rslt == SUCCESS)
        {
            // Convert raw data to standard units
            float temperature = (float)comp_data.temperature / 100.0f;
            float humidity = (float)comp_data.humidity / 1000.0f;
            float pressure = (float)comp_data.pressure / 256.0f;

            char msg[100];
            snprintf(msg, sizeof(msg), "T:%.1fC H:%.1f%% P:%.1fhPa", 
                     temperature, humidity, pressure / 100.0f);
            ESP_LOGI(TAG, "Current Readings: %s", msg);

            // Update BLE notification queue (assuming this function is defined elsewhere)
            // send_notification_safe(msg);

            // Update the OLED display
            oled_update_display(temperature, humidity, pressure);
            
        } else {
            ESP_LOGE(TAG, "Failed to read BME280 data (Error: %d). Retrying...", rslt);
        }

        // Wait for the next sampling interval (5 seconds)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

task_cleanup:
    ESP_LOGE(TAG, "BME280 Task encountered a critical error. Deleting task.");
    vTaskDelete(NULL);
}


void humidity_start(void)
{
    ESP_LOGI(TAG, "Starting BME280 and OLED on single, shared I2C Bus.");

    // 1. Initialize the single shared I2C bus and devices using the Modern API
    i2c_master_init_single_bus();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Initialize the OLED display using U8G2
    oled_init_u8g2();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Configure BME280 driver
    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    bme280_dev.intf_ptr = bme_i2c_dev_handle;

    s32 com_rslt = bme280_init(&bme280_dev);
    print_rslt("bme280_init", com_rslt);

    if (com_rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "BME280 Init failed. Cannot start sensor reading task.");
    }
    else
    {
        ESP_LOGI(TAG, "BME280 driver successfully initialized. Starting monitoring task.");
        xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
    }
}
