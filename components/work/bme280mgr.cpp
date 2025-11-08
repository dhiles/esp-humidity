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
#define SHARED_SDA_IO 1
#define SHARED_SCL_IO 2

// BME280 Device Settings
#define BME280_I2C_ADDR 0x77
#define BME280_FREQ_HZ 100000

// OLED SSD1306 Device Settings
#define SSD1306_ADDR 0x3C
#define OLED_FREQ_HZ 400000 

#define BME280_INIT_VALUE -1
#define SAMPLE_COUNT UINT8_C(50)

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

// Function to get a single humidity reading
int8_t getHumidityReading1(float *humidity_value)
{
    int8_t rslt = SUCCESS;
    uint8_t status_reg;
    struct bme280_data comp_data;
    uint32_t period;
    struct bme280_settings settings;

    // Get current settings to calculate measurement period
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    if (rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to get sensor settings for single reading");
        return rslt;
    }

    // Calculate measurement time in microseconds
    rslt = bme280_cal_meas_delay(&period, &settings);
    if (rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "Failed to calculate measurement delay for single reading");
        return rslt;
    }

    // Wait for measurement to complete
    do {
        // Read status register to check if measurement is done
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &bme280_dev);
        if (rslt != SUCCESS)
        {
            ESP_LOGE(TAG, "Failed to read status register for single reading");
            return rslt;
        }

        // If measurement isn't done, wait for the calculated time
        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            break;
        }
        bme280_dev.delay_us(period, bme280_dev.intf_ptr);
        
    } while (1);

    // Read compensated humidity data
    rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, &bme280_dev);
    if (rslt == SUCCESS)
    {
        // The scaling is needed when BME280_DOUBLE_ENABLE is NOT defined
        // because the compensated value is returned in 1000*RH% in integer mode.
#ifndef BME280_DOUBLE_ENABLE
        *humidity_value = comp_data.humidity / 1000.0f;
#else
        *humidity_value = (float)comp_data.humidity;
#endif
        ESP_LOGI(TAG, "Single humidity reading: %.2f %%RH", *humidity_value);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get sensor data for single reading");
    }

    return rslt;
}

// int8_t get_humidity(uint32_t period)
int8_t getHumidityReading(float *humidity_value)
{
    int8_t rslt = SUCCESS;
    int8_t idx = 0;
    uint8_t status_reg;

   // while (true) // (idx < SAMPLE_COUNT)
   // {
        // Read status register to check if measurement is done
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &bme280_dev);
        print_rslt("bme280_get_regs", rslt);
        if (rslt != SUCCESS)
        {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying on hard error
            return rslt;
        }

        // Check the 'measuring' bit (bit 3). If 0, measurement is complete.
        if (!(status_reg & BME280_STATUS_MEAS_DONE))
        {
            // If the measurement isn't done, wait for the calculated time
            bme280_dev.delay_us(50, bme280_dev.intf_ptr);
        }

        /* Read compensated data */
        rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, &bme280_dev);
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
            snprintf(msg, sizeof(msg), "{\"humidity\": %.2lf}", comp_data.humidity);
#else
            ESP_LOGI(TAG, "Humidity[%d]:   %lu %%RH", idx, (long unsigned int)comp_data.humidity);
            snprintf(msg, sizeof(msg), "{\"humidity\": %.2f}", (float)comp_data.humidity);
#endif

            *humidity_value = (float)comp_data.humidity;
            // Use the safe queue wrapper which now uses the global conn_handle internally
          //  send_notification_safe(msg);

            idx++;
        }

        // Add a small delay for good measure in a FreeRTOS loop
       // vTaskDelay(pdMS_TO_TICKS(5000));
  //  }

    return rslt;
}

// Task wrapper for the main sensor loop
void set_sensor_reader()
{
    int8_t rslt;
    uint32_t period;
    struct bme280_settings settings;

    // Get current settings before modifying
    rslt = bme280_get_sensor_settings(&settings, &bme280_dev);
    print_rslt("bme280_get_sensor_settings", rslt);
    if (rslt != SUCCESS)
    {
      //  vTaskDelete(NULL);
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
       // vTaskDelete(NULL);
        return;
    }

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);
    print_rslt("bme280_set_sensor_mode", rslt);
    if (rslt != SUCCESS)
    {
      //  vTaskDelete(NULL);
        return;
    }

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    print_rslt("bme280_cal_meas_delay", rslt);
    if (rslt != SUCCESS)
    {
      // vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Measurement time : %lu us", (long unsigned int)period);
    ESP_LOGI(TAG, "Starting %d humidity samples...", SAMPLE_COUNT);

   // get_humidity(period);

  //  ESP_LOGI(TAG, "Humidity sampling complete");
  //  vTaskDelete(NULL);
}

void humidity_init(void)
{
    ESP_LOGI(TAG, "Starting BME280 and OLED on single, shared I2C Bus (SDA 10 / SCL 11).");

    // 1. Initialize the single shared I2C bus and devices using the Modern API
    i2c_master_init_single_bus();
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
        ESP_LOGI(TAG, "BME280 driver successfully initialized");
        set_sensor_reader();
      //  xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
    }
}
