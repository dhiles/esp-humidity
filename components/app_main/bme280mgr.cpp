#include "bme280mgr.h"
#include "ble_provisioning.h"

static const char *TAG = "BME280";

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 10 // Check your physical wiring
#define I2C_MASTER_SCL_IO 11 // Check your physical wiring
#define I2C_MASTER_FREQ_HZ 100000
#define BME280_I2C_ADDR 0x77 // Default, try 0x76 if 0x77 fails

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define SUCCESS 0
#define BME280_INIT_VALUE -1
#define SAMPLE_COUNT UINT8_C(50)

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

// =================================================================
// ESP-IDF I2C ABSTRACTION FUNCTIONS (Your previous correct code)
// =================================================================

void i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; // Recommended change
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE; // Recommended change
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    // *** MODIFICATION HERE ***
    // Delete the driver if it was already installed, for a clean restart
    i2c_driver_delete(I2C_MASTER_NUM);
    // *** END MODIFICATION ***

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized on SDA:%d, SCL:%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
}

// Bosch I2C write (v3 API: reg_addr, data, len, intf_ptr)
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

// Bosch I2C read (v3 API: reg_addr, data, len, intf_ptr)
s8 BME280_I2C_bus_read(u8 reg_addr, u8 *data, u32 len, void *intf_ptr)
{
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
    if (len > 0)
    {
        // Read all but the last byte with ACK
        if (len > 1)
        {
            i2c_master_read(cmd, data, len - 1, (i2c_ack_type_t)I2C_MASTER_ACK);
        }
        // Read the last byte with NACK
        i2c_master_read_byte(cmd, data + len - 1, (i2c_ack_type_t)I2C_MASTER_NACK);
    }

    // 5. Stop
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

// Bosch delay (v3 API)
void BME280_delay_usec(u32 usec, void *intf_ptr)
{
    (void)intf_ptr;
    // For delays < 1ms, using esp_rom_delay_us is more accurate,
    // but for application tasks, vTaskDelay (FreeRTOS) is better.
    // The BME280 delay is usually for measurement time (~1-10ms), so vTaskDelay is safer.
    u32 ms = (usec + 999) / 1000;
    if (ms == 0)
    {
        ms = 1; // Ensure at least one tick delay if time is very short
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void i2c_scanner(void)
{
    ESP_LOGI(TAG, "Simple I2C scanner starting...");

    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at: 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan complete");
}

// =================================================================
// BME280 SENSOR LOGIC (Adapted from your example code)
// =================================================================

// Helper function to print results (since we can't use bme280_error_codes_print_result)
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
    ESP_LOGI(TAG, "Starting BME280 with Bosch driver v3 on ESP32-S3");

    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(100)); // Short delay after init

    // Simple scan helps confirm wiring/address before driver init
    i2c_scanner();

    // Setup dev struct - replaces bme280_interface_selection
    bme280_dev.intf = BME280_I2C_INTF;
    bme280_dev.read = BME280_I2C_bus_read;
    bme280_dev.write = BME280_I2C_bus_write;
    bme280_dev.delay_us = BME280_delay_usec;
    bme280_dev.intf_ptr = NULL;

    // Init
    s32 com_rslt = bme280_init(&bme280_dev);
    print_rslt("bme280_init", com_rslt);

    if (com_rslt != SUCCESS)
    {
        ESP_LOGE(TAG, "Fatal: Init failed. Check address 0x%02X and wiring.", BME280_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "BME280 successfully initialized.");

    xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
}
