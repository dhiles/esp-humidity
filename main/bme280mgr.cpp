#include "bme280mgr.h"

static const char* TAG = "BME280";

// I2C master initialization
void i2c_master_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000  // 1 MHz
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// BME280 I2C write function (required by driver)
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

// BME280 I2C read function (required by driver)
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

// Delay function (required by driver)
void BME280_delay_msek(u32 msek) {
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

// Task to read BME280 data
void bme280_reader_task(void *ignore) {
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS1,  // 0x76
        .delay_msec = BME280_delay_msek
    };

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    // Configure sensor (oversampling, standby, filter, mode)
    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);  // Humidity oversampling

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    if (com_rslt == SUCCESS) {
        ESP_LOGI(TAG, "BME280 initialized successfully");
        while (true) {
            vTaskDelay(40 / portTICK_PERIOD_MS);  // ~25 Hz sampling

            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

            if (com_rslt == SUCCESS) {
                // Compensate values (focus on humidity)
                double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                
                // Log humidity (primary output)
                ESP_LOGI(TAG, "Humidity: %.3f %%RH", hum);

                // Optional: Full readings for context
                double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100.0;  // Pa to hPa
                ESP_LOGI(TAG, "Temp: %.2f °C, Pressure: %.3f hPa (Humidity: %.3f %%RH)", 
                         temp, press, hum);
            } else {
                ESP_LOGE(TAG, "Read error: %d", com_rslt);
            }
        }
    } else {
        ESP_LOGE(TAG, "Init/config error: %d", com_rslt);
    }

    vTaskDelete(NULL);
}

void bme280start(void) {
    i2c_master_init();
    xTaskCreate(&bme280_reader_task, "bme280_reader_task", 4096, NULL, 5, NULL);  // Increased stack for safety
}