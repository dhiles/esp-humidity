#include "bme280mgr.h"

static const char *TAG = "BME280";

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_FREQ_HZ 400000 // 1 MHz

static i2c_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280_dev = NULL;

// I2C Scanner function (compatible with i2c_bus v1.4.2)
void i2c_scanner_task(void) {
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    // Step 1: Init I2C bus (uses i2c_bus component's config)
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    i2c_bus_handle_t scanner_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (scanner_bus == NULL) {
        ESP_LOGE(TAG, "Scanner bus creation failed");
        vTaskDelete(NULL);
        return;
    }

    bool found = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Create device handle (no config struct; direct addr + flag)
        i2c_bus_device_handle_t dev = i2c_bus_device_create(scanner_bus, addr, 0);  // 0 = no mem addr
        if (dev != NULL) {
            uint8_t test_data = 0x00;
            esp_err_t res = ESP_FAIL;

            // Probe: Try reading CHIPID (0xD0) for BME280 (expected 0x60)
            res = i2c_bus_read_bytes(dev, 0xD0, 1, &test_data);  // Fixed: len=1, then &test_data
            if (res == ESP_OK && test_data == 0x60) {
                ESP_LOGI(TAG, "BME280 confirmed at address 0x%02X (CHIPID=0x%02X)", addr, test_data);
                found = true;
            } else if (res == ESP_OK) {
                // General device found (non-BME280)
                ESP_LOGI(TAG, "Device found at address 0x%02X (ID=0x%02X)", addr, test_data);
                found = true;
            } else {
                // Fallback ACK test: Try dummy write (if read fails)
                res = i2c_bus_write_bytes(dev, NULL_I2C_MEM_ADDR, 0, NULL);  // 0 bytes to test ACK
                if (res == ESP_OK) {
                    ESP_LOGI(TAG, "Device ACK at address 0x%02X", addr);
                    found = true;
                }
            }
            i2c_bus_device_delete(&dev);  // Delete handle
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between probes
    }
    if (!found) {
        ESP_LOGE(TAG, "No I2C devices found—check wiring, power, or pull-ups!");
    } else {
        ESP_LOGI(TAG, "Scan complete.");
    }
    i2c_bus_delete(&scanner_bus);
    vTaskDelete(NULL);
}

// Implement init from bme280mgr.h
esp_err_t bme280_init_handle(bme280_handle_t *handle)
{
    // Step 1: Init I2C bus (uses i2c_bus component's config)
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (i2c_bus == NULL)
    {
        return ESP_FAIL;
    }

    // Step 2: Create and init BME280 handle
    *handle = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT); // 0x76
    if (*handle == NULL)
    {
        return ESP_FAIL;
    }
    return bme280_default_init(*handle);
}

// Implement read from bme280mgr.h
float bme280_read_humidity(bme280_handle_t handle)
{
    float humidity = 0.0f;
    bme280_read_humidity(handle, &humidity);
    return humidity;
}

// Task for periodic reads
void humidity_reader_task(void *ignore)
{
    float humidity = 0.0f;
    while (true)
    {
        humidity = bme280_read_humidity(bme280_dev);
        ESP_LOGI(TAG, "Humidity: %.2f %%RH", humidity);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task for periodic reads
float humidity_read(void)
{
    return bme280_read_humidity(bme280_dev);
}

void humidity_start(void)
{
    ESP_LOGI(TAG, "Starting BME280 humidity reader");
    i2c_scanner_task();
    if (bme280_init_handle(&bme280_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init BME280");
        return;
    }
    ESP_LOGI(TAG, "BME280 initialized");

    // xTaskCreate(humidity_reader_task, "humidity_reader", 4096, NULL, 5, NULL);
}
