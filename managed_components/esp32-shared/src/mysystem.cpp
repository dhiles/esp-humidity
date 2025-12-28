// mysystem.cpp
#include "mysystem.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include <string>

static const char *TAG = "MYSYSTEM";

// Static members
char MySystem::device_id[9] = {0};
bool MySystem::id_generated = false;

static const char base32_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";

void MySystem::generate_device_id()
{
    if (id_generated)
        return;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);

    snprintf(device_id, sizeof(device_id), "%02X%02X", mac[4], mac[5]);
    device_id[4] = '\0'; // Ensure null termination for 4-char string

    id_generated = true;

    ESP_LOGI(TAG, "Permanent 4-char Device ID generated: %s", device_id);
}

const char *MySystem::get_device_id()
{
    generate_device_id(); // lazy + idempotent

    // Use a static flag to track if we've prepended
    static bool prepended = false;
    static std::string full_id;

    if (!prepended)
    {
        full_id = std::string(device::TYPE) + "-" + device_id;
        prepended = true;
    }

    return full_id.c_str();
}

void MySystem::check_psram(void)
{
    if (esp_psram_is_initialized())
    {
        size_t psram_size = esp_psram_get_size();
        size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "PSRAM initialized: Total = %u bytes, Free = %u bytes | Device ID: %s",
                 psram_size, psram_free, get_device_id());
    }
    else
    {
        ESP_LOGE(TAG, "PSRAM not initialized! | Device ID: %s", get_device_id());
    }
}