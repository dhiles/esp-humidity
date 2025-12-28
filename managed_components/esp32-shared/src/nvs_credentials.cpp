#include "nvs_credentials.h"

static const char* TAG = "NVS_CREDENTIALS";

void nvs_save_ssid(char* received_ssid)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config.wifi", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    // Save SSID
    err = nvs_set_str(nvs_handle, "ssid", received_ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(err));
    }
    
    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "SSID %s saved to config.wifi namespace.", received_ssid);
}

void nvs_save_password(char* received_pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config.wifi", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    // Save Password
    err = nvs_set_str(nvs_handle, "password", received_pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Password: %s", esp_err_to_name(err));
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "password %s saved to config.wifi namespace.", received_pass);
}


void nvs_save_credentials(char* received_ssid, char* received_pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config.wifi", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    // Save SSID
    err = nvs_set_str(nvs_handle, "ssid", received_ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(err));
    }
    
    // Save Password
    err = nvs_set_str(nvs_handle, "password", received_pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save Password: %s", esp_err_to_name(err));
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Credentials saved to config.wifi namespace.");
}

/**
 * @brief NEW: Erases Wi-Fi credentials from NVS config.wifi namespace.
 * Keys "ssid" and "password" are removed. If namespace doesn't exist, logs and returns.
 */
void nvs_erase_credentials(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config.wifi", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No config.wifi namespace to erase (not found: %s)", esp_err_to_name(err));
        return;
    }

    // Erase SSID
    err = nvs_erase_key(nvs_handle, "ssid");
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to erase SSID key: %s", esp_err_to_name(err));
    }

    // Erase Password
    err = nvs_erase_key(nvs_handle, "password");
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to erase password key: %s", esp_err_to_name(err));
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS erase: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "WiFi credentials erased from config.wifi namespace.");
}

/**
 * @brief Loads Wi-Fi credentials from NVS config.wifi namespace.
 * @return ESP_OK if credentials found, ESP_ERR_NVS_NOT_FOUND otherwise.
 */
esp_err_t nvs_load_credentials(wifi_config_t *wifi_config)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config.wifi", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No config.wifi namespace found");
        return err;
    }

    size_t ssid_len = sizeof(wifi_config->sta.ssid);
    size_t pass_len = sizeof(wifi_config->sta.password);

    // Load SSID
    err = nvs_get_str(nvs_handle, "ssid", (char *)wifi_config->sta.ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No SSID found in config.wifi namespace: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err; // Credentials not found
    }

    // Load Password
    err = nvs_get_str(nvs_handle, "password", (char *)wifi_config->sta.password, &pass_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No password found in config.wifi namespace (open network?)");
        // Continue with empty password for open networks
        wifi_config->sta.password[0] = '\0';
    }

    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Credentials loaded from config.wifi namespace: SSID=%s password=%s", wifi_config->sta.ssid, wifi_config->sta.password);
    return ESP_OK; // Success: Credentials loaded
}