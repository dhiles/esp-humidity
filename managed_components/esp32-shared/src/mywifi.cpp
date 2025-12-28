#include "mywifi.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h" // For MAC2STR and MACSTR macros
#include "nvs_flash.h"
#include "lwip/ip_addr.h"
#include <cstring>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_credentials.h"
#include "esp_system.h" // For esp_restart()
#include "mdns.h"
#include "mysystem.h"
#include "esp_crt_bundle.h"

static const char *TAG = "MYWIFI";

// Initialize static members
volatile bool MyWiFi::s_connected = false;
esp_netif_t *MyWiFi::s_sta_netif = nullptr;
esp_netif_t *MyWiFi::s_ap_netif = nullptr;
esp_event_handler_instance_t MyWiFi::s_instance_any_id = nullptr;
esp_event_handler_instance_t MyWiFi::s_instance_got_ip = nullptr;
esp_event_handler_instance_t MyWiFi::s_instance_ap_start = nullptr;
int MyWiFi::s_wifi_retry_count = 0;
bool MyWiFi::s_wifi_enabled = true; // WiFi is enabled by default

// =================================================================
// NEW WIFI CONTROL METHODS
// =================================================================

/**
 * @brief Check if WiFi driver is currently active/initialized
 */
bool MyWiFi::isWiFiDriverActive()
{
    wifi_mode_t mode;
    return (esp_wifi_get_mode(&mode) == ESP_OK);
}

/**
 * @brief Get the current WiFi enabled state
 */
bool MyWiFi::isWiFiEnabled()
{
    return s_wifi_enabled;
}

/**
 * @brief Toggle WiFi enabled state
 */
void MyWiFi::toggleWiFi()
{
    setWiFiEnabled(!s_wifi_enabled);
}

/**
 * @brief Enable or disable WiFi functionality
 * @param enabled true to enable WiFi, false to disable
 */
void MyWiFi::setWiFiEnabled(bool enabled)
{
    if (enabled == s_wifi_enabled)
    {
        ESP_LOGI(TAG, "WiFi already %s", enabled ? "enabled" : "disabled");
        return;
    }

    s_wifi_enabled = enabled;
    ESP_LOGI(TAG, "WiFi %s", enabled ? "enabled" : "disabled");

    if (enabled)
    {
        // WiFi is being enabled - restart in appropriate mode
        ESP_LOGI(TAG, "Enabling WiFi...");

        if (hasValidCredentials())
        {
            ESP_LOGI(TAG, "Starting STA mode with existing credentials");
            esp_err_t result = connectWithBackoff(nullptr);
            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "WiFi STA mode started successfully");
            }
            else
            {
                ESP_LOGW(TAG, "Failed to start STA mode, falling back to AP");
                startAPWithBackoff();
            }
        }
        else
        {
            ESP_LOGI(TAG, "No credentials found, starting AP mode");
            startAPWithBackoff();
        }
    }
    else
    {
        // WiFi is being disabled - shut down completely
        ESP_LOGI(TAG, "Disabling WiFi...");

        // Stop MDNS if it was running
        mdns_free();

        // Full deinitialization to save power
        full_deinitialize();

        ESP_LOGI(TAG, "WiFi disabled - power saving mode active");
    }
}

// =================================================================
// EXISTING METHODS (with minor modifications for WiFi control)
// =================================================================

// --- Helper Functions (Internal to MyWiFi) ---
void MyWiFi::global_init()
{
    static bool initialized_once = false;

    if (initialized_once)
    {
        ESP_LOGI(TAG, "Global WiFi components already initialized. Skipping.");
        return;
    }

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize TCP/IP stack (esp_netif)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    initialized_once = true;
    ESP_LOGI(TAG, "Global WiFi components initialized (NVS, NetIF, Event Loop)");
}

// --- Public Interface of MyWiFi ---

bool MyWiFi::isConnected()
{
    return s_connected && MyWiFi::isNetworkReady() && s_wifi_enabled;
}

void MyWiFi::send_ip_to_server(char *ip_address)
{
    if (ip_address == NULL || ip_address[0] == '\0' || strcmp(ip_address, "0.0.0.0") == 0)
    {
        ESP_LOGE(TAG, "Invalid IP address");
        return;
    }

    const char *device_id = MySystem::get_device_id();

    // Manual JSON construction - minimal stack usage
    char json_payload[128]; // Fixed buffer on stack (adjust size as needed)
    snprintf(json_payload, sizeof(json_payload),
             "{\"device_id\":\"%s\",\"ip\":\"%s\"}",
             device_id, ip_address);

    ESP_LOGI(TAG, "Sending: %s", json_payload);

    // HTTP client without event handler
    esp_http_client_config_t config = {};
    config.url = device::SETIP_URL;
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = 10000;
    config.crt_bundle_attach = esp_crt_bundle_attach;

    config.buffer_size = 2048; // Larger buffer for TLS
    //config.cert_pem = NULL;    // Use NULL to use built-in cert bundle
    //config.skip_cert_common_name_check = false;
    config.keep_alive_enable = true;
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client)
    {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, json_payload, strlen(json_payload));

        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "HTTP Status = %d", esp_http_client_get_status_code(client));
        }

        esp_http_client_cleanup(client);
    }
}

/**
 * @brief Checks if the network interface has a valid IP address.
 */
bool MyWiFi::isNetworkReady()
{
    // If WiFi is disabled, network is not ready
    if (!s_wifi_enabled)
    {
        return false;
    }

    esp_netif_t *netif = s_ap_netif ? s_ap_netif : s_sta_netif;
    if (!s_connected || !netif)
        return false;

    bool is_ap_mode = (s_ap_netif != nullptr);

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to get IP info from netif.");
        return false;
    }

    int retries = 0;
    while (ip_info.ip.addr == 0 && retries++ < 20)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to get IP info during wait.");
            return false;
        }
    }

    if (is_ap_mode)
    {
        // In AP mode, only a non-zero IP is needed.
        return ip_info.ip.addr != 0;
    }
    else
    {
        char ip_str[16];
        esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
        MyWiFi::send_ip_to_server(ip_str);
        // In STA mode, both IP and Gateway must be set.
        return (ip_info.ip.addr != 0 && ip_info.gw.addr != 0);
    }
}

// `initialize()` now ONLY initializes the Wi-Fi driver, not global components.
void MyWiFi::initialize()
{
    // Don't initialize if WiFi is disabled
    if (!s_wifi_enabled)
    {
        ESP_LOGI(TAG, "WiFi disabled - skipping initialization");
        return;
    }

    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) == ESP_OK)
    {
        ESP_LOGI(TAG, "WiFi driver already initialized. Mode: %d", mode);
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGI(TAG, "WiFi driver initialized successfully.");
}

void MyWiFi::full_deinitialize()
{
    ESP_LOGI(TAG, "Performing full Wi-Fi deinitialization...");

    if (s_instance_any_id)
    {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_instance_any_id);
        s_instance_any_id = nullptr;
    }
    if (s_instance_got_ip)
    {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_instance_got_ip);
        s_instance_got_ip = nullptr;
    }
    if (s_instance_ap_start)
    {
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, s_instance_ap_start);
        s_instance_ap_start = nullptr;
    }

    if (s_connected)
    {
        ESP_LOGI(TAG, "Disconnecting WiFi...");
        esp_wifi_disconnect();
        s_connected = false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Stopping and deinitializing WiFi driver...");
    esp_wifi_stop();
    esp_wifi_deinit();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (s_sta_netif)
    {
        ESP_LOGI(TAG, "Destroying STA netif...");
        esp_netif_destroy(s_sta_netif);
        s_sta_netif = nullptr;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_ap_netif)
    {
        ESP_LOGI(TAG, "Destroying AP netif...");
        esp_netif_destroy(s_ap_netif);
        s_ap_netif = nullptr;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    s_wifi_retry_count = 0;
    ESP_LOGI(TAG, "Full Wi-Fi deinitialization complete.");
}

void MyWiFi::eventHandler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started");
            s_wifi_retry_count = 0;
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGI(TAG, "WiFi disconnected (reason: %d)", disconn->reason);
            s_connected = false;

            wifi_mode_t current_mode;
            if (esp_wifi_get_mode(&current_mode) == ESP_OK)
            {
                int retry_delay = std::min(1000 * (1 << s_wifi_retry_count), 10000);
                s_wifi_retry_count++;

                if (s_wifi_retry_count > MAX_DISCONNECT_RETRIES)
                {
                    ESP_LOGW(TAG, "Persistent disconnect, allowing main loop to handle full WiFi cycle.");
                    MyWiFi::disconnect_and_stop();
                }
                else
                {
                    ESP_LOGI(TAG, "Retrying WiFi connect in %dms (attempt %d/%d)",
                             retry_delay, s_wifi_retry_count, MAX_DISCONNECT_RETRIES);
                    vTaskDelay(pdMS_TO_TICKS(retry_delay));
                    esp_wifi_connect();
                }
            }
            else
            {
                ESP_LOGI(TAG, "WiFi driver deinitialized or not started. No automatic reconnect from handler.");
            }
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            s_connected = true;
            s_wifi_retry_count = 0;
            break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi AP started");
            s_connected = true;
            s_wifi_retry_count = 0;
            break;

        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "WiFi AP stopped");
            s_connected = false;
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED)
    {
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "AP: Station " MACSTR " assigned IP " IPSTR,
                 MAC2STR(event->mac), IP2STR(&event->ip));
    }
}

esp_err_t MyWiFi::connectWithBackoff(const char *uri)
{
    initialize();

    int retry_delay = 1000;

    for (int attempt = 1; attempt <= MAX_WIFI_RETRIES; attempt++)
    {
        if (!s_sta_netif)
        {
            if (s_ap_netif)
            {
                esp_netif_destroy(s_ap_netif);
                s_ap_netif = nullptr;
            }

            s_sta_netif = esp_netif_create_default_wifi_sta();
            if (!s_sta_netif)
            {
                ESP_LOGE(TAG, "Failed to create STA interface. This is fatal for Wi-Fi.");
                full_deinitialize();
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!s_instance_any_id)
        {
            ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                                &eventHandler, NULL, &s_instance_any_id));
        }
        if (s_instance_ap_start)
        {
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, s_instance_ap_start);
            s_instance_ap_start = nullptr;
        }

        if (!s_instance_got_ip)
        {
            ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                                &eventHandler, NULL, &s_instance_got_ip));
        }

        wifi_config_t wifi_config = {};
        if (nvs_load_credentials(&wifi_config) != ESP_OK)
        {
            ESP_LOGE(TAG, "nvs_load_credentials failed!");
            return ESP_ERR_NO_WIFI_CREDENTIALS;
        }
        ESP_LOGI(TAG, "nvs_load_credentials complete");
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.sta.pmf_cfg.capable = true;
        wifi_config.sta.pmf_cfg.required = false;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "Connecting to WiFi...");
        ESP_ERROR_CHECK(esp_wifi_connect());

        int timeout = 20;
        while (timeout-- > 0)
        {
            if (s_connected && isNetworkReady())
            {
                ESP_LOGI(TAG, "WiFi connected and network ready.");
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        ESP_LOGW(TAG, "WiFi connection timed out in connectWithBackoff(). Attempting cleanup for retry.");
        esp_wifi_disconnect();
        esp_wifi_stop();

        if (attempt < MAX_WIFI_RETRIES)
        {
            ESP_LOGW(TAG, "Connection attempt %d/%d failed, retrying in %dms",
                     attempt, MAX_WIFI_RETRIES, retry_delay);
            vTaskDelay(pdMS_TO_TICKS(retry_delay));
            retry_delay = std::min(retry_delay * 2, 10000);
        }
    }

    // Logging error fixed here by adding the missing argument
    ESP_LOGE(TAG, "Failed to connect after %d attempts. Performing full shutdown.", MAX_WIFI_RETRIES);
    full_deinitialize();
    return ESP_FAIL;
}

void MyWiFi::disconnect_and_stop()
{
    ESP_LOGI(TAG, "Disconnecting and stopping WiFi (keeping driver initialized)...");
    if (s_connected)
    {
        esp_wifi_disconnect();
        s_connected = false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) == ESP_OK)
    {
        esp_wifi_stop();
    }
    else
    {
        ESP_LOGW(TAG, "WiFi driver not initialized, skipping esp_wifi_stop().");
    }
    ESP_LOGI(TAG, "WiFi disconnected and stopped.");
}

esp_err_t MyWiFi::startAPWithBackoff(void)
{
    initialize();

    int retry_delay = 1000;

    for (int attempt = 1; attempt <= MAX_WIFI_RETRIES; attempt++)
    {
        if (!s_ap_netif)
        {
            if (s_sta_netif)
            {
                esp_netif_destroy(s_sta_netif);
                s_sta_netif = nullptr;
            }

            s_ap_netif = esp_netif_create_default_wifi_ap();
            if (!s_ap_netif)
            {
                ESP_LOGE(TAG, "Failed to create AP interface. This is fatal for Wi-Fi.");
                full_deinitialize();
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!s_instance_any_id)
        {
            ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                                &eventHandler, NULL, &s_instance_any_id));
        }

        if (s_instance_got_ip)
        {
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_instance_got_ip);
            s_instance_got_ip = nullptr;
        }

        if (!s_instance_ap_start)
        {
            ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                                                &eventHandler, NULL, &s_instance_ap_start));
        }

        wifi_config_t wifi_config = {};
        strcpy((char *)wifi_config.ap.ssid, "ESP32-AP");
        wifi_config.ap.ssid_len = strlen("ESP32-AP");
        strcpy((char *)wifi_config.ap.password, "password123");
        wifi_config.ap.channel = 1;
        wifi_config.ap.max_connection = 4;
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.ap.pmf_cfg = {.capable = true, .required = false};

        ESP_LOGI(TAG, "AP config loaded (SSID: %s)", (char *)wifi_config.ap.ssid);

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "Starting WiFi AP...");

        int timeout = 20;
        while (timeout-- > 0)
        {
            if (s_connected && isNetworkReady())
            {
                ESP_LOGI(TAG, "WiFi AP started and network ready.");
                return ESP_OK;
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        ESP_LOGW(TAG, "WiFi AP start timed out in startAPWithBackoff(). Attempting cleanup for retry.");
        esp_wifi_stop();

        if (attempt < MAX_WIFI_RETRIES)
        {
            ESP_LOGW(TAG, "AP start attempt %d/%d failed, retrying in %dms",
                     attempt, MAX_WIFI_RETRIES, retry_delay);
            vTaskDelay(pdMS_TO_TICKS(retry_delay));
            retry_delay = std::min(retry_delay * 2, 10000);
        }
    }

    ESP_LOGE(TAG, "Failed to start AP after %d attempts. Performing full shutdown.", MAX_WIFI_RETRIES);
    full_deinitialize();
    return ESP_FAIL;
}

/**
 * @brief NEW: Check if valid credentials are stored in NVS.
 * @return true if SSID length > 0, false otherwise.
 */
bool MyWiFi::hasValidCredentials()
{
    wifi_config_t wifi_config = {};
    esp_err_t err = nvs_load_credentials(&wifi_config);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No credentials in NVS.");
        return false;
    }
    if (strlen((char *)wifi_config.sta.ssid) == 0)
    {
        ESP_LOGW(TAG, "Empty SSID in NVS.");
        return false;
    }
    ESP_LOGI(TAG, "Valid credentials found for SSID: %s", (char *)wifi_config.sta.ssid);
    return true;
}

/**
 * @brief NEW: Wipe credentials from NVS (for reset/provisioning trigger).
 */
void MyWiFi::wipeCredentials()
{
    nvs_erase_credentials(); // Assuming this function exists in nvs_credentials.h
    ESP_LOGI(TAG, "Credentials wiped from NVS.");
}

/**
 * @brief NEW: Automatic WiFi start - tries STA if creds exist, else AP for provisioning.
 * On STA failure, falls back to AP. Call this in app_main() after global_init().
 * @return WIFI_MODE_STA_SUCCESS if STA mode started successfully,
 *         WIFI_MODE_AP_SUCCESS if AP mode started successfully,
 *         ESP_FAIL otherwise.
 */
esp_err_t MyWiFi::startWiFiAuto()
{
    // Check if WiFi is disabled globally
    if (!s_wifi_enabled)
    {
        ESP_LOGI(TAG, "WiFi is disabled - skipping auto start");
        return ESP_FAIL;
    }

    initialize(); // Ensure driver is ready

    if (hasValidCredentials())
    {
        ESP_LOGI(TAG, "Valid credentials found; attempting STA mode.");
        esp_err_t sta_err = connectWithBackoff(nullptr); // uri not used
        if (sta_err == ESP_OK)
        {
            ESP_LOGI(TAG, "STA mode started successfully.");
            return WIFI_MODE_STA_SUCCESS;
        }
        else
        {
            ESP_LOGE(TAG, "STA connection failed; wiping credentials and falling back to AP provisioning.");
            wipeCredentials();
            full_deinitialize(); // Clean up STA
        }
    }
    else
    {
        ESP_LOGI(TAG, "No valid credentials; starting AP provisioning mode.");
    }

    // Fallback/Initial: Start pure AP mode
    esp_err_t ap_err = startAPWithBackoff();
    if (ap_err == ESP_OK)
    {
        ESP_LOGI(TAG, "AP provisioning mode started successfully.");
        return WIFI_MODE_AP_SUCCESS;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start AP mode.");
        return ESP_FAIL;
    }
}

void MyWiFi::shutdown()
{
    ESP_LOGI(TAG, "Shutting down WiFi via MyWiFi::shutdown() (full deinit)...");
    full_deinitialize();
    ESP_LOGI(TAG, "WiFi full shutdown complete.");
}

// Initialize mDNS
void MyWiFi::start_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(WiFiConfig::MDNS_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(WiFiConfig::MDNS_INSTANCENAME));
    ESP_ERROR_CHECK(mdns_service_add("HTTP", "_http", "_tcp", 80, NULL, 0)); // Advertise HTTP service
    ESP_LOGI(TAG, "mDNS started: http://%s.local/", WiFiConfig::MDNS_HOSTNAME);
}
