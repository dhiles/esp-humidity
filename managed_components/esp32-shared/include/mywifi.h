#ifndef MYWIFI_H
#define MYWIFI_H

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include <mdns.h>
#include "cjson.h"
#include "constants.h"

#define WIFI_MODE_STA_SUCCESS 1
#define WIFI_MODE_AP_SUCCESS 2

#define ESP_ERR_NO_WIFI_CREDENTIALS (-1001) // Match the define in MyWiFi (or include a header)

class MyWiFi
{
public:
    static constexpr int MAX_WIFI_RETRIES = 5;
    static constexpr int MAX_DISCONNECT_RETRIES = 10;
    static esp_netif_t *s_sta_netif;

    /**
     * @brief Performs one-time global Wi-Fi component initializations (NVS, esp_netif, event loop).
     * Must be called ONCE at system startup (e.g., in app_main).
     */
    static void global_init();

    /**
     * @brief Initializes the ESP32 Wi-Fi driver. Can be called multiple times
     * after deinitialization. It's idempotent.
     */
    static void initialize();

    /**
     * @brief Performs a complete teardown of the Wi-Fi stack (driver, netif, etc.).
     * Use this when you want to fully power down Wi-Fi.
     */
    static void full_deinitialize();

    /**
     * @brief Disconnects from the AP and stops the Wi-Fi driver, but keeps
     * the Wi-Fi driver initialized and the netif instance alive for quicker resume.
     * Use this before entering light sleep to keep Wi-Fi in a low-power state.
     */
    static void disconnect_and_stop();

    /**
     * @brief Connects to Wi-Fi with exponential backoff retries. This is the primary
     * public entry point for establishing a Wi-Fi connection. It handles driver init,
     * netif creation, and power save mode.
     * @param uri Not directly used for Wi-Fi, but kept for interface consistency.
     * @return ESP_OK if connected and network ready, ESP_FAIL otherwise after all retries.
     */
    static esp_err_t connectWithBackoff(const char *uri);
    static esp_err_t startAPWithBackoff(void);

    /**
     * @brief Checks if the Wi-Fi is currently connected and has an IP address.
     * @return true if connected and network ready, false otherwise.
     */
    static bool isConnected();

    /**
     * @brief Checks if the network interface has a valid IP and Gateway.
     * @return true if network is ready, false otherwise.
     */
    static bool isNetworkReady();

    /**
     * @brief Public method to perform a full shutdown of the Wi-Fi stack.
     * This is a wrapper for full_deinitialize().
     */
    static void shutdown();
    static bool hasValidCredentials();
    static void wipeCredentials();
    static esp_err_t startWiFiAuto();
    static void start_mdns(void);

    // NEW WIFI CONTROL FUNCTIONS
    /**
     * @brief Enable or disable WiFi functionality
     * @param enabled true to enable WiFi, false to disable
     */
    static void setWiFiEnabled(bool enabled);

    /**
     * @brief Get the current WiFi enabled state
     * @return true if WiFi is enabled, false if disabled
     */
    static bool isWiFiEnabled();

    /**
     * @brief Toggle WiFi enabled state
     */
    static void toggleWiFi();

    /**
     * @brief Check if WiFi driver is currently active/initialized
     * @return true if WiFi driver is active, false otherwise
     */
    static bool isWiFiDriverActive();
    
    static esp_err_t http_event_handler(esp_http_client_event_t *evt);
    static void send_ip_to_server(char* ip_address);

private:
    // Static member variables to maintain Wi-Fi state
    static volatile bool s_connected; /**< True if Wi-Fi is connected and has IP */
    static esp_netif_t *s_ap_netif;
    static esp_event_handler_instance_t s_instance_ap_start; /**< Event handler instance for all Wi-Fi events */ /**< Handle to the AP network interface */ /**< Handle to the STA network interface */
    static esp_event_handler_instance_t s_instance_any_id;                                                                                                  /**< Event handler instance for all Wi-Fi events */
    static esp_event_handler_instance_t s_instance_got_ip;                                                                                                  /**< Event handler instance for IP_EVENT_STA_GOT_IP */
    static int s_wifi_retry_count;                                                                                                                          /**< Current retry count for connection attempts */
    
    // NEW: WiFi enabled state tracking
    static bool s_wifi_enabled; /**< Global WiFi enabled/disabled state */

    // Event handler for Wi-Fi and IP events.
    static void eventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
};

#endif // MYWIFI_H