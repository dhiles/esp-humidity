#include "mymqtt.h"
#include "mywifi.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <vector>
#include <algorithm> // For std::find

static const char* TAG = "MYMQTT";
volatile bool MyMQTT::s_connected = false;
esp_mqtt_client_handle_t MyMQTT::s_client = nullptr;
SemaphoreHandle_t MyMQTT::s_connection_sem = nullptr;
std::vector<std::string> MyMQTT::s_subscribed_topics;
void (*MyMQTT::s_message_handler)(const char *, const char *) = nullptr;

bool MyMQTT::isConnected()
{
    return s_connected;
}

void MyMQTT::initialize()
{
    if (s_connection_sem == nullptr)
    {
        s_connection_sem = xSemaphoreCreateBinary();
        if (s_connection_sem == nullptr)
        {
            ESP_LOGE(TAG, "Failed to create connection semaphore");
        }
    }
    // Pre-allocate space for 10 topics to reduce heap fragmentation
    s_subscribed_topics.reserve(10);
}

void MyMQTT::waitForConnection()
{
    // Wait up to 15 seconds for connection
    if (xSemaphoreTake(s_connection_sem, pdMS_TO_TICKS(15000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "MQTT connection timeout");
        if (s_client)
        {
            esp_mqtt_client_stop(s_client);
            s_client = nullptr;
        }
    }
}

esp_err_t MyMQTT::connect(const char *uri)
{
    initialize(); // Ensure WiFi is connected first
    if (!MyWiFi::isConnected())
    {
        ESP_LOGE(TAG, "WiFi not connected - cannot connect MQTT");
        return ESP_FAIL;
    }
    // Clean up existing connection if needed
    if (s_client)
    {
        esp_mqtt_client_disconnect(s_client);
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client); // Free resources
        s_client = nullptr;
        s_connected = false;
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for cleanup
    }
    // Configure MQTT client with longer timeouts
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = uri;
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.session.keepalive = 60;
    mqtt_cfg.network.timeout_ms = 30000;
    mqtt_cfg.network.reconnect_timeout_ms = 2000;
    // Initialize new client
    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client)
    {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    // Register event handler
    esp_mqtt_client_register_event(s_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, eventHandler, NULL);
    // Start connection
    ESP_LOGI(TAG, "Connecting to MQTT broker...");
    esp_err_t ret = esp_mqtt_client_start(s_client);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }
    waitForConnection();
    // Resubscribe to topics if connected
    if (s_connected)
    {
        ESP_LOGI(TAG, "Resubscribing to %d topics", s_subscribed_topics.size());
        for (const auto &topic : s_subscribed_topics)
        {
            esp_mqtt_client_subscribe(s_client, topic.c_str(), 0);
            vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between subscriptions
        }
    }
    return s_connected ? ESP_OK : ESP_FAIL;
}

void MyMQTT::shutdown()
{
    ESP_LOGI(TAG, "Starting MQTT shutdown sequence");
    if (!s_client)
    {
        ESP_LOGD(TAG, "No active MQTT client");
    }
    else
    {
        esp_err_t err = esp_mqtt_client_disconnect(s_client);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "MQTT disconnect failed: %s", esp_err_to_name(err));
        }
        err = esp_mqtt_client_stop(s_client);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "MQTT stop failed: %s", esp_err_to_name(err));
        }
        err = esp_mqtt_client_destroy(s_client);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "MQTT destroy failed: %s", esp_err_to_name(err));
        }
        s_client = nullptr;
        s_connected = false;
    }
    // Free the semaphore
    if (s_connection_sem)
    {
        vSemaphoreDelete(s_connection_sem);
        s_connection_sem = nullptr;
    }
    // Clear subscribed topics
    s_subscribed_topics.clear();
    ESP_LOGI(TAG, "MQTT shutdown complete");
}

void MyMQTT::publish(const char *topic, const char *message)
{
    if (s_connected && s_client)
    {
        int msg_id = esp_mqtt_client_publish(s_client, topic, message, 0, 0, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "Failed to publish message");
        }
        else
        {
            ESP_LOGD(TAG, "Published to %s: %s (msg_id %d)", topic, message, msg_id);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Cannot publish - MQTT not connected");
    }
}

void MyMQTT::subscribe(const char *topic)
{
    // Avoid duplicate subscriptions
    if (std::find(s_subscribed_topics.begin(), s_subscribed_topics.end(), topic) == s_subscribed_topics.end())
    {
        s_subscribed_topics.push_back(topic);
    }
    if (s_connected && s_client)
    {
        int msg_id = esp_mqtt_client_subscribe(s_client, topic, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "Failed to subscribe to topic");
        }
        else
        {
            ESP_LOGI(TAG, "Subscribed to %s (msg_id %d)", topic, msg_id);
        }
    }
}

void MyMQTT::registerMessageHandler(void (*handler)(const char *, const char *))
{
    s_message_handler = handler;
}

void MyMQTT::eventHandler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        if (s_connection_sem)
        {
            xSemaphoreGive(s_connection_sem);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        s_connected = false;
        break;
    case MQTT_EVENT_DATA:
        if (s_message_handler)
        {
            char topic[event->topic_len + 1];
            char payload[event->data_len + 1];
            strncpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            strncpy(payload, event->data, event->data_len);
            payload[event->data_len] = '\0';
            s_message_handler(topic, payload);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        s_connected = false;
        break;
    default:
        ESP_LOGD(TAG, "Unhandled MQTT event: %ld", event_id);
        break;
    }
}