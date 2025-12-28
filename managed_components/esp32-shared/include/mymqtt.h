#ifndef MYMQTT_H
#define MYMQTT_H

#include "esp_err.h"
#include "mqtt_client.h"
#include <string>
#include <vector>

class MyMQTT {
private:
    static volatile bool s_connected;
    static SemaphoreHandle_t s_connection_sem;
    static std::vector<std::string> s_subscribed_topics;
    static void (*s_message_handler)(const char*, const char*);
    
public:
    static esp_mqtt_client_handle_t s_client;

    static bool isConnected();
    static void initialize();
    static void waitForConnection();
    static esp_err_t connect(const char* uri);
    static void shutdown();
    static void publish(const char* topic, const char* message);
    static void subscribe(const char* topic);
    static void registerMessageHandler(void (*handler)(const char*, const char*));
    static void eventHandler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data);
};

#endif