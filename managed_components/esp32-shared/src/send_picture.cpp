#include "send_picture.h"
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "ca_cert_def.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
//#include "esp_heap_trace.h"
#include "mymqtt.h"
#include "mywifi.h"
#include "myntp.h"

static const char *TAG = "SEND_PICTURE";

// MQTT topics
static const char *mqtt_topic = "picsrv/image";
//static const char *mqtt_result_topic = "picsrv/send-picture";

// Semaphore for MQTT operations
static SemaphoreHandle_t mqtt_mutex = NULL;

// Heap tracing buffer
#define NUM_RECORDS 100

static void escape_json_string(char *dest, size_t dest_size, const char *src)
{
    size_t src_index = 0, dest_index = 0;
    while (src[src_index] && dest_index < dest_size - 1)
    {
        char c = src[src_index];
        if (c == '"' || c == '\\')
        {
            if (dest_index + 2 >= dest_size)
                break;
            dest[dest_index++] = '\\';
            dest[dest_index++] = c;
        }
        else if (c >= 0x20 || c == '\t')
        {
            dest[dest_index++] = c;
        }
        else
        {
            dest[dest_index++] = ' ';
        }
        src_index++;
    }
    dest[dest_index] = '\0';
}

bool response_to_json(UploadResponse &response, char *buffer, size_t buffer_size)
{
    char escaped_message[128] = {0};
    char escaped_fileName[64] = {0};
    char escaped_error[128] = {0};

    escape_json_string(escaped_message, sizeof(escaped_message), response.message);
    escape_json_string(escaped_fileName, sizeof(escaped_fileName), response.fileName);
    escape_json_string(escaped_error, sizeof(escaped_error), response.error);

    if (response.success)
    {
        if (response.fileName[0])
        {
            snprintf(buffer, buffer_size,
                     "{\"success\":true,\"message\":\"%s\",\"fileName\":\"%s\",\"bytesReceived\":%d}",
                     escaped_message, escaped_fileName, response.bytesReceived);
        }
        else
        {
            snprintf(buffer, buffer_size,
                     "{\"success\":true,\"message\":\"%s\",\"bytesReceived\":%d}",
                     escaped_message, response.bytesReceived);
        }
    }
    else
    {
        if (response.error[0])
        {
            snprintf(buffer, buffer_size,
                     "{\"success\":false,\"message\":\"%s\",\"error\":\"%s\"}",
                     escaped_message, escaped_error);
        }
        else
        {
            snprintf(buffer, buffer_size,
                     "{\"success\":false,\"message\":\"%s\"}",
                     escaped_message);
        }
    }
    return true;
}

void upload_file2(const char *imageFileName, const uint8_t *data, size_t data_len,
                 const char *contentType, const char *fileExtension, UploadResponse &response)
{
    uint32_t free_heap_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "upload_file start - Free Heap: %lu", free_heap_before);

    response.success = false;
    response.bytesReceived = -1;
    snprintf(response.fileName, sizeof(response.fileName), "%s.%s", imageFileName, fileExtension);
    response.message[0] = '\0';
    response.error[0] = '\0';

    if (!MyWiFi::isConnected() || !MyWiFi::isNetworkReady()) {
        strcpy(response.message, "Wi-Fi not connected");
        strcpy(response.error, "Network unavailable");
        ESP_LOGE(TAG, "Wi-Fi not connected or network not ready, aborting upload");
        return;
    }

    if (!mqtt_mutex) {
        mqtt_mutex = xSemaphoreCreateMutex();
        if (mqtt_mutex == NULL) {
            strcpy(response.message, "Failed to create MQTT mutex");
            strcpy(response.error, "Mutex creation failure");
            ESP_LOGE(TAG, "Failed to create MQTT mutex");
            return;
        }
        ESP_LOGI(TAG, "MQTT mutex created successfully");
    }

    if (xSemaphoreTake(mqtt_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        strcpy(response.message, "Failed to acquire MQTT mutex");
        strcpy(response.error, "Mutex lock failure");
        ESP_LOGE(TAG, "Failed to acquire MQTT mutex");
        xSemaphoreGive(mqtt_mutex);
        return;
    }

    if (!MyMQTT::isConnected()) {
        strcpy(response.message, "MQTT client not connected");
        strcpy(response.error, "MQTT connection failure");
        ESP_LOGE(TAG, "MQTT client not connected");
        xSemaphoreGive(mqtt_mutex);
        return;
    }

    // Publish filename to a separate topic
    const char *filename_topic = "suncam/image/filename";
    int filename_msg_id = esp_mqtt_client_publish(MyMQTT::s_client, filename_topic, response.fileName, strlen(response.fileName), 1, 0);
    if (filename_msg_id < 0) {
        strcpy(response.message, "Failed to publish filename");
        snprintf(response.error, sizeof(response.error), "MQTT publish failed, msg_id: %d", filename_msg_id);
        ESP_LOGE(TAG, "Failed to publish filename, msg_id: %d", filename_msg_id);
        xSemaphoreGive(mqtt_mutex);
        return;
    }
    ESP_LOGI(TAG, "Published filename %s to %s (msg_id %d)", response.fileName, filename_topic, filename_msg_id);

    // Publish image data with QoS 1
    int msg_id = esp_mqtt_client_publish(MyMQTT::s_client, mqtt_topic, (const char *)data, data_len, 1, 0);
    if (msg_id >= 0) {
        response.success = true;
        strcpy(response.message, "Image uploaded successfully via MQTT");
        response.bytesReceived = data_len;
        ESP_LOGI(TAG, "Published %zu bytes to %s (msg_id %d)", data_len, mqtt_topic, msg_id);
    } else {
        strcpy(response.message, "Failed to publish image");
        snprintf(response.error, sizeof(response.error), "MQTT publish failed, msg_id: %d", msg_id);
        ESP_LOGE(TAG, "Failed to publish image, msg_id: %d", msg_id);
    }

    xSemaphoreGive(mqtt_mutex);

    uint32_t free_heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "upload_file end - Free Heap: %lu, Delta: %d", free_heap_after, (int)(free_heap_after - free_heap_before));
}

void upload_file(const char *imageFileName, const uint8_t *data, size_t data_len,
                 const char *contentType, const char *fileExtension, UploadResponse &response)
{
    // Construct full filename
    char full_filename[256];  // Adjust size as needed for max filename length
    snprintf(full_filename, sizeof(full_filename), "%s.%s", imageFileName, fileExtension);
    size_t filename_len = strlen(full_filename);

    // Allocate combined buffer: len (2 bytes) + filename + image
    size_t total_len = 2 + filename_len + data_len;
    uint8_t* combined_buf = (uint8_t*)malloc(total_len);
    if (!combined_buf) {
        strcpy(response.message, "Failed to allocate buffer");
        snprintf(response.error, sizeof(response.error), "Malloc failed for %zu bytes", total_len);
        response.success = false;
        ESP_LOGE(TAG, "Malloc failed for %zu bytes", total_len);
        return;
    }

    // Write little-endian uint16 length
    uint16_t len = (uint16_t)filename_len;
    combined_buf[0] = len & 0xFF;
    combined_buf[1] = (len >> 8) & 0xFF;

    // Copy filename and image data
    memcpy(combined_buf + 2, full_filename, filename_len);
    memcpy(combined_buf + 2 + filename_len, data, data_len);

    // Publish combined buffer with QoS 1
    int msg_id = esp_mqtt_client_publish(MyMQTT::s_client, mqtt_topic, (const char *)combined_buf, total_len, 1, 0);
    free(combined_buf);  // Always free

    if (msg_id >= 0) {
        response.success = true;
        strcpy(response.message, "Image uploaded successfully via MQTT");
        response.bytesReceived = data_len;  // Report original image size
        ESP_LOGI(TAG, "Published %zu bytes (image: %zu) to %s (msg_id %d)", total_len, data_len, mqtt_topic, msg_id);
    } else {
        response.success = false;
        strcpy(response.message, "Failed to publish image");
        snprintf(response.error, sizeof(response.error), "MQTT publish failed, msg_id: %d", msg_id);
        ESP_LOGE(TAG, "Failed to publish image, msg_id: %d", msg_id);
    }
}

void upload_file1(const char *imageFileName, const uint8_t *data, size_t data_len,
                 const char *contentType, const char *fileExtension, UploadResponse &response)
{
    uint32_t free_heap_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "upload_file start - Free Heap: %lu", free_heap_before);

    response.success = false;
    response.bytesReceived = -1;
    snprintf(response.fileName, sizeof(response.fileName), "%s.%s", imageFileName, fileExtension);
    response.message[0] = '\0';
    response.error[0] = '\0';

    if (!MyWiFi::isConnected() || !MyWiFi::isNetworkReady()) {
        strcpy(response.message, "Wi-Fi not connected");
        strcpy(response.error, "Network unavailable");
        ESP_LOGE(TAG, "Wi-Fi not connected or network not ready, aborting upload");
        return;
    }

    if (!mqtt_mutex) {
        mqtt_mutex = xSemaphoreCreateMutex();
        if (mqtt_mutex == NULL) {
            strcpy(response.message, "Failed to create MQTT mutex");
            strcpy(response.error, "Mutex creation failure");
            ESP_LOGE(TAG, "Failed to create MQTT mutex");
            return;
        }
        ESP_LOGI(TAG, "MQTT mutex created successfully");
    }

    if (xSemaphoreTake(mqtt_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        strcpy(response.message, "Failed to acquire MQTT mutex");
        strcpy(response.error, "Mutex lock failure");
        ESP_LOGE(TAG, "Failed to acquire MQTT mutex");
        xSemaphoreGive(mqtt_mutex);
        return;
    }

    if (!MyMQTT::isConnected()) {
        strcpy(response.message, "MQTT client not connected");
        strcpy(response.error, "MQTT connection failure");
        ESP_LOGE(TAG, "MQTT client not connected");
        xSemaphoreGive(mqtt_mutex);
        return;
    }

    // Publish image data with QoS 1
    int msg_id = esp_mqtt_client_publish(MyMQTT::s_client, mqtt_topic, (const char *)data, data_len, 1, 0);
    if (msg_id >= 0) {
        response.success = true;
        strcpy(response.message, "Image uploaded successfully via MQTT");
        response.bytesReceived = data_len;
        ESP_LOGI(TAG, "Published %zu bytes to %s (msg_id %d)", data_len, mqtt_topic, msg_id);
    } else {
        strcpy(response.message, "Failed to publish image");
        snprintf(response.error, sizeof(response.error), "MQTT publish failed, msg_id: %d", msg_id);
        ESP_LOGE(TAG, "Failed to publish image, msg_id: %d", msg_id);
    }

    xSemaphoreGive(mqtt_mutex);

    uint32_t free_heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "upload_file end - Free Heap: %lu, Delta: %d", free_heap_after, (int)(free_heap_after - free_heap_before));
}

bool sendPicture(const char *camName, camera_fb_t *fb, UploadResponse &response)
{
    uint32_t free_heap_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "sendPicture start - Free Heap: %lu", free_heap_before);

    if (!fb || fb->len == 0)
    {
        response.success = false;
        strcpy(response.message, "Invalid frame buffer");
        ESP_LOGE(TAG, "Invalid frame buffer");
        return false;
    }

    ESP_LOGI(TAG, "Image captured! Size: %zu bytes", fb->len);

    char timestamp[32];
    MyNTP::getTimestamp(timestamp, sizeof(timestamp));
    char imageFileName[64];
    snprintf(imageFileName, sizeof(imageFileName), "%s_%s", camName, timestamp);

    upload_file(imageFileName, fb->buf, fb->len, "image/jpeg", "jpg", response);

    uint32_t free_heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "sendPicture end - Free Heap: %lu, Delta: %d", free_heap_after, (int)(free_heap_after - free_heap_before));
    return response.success;
}

void transfer_fb(const char *camName, camera_fb_t *fb)
{
    uint32_t free_heap_before = esp_get_free_heap_size();
    ESP_LOGI(TAG, "transfer_fb start - Free Heap: %lu", free_heap_before);

    UploadResponse response = {false, -1, "", "", ""};

    ESP_LOGI(TAG, "Starting fb transfer...");
    if (fb == NULL)
    {
        ESP_LOGE(TAG, "Failed to capture image");
        response.success = false;
        strcpy(response.message, "Failed to capture image");
        strcpy(response.error, "Camera capture failed");
        char json_response[512] = {0};
        response_to_json(response, json_response, sizeof(json_response));
        uint32_t heap_before_publish = esp_get_free_heap_size();
        if (MyMQTT::isConnected()) {
          //  esp_mqtt_client_publish(MyMQTT::s_client, mqtt_result_topic, json_response, 0, 1, 0);
            ESP_LOGI(TAG, "Published error response: %s", json_response);
        } else {
            ESP_LOGW(TAG, "MQTT not connected, cannot publish error response");
        }
        ESP_LOGI(TAG, "1. (error) - Free Heap: %lu, Delta: %d", esp_get_free_heap_size(), (int)(esp_get_free_heap_size() - heap_before_publish));
        return;
    }
    ESP_LOGI(TAG, "Image captured! Size: %zu bytes", fb->len);

    uint32_t heap_before_send = esp_get_free_heap_size();
    bool send_success = sendPicture(camName, fb, response);
    ESP_LOGI(TAG, "sendPicture - Free Heap: %lu, Delta: %d", esp_get_free_heap_size(), (int)(esp_get_free_heap_size() - heap_before_send));

    uint32_t heap_before_fb_return = esp_get_free_heap_size();
    esp_camera_fb_return(fb);
    ESP_LOGI(TAG, "esp_camera_fb_return - Free Heap: %lu, Delta: %d", esp_get_free_heap_size(), (int)(esp_get_free_heap_size() - heap_before_fb_return));

    if (send_success)
    {
        ESP_LOGI(TAG, "Transfer complete");
    }
    else
    {
        ESP_LOGE(TAG, "Transfer error: %s", response.error);
    }
/*
    char json_response[512] = {0};
    response_to_json(response, json_response, sizeof(json_response));
    if (json_response[0])
    {
        ESP_LOGI(TAG, "Publishing transfer result: %s", json_response);
        uint32_t heap_before_publish = esp_get_free_heap_size();
        if (MyMQTT::isConnected()) {
            esp_mqtt_client_publish(MyMQTT::s_client, mqtt_result_topic, json_response, 0, 1, 0);
        } else {
            ESP_LOGW(TAG, "MQTT not connected, cannot publish response");
        }
        ESP_LOGI(TAG, "2. Free Heap: %lu, Delta: %d", esp_get_free_heap_size(), (int)(esp_get_free_heap_size() - heap_before_publish));
    }
    else
    {
        ESP_LOGE(TAG, "Failed to convert response to JSON");
        uint32_t heap_before_publish = esp_get_free_heap_size();
        if (MyMQTT::isConnected()) {
            esp_mqtt_client_publish(MyMQTT::s_client, mqtt_result_topic, "{\"success\":false,\"message\":\"Failed to serialize response\"}", 0, 1, 0);
        } else {
            ESP_LOGW(TAG, "MQTT not connected, cannot publish error response");
        }
        ESP_LOGI(TAG, "3. (error) - Free Heap: %lu, Delta: %d", esp_get_free_heap_size(), (int)(esp_get_free_heap_size() - heap_before_publish));
    }
*/
    uint32_t free_heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "transfer_fb end - Free Heap: %lu, Delta: %d", free_heap_after, (int)(free_heap_after - free_heap_before));

    static int loop_count = 0;
    if (++loop_count % 10 == 0)
    {
        ESP_LOGI(TAG, "Dumping heap trace after %d loops", loop_count);
      //  heap_trace_dump();
    }
}

