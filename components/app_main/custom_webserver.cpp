#include "custom_webserver.h"

static const char *TAG = "CUSTOM_WEBSERVER";

/**
 * @brief Handler for the /message endpoint
 */
esp_err_t custom_get_handler(httpd_req_t *req)
{
    const char *json = WorkImplementation::getInstance().getMessage();

    // Parse the JSON string
    cJSON *root = cJSON_Parse(json);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON from getMessage()");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send(req, "humidity=error: invalid JSON", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    // Extract humidity value
    cJSON *humidity_json = cJSON_GetObjectItem(root, "humidity");
    if (humidity_json == NULL || !cJSON_IsNumber(humidity_json))
    {
        ESP_LOGE(TAG, "Humidity field not found or not a number in JSON");
        cJSON_Delete(root);
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send(req, "humidity=error: humidity field not found", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    // Get humidity value
    double humidity_value = humidity_json->valuedouble;

    // Create response string
    char response[50];
    snprintf(response, sizeof(response), "humidity=%.2f", humidity_value);

    // Clean up JSON
    cJSON_Delete(root);

    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/plain");

    // Send response with the formatted string
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    // Log the request
    ESP_LOGI(TAG, "Serving /message endpoint - humidity: %.2f", humidity_value);

    return ESP_OK;
}


void start_custom_webserver() {
    httpd_handle_t server = start_webserver(true); 
    // Register custom URI handler
    httpd_uri_t custom_uri = {
        .uri = "/custom",
        .method = HTTP_GET,
        .handler = custom_get_handler,
        .user_ctx = NULL};
    register_uri_handler(server, &custom_uri);

}