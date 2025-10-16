
#include "webserver.h"

static const char *TAG = "WEBSERVER";

// ----------------------------------------------------------------------
// 1. HTTP Handler Function
// ----------------------------------------------------------------------

/**
 * @brief Handler for the /hello endpoint
 */
esp_err_t hello_get_handler(httpd_req_t *req)
{
    const char* resp_str = "Humidity = ";
    std::string result = std::string(resp_str) + std::to_string(comp_data.humidity);
    const char* final_result = result.c_str();
    
    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/plain");

    // Send response with the string
    httpd_resp_send(req, final_result, HTTPD_RESP_USE_STRLEN);

    // Log the request
    ESP_LOGI(TAG, "Serving /hello endpoint");

    return ESP_OK;
}

// ----------------------------------------------------------------------
// 2. URI Definition
// ----------------------------------------------------------------------

/**
 * @brief URI structure for /hello endpoint
 */
httpd_uri_t hello = {
    .uri      = "/hello",
    .method   = HTTP_GET,
    .handler  = hello_get_handler,
    .user_ctx = NULL
};

// ----------------------------------------------------------------------
// 3. Server Initialization
// ----------------------------------------------------------------------

/**
 * @brief Starts the HTTP server instance
 */
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}