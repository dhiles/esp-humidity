#include "webserver.h"

static const char *TAG = "WEBSERVER";

// ----------------------------------------------------------------------
// 1. HTTP Handler Functions
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

/**
 * @brief Handler for the root (/) endpoint
 */
esp_err_t root_get_handler(httpd_req_t *req)
{
    const char* resp_str = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Welcome</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f0f0f0;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            padding: 20px;
            box-sizing: border-box;
        }
        .container {
            text-align: center;
            background: white;
            padding: 40px;
            border-radius: 10px;
            box-shadow: 0 0 20px rgba(0,0,0,0.1);
            max-width: 90%;
            width: 600px;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        p {
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }
        a {
            display: inline-block;
            color: #007bff;
            text-decoration: none;
            font-size: 1.2em;
            padding: 10px 20px;
            background-color: #007bff;
            color: white;
            border-radius: 5px;
            transition: background-color 0.3s;
        }
        a:hover {
            background-color: #0056b3;
        }
        @media (max-width: 600px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 1.5em;
            }
            p {
                font-size: 1em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Welcome to the Humidoer Web Server!</h1>
        <p>This is a simple welcome page.</p>
        <a href="/hello">Check Humidity</a>
    </div>
</body>
</html>
)";
    
    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/html");

    // Send response with the string
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    // Log the request
    ESP_LOGI(TAG, "Serving root endpoint");

    return ESP_OK;
}

// ----------------------------------------------------------------------
// 2. URI Definitions
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

/**
 * @brief URI structure for root (/) endpoint
 */
httpd_uri_t root = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = root_get_handler,
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
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &hello);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}