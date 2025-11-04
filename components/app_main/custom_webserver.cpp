#include "custom_webserver.h"

static const char *TAG = "CUSTOM_WEBSERVER";

static double current_humidity = 0.0;
static char current_response[50] = "humidity=0.00";
static esp_timer_handle_t humidity_timer_handle;

/**
 * @brief Timer callback function to update humidity data
 */
static void humidity_timer_callback(void* arg)
{
    const char *json = WorkImplementation::getInstance().getMessage();
    
    // Parse the JSON string
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON in timer callback");
        return;
    }
    
    // Extract humidity value
    cJSON *humidity_json = cJSON_GetObjectItem(root, "humidity");
    if (humidity_json != NULL && cJSON_IsNumber(humidity_json)) {
        current_humidity = humidity_json->valuedouble;
        snprintf(current_response, sizeof(current_response), "humidity=%.2f", current_humidity);
        ESP_LOGI(TAG, "Humidity updated: %.2f", current_humidity);
    } else {
        ESP_LOGE(TAG, "Failed to extract humidity in timer callback");
        strlcpy(current_response, "humidity=error", sizeof(current_response));
    }
    
    cJSON_Delete(root);
}

/**
 * @brief Handler for the /custom endpoint
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

/**
 * @brief Handler for the /humidity-gauge endpoint
 */
esp_err_t humidity_gauge_get_handler(httpd_req_t *req)
{
    // Use heap allocation for large HTML response
    char *html_response = (char *)malloc(4096);
    if (html_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML response");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Calculate gauge rotation (0° to 180° for 0-100% humidity)
    double gauge_rotation = (current_humidity / 100.0) * 180.0;
    if (gauge_rotation > 180.0) gauge_rotation = 180.0;
    if (gauge_rotation < 0.0) gauge_rotation = 0.0;
    
    // Determine color based on humidity level
    const char* gauge_color;
    if (current_humidity < 30) {
        gauge_color = "#ff6b6b"; // Red for low humidity
    } else if (current_humidity < 70) {
        gauge_color = "#51cf66"; // Green for normal humidity
    } else {
        gauge_color = "#339af0"; // Blue for high humidity
    }
    
    // Build HTML in chunks to avoid large stack usage
    int written = 0;
    
    // HTML header and CSS
    written += snprintf(html_response + written, 4096 - written,
        "<!DOCTYPE html>"
        "<html lang=\"en\">"
        "<head>"
            "<meta charset=\"UTF-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
            "<title>Humidity Gauge</title>"
            "<style>"
                "body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#667eea 0%%,#764ba2 100%%);margin:0;padding:20px;display:flex;justify-content:center;align-items:center;min-height:100vh;}"
                ".container{background:white;border-radius:20px;padding:40px;box-shadow:0 20px 40px rgba(0,0,0,0.1);text-align:center;max-width:500px;width:100%%;}"
                "h1{color:#333;margin-bottom:30px;font-size:2em;}"
                ".gauge{width:300px;height:150px;position:relative;margin:0 auto 30px;}"
                ".gauge-background{width:100%%;height:100%%;background:#f8f9fa;border-radius:150px 150px 0 0;position:relative;overflow:hidden;border:10px solid #e9ecef;box-sizing:border-box;}"
                ".gauge-fill{position:absolute;bottom:0;left:0;width:100%%;height:100%%;background:conic-gradient(from 0.25turn at 50%% 100%%,#ff6b6b 0%%,#ffd43b 50%%,#51cf66 100%%);transform-origin:50%% 100%%;clip-path:polygon(0 0,100%% 0,100%% 100%%,0 100%%);border-radius:150px 150px 0 0;}"
                ".gauge-mask{position:absolute;bottom:0;left:20px;width:calc(100%% - 40px);height:calc(100%% - 20px);background:white;border-radius:130px 130px 0 0;z-index:1;}"
    );
    
    // Dynamic gauge styles
    written += snprintf(html_response + written, 4096 - written,
                ".gauge-needle{position:absolute;bottom:0;left:50%%;width:4px;height:120px;background:%s;transform-origin:50%% 100%%;transform:translateX(-50%%) rotate(%.2fdeg);z-index:2;border-radius:2px 2px 0 0;transition:transform 1s ease-in-out;box-shadow:0 0 10px rgba(0,0,0,0.3);}"
                ".gauge-center{position:absolute;bottom:-10px;left:50%%;width:30px;height:30px;background:%s;border:4px solid white;border-radius:50%%;transform:translateX(-50%%);z-index:3;box-shadow:0 0 10px rgba(0,0,0,0.3);}"
                ".gauge-labels{display:flex;justify-content:space-between;width:300px;margin:10px auto 0;padding:0 20px;box-sizing:border-box;}"
                ".gauge-label{font-size:14px;color:#666;font-weight:bold;}"
                ".humidity-value{font-size:3em;font-weight:bold;color:#333;margin:20px 0;text-shadow:2px 2px 4px rgba(0,0,0,0.1);}"
                ".humidity-unit{font-size:1.5em;color:#666;margin-left:5px;}"
                ".last-update{color:#888;font-size:14px;margin-top:20px;}"
                ".auto-refresh{margin-top:20px;padding:10px 20px;background:#4dabf7;color:white;border:none;border-radius:25px;cursor:pointer;font-size:14px;transition:background 0.3s;}"
                ".auto-refresh:hover{background:#339af0;}"
            "</style>"
        "</head>"
        "<body>"
            "<div class=\"container\">"
                "<h1>🌡️ Humidity Monitor</h1>"
                "<div class=\"gauge\">"
                    "<div class=\"gauge-background\">"
                        "<div class=\"gauge-fill\"></div>"
                        "<div class=\"gauge-mask\"></div>"
                        "<div class=\"gauge-needle\"></div>"
                        "<div class=\"gauge-center\"></div>"
                    "</div>"
                    "<div class=\"gauge-labels\">"
                        "<span class=\"gauge-label\">0%%</span>"
                        "<span class=\"gauge-label\">50%%</span>"
                        "<span class=\"gauge-label\">100%%</span>"
                    "</div>"
                "</div>"
                "<div class=\"humidity-value\">"
                    "%.2f<span class=\"humidity-unit\">%%</span>"
                "</div>"
                "<div class=\"last-update\" id=\"lastUpdate\">"
                    "Last updated: just now"
                "</div>"
                "<button class=\"auto-refresh\" onclick=\"location.reload()\">"
                    "🔄 Refresh Now"
                "</button>"
            "</div>"
        "<script>"
            "setTimeout(function(){location.reload();},30000);"
            "function updateLastUpdateTime(){"
                "const now=new Date();"
                "const timeString=now.toLocaleTimeString();"
                "document.getElementById('lastUpdate').textContent='Last updated: '+timeString;"
            "}"
            "updateLastUpdateTime();"
        "</script>"
        "</body>"
        "</html>",
        gauge_color, gauge_rotation - 90.0, gauge_color, current_humidity);
    
    // Check if we exceeded buffer
    if (written >= 4096) {
        ESP_LOGE(TAG, "HTML response too large: %d bytes", written);
        free(html_response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/html");

    // Send response with the gauge HTML
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);

    // Free allocated memory
    free(html_response);

    // Log the request
    ESP_LOGI(TAG, "Serving /humidity-gauge endpoint - humidity: %.2f", current_humidity);

    return ESP_OK;
}

/**
 * @brief Initialize the humidity update timer
 */
void init_humidity_timer(void)
{
    esp_timer_create_args_t timer_args = {
        .callback = &humidity_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "humidity_timer"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &humidity_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(humidity_timer_handle, 30000000)); // 30 seconds in microseconds
    
    ESP_LOGI(TAG, "Humidity update timer started (30 seconds interval)");
    
    // Get initial value
    humidity_timer_callback(NULL);
}

void start_custom_webserver()
{
    httpd_handle_t server = start_webserver(true);
    // Register custom URI handler
    httpd_uri_t custom_uri = {
        .uri = "/custom",
        .method = HTTP_GET,
        .handler = custom_get_handler,
        .user_ctx = NULL};

    httpd_uri_t gauge_uri = {
        .uri = "/humidity-gauge",
        .method = HTTP_GET,
        .handler = humidity_gauge_get_handler,
        .user_ctx = NULL};

    register_uri_handler(server, &custom_uri);
    register_uri_handler(server, &gauge_uri);
    init_humidity_timer();
}