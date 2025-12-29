#include "custom_webserver.h"

static const char *TAG = "CUSTOM_WEBSERVER";

static double current_humidity = 0.0;
static char current_response[50] = "humidity=0.00";
static esp_timer_handle_t humidity_timer_handle;

/**
 * @brief Timer callback function to update humidity data
 */
static void humidity_timer_callback(void *arg)
{
    const char *json = WorkImplementation::getInstance().getMessage();

    // Parse the JSON string
    cJSON *root = cJSON_Parse(json);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON in timer callback");
        return;
    }

    // Extract humidity value
    cJSON *humidity_json = cJSON_GetObjectItem(root, "humidity");
    if (humidity_json != NULL && cJSON_IsNumber(humidity_json))
    {
        current_humidity = humidity_json->valuedouble;
        snprintf(current_response, sizeof(current_response), "humidity=%.2f", current_humidity);
        ESP_LOGI(TAG, "Humidity updated: %.2f", current_humidity);
    }
    else
    {
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
    if (html_response == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML response");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Calculate gauge rotation (0° to 180° for 0-100% humidity)
    double gauge_rotation = (current_humidity / 100.0) * 180.0;
    if (gauge_rotation > 180.0)
        gauge_rotation = 180.0;
    if (gauge_rotation < 0.0)
        gauge_rotation = 0.0;

    // Determine color based on humidity level
    const char *gauge_color;
    if (current_humidity < 30)
    {
        gauge_color = "#ff6b6b"; // Red for low humidity
    }
    else if (current_humidity < 70)
    {
        gauge_color = "#51cf66"; // Green for normal humidity
    }
    else
    {
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
                        ".gauge-mask{position:absolute;bottom:0;left:20px;width:calc(100%% - 40px);height:calc(100%% - 20px);background:white;border-radius:130px 130px 0 0;z-index:1;}");

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
    if (written >= 4096)
    {
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
        .name = "humidity_timer"};

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &humidity_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(humidity_timer_handle, 1000000)); // 30 seconds in microseconds

    ESP_LOGI(TAG, "Humidity update timer started (30 seconds interval)");

    // Get initial value
    humidity_timer_callback(NULL);
}

/**
 * @brief Handler for GET /api/sensors?count=N
 *        Returns the last N sensor readings (newest first) as JSON array
 */
esp_err_t api_sensors_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "API: GET /api/sensors (with optional count query)");

    SensorDataManager &mgr = SensorDataManager::getInstance();
    size_t total_available = mgr.size();

    if (total_available == 0)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No sensor data available yet");
        return ESP_OK;
    }

    // Parse query parameter: count
    char query[32] = {0};
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0 && query_len < sizeof(query))
    {
        httpd_req_get_url_query_str(req, query, sizeof(query));
    }

    char count_str[16] = {0};
    if (httpd_query_key_value(query, "count", count_str, sizeof(count_str)) != ESP_OK)
    {
        // No count parameter → default to 1
        strcpy(count_str, "1");
    }

    // Convert to integer
    char *endptr;
    long requested_count = strtol(count_str, &endptr, 10);

    if (*endptr != '\0' || requested_count < 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid 'count' parameter");
        return ESP_OK;
    }

    size_t count = (size_t)requested_count;
    if (count == 0 || count > total_available)
    {
        count = total_available; // Return all if 0 or too large
    }

    // Create JSON array of readings
    cJSON *readings_array = cJSON_CreateArray();
    if (!readings_array)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
        return ESP_FAIL;
    }

    // Iterate from newest to oldest
    for (size_t i = 0; i < count; ++i)
    {
        size_t index = total_available - 1 - i; // newest = size-1, then size-2, etc.
        const SensorData *data = mgr.getAt(index);
        if (!data || data->isEmpty())
            continue;

        cJSON *reading_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(reading_obj, "timestamp", data->getTimestamp());
        cJSON_AddNumberToObject(reading_obj, "sensor_id", data->getSensorId());

        cJSON *values_array = cJSON_CreateArray();
        for (uint8_t j = 0; j < data->getValueCount(); ++j)
        {
            const SensorValue *val = data->getValueByIndex(j);
            if (!val || !val->isValid())
                continue;

            cJSON *value_obj = cJSON_CreateObject();
            cJSON_AddStringToObject(value_obj, "type", val->getTypeString().c_str());
            cJSON_AddNumberToObject(value_obj, "value", val->value);
            cJSON_AddNumberToObject(value_obj, "quality", val->quality);
            cJSON_AddItemToArray(values_array, value_obj);
        }
        cJSON_AddItemToObject(reading_obj, "values", values_array);
        cJSON_AddItemToArray(readings_array, reading_obj);
    }

    // Convert to string
    char *json_str = cJSON_Print(readings_array);
    if (!json_str)
    {
        cJSON_Delete(readings_array);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        return ESP_FAIL;
    }

    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    // Clean up
    cJSON_free(json_str);
    cJSON_Delete(readings_array);

    ESP_LOGI(TAG, "Sent %u latest sensor readings", count);
    return ESP_OK;
}

/**
 * @brief Handler for GET /api/sensors/latest - returns latest sensor data as JSON
 */
esp_err_t api_sensors_latest_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "API: GET /api/sensors/latest");

    SensorDataManager &mgr = SensorDataManager::getInstance();
    size_t total = mgr.size();

    if (total == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No sensor data available yet");
        return ESP_OK;
    }

    // Use the suggested reliable method to get the latest data
    const size_t headIndex = mgr.getHeadIndex();
    const size_t capacity = mgr.getBuffer().capacity();
    const SensorData *latest = mgr.getAt((headIndex == 0) ? (capacity - 1) : (headIndex - 1));

    if (!latest || latest->isEmpty()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Latest reading is empty");
        return ESP_OK;
    }

    // Create JSON root
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        ESP_LOGE(TAG, "Failed to create JSON root");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
        return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "timestamp", latest->getTimestamp());
    cJSON_AddNumberToObject(root, "sensor_id", latest->getSensorId());

    // Array of sensor values
    cJSON *values_array = cJSON_CreateArray();
    if (!values_array)
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
        return ESP_FAIL;
    }

    for (uint8_t i = 0; i < latest->getValueCount(); ++i)
    {
        const SensorValue *val = latest->getValueByIndex(i);
        if (!val || !val->isValid())
            continue;

        cJSON *value_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(value_obj, "type", val->getTypeString().c_str());
        cJSON_AddNumberToObject(value_obj, "value", val->value);
        cJSON_AddNumberToObject(value_obj, "quality", val->quality);
        cJSON_AddItemToArray(values_array, value_obj);
    }

    cJSON_AddItemToObject(root, "values", values_array);

    // Convert to string
    char *json_str = cJSON_Print(root);
    if (!json_str)
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        return ESP_FAIL;
    }

    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    // Clean up
    cJSON_free(json_str);
    cJSON_Delete(root);

    return ESP_OK;
}

/**
 * @brief Handler for POST /api/sensors/clear
 *        Clears all stored sensor data from the circular buffer
 */
esp_err_t api_sensors_clear_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "API: POST /api/sensors/clear - clearing sensor buffer");

    // Only allow POST method
    if (req->method != HTTP_POST)
    {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method Not Allowed - Use POST");
        return ESP_OK;
    }

    SensorDataManager &mgr = SensorDataManager::getInstance();
    size_t previous_count = mgr.size();

    mgr.clear(); // This resets head, tail, and count to 0

    // Prepare JSON response
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
        return ESP_FAIL;
    }

    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddNumberToObject(root, "previous_count", previous_count);
    cJSON_AddStringToObject(root, "message", "Sensor data buffer cleared");

    char *json_str = cJSON_Print(root);
    if (!json_str)
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        return ESP_FAIL;
    }

    // Send success response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    // Clean up
    cJSON_free(json_str);
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Sensor buffer cleared (was %u entries)", previous_count);
    return ESP_OK;
}

/**
 * @brief Handler for GET /api/sensors/count
 *        Returns the current number of stored sensor records
 */
esp_err_t api_sensors_count_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "API: GET /api/sensors/count");

    SensorDataManager &mgr = SensorDataManager::getInstance();
    size_t current_count = mgr.size();

    // Create simple JSON response
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal server error");
        return ESP_FAIL;
    }

    cJSON_AddNumberToObject(root, "count", current_count);
    cJSON_AddNumberToObject(root, "capacity", mgr.getBuffer().capacity()); // Optional: show max capacity

    char *json_str = cJSON_Print(root);
    if (!json_str)
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        return ESP_FAIL;
    }

    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);

    // Clean up
    cJSON_free(json_str);
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Reported sensor record count: %u", current_count);
    return ESP_OK;
}

void start_custom_webserver()
{
    httpd_handle_t server = start_webserver(true);
    // Register custom URI handler
    httpd_uri_t api_sensors_uri = {
        .uri = "/api/sensors",
        .method = HTTP_GET,
        .handler = api_sensors_get_handler,
        .user_ctx = NULL};

    httpd_uri_t api_sensors_latest_uri = {
        .uri = "/api/sensors/latest",
        .method = HTTP_GET,
        .handler = api_sensors_latest_get_handler,
        .user_ctx = NULL};

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

    httpd_uri_t api_sensors_clear_uri = {
        .uri = "/api/sensors/clear",
        .method = HTTP_POST,
        .handler = api_sensors_clear_post_handler,
        .user_ctx = NULL};

    httpd_uri_t api_sensors_count_uri = {
        .uri = "/api/sensors/count",
        .method = HTTP_GET,
        .handler = api_sensors_count_get_handler,
        .user_ctx = NULL};

    register_uri_handler(server, &api_sensors_uri);
    register_uri_handler(server, &api_sensors_latest_uri);
    register_uri_handler(server, &custom_uri);
    register_uri_handler(server, &gauge_uri);
    register_uri_handler(server, &api_sensors_clear_uri);
    register_uri_handler(server, &api_sensors_count_uri);
    init_humidity_timer();
}
