#include "webserver.h"

static const char *TAG = "WEBSERVER";

// Global variables
SemaphoreHandle_t provisioning_sem = NULL;

// ----------------------------------------------------------------------
// 1. HTTP Handler Functions
// ----------------------------------------------------------------------

/**
 * @brief Handler for the /start endpoint
 */
esp_err_t start_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving / endpoint");

    // Method 1: Check if we're in AP mode by examining WiFi mode
    wifi_mode_t wifi_mode;
    esp_err_t ret = esp_wifi_get_mode(&wifi_mode);
    bool is_ap_mode = false;

    if (ret == ESP_OK)
    {
        // If we're in AP mode or AP+STA mode, consider it AP mode for this purpose
        is_ap_mode = (wifi_mode == WIFI_MODE_AP || wifi_mode == WIFI_MODE_APSTA);
        ESP_LOGI(TAG, "WiFi mode: %d, AP mode: %s", wifi_mode, is_ap_mode ? "true" : "false");
    }
    else
    {
        ESP_LOGW(TAG, "Failed to get WiFi mode: 0x%x", ret);
        // Fallback: assume AP mode if we can't determine
        is_ap_mode = true;
    }

    const char *resp_str;

    if (is_ap_mode)
    {
        // AP mode - show provisioning information
        resp_str = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Device Setup</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f0f0f0;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
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
            width: 500px;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        p {
            color: #666;
            line-height: 1.6;
            margin-bottom: 20px;
        }
        .button {
            display: inline-block;
            background-color: #007bff;
            color: white;
            padding: 12px 30px;
            text-decoration: none;
            border-radius: 5px;
            font-size: 16px;
            transition: background-color 0.3s;
            margin: 10px;
        }
        .button:hover {
            background-color: #0056b3;
        }
        .info-box {
            background-color: #e7f3ff;
            border-left: 4px solid #007bff;
            padding: 15px;
            margin: 20px 0;
            text-align: left;
        }
        @media (max-width: 600px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 1.5em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Welcome to Your Device Setup</h1>
        <p>Your device is currently in Access Point mode and ready to be configured.</p>
        
        <div class="info-box">
            <strong>Next Steps:</strong><br>
            1. Connect your device to your WiFi network<br>
            2. The device will restart and connect to your network<br>
            3. You can then update firmware if needed
        </div>
        
        <a href="/wifisetup" class="button">Configure WiFi</a>
        <a href="/ota" class="button" style="background-color: #6c757d;">Firmware Update</a>
    </div>
</body>
</html>
)";
    }
    else
    {
        // Station mode - show firmware update information
        resp_str = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Device Management</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f0f0f0;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
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
            width: 500px;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        p {
            color: #666;
            line-height: 1.6;
            margin-bottom: 20px;
        }
        .button {
            display: inline-block;
            background-color: #28a745;
            color: white;
            padding: 12px 30px;
            text-decoration: none;
            border-radius: 5px;
            font-size: 16px;
            transition: background-color 0.3s;
            margin: 10px;
        }
        .button:hover {
            background-color: #218838;
        }
        .info-box {
            background-color: #f8f9fa;
            border-left: 4px solid #28a745;
            padding: 15px;
            margin: 20px 0;
            text-align: left;
        }
        .steps {
            text-align: left;
            margin: 20px 0;
        }
        .steps ol {
            padding-left: 20px;
        }
        .steps li {
            margin-bottom: 10px;
        }
        @media (max-width: 600px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 1.5em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Device Management</h1>
        <p>Your device is connected to your network and ready for use.</p>
        
        <div class="info-box">
            <strong>Firmware Update Available</strong><br>
            You can update your device's firmware to get the latest features and improvements.
        </div>
        
        <div class="steps">
            <strong>How to update firmware:</strong>
            <ol>
                <li>Click the "Update Firmware" button below</li>
                <li>Select your firmware file (.bin format)</li>
                <li>Wait for the upload to complete</li>
                <li>The device will automatically restart</li>
            </ol>
        </div>
        
        <a href="/ota" class="button">Update Firmware</a>
        <a href="/wifisetup" class="button">Configure WiFi</a>
    </div>
</body>
</html>
)";
    }

    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/html");

    // Send response with the appropriate HTML
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/**
 * @brief Handler for OTA firmware updates
 */
esp_err_t ota_update_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Starting OTA update");

    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL)
    {
        ESP_LOGE(TAG, "No OTA update partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);

    // Initialize OTA
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA begin failed: 0x%x", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char ota_buffer[1024];
    int content_received = 0;
    int total_content_len = req->content_len;

    ESP_LOGI(TAG, "OTA update started, content length: %d", total_content_len);

    // Receive and write OTA data in chunks
    while (content_received < total_content_len)
    {
        int recv_len = httpd_req_recv(req, ota_buffer, sizeof(ota_buffer));

        if (recv_len < 0)
        {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
            {
                continue; // Retry on timeout
            }
            ESP_LOGE(TAG, "OTA receive error: %d", recv_len);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA receive failed");
            return ESP_FAIL;
        }

        if (recv_len > 0)
        {
            // Write received data to OTA partition
            err = esp_ota_write(ota_handle, (const void *)ota_buffer, recv_len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "OTA write failed: 0x%x", err);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
                return ESP_FAIL;
            }

            content_received += recv_len;

            // Log progress every 10KB
            if (content_received % (10 * 1024) == 0)
            {
                ESP_LOGI(TAG, "OTA progress: %d/%d bytes", content_received, total_content_len);
            }
        }
    }

    // Finalize OTA update
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA end failed: 0x%x", err);
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "OTA validation failed");
        }
        else
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        }
        return ESP_FAIL;
    }

    // Set boot partition and respond
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA set boot partition failed: 0x%x", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set boot partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA update successful, restarting...");

    // Send success response
    httpd_resp_set_type(req, "application/json");
    const char *success_resp = "{\"status\":\"success\",\"message\":\"OTA update completed, restarting device\"}";
    httpd_resp_send(req, success_resp, HTTPD_RESP_USE_STRLEN);

    // Give time for response to be sent
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Restart device
    esp_restart();

    return ESP_OK;
}

/**
 * @brief Handler for the /ota endpoint - displays OTA update form
 */
esp_err_t ota_get_handler(httpd_req_t *req)
{
    const char *resp_str = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Firmware Update</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f0f0f0;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
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
            width: 500px;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        .form-group {
            margin-bottom: 20px;
            text-align: left;
        }
        label {
            display: block;
            margin-bottom: 5px;
            color: #333;
            font-weight: bold;
        }
        input[type="file"] {
            width: 100%;
            padding: 12px;
            border: 2px dashed #ddd;
            border-radius: 5px;
            font-size: 16px;
            box-sizing: border-box;
            background-color: #f9f9f9;
        }
        input[type="file"]:hover {
            border-color: #007bff;
        }
        button {
            background-color: #28a745;
            color: white;
            border: none;
            padding: 12px 30px;
            font-size: 16px;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s;
            margin-top: 10px;
        }
        button:hover {
            background-color: #218838;
        }
        button:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        .message {
            margin-top: 20px;
            padding: 15px;
            border-radius: 5px;
            display: none;
        }
        .success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .progress {
            margin-top: 20px;
            display: none;
        }
        .progress-bar {
            width: 100%;
            height: 20px;
            background-color: #f0f0f0;
            border-radius: 10px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background-color: #007bff;
            width: 0%;
            transition: width 0.3s;
        }
        .back-link {
            display: block;
            margin-top: 20px;
            color: #007bff;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
        @media (max-width: 600px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 1.5em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Firmware Update</h1>
        <p>Upload a new firmware image to update the device.</p>
        
        <form id="otaForm" enctype="multipart/form-data">
            <div class="form-group">
                <label for="firmware">Select Firmware File:</label>
                <input type="file" id="firmware" name="firmware" accept=".bin" required>
            </div>
            
            <button type="submit" id="otaBtn">Update Firmware</button>
        </form>
        
        <div class="progress" id="progressContainer">
            <div class="progress-bar">
                <div class="progress-fill" id="progressFill"></div>
            </div>
            <div id="progressText">0%</div>
        </div>
        
        <div id="message" class="message"></div>
        
        <a href="/" class="back-link">Main Page</a>
    </div>

    <script>
        document.getElementById('otaForm').addEventListener('submit', async function(e) {
            e.preventDefault();
            
            const fileInput = document.getElementById('firmware');
            const button = document.getElementById('otaBtn');
            const messageDiv = document.getElementById('message');
            const progressContainer = document.getElementById('progressContainer');
            const progressFill = document.getElementById('progressFill');
            const progressText = document.getElementById('progressText');
            
            if (!fileInput.files.length) {
                showMessage('Please select a firmware file', 'error');
                return;
            }
            
            const file = fileInput.files[0];
            if (!file.name.endsWith('.bin')) {
                showMessage('Please select a .bin file', 'error');
                return;
            }
            
            // Show progress bar
            progressContainer.style.display = 'block';
            progressFill.style.width = '0%';
            progressText.textContent = '0%';
            
            // Disable button and show loading state
            button.disabled = true;
            button.textContent = 'Uploading...';
            messageDiv.style.display = 'none';
            
            try {
                const xhr = new XMLHttpRequest();
                
                // Track upload progress
                xhr.upload.addEventListener('progress', function(e) {
                    if (e.lengthComputable) {
                        const percentComplete = (e.loaded / e.total) * 100;
                        progressFill.style.width = percentComplete + '%';
                        progressText.textContent = Math.round(percentComplete) + '%';
                    }
                });
                
                xhr.addEventListener('load', function() {
                    if (xhr.status === 200) {
                        showMessage('Firmware update successful! Device will restart in a few seconds...', 'success');
                        // Wait a bit before page becomes unresponsive
                        setTimeout(() => {
                            showMessage('Device is restarting... You may need to reconnect.', 'success');
                        }, 3000);
                    } else {
                        showMessage('Update failed: ' + xhr.responseText, 'error');
                    }
                });
                
                xhr.addEventListener('error', function() {
                    showMessage('Network error during upload', 'error');
                });
                
                xhr.open('POST', '/update');
                xhr.setRequestHeader('Content-Type', 'application/octet-stream');
                xhr.send(file);
                
            } catch (error) {
                showMessage('Error: ' + error.message, 'error');
            } finally {
                // Re-enable button
                button.disabled = false;
                button.textContent = 'Update Firmware';
            }
        });
        
        function showMessage(text, type) {
            const messageDiv = document.getElementById('message');
            messageDiv.textContent = text;
            messageDiv.className = 'message ' + type;
            messageDiv.style.display = 'block';
            
            // Scroll to message
            messageDiv.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        }
        
        // Add some client-side validation for file size (optional)
        document.getElementById('firmware').addEventListener('change', function(e) {
            const file = e.target.files[0];
            if (file) {
                const fileSizeMB = file.size / (1024 * 1024);
                if (fileSizeMB > 2) {
                    showMessage('Warning: File size exceeds 2MB. Make sure your firmware fits in the OTA partition.', 'error');
                } else {
                    showMessage('File selected: ' + file.name + ' (' + fileSizeMB.toFixed(2) + ' MB)', 'success');
                }
            }
        });
    </script>
</body>
</html>
)";

    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/html");

    // Send response with the HTML form
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    // Log the request
    ESP_LOGI(TAG, "Serving /ota endpoint");

    return ESP_OK;
}

/**
 * @brief Handler for the /message endpoint
 */
esp_err_t message_get_handler(httpd_req_t *req)
{
    const char *final_result = WorkImplementation::getInstance().getMessage();

    // Set the HTTP Content-type header
    httpd_resp_set_type(req, "text/plain");

    // Send response with the string
    httpd_resp_send(req, final_result, HTTPD_RESP_USE_STRLEN);

    // Log the request
    ESP_LOGI(TAG, "Serving /message endpoint");

    return ESP_OK;
}

/**
 * @brief Handler for the wifisetup (/wifisetup) endpoint
 */
esp_err_t wifisetup_get_handler(httpd_req_t *req)
{
    const char *resp_str = R"(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WiFi Provisioning</title>
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
            width: 500px;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        .form-group {
            margin-bottom: 20px;
            text-align: left;
        }
        label {
            display: block;
            margin-bottom: 5px;
            color: #333;
            font-weight: bold;
        }
        input[type="text"],
        input[type="password"] {
            width: 100%;
            padding: 12px;
            border: 1px solid #ddd;
            border-radius: 5px;
            font-size: 16px;
            box-sizing: border-box;
        }
        button {
            background-color: #007bff;
            color: white;
            border: none;
            padding: 12px 30px;
            font-size: 16px;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        button:hover {
            background-color: #0056b3;
        }
        button:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        .message {
            margin-top: 20px;
            padding: 10px;
            border-radius: 5px;
            display: none;
        }
        .success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        @media (max-width: 600px) {
            .container {
                padding: 20px;
            }
            h1 {
                font-size: 1.5em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>WiFi Provisioning</h1>
        <p>Enter your WiFi credentials to connect the device to your network.</p>
        
        <form id="provisionForm">
            <div class="form-group">
                <label for="ssid">WiFi SSID:</label>
                <input type="text" id="ssid" name="ssid" required placeholder="Enter your WiFi name">
            </div>
            
            <div class="form-group">
                <label for="password">WiFi Password:</label>
                <input type="password" id="password" name="password" required placeholder="Enter your WiFi password">
            </div>
            
            <button type="submit" id="provisionBtn">Provision Device</button>
        </form>
        
        <div id="message" class="message"></div>
    </div>

    <script>
        document.getElementById('provisionForm').addEventListener('submit', async function(e) {
            e.preventDefault();
            
            const ssid = document.getElementById('ssid').value;
            const password = document.getElementById('password').value;
            const button = document.getElementById('provisionBtn');
            const messageDiv = document.getElementById('message');
            
            // Validate inputs
            if (!ssid || !password) {
                showMessage('Please enter both SSID and password', 'error');
                return;
            }
            
            // Disable button and show loading state
            button.disabled = true;
            button.textContent = 'Provisioning...';
            messageDiv.style.display = 'none';
            
            try {
                const response = await fetch('/provision', {
                    method: 'PUT',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        ssid: ssid,
                        password: password
                    })
                });
                
                if (response.ok) {
                    const result = await response.json();
                    showMessage('Provisioning successful! Device is connecting to WiFi...', 'success');
                    // Clear form
                    document.getElementById('provisionForm').reset();
                } else {
                    const error = await response.text();
                    showMessage('Provisioning failed: ' + error, 'error');
                }
            } catch (error) {
                showMessage('Network error: ' + error.message, 'error');
            } finally {
                // Re-enable button
                button.disabled = false;
                button.textContent = 'Provision Device';
            }
        });
        
        function showMessage(text, type) {
            const messageDiv = document.getElementById('message');
            messageDiv.textContent = text;
            messageDiv.className = 'message ' + type;
            messageDiv.style.display = 'block';
        }
    </script>
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

/**
 * @brief Handler for the /provision endpoint (PUT method)
 */
esp_err_t provision_put_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received provisioning request");

    // Check content length
    size_t content_len = req->content_len;
    if (content_len > 512)
    { // Limit to 512 bytes for safety
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    // Allocate buffer for received data
    char *content = (char *)malloc(content_len + 1);
    if (content == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for request content");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    // Receive the data
    int ret = httpd_req_recv(req, content, content_len);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timeout");
        }
        free(content);
        return ESP_FAIL;
    }
    content[ret] = '\0'; // Null-terminate the string

    ESP_LOGI(TAG, "Received provision data: %s", content);

    // Parse JSON data
    cJSON *root = cJSON_Parse(content);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
        free(content);
        return ESP_FAIL;
    }

    // Extract SSID and password
    cJSON *ssid_json = cJSON_GetObjectItem(root, "ssid");
    cJSON *password_json = cJSON_GetObjectItem(root, "password");

    if (!cJSON_IsString(ssid_json) || !cJSON_IsString(password_json))
    {
        ESP_LOGE(TAG, "Missing or invalid ssid/password in JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid ssid/password");
        cJSON_Delete(root);
        free(content);
        return ESP_FAIL;
    }

    char *ssid = ssid_json->valuestring;
    char *password = password_json->valuestring;

    // Validate parameters
    if (strlen(ssid) == 0 || strlen(password) == 0)
    {
        ESP_LOGE(TAG, "Empty ssid or password");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID and password cannot be empty");
        cJSON_Delete(root);
        free(content);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Provisioning with SSID: %s, Password: [hidden]", ssid);

    nvs_save_credentials(ssid, password);

    if (provisioning_sem != NULL)
    {
        xSemaphoreGive(provisioning_sem);
    }

    // Clean up
    cJSON_Delete(root);
    free(content);

    // Send success response
    httpd_resp_set_type(req, "application/json");
    const char *success_resp = "{\"status\":\"success\",\"message\":\"WiFi credentials received\"}";
    httpd_resp_send(req, success_resp, HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Provisioning completed successfully");
    return ESP_OK;
}

// ----------------------------------------------------------------------
// 2. URI Definitions
// ----------------------------------------------------------------------
/**
 * @brief URI structure for /api/sensors/latest endpoint
 */
/**
 * @brief URI structure for /start endpoint
 */
httpd_uri_t start_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = start_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI structure for OTA update endpoint
 */
httpd_uri_t ota_update_uri = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = ota_update_handler,
    .user_ctx = NULL};

/**
 * @brief URI structure for OTA page endpoint
 */
httpd_uri_t ota_page_uri = {
    .uri = "/ota",
    .method = HTTP_GET,
    .handler = ota_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI structure for /message endpoint
 */
httpd_uri_t message_uri = {
    .uri = "/message",
    .method = HTTP_GET,
    .handler = message_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI structure for /wifisetup endpoint
 */
httpd_uri_t wifisetup_uri = {
    .uri = "/wifisetup",
    .method = HTTP_GET,
    .handler = wifisetup_get_handler,
    .user_ctx = NULL};

/**
 * @brief URI structure for /provision endpoint (PUT method)
 */
httpd_uri_t provision_uri = {
    .uri = "/provision",
    .method = HTTP_PUT,
    .handler = provision_put_handler,
    .user_ctx = NULL};

// ----------------------------------------------------------------------
// 3. Server Initialization and Handler Registration
// ----------------------------------------------------------------------

/**
 * @brief Register a URI handler with the server
 */
esp_err_t register_uri_handler(httpd_handle_t server, const httpd_uri_t *uri_handler)
{
    if (server == NULL || uri_handler == NULL)
    {
        ESP_LOGE(TAG, "Invalid parameters for register_uri_handler");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = httpd_register_uri_handler(server, uri_handler);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Registered URI: %s, method: %d", uri_handler->uri, uri_handler->method);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to register URI: %s, error: %d", uri_handler->uri, ret);
    }

    return ret;
}

/**
 * @brief Register multiple URI handlers at once
 */
esp_err_t register_uri_handlers(httpd_handle_t server, const httpd_uri_t *uri_handlers, size_t count)
{
    if (server == NULL || uri_handlers == NULL)
    {
        ESP_LOGE(TAG, "Invalid parameters for register_uri_handlers");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t overall_result = ESP_OK;

    for (size_t i = 0; i < count; i++)
    {
        esp_err_t ret = register_uri_handler(server, &uri_handlers[i]);
        if (ret != ESP_OK)
        {
            overall_result = ret;
            // Continue registering other handlers even if one fails
        }
    }

    return overall_result;
}

/**
 * @brief Register default URI handlers (root, message, provision)
 */
esp_err_t register_default_uri_handlers(httpd_handle_t server)
{
    if (server == NULL)
    {
        ESP_LOGE(TAG, "Invalid server handle for register_default_uri_handlers");
        return ESP_ERR_INVALID_ARG;
    }

    // Array of default URI handlers
    const httpd_uri_t *default_handlers[] = {
        &wifisetup_uri,
        &message_uri,
        &provision_uri,
        &ota_update_uri,
        &ota_page_uri,
        &start_uri};

    size_t num_handlers = sizeof(default_handlers) / sizeof(default_handlers[0]);
    esp_err_t overall_result = ESP_OK;

    for (size_t i = 0; i < num_handlers; i++)
    {
        esp_err_t ret = register_uri_handler(server, default_handlers[i]);
        if (ret != ESP_OK)
        {
            overall_result = ret;
        }
    }

    return overall_result;
}

/**
 * @brief Starts the HTTP server instance
 * @param register_default_handlers If true, registers the default handlers (root, message, provision)
 * @return httpd_handle_t Handle to the started server, or NULL on failure
 */
httpd_handle_t start_webserver(bool register_default_handlers)
{
    // Create provisioning semaphore if it doesn't exist
    if (provisioning_sem == NULL)
    {
        provisioning_sem = xSemaphoreCreateBinary();
        if (provisioning_sem == NULL)
        {
            ESP_LOGE(TAG, "Failed to create provisioning semaphore");
            return NULL;
        }
    }

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;                   // Default is 8, increase to 16
    config.uri_match_fn = httpd_uri_match_wildcard; // Optional: enable wildcard matching

    // Start the httpd server
    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Register default URI handlers if requested
        if (register_default_handlers)
        {
            ESP_LOGI(TAG, "Registering default URI handlers");
            register_default_uri_handlers(server);
        }
        else
        {
            ESP_LOGI(TAG, "Skipping default URI handlers registration");
        }
        return server;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return NULL;
}

/**
 * @brief Alternative function to start webserver with custom configuration
 * @param config Server configuration (can be NULL for defaults)
 * @param register_default_handlers If true, registers the default handlers
 * @return httpd_handle_t Handle to the started server, or NULL on failure
 */
httpd_handle_t start_webserver_with_config(const httpd_config_t *config, bool register_default_handlers)
{
    // Create provisioning semaphore if it doesn't exist
    if (provisioning_sem == NULL)
    {
        provisioning_sem = xSemaphoreCreateBinary();
        if (provisioning_sem == NULL)
        {
            ESP_LOGE(TAG, "Failed to create provisioning semaphore");
            return NULL;
        }
    }

    httpd_handle_t server = NULL;
    httpd_config_t server_config;

    // Use provided config or default config
    if (config != NULL)
    {
        server_config = *config;
    }
    else
    {
        server_config = HTTPD_DEFAULT_CONFIG();
    }

    // Start the httpd server
    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", server_config.server_port);
    if (httpd_start(&server, &server_config) == ESP_OK)
    {
        // Register default URI handlers if requested
        if (register_default_handlers)
        {
            ESP_LOGI(TAG, "Registering default URI handlers");
            register_default_uri_handlers(server);
        }
        else
        {
            ESP_LOGI(TAG, "Skipping default URI handlers registration");
        }
        return server;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return NULL;
}