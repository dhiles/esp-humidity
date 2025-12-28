#include "myntp.h"

const char *MyNTP::TAG = "MYNTP";
const char *timeZone = "MST7MDT,M3.2.0/2,M11.1.0";

void MyNTP::initialize()
{
    ESP_LOGI(TAG, "Initializing SNTP client");
    setenv("TZ", timeZone, 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "192.168.1.101"); // Local NTP server first
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_setservername(2, "time.google.com");
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED); // Immediate sync for faster response
    esp_sntp_set_time_sync_notification_cb(timeSyncCallback);
    esp_sntp_init();
}

bool MyNTP::syncTime()
{
    if (!MyWiFi::isConnected() || !MyWiFi::isNetworkReady())
    {
        ESP_LOGE(TAG, "Cannot sync time: Wi-Fi not connected or network not ready");
        return false;
    }

    ESP_LOGI(TAG, "Starting NTP time sync");
    int retry = 0;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < RETRY_COUNT)
    {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        retry++;
    }

    time_t now;
    time(&now);
    struct tm *timeinfo = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);

    if (now >= 946684800)
    { // After 2000-01-01
        ESP_LOGI(TAG, "Time synced successfully: %s (epoch: %lld)", time_str, (long long)now);
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to sync time: %s (epoch: %lld)", time_str, (long long)now);
        // Try restarting SNTP with a different server
        ESP_LOGI(TAG, "Retrying with time.google.com...");
        esp_sntp_setservername(0, "time.google.com");
        esp_sntp_restart();
        retry = 0;
        while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < RETRY_COUNT)
        {
            ESP_LOGI(TAG, "Retry: Waiting for time sync... (%d/%d)", retry, RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            retry++;
        }
        time(&now);
        timeinfo = localtime(&now);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
        if (now >= 946684800)
        {
            ESP_LOGI(TAG, "Time synced successfully on retry: %s (epoch: %lld)", time_str, (long long)now);
            return true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to sync time on retry: %s (epoch: %lld)", time_str, (long long)now);
            return false;
        }
    }
}

// Helper: Parse "YYYYMMDD_HHMMSS" back to epoch timestamp
uint32_t MyNTP::parse_timestamp(const char *timestamp_str)
{
    if (!timestamp_str || strlen(timestamp_str) != 15)
    { // exactly "YYYYMMDD_HHMMSS"
        ESP_LOGE(TAG, "Invalid timestamp length: %s", timestamp_str);
        return 0;
    }

    struct tm timeinfo = {0};

    // Manual parsing – very safe and portable
    if (sscanf(timestamp_str,
               "%4u%2u%2u_%2u%2u%2u",
               (unsigned int *)&timeinfo.tm_year,
               (unsigned int *)&timeinfo.tm_mon,
               (unsigned int *)&timeinfo.tm_mday,
               (unsigned int *)&timeinfo.tm_hour,
               (unsigned int *)&timeinfo.tm_min,
               (unsigned int *)&timeinfo.tm_sec) != 6)
    {
        ESP_LOGE(TAG, "sscanf failed to parse: %s", timestamp_str);
        return 0;
    }

    // Adjust for struct tm fields
    timeinfo.tm_year -= 1900; // struct tm expects years since 1900
    timeinfo.tm_mon -= 1;     // struct tm expects 0-11

    time_t epoch = mktime(&timeinfo);
    if (epoch == (time_t)-1)
    {
        ESP_LOGE(TAG, "mktime failed for: %s", timestamp_str);
        return 0;
    }

    return static_cast<uint32_t>(epoch);
}

// New method: Get current epoch timestamp from formatted string
uint32_t MyNTP::getEpochTimestamp()
{
    char timestamp_buf[32] = {0};
    if (!MyNTP::getTimestamp(timestamp_buf, sizeof(timestamp_buf)))
    {
        ESP_LOGE(TAG, "Failed to get timestamp - returning 0");
        return 0;
    }

    uint32_t epoch = parse_timestamp(timestamp_buf);
    if (epoch == 0)
    {
        ESP_LOGE(TAG, "Failed to parse timestamp: %s", timestamp_buf);
    }
    else
    {
        ESP_LOGD(TAG, "Parsed epoch timestamp: %u from %s", epoch, timestamp_buf);
    }

    return epoch;
}

bool MyNTP::getTimestamp(char *buf, size_t buf_size)
{
    time_t now;
    time(&now);
    if (now < 946684800)
    { // Before 2000-01-01
        ESP_LOGE(TAG, "Invalid system time, using uptime-based timestamp");
        snprintf(buf, buf_size, "default_%lld", (long long)(esp_timer_get_time() / 1000000));
        return false;
    }
    else
    {
        struct tm *timeinfo = localtime(&now);
        strftime(buf, buf_size, "%Y%m%d_%H%M%S", timeinfo);
        ESP_LOGI(TAG, "Generated timestamp: %s", buf);
        return true;
    }
}

bool MyNTP::isTimeSynced()
{
    time_t now;
    time(&now);
    return now >= 946684800; // After 2000-01-01
}

void MyNTP::shutdown()
{
    ESP_LOGI(TAG, "Shutting down SNTP client");
    esp_sntp_stop();
}

void MyNTP::timeSyncCallback(struct timeval *tv)
{
    time_t now = tv->tv_sec;
    struct tm *timeinfo = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
    ESP_LOGI(TAG, "Time sync callback triggered: %s (epoch: %lld)", time_str, (long long)now);
}