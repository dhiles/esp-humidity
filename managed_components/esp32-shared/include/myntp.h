#ifndef MYNTP_H
#define MYNTP_H

#include <string.h>
#include <time.h>
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mywifi.h"

class MyNTP {
public:
    // Initialize SNTP client
    static void initialize();

    // Sync time with NTP servers, returns true if successful
    static bool syncTime();

    // Get formatted timestamp for filenames (e.g., "20250816_134503")
    static bool getTimestamp(char *buf, size_t buf_size);

    // Check if time is synchronized
    static bool isTimeSynced();

    // Shutdown SNTP client
    static void shutdown();

    static uint32_t parse_timestamp(const char* timestamp_str);
    static uint32_t getEpochTimestamp();

private:
    static const char *TAG;
    static const int RETRY_COUNT = 15;
    static const int RETRY_DELAY_MS = 2000;
    static void timeSyncCallback(struct timeval *tv);
};

#endif