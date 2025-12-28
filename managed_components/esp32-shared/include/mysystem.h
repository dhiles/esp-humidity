#ifndef MYSYSTEM_H
#define MYSYSTEM_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_psram.h"
#include "constants.h"

class MySystem {
public:

    // Returns device type + permanent 8-char unique ID (e.g. "HUMD-K9M4P2X7")
    // Thread-safe, zero-cost after first call
    static const char* get_device_id();

    // Existing function
    static void check_psram();

private:
    static void generate_device_id();   // called only once

    static char device_id[9];           // 8 chars + null terminator
    static bool id_generated;
};

#endif

