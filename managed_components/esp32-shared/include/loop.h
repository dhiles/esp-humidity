#ifndef LOOP_H
#define LOOP_H

#include "mymqtt.h"
#include "mywifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include "work_interface.h"

typedef struct {
    uint32_t sleep_duration;
    const char* mqtt_uri;
    WorkInterface* work_impl;
} sleep_loop_params_t;

void sleep_loop(int sleep_seconds, const char* uri, WorkInterface& work);
void sleep_loop_task(void *pvParameters);

#endif
