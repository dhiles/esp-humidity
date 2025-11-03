#ifndef CUSTOM_WEBSERVER_H
#define CUSTOM_WEBSERVER_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "cjson.h"
#include "esp_log.h"
#include "webserver.h"

void start_custom_webserver();

#endif