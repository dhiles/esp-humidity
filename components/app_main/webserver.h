#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdio.h>
#include <string>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "bme280mgr.h"

#include "esp_http_server.h"

httpd_handle_t start_webserver(void);
esp_err_t hello_get_handler(httpd_req_t *req);

#endif


