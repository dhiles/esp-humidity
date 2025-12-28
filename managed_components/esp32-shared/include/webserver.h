#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdio.h>
#include <string>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include "freertos/semphr.h"
#include <nvs_flash.h>
#include "nvs_credentials.h"
#include <sys/param.h>
#include "cJSON.h" 
#include "esp_http_server.h"
#include "work.h"
#include "esp_ota_ops.h"
#include "sensor_data_manager.h"   // Your singleton class

extern SemaphoreHandle_t provisioning_sem;

esp_err_t register_uri_handler(httpd_handle_t server, const httpd_uri_t *uri_handler);
esp_err_t register_uri_handlers(httpd_handle_t server, const httpd_uri_t *uri_handlers, size_t count);
esp_err_t register_default_uri_handlers(httpd_handle_t server);
httpd_handle_t start_webserver(bool register_default_handlers);
httpd_handle_t start_webserver_with_config(const httpd_config_t *config, bool register_default_handlers);

#endif


