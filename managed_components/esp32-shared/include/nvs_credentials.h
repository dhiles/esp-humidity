#ifndef NVS_CREDENTIALS_H
#define NVS_CREDENTIALS_H

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

void nvs_save_ssid(char* received_ssid);
void nvs_save_password(char* received_password);
void nvs_save_credentials(char* received_ssid, char* received_pass);
esp_err_t nvs_load_credentials(wifi_config_t *wifi_config);
void nvs_erase_credentials(void);

#ifdef __cplusplus
}
#endif

#endif // NVS_JSON_H