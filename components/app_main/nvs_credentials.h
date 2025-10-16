#ifndef NVS_CREDENTIALS_H
#define NVS_CREDENTIALS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void nvs_save_credentials();
esp_err_t nvs_load_credentials(wifi_config_t *wifi_config);

#ifdef __cplusplus
}
#endif

#endif // NVS_JSON_H