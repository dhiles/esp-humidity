#ifndef NVS_JSON_H
#define NVS_JSON_H

#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize JSON objects for all NVS namespaces
esp_err_t init_nvs_json_all(void);

// Get the JSON object for a specific namespace
cJSON *get_nvs_json(const char *ns);

// Free all global JSON objects
void free_nvs_json_all(void);

#ifdef __cplusplus
}
#endif

#endif // NVS_JSON_H