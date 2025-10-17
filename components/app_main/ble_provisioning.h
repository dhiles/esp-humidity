#ifndef BLE_PROVISIONING_H
#define BLE_PROVISIONING_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h" // Added for C++ compatibility

#ifdef __cplusplus
extern "C" {
#endif

// Core NimBLE Headers
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_credentials.h"
#include <freertos/semphr.h>

extern SemaphoreHandle_t provisioning_sem;

void ble_provisioning_init(void);

#undef min
#undef max

#ifdef __cplusplus
}
#endif


#endif