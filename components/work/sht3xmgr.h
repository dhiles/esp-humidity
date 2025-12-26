// components/work/sht3xmgr.h
#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

void sht3x_init(void);
esp_err_t get_sht3x_reading(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif