#ifndef SHT4X_H
#define SHT4X_H

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

void sht4x_init(void);
esp_err_t sht4x_measure_high_precision(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif // SHT4X_H