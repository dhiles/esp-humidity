#ifndef MYLED_H
#define MYLED_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdbool.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON,
    LED_STATE_FLASHING
} led_state_t;

// Existing functions (unchanged API)
extern "C" void init_led(gpio_num_t gpio_pin);
extern "C" void led_set_state(gpio_num_t gpio_pin, bool state);
extern "C" void led_start_flashing(gpio_num_t gpio_pin, uint64_t period_ms);
extern "C" void led_stop_flashing(gpio_num_t gpio_pin, bool stop_level);
void led_cleanup(gpio_num_t gpio_pin);
void led_cleanup_all(void);

void led_global_set_enabled(bool enabled);
bool led_global_is_enabled(void);

//#ifdef __cplusplus
//}
//#endif

#endif // MYLED_H