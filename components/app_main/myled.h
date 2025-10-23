#ifndef MYLED_H
#define MYLED_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"


#ifdef __cplusplus
extern "C"
{
#endif

void init_led(gpio_num_t gpio_pin);
void led_set_state(gpio_num_t gpio_pin, bool state);
void led_start_flashing(gpio_num_t gpio_pin, uint64_t period_ms);
void led_stop_flashing(gpio_num_t gpio_pin, bool stop_level);
void led_cleanup(gpio_num_t gpio_pin);

#ifdef __cplusplus
}
#endif



#endif