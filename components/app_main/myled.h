#ifndef MYLED_H
#define MYLED_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define RED_LED GPIO_NUM_13
#define GREEN_LED GPIO_NUM_14

#ifdef __cplusplus
extern "C"
{
#endif

void init_led(gpio_num_t gpio_pin);
void led_set_state(gpio_num_t gpio_pin, bool state);
void led_start_flashing(gpio_num_t gpio_pin);
void led_stop_flashing(gpio_num_t gpio_pin, int stop_level);
void led_cleanup(void);

#ifdef __cplusplus
}
#endif



#endif