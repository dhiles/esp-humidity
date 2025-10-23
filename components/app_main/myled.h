#ifndef MYLED_H
#define MYLED_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define RED_LED GPIO_NUM_13
#define GREEN_LED GPIO_NUM_14
#define BLUE_LED GPIO_NUM_15


#ifdef __cplusplus
extern "C"
{
#endif

void init_led(gpio_num_t gpio_pin);
void led_set_state(gpio_num_t gpio_pin, bool state);
void led_start_flashing(int index, gpio_num_t gpio_pin, uint64_t period_ms);
void led_stop_flashing(int index, bool stop_level);
void led_cleanup(int index);

#ifdef __cplusplus
}
#endif



#endif