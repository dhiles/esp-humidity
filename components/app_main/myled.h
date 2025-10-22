#ifndef MYLED_H
#define MYLED_H

#include "driver/gpio.h"

#define RED_LED GPIO_NUM_13
#define GREEN_LED GPIO_NUM_14

#ifdef __cplusplus
extern "C"
{
#endif

void init_leds();
void led_set_state(gpio_num_t gpio_pin, bool state);

#ifdef __cplusplus
}
#endif



#endif