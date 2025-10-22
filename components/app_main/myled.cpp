#ifdef __cplusplus
extern "C"
{
#endif


#include "myled.h"

void init_leds()
{
    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
}

void led_set_state(gpio_num_t gpio_pin, bool state)
{
    gpio_set_level(gpio_pin, state ? 1 : 0);
}

#ifdef __cplusplus
}
#endif

