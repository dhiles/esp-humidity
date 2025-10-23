#ifdef __cplusplus
extern "C"
{
#endif

#include "myled.h"

    static const char *TAG = "MYLED";

    // Timer handle (global for start/stop access)
    static esp_timer_handle_t blink_timer = NULL;

    // Track LED state for toggling in callback
    static bool led_state = false;

    void init_led(gpio_num_t gpio_pin)
    {
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
    }

    void led_set_state(gpio_num_t gpio_pin, bool state)
    {
        gpio_set_level(gpio_pin, state ? 1 : 0);
    }

    // Timer callback: Toggles LED every period using passed arg (GPIO pin)
    static void blink_timer_callback(void *arg)
    {
        gpio_num_t pin = (gpio_num_t)(uintptr_t)arg; // Safe cast: void* -> uintptr_t -> gpio_num_t        
        led_state = !led_state;
        gpio_set_level(pin, led_state ? 1 : 0);
        ESP_LOGI(TAG, "LED on pin %d %s", pin, led_state ? "ON" : "OFF");
    }

    // Function to start flashing: Create if needed, then start periodic
    void led_start_flashing(gpio_num_t gpio_pin)
    {
        if (blink_timer == NULL)
        {
            // Configure GPIO (done once on first start)
            gpio_reset_pin(gpio_pin);
            gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
            gpio_set_level(gpio_pin, 0); // Start off

            // Create timer and pass gpio_pin as arg
            esp_timer_create_args_t timer_args = {};
            timer_args.callback = &blink_timer_callback;
            timer_args.arg = (void *)gpio_pin; // Pass the pin to every callback
            timer_args.name = "blink_timer";
            timer_args.dispatch_method = ESP_TIMER_TASK; // Default: Run in timer service task
            timer_args.skip_unhandled_events = false;
            esp_err_t err = esp_timer_create(&timer_args, &blink_timer);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "Blink timer created for pin %d", gpio_pin);
        }

        // Start periodic (period in microseconds; 500,000 µs = 500 ms half-cycle)
        esp_timer_start_periodic(blink_timer, 500000);
        ESP_LOGI(TAG, "LED flashing started on pin %d", gpio_pin);
    }

    // Function to stop flashing: Stop the timer
    void led_stop_flashing(gpio_num_t gpio_pin, int stop_level)
    {
        if (blink_timer != NULL)
        {
            esp_timer_stop(blink_timer);
            ESP_LOGI(TAG, "LED flashing stopped");
            gpio_set_level(gpio_pin, stop_level); // Ensure LED is off or on
        }
        else
        {
            ESP_LOGW(TAG, "No timer to stop; flashing not started");
        }
    }

    // Optional: Permanent cleanup (call when app ends)
    void led_cleanup(void)
    {
        if (blink_timer != NULL)
        {
            esp_timer_stop(blink_timer);
            esp_timer_delete(blink_timer);
            blink_timer = NULL;
            ESP_LOGI(TAG, "Blink timer deleted");
        }
    }

#ifdef __cplusplus
}
#endif
