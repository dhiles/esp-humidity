#ifdef __cplusplus
extern "C"
{
#endif

#include "myled.h"
#include <stdint.h>  // For uintptr_t

static const char *TAG = "MYLED";

#define MAX_LEDS 4  // Support up to 4 LEDs; increase as needed

// Structure for per-LED blinking state
typedef struct {
    gpio_num_t pin;
    esp_timer_handle_t timer;
    bool state;
    uint64_t period_us;
    bool active;
} led_blink_t;

// Global array for up to MAX_LEDS
static led_blink_t leds[MAX_LEDS];

void init_led(gpio_num_t gpio_pin)
{
    gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
}

void led_set_state(gpio_num_t gpio_pin, bool state)
{
    gpio_set_level(gpio_pin, state ? 1 : 0);
}

// Timer callback: Toggles specified LED every period using passed arg (LED index)
static void blink_timer_callback(void *arg)
{
    int index = (int)(uintptr_t)arg;  // Safe cast: void* -> uintptr_t -> int (LED index)
    if (index < 0 || index >= MAX_LEDS || !leds[index].active) {
        return;  // Invalid or inactive
    }

    leds[index].state = !leds[index].state;
    gpio_set_level(leds[index].pin, leds[index].state ? 1 : 0);
    ESP_LOGI(TAG, "LED on pin %d %s (index %d)", leds[index].pin, leds[index].state ? "ON" : "OFF", index);
}

// Function to start flashing: Specify LED index (0 to MAX_LEDS-1), pin, and dynamic period in ms
void led_start_flashing(int index, gpio_num_t gpio_pin, uint64_t period_ms)
{
    if (index < 0 || index >= MAX_LEDS) {
        ESP_LOGE(TAG, "Invalid LED index: %d (must be 0 to %d)", index, MAX_LEDS - 1);
        return;
    }

    led_blink_t *led = &leds[index];

    if (led->timer == NULL) {
        // Configure GPIO (done once on first start)
        gpio_reset_pin(gpio_pin);
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_pin, 0);  // Start off

        // Create timer and pass index as arg
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &blink_timer_callback;
        timer_args.arg = (void *)(uintptr_t)index;  // Pass LED index to every callback
        timer_args.name = "blink_timer";  // Note: Shared name; could make dynamic if needed
        timer_args.dispatch_method = ESP_TIMER_TASK;  // Default: Run in timer service task
        timer_args.skip_unhandled_events = false;
        esp_err_t err = esp_timer_create(&timer_args, &led->timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create timer for index %d: %s", index, esp_err_to_name(err));
            return;
        }

        // Initialize LED struct
        led->pin = gpio_pin;
        led->state = false;
        led->active = false;

        ESP_LOGI(TAG, "Blink timer created for LED index %d on pin %d", index, gpio_pin);
    }

    // Update period (convert ms to µs for storage and API)
    led->period_us = period_ms * 1000ULL;

    // Start periodic
    esp_timer_start_periodic(led->timer, led->period_us);
    led->active = true;
    ESP_LOGI(TAG, "LED flashing started on pin %d (index %d) with period %llu ms", gpio_pin, index, period_ms);
}

// Function to stop flashing: Stop the timer for specified LED index and set final level
void led_stop_flashing(int index, bool stop_level)
{
    if (index < 0 || index >= MAX_LEDS) {
        ESP_LOGW(TAG, "Invalid LED index: %d (must be 0 to %d)", index, MAX_LEDS - 1);
        return;
    }

    led_blink_t *led = &leds[index];

    if (led->timer != NULL && led->active) {
        esp_timer_stop(led->timer);
        led->active = false;
        ESP_LOGI(TAG, "LED flashing stopped for index %d", index);
        gpio_set_level(led->pin, stop_level ? 1 : 0);  // Ensure LED is on/off as specified
    } else {
        ESP_LOGW(TAG, "No active timer to stop for index %d", index);
    }
}

// Optional: Permanent cleanup for specified LED index (call when app ends)
void led_cleanup(int index)
{
    if (index < 0 || index >= MAX_LEDS) {
        ESP_LOGE(TAG, "Invalid LED index: %d (must be 0 to %d)", index, MAX_LEDS - 1);
        return;
    }

    led_blink_t *led = &leds[index];

    if (led->timer != NULL) {
        esp_timer_stop(led->timer);
        esp_timer_delete(led->timer);
        led->timer = NULL;
        led->active = false;
        led->state = false;
        ESP_LOGI(TAG, "Blink timer deleted for LED index %d", index);
    }
}

// Optional: Cleanup all LEDs
void led_cleanup_all(void)
{
    for (int i = 0; i < MAX_LEDS; i++) {
        led_cleanup(i);
    }
    ESP_LOGI(TAG, "All %d blink timers cleaned up", MAX_LEDS);
}

#ifdef __cplusplus
}
#endif