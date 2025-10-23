#ifdef __cplusplus
extern "C"
{
#endif

#include "myled.h"
#include <stdint.h>  // For uintptr_t
#include <stdio.h>   // For snprintf

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

// Helper: Find index by pin (returns -1 if not found)
static int find_index_by_pin(gpio_num_t pin)
{
    for (int i = 0; i < MAX_LEDS; i++) {
        if (leds[i].pin == pin) {
            return i;
        }
    }
    return -1;
}

// Helper: Find free index (timer == NULL; returns -1 if none)
static int find_free_index(void)
{
    for (int i = 0; i < MAX_LEDS; i++) {
        if (leds[i].timer == NULL) {
            return i;
        }
    }
    return -1;
}

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
    ESP_LOGI(TAG, "LED on pin %d %s", leds[index].pin, leds[index].state ? "ON" : "OFF");
}

// Function to start flashing: Specify pin and dynamic period in ms
void led_start_flashing(gpio_num_t gpio_pin, uint64_t period_ms)
{
    int index = find_index_by_pin(gpio_pin);
    if (index == -1) {
        index = find_free_index();
        if (index == -1) {
            ESP_LOGE(TAG, "No free LED slots available (max %d)", MAX_LEDS);
            return;
        }
    }

    led_blink_t *led = &leds[index];
    bool creating = (led->timer == NULL);

    if (creating) {
        bool is_new_pin = (leds[index].pin != gpio_pin);
        if (is_new_pin) {
            gpio_reset_pin(gpio_pin);
        }
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_pin, 0);
        if (is_new_pin) {
            led->pin = gpio_pin;
        }
        led->state = false;

        // Create timer
        char timer_name[20];
        snprintf(timer_name, sizeof(timer_name), "blink_timer_%d", index);
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &blink_timer_callback;
        timer_args.arg = (void *)(uintptr_t)index;
        timer_args.name = timer_name;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.skip_unhandled_events = false;
        esp_err_t err = esp_timer_create(&timer_args, &led->timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create timer for pin %d: %s", gpio_pin, esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "Blink timer created for LED on pin %d", gpio_pin);
    }

    // Update period (convert ms to µs for storage and API)
    led->period_us = period_ms * 1000ULL;

    // Start periodic
    esp_timer_start_periodic(led->timer, led->period_us);
    led->active = true;
    ESP_LOGI(TAG, "LED flashing started on pin %d with period %llu ms", gpio_pin, period_ms);
}

// Function to stop flashing: Stop the timer for specified pin and set final level
void led_stop_flashing(gpio_num_t gpio_pin, bool stop_level)
{
    int index = find_index_by_pin(gpio_pin);
    if (index < 0) {
        ESP_LOGW(TAG, "No LED configured for pin %d", gpio_pin);
        return;
    }

    led_blink_t *led = &leds[index];

    if (led->timer != NULL && led->active) {
        esp_timer_stop(led->timer);
        led->active = false;
        led->state = stop_level;
        gpio_set_level(led->pin, stop_level ? 1 : 0);
        ESP_LOGI(TAG, "LED flashing stopped for pin %d", gpio_pin);
    } else {
        ESP_LOGW(TAG, "No active timer to stop for pin %d", gpio_pin);
    }
}

// Optional: Permanent cleanup for specified pin (call when app ends)
void led_cleanup(gpio_num_t gpio_pin)
{
    int index = find_index_by_pin(gpio_pin);
    if (index < 0) {
        ESP_LOGW(TAG, "No LED configured for pin %d", gpio_pin);
        return;
    }

    led_blink_t *led = &leds[index];

    if (led->timer != NULL) {
        esp_timer_stop(led->timer);
        led->active = false;
        led->state = false;
        gpio_set_level(led->pin, 0);
        esp_timer_delete(led->timer);
        led->timer = NULL;
        ESP_LOGI(TAG, "Blink timer deleted for LED on pin %d", gpio_pin);
    } else {
        ESP_LOGW(TAG, "No timer to delete for pin %d", gpio_pin);
    }
}

// Optional: Cleanup all LEDs
void led_cleanup_all(void)
{
    int cleaned = 0;
    for (int i = 0; i < MAX_LEDS; i++) {
        if (leds[i].timer != NULL) {
            esp_timer_stop(leds[i].timer);
            leds[i].active = false;
            leds[i].state = false;
            gpio_set_level(leds[i].pin, 0);
            esp_timer_delete(leds[i].timer);
            leds[i].timer = NULL;
            ESP_LOGI(TAG, "Blink timer deleted for LED on pin %d", leds[i].pin);
            cleaned++;
        }
    }
    ESP_LOGI(TAG, "All %d blink timers cleaned up", cleaned);
}

#ifdef __cplusplus
}
#endif