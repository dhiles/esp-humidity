// #ifdef __cplusplus
// extern "C" {
// #endif

#include "myled.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <map>

static const char *TAG = "MYLED";

// Per-LED data stored in the map
typedef struct
{
    led_state_t desired_state; // OFF, ON, or FLASHING
    uint64_t period_us;        // blinking period in microseconds
    esp_timer_handle_t timer;
    bool current_level; // current physical level (for toggle)
} led_data_t;

// C++ map: key = gpio_num_t → value = led_data_t
static std::map<gpio_num_t, led_data_t> *led_map = NULL;

// One global switch — turn all LEDs on/off together
static bool g_leds_enabled = true;

// Timer callback — toggles one LED
static void blink_timer_callback(void *arg)
{
    gpio_num_t pin = (gpio_num_t)(uintptr_t)arg;

    if (!g_leds_enabled)
        return;

    if (led_map && led_map->find(pin) != led_map->end())
    {
        led_data_t &led = led_map->at(pin);
        if (led.desired_state != LED_STATE_FLASHING)
            return;

        led.current_level = !led.current_level;
        gpio_set_level(pin, led.current_level);
        ESP_LOGI(TAG, "LED on pin %d %s", pin, led.current_level ? "ON" : "OFF");
    }
}

// Apply desired state to hardware (only if globally enabled)
static void apply_state(gpio_num_t pin, led_data_t *led)
{
    if (!g_leds_enabled)
    {
        gpio_set_level(pin, 0);
        return;
    }

    switch (led->desired_state)
    {
    case LED_STATE_OFF:
        gpio_set_level(pin, 0);
        led->current_level = false;
        break;
    case LED_STATE_ON:
        gpio_set_level(pin, 1);
        led->current_level = true;
        break;
    case LED_STATE_FLASHING:
        // Timer controls it
        break;
    }
}

// ==============================================================
extern "C" void init_led(gpio_num_t gpio_pin)
{
    if (!led_map)
    {
        led_map = new std::map<gpio_num_t, led_data_t>();
    }

    if (led_map->find(gpio_pin) != led_map->end())
    {
        ESP_LOGW(TAG, "LED on GPIO%d already initialized", gpio_pin);
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(gpio_pin, 0);

    led_data_t data = {
        .desired_state = LED_STATE_OFF,
        .period_us = 500000, // 500ms default
        .timer = NULL,
        .current_level = false};

    (*led_map)[gpio_pin] = data;
    ESP_LOGI(TAG, "LED initialized on GPIO%d → added to map", gpio_pin);
}

extern "C" void led_set_state(gpio_num_t gpio_pin, bool state)
{
    if (!led_map)
        return;

    auto it = led_map->find(gpio_pin);
    if (it == led_map->end())
        return;

    led_data_t &led = it->second;

    if (led.timer)
    {
        esp_timer_stop(led.timer);
    }

    led.desired_state = state ? LED_STATE_ON : LED_STATE_OFF;
    apply_state(gpio_pin, &led);
}

// ==============================================================
extern "C" void led_start_flashing(gpio_num_t gpio_pin, uint64_t period_ms)
{
    if (!led_map)
    {
        ESP_LOGE(TAG, "LED map not initialized");
        return;
    }

    auto it = led_map->find(gpio_pin);
    if (it == led_map->end())
    {
        ESP_LOGE(TAG, "LED on GPIO%d not initialized", gpio_pin);
        return;
    }

    led_data_t &led = it->second;
    uint64_t period_us = period_ms * 1000ULL;

    // Create timer if needed
    if (!led.timer)
    {
        char name[24];
        snprintf(name, sizeof(name), "blink_%d", gpio_pin);
        esp_timer_create_args_t t = {};
        t.callback = blink_timer_callback;
        t.arg = (void *)(uintptr_t)gpio_pin;
        t.name = name;
        t.dispatch_method = ESP_TIMER_TASK;
        esp_timer_create(&t, &led.timer);
    }

    led.period_us = period_us;
    led.desired_state = LED_STATE_FLASHING;
    led.current_level = false;

    if (g_leds_enabled)
    {
        esp_timer_start_periodic(led.timer, period_us);
        gpio_set_level(gpio_pin, 0);
    }

    ESP_LOGI(TAG, "LED flashing started on GPIO%d (%llu ms)", gpio_pin, period_ms);
}

extern "C" void led_stop_flashing(gpio_num_t gpio_pin, bool stop_level)
{
    if (!led_map)
        return;

    auto it = led_map->find(gpio_pin);
    if (it == led_map->end())
        return;

    led_data_t &led = it->second;

    if (led.timer)
    {
        esp_timer_stop(led.timer);
    }

    led.desired_state = stop_level ? LED_STATE_ON : LED_STATE_OFF;
    led.current_level = stop_level;
    apply_state(gpio_pin, &led);

    ESP_LOGI(TAG, "LED flashing stopped on GPIO%d → final %s", gpio_pin, stop_level ? "ON" : "OFF");
}

// ==============================================================
// Global control — one function to rule them all
void led_global_set_enabled(bool enabled)
{
    if (g_leds_enabled == enabled)
        return;

    g_leds_enabled = enabled;
    ESP_LOGI(TAG, "All LEDs globally %s", enabled ? "ENABLED" : "DISABLED");

    if (!led_map)
        return;

    for (auto &pair : *led_map)
    {
        gpio_num_t pin = pair.first;
        led_data_t &led = pair.second;

        if (!enabled)
        {
            if (led.timer)
                esp_timer_stop(led.timer);
            gpio_set_level(pin, 0);
        }
        else
        {
            if (led.desired_state == LED_STATE_FLASHING && led.timer)
            {
                esp_timer_start_periodic(led.timer, led.period_us);
                gpio_set_level(pin, 0);
            }
            else
            {
                apply_state(pin, &led);
            }
        }
    }
}

bool led_global_is_enabled(void)
{
    return g_leds_enabled;
}

// ==============================================================
void led_cleanup(gpio_num_t gpio_pin)
{
    if (!led_map)
        return;

    auto it = led_map->find(gpio_pin);
    if (it == led_map->end())
        return;

    led_data_t &led = it->second;

    if (led.timer)
    {
        esp_timer_stop(led.timer);
        esp_timer_delete(led.timer);
    }
    gpio_reset_pin(gpio_pin);
    led_map->erase(it);

    ESP_LOGI(TAG, "LED cleanup complete for GPIO%d", gpio_pin);
}

void led_cleanup_all(void)
{
    if (!led_map)
        return;

    for (auto &pair : *led_map)
    {
        gpio_num_t pin = pair.first;
        led_data_t &led = pair.second;
        if (led.timer)
        {
            esp_timer_stop(led.timer);
            esp_timer_delete(led.timer);
        }
        gpio_reset_pin(pin);
    }

    delete led_map;
    led_map = NULL;
    g_leds_enabled = true;

    ESP_LOGI(TAG, "All LEDs cleaned up");
}

// #ifdef __cplusplus
// }
// #endif