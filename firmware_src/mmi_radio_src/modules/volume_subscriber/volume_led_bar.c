// volume_led_bar.c
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *VOLUME_LED_BAR_TAG = "VOLUME_LED_BAR";

// ---- PIN CONFIG (adapt to your wiring) ----
#define SR_DATA_GPIO GPIO_NUM_23 // SER   (pin 14)
#define SR_CLK_GPIO GPIO_NUM_18  // SRCLK (pin 11)
#define SR_LATCH_GPIO GPIO_NUM_5 // RCLK  (pin 12)

// Last 2 LEDs directly on ESP32
#define LED8_GPIO GPIO_NUM_33 // segment index 8 (9th LED)
#define LED9_GPIO GPIO_NUM_19 // segment index 9 (10th LED)

// If you wired bar as common-cathode with anodes driven HIGH, keep 0.
// If you wired it inverted (common-anode), set to 1.
#define LED_BAR_INVERT_OUTPUT 0

static inline void sr_pulse(gpio_num_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}

// Shift out 1 byte MSB first
static void sr_shift_out(uint8_t value)
{
    for (int i = 7; i >= 0; --i)
    {
        int bit = (value >> i) & 0x01;
        gpio_set_level(SR_DATA_GPIO, bit);
        sr_pulse(SR_CLK_GPIO);
    }
}

static void sr_latch(void)
{
    sr_pulse(SR_LATCH_GPIO);
}

void led_bar_set_level(float level)
{
    // clamp
    if (level < 0.0f)
        level = 0.0f;
    if (level > 1.0f)
        level = 1.0f;

    const int total_leds = 10;
    int leds_on = (int)roundf(level * total_leds);
    if (leds_on < 0)
        leds_on = 0;
    if (leds_on > total_leds)
        leds_on = total_leds;

    // first 8 via shift register
    int leds_on_sr = leds_on;
    if (leds_on_sr > 8)
        leds_on_sr = 8;

    uint8_t pattern = 0;
    for (int i = 0; i < leds_on_sr; ++i)
    {
        pattern |= (1u << i); // bit 0 = first (bottom) LED, etc.
    }

#if LED_BAR_INVERT_OUTPUT
    pattern = ~pattern;
#endif

    sr_shift_out(pattern);
    sr_latch();

    // last 2 LEDs directly
    int led8_on = (leds_on >= 9) ? 1 : 0;  // 9th LED
    int led9_on = (leds_on >= 10) ? 1 : 0; // 10th LED

#if LED_BAR_INVERT_OUTPUT
    led8_on = !led8_on;
    led9_on = !led9_on;
#endif

    gpio_set_level(LED8_GPIO, led8_on);
    gpio_set_level(LED9_GPIO, led9_on);
}

void led_bar_init(void)
{
    ESP_LOGI(VOLUME_LED_BAR_TAG, "Init pins");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SR_DATA_GPIO) |
                        (1ULL << SR_CLK_GPIO) |
                        (1ULL << SR_LATCH_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED8_GPIO) |
                        (1ULL << LED9_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&led_conf);

    led_bar_set_level(5.0f);

    ESP_LOGI(VOLUME_LED_BAR_TAG, "LED bar initialized");
}
