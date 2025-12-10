#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdbool.h>

#define TAG "SEVENSEG"

// -------- Pin configuration --------

// SECOND 74HC595 (for 7-seg)
#define SEVSEG_SR_DATA GPIO_NUM_16  // SER (DS)
#define SEVSEG_SR_CLK GPIO_NUM_17   // SRCLK (SHCP)
#define SEVSEG_SR_LATCH GPIO_NUM_4 // RCLK  (STCP)

// Digit select pins (common anodes)
#define SEVSEG_DIGIT_0 GPIO_NUM_25 // leftmost
#define SEVSEG_DIGIT_1 GPIO_NUM_26
#define SEVSEG_DIGIT_2 GPIO_NUM_27
#define SEVSEG_DIGIT_3 GPIO_NUM_32 // rightmost

// Refresh timing: 4 digits * 3ms = ~12ms per cycle (~83Hz)
#define SEVSEG_DIGIT_ON_TIME_MS 3
#define SEVSEG_REFRESH_TASK_STACK 2048
#define SEVSEG_REFRESH_TASK_PRIO 4

// ---------- Segment bit mapping on 74HC595 ----------
// Assume Q0..Q7: A,B,C,D,E,F,G,DP
#define SEG_A (1 << 0)
#define SEG_B (1 << 1)
#define SEG_C (1 << 2)
#define SEG_D (1 << 3)
#define SEG_E (1 << 4)
#define SEG_F (1 << 5)
#define SEG_G (1 << 6)
#define SEG_DP (1 << 7)

// Common ANODE assumption:
// We'll define "active-high" patterns (1 = segment lit),
// then invert before shifting (since ON = LOW on the IC output).
static const uint8_t digit_patterns_active_high[10] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         // 0
    SEG_B | SEG_C,                                         // 1
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                 // 2
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                 // 3
    SEG_B | SEG_C | SEG_F | SEG_G,                         // 4
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                 // 5
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,         // 6
    SEG_A | SEG_B | SEG_C,                                 // 7
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 8
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G          // 9
};

// Special value meaning "blank digit"
#define DIGIT_BLANK 10

// Four digits (0-9 or DIGIT_BLANK), index 0 = leftmost
volatile uint8_t s_digits[4] = {DIGIT_BLANK, DIGIT_BLANK, 0, 0};
// Decimal point flags (we'll only use the last digit, but keep generic)
volatile bool s_dp[4] = {false, false, false, true};

void gpio_set_output(gpio_num_t pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void shift_register_init(void)
{
    gpio_set_output(SEVSEG_SR_DATA);
    gpio_set_output(SEVSEG_SR_CLK);
    gpio_set_output(SEVSEG_SR_LATCH);

    gpio_set_level(SEVSEG_SR_DATA, 0);
    gpio_set_level(SEVSEG_SR_CLK, 0);
    gpio_set_level(SEVSEG_SR_LATCH, 0);
}

void digit_pins_init(void)
{
    gpio_set_output(SEVSEG_DIGIT_0);
    gpio_set_output(SEVSEG_DIGIT_1);
    gpio_set_output(SEVSEG_DIGIT_2);
    gpio_set_output(SEVSEG_DIGIT_3);

    // All digits off initially (common anode => LOW off, HIGH on)
    gpio_set_level(SEVSEG_DIGIT_0, 0);
    gpio_set_level(SEVSEG_DIGIT_1, 0);
    gpio_set_level(SEVSEG_DIGIT_2, 0);
    gpio_set_level(SEVSEG_DIGIT_3, 0);
}

// Shift out 8 bits MSB first to the 7-seg 74HC595
void shift_register_write(uint8_t value)
{
    gpio_set_level(SEVSEG_SR_LATCH, 0); // latch low while shifting

    for (int i = 7; i >= 0; i--)
    {
        uint8_t bit = (value >> i) & 0x01;
        gpio_set_level(SEVSEG_SR_DATA, bit);
        gpio_set_level(SEVSEG_SR_CLK, 1);
        gpio_set_level(SEVSEG_SR_CLK, 0);
    }

    gpio_set_level(SEVSEG_SR_LATCH, 1); // latch high to output
}

void digits_all_off(void)
{
    gpio_set_level(SEVSEG_DIGIT_0, 0);
    gpio_set_level(SEVSEG_DIGIT_1, 0);
    gpio_set_level(SEVSEG_DIGIT_2, 0);
    gpio_set_level(SEVSEG_DIGIT_3, 0);
}

void digit_enable(unsigned int idx)
{
    switch (idx)
    {
    case 0:
        gpio_set_level(SEVSEG_DIGIT_0, 1);
        break;
    case 1:
        gpio_set_level(SEVSEG_DIGIT_1, 1);
        break;
    case 2:
        gpio_set_level(SEVSEG_DIGIT_2, 1);
        break;
    case 3:
        gpio_set_level(SEVSEG_DIGIT_3, 1);
        break;
    default:
        break;
    }
}

void sevenseg_refresh_task(void *arg)
{
    // ChatGPT said, that there needs to be a background tasks running, altough I am not a fan of it
    // Maybe I will remove it later on
    ESP_LOGI(TAG, "Refresh task started");

    while (1)
    {
        for (unsigned int i = 0; i < 4; i++)
        {

            // 1) Turn off all digits
            digits_all_off();

            // 2) Select digit value
            uint8_t d = s_digits[i];
            uint8_t pattern_high;

            if (d <= 9)
            {
                pattern_high = digit_patterns_active_high[d];
            }
            else
            {
                // blank digit
                pattern_high = 0;
            }

            if (s_dp[i])
            {
                pattern_high |= SEG_DP;
            }

            // Common anode => ON = LOW, so invert
            uint8_t pattern_to_shift = (uint8_t)(~pattern_high);

            // 3) Update 74HC595 outputs
            shift_register_write(pattern_to_shift);

            // 4) Enable this digit
            digit_enable(i);

            // 5) Hold for a short time
            vTaskDelay(pdMS_TO_TICKS(SEVSEG_DIGIT_ON_TIME_MS));
        }
    }
}

void sevenseg_init(void)
{
    shift_register_init();
    digit_pins_init();
    digits_all_off();

    // start background refresh
    xTaskCreate(
        sevenseg_refresh_task,
        "sevenseg_refresh",
        SEVSEG_REFRESH_TASK_STACK,
        NULL,
        SEVSEG_REFRESH_TASK_PRIO,
        NULL);

    ESP_LOGI(TAG, "Seven-seg initialized");
}

void sevenseg_set_and_show_number_int(int number)
{
    // Clamp range: 0..9999
    if (number < 0)
        number = 0;
    if (number > 9999)
        number = 9999;

    // Extract digits
    int d4 = number % 10;
    int d3 = (number / 10) % 10;
    int d2 = (number / 100) % 10;
    int d1 = (number / 1000) % 10;

    uint8_t digit0, digit1, digit2, digit3;

    if (number < 1000)
    {
        //
        // Format: [blank][d2][d3].[d4]
        //
        digit0 = DIGIT_BLANK; // always blank
        digit1 = (uint8_t)d2; // hundreds (or blank if <100)
        digit2 = (uint8_t)d3; // tens (or blank if <10)
        digit3 = (uint8_t)d4; // ones
    }
    else
    {
        //
        // Format: [d1][d2][d3].[d4]
        //
        digit0 = (uint8_t)d1; // thousands
        digit1 = (uint8_t)d2; // hundreds
        digit2 = (uint8_t)d3; // tens
        digit3 = (uint8_t)d4; // ones
    }

    // Apply to global buffers
    s_digits[0] = digit0;
    s_digits[1] = digit1;
    s_digits[2] = digit2;
    s_digits[3] = digit3;

    // DP always ON on digit index 2
    s_dp[0] = false;
    s_dp[1] = false;
    s_dp[2] = true; // DP ON here
    s_dp[3] = false;
}