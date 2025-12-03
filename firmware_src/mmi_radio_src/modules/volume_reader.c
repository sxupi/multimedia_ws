#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_log.h"

typedef struct
{
    QueueHandle_t queue;       // Queue to send samples to (uint16_t)
    uint16_t change_threshold; // Minimum delta to trigger a send
    TickType_t poll_delay;     // Delay between reads (in ticks)
} volume_reader_task_config_t;

#define VOLUME_READER_TAG "VOLUME_READER"

// ADC config (adjust GPIO/channel if needed)
// Example: GPIO34 is ADC1_CHANNEL_6 on classic ESP32
#define VOLUME_READER_ADC_UNIT ADC_UNIT_1
#define VOLUME_READER_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34
#define VOLUME_READER_ADC_WIDTH ADC_WIDTH_BIT_12 // 0..4095
#define VOLUME_READER_ADC_ATTEN ADC_ATTEN_DB_11  // ~0–3.3V range

void volume_reader_init(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    // Configure ADC1 width (resolution)
    adc1_config_width(VOLUME_READER_ADC_WIDTH);

    // Configure the channel attenuation
    adc1_config_channel_atten(VOLUME_READER_ADC_CHANNEL, VOLUME_READER_ADC_ATTEN);

    initialized = true;
    ESP_LOGI(VOLUME_READER_TAG,
             "ADC initialized: unit=ADC1, channel=%d, width=%d, atten=%d",
             VOLUME_READER_ADC_CHANNEL, VOLUME_READER_ADC_WIDTH, VOLUME_READER_ADC_ATTEN);
}

uint16_t read_volume_potentiometer_raw(void)
{
    // For ADC1, use adc1_get_raw
    int raw = adc1_get_raw(VOLUME_READER_ADC_CHANNEL);
    if (raw < 0)
    {
        raw = 0;
    }
    if (raw > 4095)
    {
        raw = 4095;
    }

    // Remove or comment out this log if it spams too much
    // ESP_LOGI(VOLUME_READER_TAG, "Read raw ADC value: %d", raw);

    return (uint16_t)raw;
}

void volume_reader_task(void *pvParameters)
{
    volume_reader_task_config_t *cfg = (volume_reader_task_config_t *)pvParameters;

    volume_reader_init();

    uint16_t last_value = read_volume_potentiometer_raw();

    // Optionally send the initial value once at startup
    //(void)xQueueSend(cfg->queue, &last_value, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        vTaskDelay(cfg->poll_delay);

        uint16_t current = read_volume_potentiometer_raw();

        uint16_t diff = (current > last_value)
                            ? (current - last_value)
                            : (last_value - current);

        if (diff >= cfg->change_threshold)
        {
            last_value = current;
            // Send new value (non-blocking; change 0 to a timeout if desired)
            (void)xQueueSend(cfg->queue, &current, 0);
        }
    }
}

BaseType_t volume_reader_task_start(
    const char *name,
    uint16_t stack_size,
    UBaseType_t priority,
    volume_reader_task_config_t *config)
{
    return xTaskCreate(
        volume_reader_task,
        name,
        stack_size,
        (void *)config,
        priority,
        NULL);
}
