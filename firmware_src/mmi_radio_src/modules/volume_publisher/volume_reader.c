#include <stdint.h>
#include <stdbool.h>

#include "driver/adc.h"
#include "esp_log.h"

#define VOLUME_READER_TAG "VOLUME_READER"

// ADC config (adjust GPIO/channel if needed)
// Example: GPIO34 is ADC1_CHANNEL_6 on classic ESP32
#define VOLUME_READER_ADC_UNIT ADC_UNIT_1
#define VOLUME_READER_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34
#define VOLUME_READER_ADC_WIDTH ADC_WIDTH_BIT_12 // 0..4095
#define VOLUME_READER_ADC_ATTEN ADC_ATTEN_DB_11  // ~0–3.3V range

// Other static variables
#define VOLUME_MIN_RAW_VALUE 0.0f
#define VOLUME_MAX_RAW_VALUE 3300.0f

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

    if (raw < VOLUME_MIN_RAW_VALUE)
        raw = VOLUME_MIN_RAW_VALUE;

    if (raw > VOLUME_MAX_RAW_VALUE)
        raw = VOLUME_MAX_RAW_VALUE;

    return (uint16_t)raw;
}