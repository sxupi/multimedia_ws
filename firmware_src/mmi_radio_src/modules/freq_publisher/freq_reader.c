#include <stdint.h>
#include <stdbool.h>

#include "driver/adc.h"
#include "esp_log.h"

#define FREQ_READER_TAG "FREQ_READER"

// ADC config (adjust GPIO/channel if needed)
#define FREQ_READER_ADC_UNIT ADC_UNIT_1
#define FREQ_READER_ADC_CHANNEL ADC1_CHANNEL_7 // GPIO35
#define FREQ_READER_ADC_WIDTH ADC_WIDTH_BIT_12 // 0..4095
#define FREQ_READER_ADC_ATTEN ADC_ATTEN_DB_11  // ~0–3.3V range

// Other static variables
#define FREQ_MIN_RAW_VALUE 0
#define FREQ_MAX_RAW_VALUE 4095

void freq_reader_init(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    // Configure ADC1 width (resolution)
    adc1_config_width(FREQ_READER_ADC_WIDTH);

    // Configure the channel attenuation
    adc1_config_channel_atten(FREQ_READER_ADC_CHANNEL, FREQ_READER_ADC_ATTEN);

    initialized = true;
    ESP_LOGI(FREQ_READER_TAG,
             "ADC initialized: unit=ADC1, channel=%d, width=%d, atten=%d",
             FREQ_READER_ADC_CHANNEL, FREQ_READER_ADC_WIDTH, FREQ_READER_ADC_ATTEN);
}

uint16_t read_freq_potentiometer_raw(void)
{
    // For ADC1, use adc1_get_raw
    int raw = adc1_get_raw(FREQ_READER_ADC_CHANNEL);

    if (raw < FREQ_MIN_RAW_VALUE)
        raw = FREQ_MIN_RAW_VALUE;
    if (raw > FREQ_MAX_RAW_VALUE)
        raw = FREQ_MAX_RAW_VALUE;

    return (uint16_t)raw;
}