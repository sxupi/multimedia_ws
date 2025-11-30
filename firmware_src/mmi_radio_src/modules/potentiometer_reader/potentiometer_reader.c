#include "potentiometer_reader.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Internal copy of configuration for each task
typedef struct
{
    adc1_channel_t adc_channel;
    QueueHandle_t  output_queue;
    TickType_t     sample_period;
    uint16_t       min_raw;
    uint16_t       max_raw;
    const char    *task_name;
} knob_reader_task_cfg_t;

static int map_raw_to_percent(int raw, uint16_t min_raw, uint16_t max_raw)
{
    if (max_raw <= min_raw) {
        // Avoid division by zero: treat as 0–4095
        min_raw = 0;
        max_raw = 4095;
    }

    if (raw < (int)min_raw) raw = min_raw;
    if (raw > (int)max_raw) raw = max_raw;

    int range = (int)max_raw - (int)min_raw;
    int shifted = raw - (int)min_raw;

    int percent = (shifted * 100) / range;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;
}

static void knob_reader_task(void *pvParameters)
{
    knob_reader_task_cfg_t *cfg = (knob_reader_task_cfg_t *)pvParameters;

    // ADC configuration (12-bit width, 0–3.3V approx with 11 dB attenuation)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(cfg->adc_channel, ADC_ATTEN_DB_11);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int raw = adc1_get_raw(cfg->adc_channel);
        int percent = map_raw_to_percent(raw, cfg->min_raw, cfg->max_raw);

        int32_t value = (int32_t)percent;
        // Use overwrite so the consumer always sees latest value (queue length = 1 recommended)
        xQueueOverwrite(cfg->output_queue, &value);

        vTaskDelayUntil(&last_wake, cfg->sample_period);
    }

    // We should never reach here, but in case:
    vPortFree(cfg);
    vTaskDelete(NULL);
}

BaseType_t knob_reader_start(const knob_reader_config_t *config,
                             TaskHandle_t *out_task_handle)
{
    if (config == NULL || config->output_queue == NULL) {
        return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    knob_reader_task_cfg_t *cfg = pvPortMalloc(sizeof(knob_reader_task_cfg_t));
    if (cfg == NULL) {
        return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    cfg->adc_channel   = config->adc_channel;
    cfg->output_queue  = config->output_queue;
    cfg->sample_period = config->sample_period;
    cfg->min_raw       = config->min_raw;
    cfg->max_raw       = config->max_raw;
    cfg->task_name     = config->task_name ? config->task_name : "knob_reader";

    TaskHandle_t handle = NULL;
    BaseType_t res = xTaskCreate(
        knob_reader_task,
        cfg->task_name,
        config->task_stack_size > 0 ? config->task_stack_size : 2048,
        cfg,
        config->task_priority,
        &handle
    );

    if (res != pdPASS) {
        vPortFree(cfg);
        return res;
    }

    if (out_task_handle != NULL) {
        *out_task_handle = handle;
    }

    return pdPASS;
}