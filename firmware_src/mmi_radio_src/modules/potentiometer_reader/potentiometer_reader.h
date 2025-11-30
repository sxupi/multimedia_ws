#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for a single knob reader instance.
 *
 * You can create multiple knobs (e.g. volume + frequency) by using different
 * adc_channel and output_queue for each instance.
 */
typedef struct
{
    adc1_channel_t adc_channel;     ///< ADC1 channel used for this knob
    QueueHandle_t  output_queue;    ///< Queue that receives int32_t percentage values (0–100)
    TickType_t     sample_period;   ///< Sampling period in FreeRTOS ticks
    uint16_t       min_raw;         ///< Raw ADC value mapped to 0%
    uint16_t       max_raw;         ///< Raw ADC value mapped to 100%
    const char    *task_name;       ///< Optional: task name (for debugging)
    uint32_t       task_stack_size; ///< Stack size in words (not bytes!)
    UBaseType_t    task_priority;   ///< FreeRTOS task priority
} knob_reader_config_t;

/**
 * @brief Start a knob reader task using the given configuration.
 *
 * The function:
 *  - Configures ADC1 width and attenuation
 *  - Spawns a FreeRTOS task that:
 *      * periodically reads the ADC channel
 *      * maps the value to 0–100%
 *      * sends it to config->output_queue (int32_t)
 *
 * @param config Pointer to configuration (will be copied internally).
 * @param out_task_handle Optional: returned task handle (can be NULL).
 * @return pdPASS on success, or an error code from xTaskCreate if task creation fails.
 */
BaseType_t knob_reader_start(const knob_reader_config_t *config,
                             TaskHandle_t *out_task_handle);

#ifdef __cplusplus
}
#endif