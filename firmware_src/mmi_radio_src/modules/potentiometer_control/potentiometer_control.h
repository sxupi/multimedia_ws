// modules/potentiometer_control/potentiometer_control.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        adc1_channel_t adc_channel; // ADC1 channel for the potentiometer
        QueueHandle_t output_queue; // Queue to send raw ADC readings (uint32_t)
        uint32_t sample_period_ms;
        const char *task_name;
        UBaseType_t task_priority;
        uint32_t task_stack_size; // in words (not bytes)
    } potentiometer_config_t;

    /**
     * @brief Start a potentiometer sampling task.
     *
     * This function:
     *  - Configures the given ADC1 channel
     *  - Creates a FreeRTOS task which periodically reads adc1_get_raw(...)
     *    and pushes the raw value (uint32_t) into the output_queue.
     *
     * You can call this multiple times with different configs to handle
     * different potentiometers (each gets its own task).
     */
    void potentiometer_control_start(const potentiometer_config_t *config);

#ifdef __cplusplus
}
#endif
