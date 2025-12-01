// modules/volume_publisher/volume_publisher.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        rcl_publisher_t *publisher; // Must be initialized in app.c
        QueueHandle_t input_queue;  // Receives raw ADC (uint32_t) values
        const char *task_name;
        UBaseType_t task_priority;
        uint32_t task_stack_size; // in words
        uint32_t max_adc_value;   // typically 4095 for 12-bit
        bool log_raw_values;      // if true, logs every received value
    } volume_publisher_config_t;

    /**
     * @brief Start the volume publisher task.
     *
     * This task:
     *  - Waits for ADC values from the input_queue
     *  - Normalizes them to [0.0, 1.0]
     *  - Publishes std_msgs/msg/Float32 on the provided publisher
     *
     * micro-ROS node + publisher must already be initialized.
     */
    void volume_publisher_start(const volume_publisher_config_t *config);

#ifdef __cplusplus
}
#endif
