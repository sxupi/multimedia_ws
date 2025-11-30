#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for a volume publisher task.
 *
 * The task waits for int32_t values on input_queue and publishes them
 * via the given rcl_publisher_t and message object.
 */
typedef struct
{
    QueueHandle_t           input_queue;      ///< Queue with int32_t volume values (0–100)
    rcl_publisher_t        *publisher;        ///< Pointer to an initialized rcl_publisher_t
    std_msgs__msg__Int32   *msg;              ///< Pointer to a pre-allocated message
    const char             *task_name;        ///< Optional: task name
    uint32_t                task_stack_size;  ///< Stack size in words
    UBaseType_t             task_priority;    ///< Task priority
} volume_publisher_config_t;

/**
 * @brief Start the volume publisher FreeRTOS task.
 *
 * The task:
 *  - blocks on input_queue
 *  - for each received value, sets msg->data and calls rcl_publish
 */
BaseType_t volume_publisher_start(const volume_publisher_config_t *config,
                                  TaskHandle_t *out_task_handle);

#ifdef __cplusplus
}
#endif