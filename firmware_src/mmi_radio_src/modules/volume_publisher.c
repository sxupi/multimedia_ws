#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_log.h"

#include "rcchecker.c"

typedef struct
{
    QueueHandle_t    queue;      // queue of uint16_t values from volume_reader
} volume_publisher_task_config_t;

#define VOLUME_PUBLISHER_TAG "VOLUME_PUBLISHER"

// Global node + publisher (must outlive the task)
//static rcl_node_t      volume_node;
static rcl_publisher_t volume_publisher;
static std_msgs__msg__Float32 volume_msg;

/**
 * Initialize the node and publisher.
 * Call this ONCE from appMain() before starting the volume_publisher_task.
 */
void volume_publisher_init(rclc_support_t *support, rcl_allocator_t *allocator, rcl_node_t *node)
{
    (void)allocator;  // not needed in this simple init, but kept for symmetry

    // Moved into appMain() so that I have only one node
    // Create node
    //RCCHECK(rclc_node_init_default(
    //    &volume_node,
    //    "volume_from_potentiometer",
    //    "",
    //    support));

    // Initialize the publisher
    RCCHECK(rclc_publisher_init_default(
        &volume_publisher,
        //&volume_node,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "volume_float32"));

    // Zero-init message
    volume_msg.data = 0.0f;

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Volume publisher initialized");
}

/**
 * Task: wait for ADC values on the queue, convert to float, publish.
 * Expects queue elements of type uint16_t (0..4095).
 */
void volume_publisher_task(void *pvParameters)
{
    volume_publisher_task_config_t *cfg =
        (volume_publisher_task_config_t *)pvParameters;

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Volume publisher task started");

    for (;;)
    {
        uint16_t pot_value = 0;

        // Block until a new value arrives
        if (xQueueReceive(cfg->queue, &pot_value, portMAX_DELAY) == pdTRUE)
        {
            // Map 0..4095 -> 0.0..1.0
            float normalized = (float)pot_value / 4095.0f;
            volume_msg.data = normalized;

            // Publish (soft check so we don't hard-abort on transient errors)
            RCSOFTCHECK(rcl_publish(&volume_publisher, &volume_msg, NULL));

            ESP_LOGI(
                VOLUME_PUBLISHER_TAG,
                "Published volume: raw=%u, normalized=%f",
                pot_value,
                normalized
            );
        }
    }
}

/**
 * Start the volume publisher task.
 * Make sure volume_publisher_init() has been called BEFORE this.
 */
BaseType_t volume_publisher_task_start(
    const char *name,
    uint16_t    stack_size,
    UBaseType_t priority,
    volume_publisher_task_config_t *config)
{
    return xTaskCreate(
        volume_publisher_task,
        name,
        stack_size,
        (void *)config,
        priority,
        NULL);
}
