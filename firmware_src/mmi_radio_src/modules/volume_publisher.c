#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_log.h"

#include "rcchecker.c"

typedef struct
{
    QueueHandle_t queue; // queue of uint16_t values from volume_reader
} volume_publisher_task_config_t;

#define VOLUME_PUBLISHER_TAG "VOLUME_PUBLISHER"

// Global node + publisher (must outlive the task)
// rcl_node_t      volume_pub_node;
rcl_publisher_t volume_publisher;
std_msgs__msg__Float32 volume_pub_msg;

void volume_publisher_init(rclc_support_t *support, rcl_allocator_t *allocator, rcl_node_t *node)
{
    (void)allocator; // not needed in this simple init, but kept for symmetry

    // Moved into appMain() so that I have only one node
    // Create node
    // RCCHECK(rclc_node_init_default(
    //    &volume_pub_node,
    //    "volume_from_potentiometer",
    //    "",
    //    support));

    // Initialize the publisher
    RCCHECK(rclc_publisher_init_default(
        &volume_publisher,
        //&volume_pub_node,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "volume_float32"));

    // Zero-init message
    volume_pub_msg.data = 0.0f;

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Volume publisher initialized");
}

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
            volume_pub_msg.data = normalized;

            // Publish (soft check so we don't hard-abort on transient errors)
            RCSOFTCHECK(rcl_publish(&volume_publisher, &volume_pub_msg, NULL));

            ESP_LOGI(
                VOLUME_PUBLISHER_TAG,
                "Published volume: raw=%u, normalized=%f",
                pot_value,
                normalized);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

BaseType_t volume_publisher_task_start(
    const char *name,
    uint16_t stack_size,
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
