#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "rcchecker.c"

typedef struct
{
    QueueHandle_t queue;
} volume_subscriber_config_t;

#define VOLUME_SUBSCRIBER_TAG "VOLUME_SUBSCRIBER"

rcl_subscription_t volume_subscriber;
std_msgs__msg__Float32 volume_sub_msg;

volume_subscriber_config_t *volume_sub_cfg = NULL;

void volume_subscription_callback(const void *msgin)
{
    if (volume_sub_cfg == NULL || volume_sub_cfg->queue == NULL)
    {
        ESP_LOGE(VOLUME_SUBSCRIBER_TAG, "No queue configured, dropping value");
    }
    else
    {
        const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

        float volume_level = msg->data;

        if (volume_level < 0.0f)
            volume_level = 0.0f;
        if (volume_level > 1.0f)
            volume_level = 1.0f;

        (void)xQueueSend(volume_sub_cfg->queue, &volume_level, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

void volume_subscriber_set_config(volume_subscriber_config_t *cfg)
{
    volume_sub_cfg = cfg;
}

void volume_subscriber_init(rclc_support_t *support, rcl_allocator_t *allocator, rcl_node_t *node, rclc_executor_t *executor)
{
    (void)allocator; // not needed in this simple init, but kept for symmetry

    // Moved into appMain() so that I have only one node
    // Create node
    // RCCHECK(rclc_node_init_default(
    //    &volume_sub_node,
    //    "volume_converter",
    //    "",
    //    support));

    // Initialize the publisher
    RCCHECK(rclc_subscription_init_best_effort(
        &volume_subscriber,
        //&volume_sub_node,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "volume_float32"));

    RCCHECK(rclc_executor_add_subscription(executor, &volume_subscriber, &volume_sub_msg,
                                           &volume_subscription_callback, ON_NEW_DATA));

    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Volume subscriber initialized");
}