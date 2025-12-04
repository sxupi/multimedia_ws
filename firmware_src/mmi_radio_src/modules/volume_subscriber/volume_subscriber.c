#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "../rcchecker.c"
#include "volume_led_bar.c"

#define VOLUME_SUBSCRIBER_TAG "VOLUME_SUBSCRIBER"
#define VOLUME_SUBSCRIBER_NODE_TAG "mmi_volume_subscriber"
#define VOLUME_SUBSCRIBER_TOPIC "volume_float32"

// Global micro-ROS variables
rcl_node_t volume_sub_node;
rcl_subscription_t volume_subscriber;
std_msgs__msg__Float32 volume_sub_msg;

void volume_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

    float volume_level = msg->data;

    if (volume_level < 0.0f)
        volume_level = 0.0f;
    if (volume_level > 1.0f)
        volume_level = 1.0f;

    led_bar_set_level(volume_level);
}

void volume_subscriber_init(rclc_support_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Initializing node %s", VOLUME_SUBSCRIBER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &volume_sub_node,
        VOLUME_SUBSCRIBER_NODE_TAG,
        "",
        support));
    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Node (%s) initialized", VOLUME_SUBSCRIBER_NODE_TAG);

    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Initializing subscription");
    RCCHECK(rclc_subscription_init_best_effort(
        &volume_subscriber,
        &volume_sub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        VOLUME_SUBSCRIBER_TOPIC));
    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Subscription initialized");

    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Adding subscriber to executor");
    RCCHECK(rclc_executor_add_subscription(executor,
                                           &volume_subscriber,
                                           &volume_sub_msg,
                                           &volume_subscription_callback,
                                           ON_NEW_DATA));
    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Subscriber was added to executor");

    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "Initializing the LED bar")
    led_bar_init();
    ESP_LOGI(VOLUME_SUBSCRIBER_TAG, "LED bar initialized")
}

void volume_subscriber_cleanup(void)
{
    RCCHECK(rcl_subscription_fini(&volume_subscriber, &volume_sub_node));
    RCCHECK(rcl_node_fini(&volume_sub_node));
    RCCHECK(std_msgs__msg__Float32__fini(&volume_sub_msg));
}