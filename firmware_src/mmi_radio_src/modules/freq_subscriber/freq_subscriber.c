#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "../rcchecker.c"
#include "freq_sevseg.c"

#define FREQ_SUBSCRIBER_TAG "FREQUENCY_SUBSCRIBER"
#define FREQ_SUBSCRIBER_NODE_TAG "mmi_freq_subscriber"
#define FREQ_SUBSCRIBER_TOPIC "frequency_float32"

// Global micro-ROS variables
rcl_node_t freq_sub_node;
rcl_subscription_t freq_subscriber;
std_msgs__msg__Float32 freq_sub_msg;

void freq_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

    float frequency = msg->data;

    if (frequency < 0.0f)
        frequency = 0.0f;
    if (frequency > 1000.0f)
        frequency = 999.0f;

    sevenseg_set_and_show_number(frequency);
}

void freq_subscriber_init(rclc_support_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Initializing node %s", FREQ_SUBSCRIBER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &freq_sub_node,
        FREQ_SUBSCRIBER_NODE_TAG,
        "",
        support));
    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Node (%s) initialized", FREQ_SUBSCRIBER_NODE_TAG);

    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Initializing subscription");
    RCCHECK(rclc_subscription_init_best_effort(
        &freq_subscriber,
        &freq_sub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        FREQ_SUBSCRIBER_TOPIC));
    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Subscription initialized");

    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Adding subscriber to executor");
    RCCHECK(rclc_executor_add_subscription(executor,
                                           &freq_subscriber,
                                           &freq_sub_msg,
                                           &freq_subscription_callback,
                                           ON_NEW_DATA));
    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Subscriber was added to executor");

    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "Initializing the 7 segment LED display")
    sevenseg_init();
    ESP_LOGI(FREQ_SUBSCRIBER_TAG, "7 segment LED display initialized")
}

void freq_subscriber_cleanup(void)
{
    RCCHECK(rcl_subscription_fini(&freq_subscriber, &freq_sub_node));
    RCCHECK(rcl_node_fini(&freq_sub_node));
    RCCHECK(std_msgs__msg__Float32__fini(&freq_sub_msg));
}