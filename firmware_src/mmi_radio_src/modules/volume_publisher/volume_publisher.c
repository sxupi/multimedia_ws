#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "esp_log.h"

#include "../rcchecker.c"
#include "volume_reader.c"

// Static variables
#define VOLUME_PUBLISHER_TAG "VOLUME_PUBLISHER"
#define VOLUME_PUBLISHER_NODE_TAG "mmi_volume_publisher"
#define VOLUME_PUBLISHER_TOPIC "potentiometer/volume_float32"

#define VOLUME_PUBLISHER_CHANGE_THRESHOLD 50
#define VOLUME_PUBLISHER_TIMER_DELAY_MS 50

// Global micro-ROS variables
rcl_node_t volume_pub_node;
rcl_publisher_t volume_publisher;
rcl_timer_t volume_pub_timer;
std_msgs__msg__Float32 volume_pub_msg;

// Global variables
uint16_t volume_last_raw_value;

void volume_publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL)
    {
        uint16_t current = read_volume_potentiometer_raw();

        uint16_t diff = (current > volume_last_raw_value)
                            ? (current - volume_last_raw_value)
                            : (volume_last_raw_value - current);

        if (diff >= VOLUME_PUBLISHER_CHANGE_THRESHOLD)
        {
            float normalized = (current - VOLUME_MIN_RAW_VALUE) / (VOLUME_MAX_RAW_VALUE - VOLUME_MIN_RAW_VALUE);
            volume_pub_msg.data = normalized;

            RCSOFTCHECK(rcl_publish(&volume_publisher, &volume_pub_msg, NULL));

            ESP_LOGI(
                VOLUME_PUBLISHER_TAG,
                "Published volume: raw=%u, normalized=%f",
                current,
                normalized);
        }
        volume_last_raw_value = current;
    }
}

void volume_publisher_init(rcl_allocator_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Initializing node %s", VOLUME_PUBLISHER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &volume_pub_node,
        VOLUME_PUBLISHER_NODE_TAG,
        "",
        support));
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Node (%s) initialized", VOLUME_PUBLISHER_NODE_TAG);

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Initializing publisher");
    RCCHECK(rclc_publisher_init_default(
        &volume_publisher,
        &volume_pub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        VOLUME_PUBLISHER_TOPIC));
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Publisher initialized");

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Initializing timer with %d ms poll rate", VOLUME_PUBLISHER_TIMER_DELAY_MS);
    RCCHECK(rclc_timer_init_default(
        &volume_pub_timer,
        support,
        RCL_MS_TO_NS(VOLUME_PUBLISHER_TIMER_DELAY_MS),
        volume_publisher_timer_callback));
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Timer initialized");

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Adding timer to executor");
    RCCHECK(rclc_executor_add_timer(executor, &volume_pub_timer));
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Timer was added to executor");

    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Initializing the volume reader");
    volume_reader_init();
    ESP_LOGI(VOLUME_PUBLISHER_TAG, "Volume reader initialized");

    // Do one read for initializing the reader
    volume_last_raw_value = read_volume_potentiometer_raw();
}

void volume_publisher_cleanup(void)
{
    RCCHECK(rcl_publisher_fini(&volume_publisher, &volume_pub_node));
    RCCHECK(rcl_timer_fini(&volume_pub_timer));
    RCCHECK(rcl_node_fini(&volume_pub_node));
    std_msgs__msg__Float32__fini(&volume_pub_msg);
}