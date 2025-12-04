#include <stdint.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

#include "driver/adc.h"
#include "esp_log.h"

#include "rcchecker.c"
#include "freq_reader.c"

// Static variables
#define FREQ_PUBLISHER_TAG "FREQUENCY_PUBLISHER"
#define FREQ_PUBLISHER_NODE_TAG "mmi_frequency_publisher"
#define FREQ_PUBLISHER_TOPIC "frequency_float32

#define FREQ_PUBLISHER_CHANGE_THRESHOLD 100
#define FREQ_PUBLISHER_TIMER_DELAY_MS 50

// Global micro-ROS variables
rcl_node_t freq_pub_node;
rcl_timer_t freq_pub_timer;
rcl_publisher_t freq_publisher;
std_msgs__msg__Float32 freq_pub_msg;

// Global variables
uint16_t freq_last_raw_value;

void freq_publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL)
    {
        uint16_t current = read_freq_potentiometer_raw();

        uint16_t diff = (current > freq_last_raw_value)
                            ? (current - freq_last_raw_value)
                            : (freq_last_raw_value - current);

        if (diff > FREQ_PUBLISHER_CHANGE_THRESHOLD)
        {
            float normalized = (float)current / FREQ_MAX_RAW_VALUE;
            freq_pub_msg.data = normalized;

            RCSOFTCHECK(rcl_publish(&freq_publisher, &freq_pub_msg, NULL));

            ESP_LOGI(
                FREQ_PUBLISHER_TAG,
                "Published frequency: raw=%u, normalized=%f",
                current,
                normalized);
        }
        freq_last_raw_value = current;
    }
}

void freq_publisher_init(rcl_allocator_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Initializing node %s", FREQ_PUBLISHER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &freq_pub_node,
        FREQ_PUBLISHER_NODE_TAG,
        "",
        support));
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Node (%s) initialized", FREQ_PUBLISHER_NODE_TAG);

    ESP_LOGI(FREQ_PUBLISHER_TAG, "Initializing publisher");
    RCCHECK(rclc_publisher_init_default(
        &freq_publisher,
        &freq_pub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        FREQ_PUBLISHER_TOPIC));
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Publisher initialized");

    ESP_LOGI(FREQ_PUBLISHER_TAG, "Initializing timer with %d ms poll rate", FREQ_PUBLISHER_TIMER_DELAY_MS);
    RCCHECK(rclc_timer_init_default(
        &freq_pub_timer,
        support,
        RCL_MS_TO_NS(FREQ_PUBLISHER_TIMER_DELAY_MS),
        freq_publisher_timer_callback));
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Timer initialized");

    ESP_LOGI(FREQ_PUBLISHER_TAG, "Adding timer to executor");
    RCCHECK(rclc_executor_add_timer(executor, &freq_pub_timer));
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Timer was added to executor");

    ESP_LOGI(FREQ_PUBLISHER_TAG, "Initializing the frequency reader")
    freq_reader_init();
    ESP_LOGI(FREQ_PUBLISHER_TAG, "Frequency reader initialized")

    // Do one read for initializing the reader
    freq_last_raw_value = read_freq_potentiometer_raw();
}

void freq_publisher_cleanup(void)
{
    RCCHECK(rcl_publisher_fini(&freq_publisher, &freq_pub_node));
    RCCHECK(rcl_timer_fini(&freq_pub_timer));
    RCCHECK(rcl_node_fini(&freq_pub_node));
    RCCHECK(std_msgs__msg__Float32__fini(&freq_pub_msg));
}