// app.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// micro-ROS / rclc
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <rcl/error_handling.h>

#include "modules/rcchecker.c"

#include "modules/volume_reader.c"
#include "modules/volume_publisher.c"

#include "modules/freq_reader.c"
#include "modules/freq_publisher.c"

#include "modules/volume_led_bar.c"
#include "modules/volume_subscriber.c"

static const char *MAIN_TAG = "APP";

void appMain(void)
{
    ESP_LOGI(MAIN_TAG, "Starting mmi_mr_multimedia_src app...");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    ESP_LOGI(MAIN_TAG, "Initializing support and allocator...");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    ESP_LOGI(MAIN_TAG, "Support and allocator initialized...");

    ESP_LOGI(MAIN_TAG, "Initializing node (mmi_esp32_node)...");
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(
        &node,
        "mmi_esp32_node",
        "",
        &support));
    ESP_LOGI(MAIN_TAG, "Node (mmi_esp32_node) initialized...");

    ESP_LOGI(MAIN_TAG, "Initializing executor...");
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    ESP_LOGI(MAIN_TAG, "Executor initialized...");

    // Initialize the volume publisher
    volume_publisher_init(&support, &allocator, &node);
    // Initialize the frequency publisher
    freq_publisher_init(&support, &allocator, &node);

    // -------------------------------------------------------------------------------
    // Volume potentiometer reader and publisher
    // -------------------------------------------------------------------------------
    // Initialize the volume (raw values) queue
    QueueHandle_t volume_pub_queue = xQueueCreate(10, sizeof(uint16_t));
    // Start the volume (potentiometer) reader task
    volume_reader_task_config_t volume_reader_cfg = {
        .queue = volume_pub_queue,
        .change_threshold = 100,
        .poll_delay = pdMS_TO_TICKS(50)};
    volume_reader_task_start("volume_reader", 2048, 4, &volume_reader_cfg);

    // Start the volume publisher task to publish the values from potentiometer
    volume_publisher_task_config_t volume_publisher_cfg = {
        .queue = volume_pub_queue};
    volume_publisher_task_start("volume_publisher", 4096, 4, &volume_publisher_cfg);

    // -------------------------------------------------------------------------------
    // Frequency potentiometer reader and publisher
    // -------------------------------------------------------------------------------
    // Initialize the frequency (raw values) queue
    QueueHandle_t freq_queue = xQueueCreate(10, sizeof(uint16_t));

    // Start the frequency (potentiometer) reader task
    freq_reader_task_config_t freq_reader_cfg = {
        .queue = freq_queue,
        .change_threshold = 50,
        .poll_delay = pdMS_TO_TICKS(50)};
    freq_reader_task_start("freq_reader", 2048, 4, &freq_reader_cfg);

    // Start the frequency publisher task to publish the values from potentiometer
    freq_publisher_task_config_t freq_publisher_cfg = {
        .queue = freq_queue};
    freq_publisher_task_start("freq_publisher", 4096, 4, &freq_publisher_cfg);

    // -------------------------------------------------------------------------------
    // Setting up volume subscriber and volume LED bar
    // -------------------------------------------------------------------------------
    QueueHandle_t volume_sub_queue = xQueueCreate(5, sizeof(float));
    // Start the volume subscriber
    volume_subscriber_config_t volume_subscriber_cfg = {
        .queue = volume_sub_queue};
    volume_subscriber_set_config(&volume_subscriber_cfg);
    volume_subscriber_init(&support, &allocator, &node, &executor);

    // Setting up the LED bar
    volume_led_bar_task_config_t volume_led_bar_cfg = {
        .queue = volume_sub_queue};
    led_bar_init();
    volume_led_bar_task_start("volume_led_bar", 4096, 4, &volume_led_bar_cfg);

    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Free the resources
    // TODO: Free the other tasks and also publisher/subscriber
    RCCHECK(rcl_node_fini(&node));
}
