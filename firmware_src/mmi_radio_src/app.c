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

// Import from local modules
#include "modules/rcchecker.c"
#include "modules/volume_publisher/volume_publisher.c"
#include "modules/freq_publisher/freq_publisher.c"
#include "modules/volume_subscriber/volume_subscriber.c"
#include "modules/freq_subscriber/freq_subscriber.c"

static const char *MAIN_TAG = "APP";

void appMain(void)
{
    ESP_LOGI(MAIN_TAG, "Starting mmi_radio_src app");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    ESP_LOGI(MAIN_TAG, "Initializing support and allocator");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    ESP_LOGI(MAIN_TAG, "Support and allocator initialized");

    // Maybe I need to delay it afterwards
    // vTaskDelay(pdMS_TO_TICKS(250));
    rclc_executor_t executor;
    ESP_LOGI(MAIN_TAG, "Initializing executor");
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    ESP_LOGI(MAIN_TAG, "Executor initialized");

    // Maybe I need to delay it afterwards
    // vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(MAIN_TAG, "Initializing volume publisher");
    volume_publisher_init(&support, &executor);
    ESP_LOGI(MAIN_TAG, "Volume publisher initialized");

    // Maybe I need to delay it afterwards
    // vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(MAIN_TAG, "Initializing frequency publisher");
    freq_publisher_init(&support, &executor);
    ESP_LOGI(MAIN_TAG, "Volume frequency initialized");

    // Maybe I need to delay it afterwards
    // vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(MAIN_TAG, "Initializing volume subscriber");
    volume_subscriber_init(&support, &executor);
    ESP_LOGI(MAIN_TAG, "Volume subscriber initialized");

    ESP_LOGI(MAIN_TAG, "Initializing frequency subscriber");
    freq_subscriber_init(&support, &executor);
    ESP_LOGI(MAIN_TAG, "Frequency subscriber initialized");

    ESP_LOGI(MAIN_TAG, "Starting executor spin");
    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(MAIN_TAG, "Starting cleanup");
    RCCHECK(rclc_executor_fini(&executor));
    volume_publisher_cleanup();
    freq_publisher_cleanup();
    volume_subscriber_cleanup();
    freq_subscriber_cleanup();
    RCCHECK(rclc_support_fini(&support));
}
