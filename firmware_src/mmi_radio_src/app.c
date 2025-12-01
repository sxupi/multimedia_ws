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

#include "modules/potentiometer_control/potentiometer_control.h"
#include "modules/volume_publisher/volume_publisher.h"

static const char *TAG = "APP";

// ---- micro-ROS globals ----
static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rcl_publisher_t g_volume_pub;

// Simple error handling macros
#define RCCHECK(fn)                                                           \
    do                                                                        \
    {                                                                         \
        rcl_ret_t rc = (fn);                                                  \
        if (rc != RCL_RET_OK)                                                 \
        {                                                                     \
            ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int)rc); \
            return;                                                           \
        }                                                                     \
    } while (0)

#define RCSOFTCHECK(fn)                                                          \
    do                                                                           \
    {                                                                            \
        rcl_ret_t rc = (fn);                                                     \
        if (rc != RCL_RET_OK)                                                    \
        {                                                                        \
            ESP_LOGW(TAG, "Soft fail status on line %d: %d", __LINE__, (int)rc); \
        }                                                                        \
    } while (0)

// You can change this pin/channel as needed (ADC1 on ESP32)
#define POT_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34

// Queue length and type for raw ADC readings
#define POT_QUEUE_LENGTH 16

void app_main(void)
{
    ESP_LOGI(TAG, "Starting mmi_mr_multimedia_src app...");

    // 1) Initialize micro-ROS core
    g_allocator = rcl_get_default_allocator();

    // Initialize the support structure (no CLI args)
    RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

    // Create node
    RCCHECK(rclc_node_init_default(
        &g_node,
        "mmi_mr_multimedia_node",
        "",
        &g_support));

    // 2) Initialize volume publisher
    //    Topic: /mmi/volume, Type: std_msgs/msg/Float32
    RCCHECK(rclc_publisher_init_default(
        &g_volume_pub,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/mmi/volume"));

    ESP_LOGI(TAG, "micro-ROS node and volume publisher initialized.");

    // 3) Create queue for ADC → volume pipeline
    QueueHandle_t pot_queue = xQueueCreate(POT_QUEUE_LENGTH, sizeof(uint32_t));
    if (pot_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create pot_queue");
        return;
    }

    // 4) Start potentiometer module (own task)
    potentiometer_config_t pot_cfg = {
        .adc_channel = POT_ADC_CHANNEL,
        .output_queue = pot_queue,
        .sample_period_ms = 50, // sample every 50ms; adjust as needed
        .task_name = "pot_task",
        .task_priority = 5,
        .task_stack_size = 4096};
    potentiometer_control_start(&pot_cfg);
    ESP_LOGI(TAG, "Potentiometer control task started.");

    // 5) Start volume publisher module (own task)
    volume_publisher_config_t vol_cfg = {
        .publisher = &g_volume_pub,
        .input_queue = pot_queue,
        .task_name = "volume_pub_task",
        .task_priority = 6,
        .task_stack_size = 4096,
        .max_adc_value = 4095,  // 12-bit ADC
        .log_raw_values = false // set true for debugging
    };
    volume_publisher_start(&vol_cfg);
    ESP_LOGI(TAG, "Volume publisher task started.");

    // 6) app_main task can gracefully end if nothing else to do
    ESP_LOGI(TAG, "App initialization done, deleting app_main task.");
    vTaskDelete(NULL); // Other tasks keep running
}
