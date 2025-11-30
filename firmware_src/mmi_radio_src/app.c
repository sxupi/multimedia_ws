#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// micro-ROS / ROS 2
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

// ADC channels etc. are configured in the potentiometer module
#include "modules/potentiometer_reader/potentiometer_reader.h"
#include "modules/volume_publisher/volume_publisher.h"

// IMPORTANT: in this micro-ROS FreeRTOS setup, only app.c is compiled.
// We pull in module implementations here so they become part of this TU.
#include "modules/potentiometer_reader/potentiometer_reader.c"
#include "modules/volume_publisher/volume_publisher.c"

// Provided by the micro-ROS platform layer for your board/transport
// (UART, Wi-Fi, etc.). This is defined in the microros_esp32_extensions.
void micro_ros_platform_configure(void);

// =======================
// Global handles
// =======================

// ROS entities
static rcl_allocator_t g_allocator;
static rclc_support_t  g_support;
static rcl_node_t      g_node;

// Publishers + messages
static rcl_publisher_t g_volume_pub;
static std_msgs__msg__Int32 g_volume_msg;

static rcl_publisher_t g_frequency_pub;
static std_msgs__msg__Int32 g_frequency_msg;

// Queues (potentiometer → publisher)
static QueueHandle_t g_volume_queue;
static QueueHandle_t g_frequency_queue;

// =======================
// Helper: abort on RCL error
// =======================
static void check_ret(rcl_ret_t rc, const char *what)
{
    if (rc != RCL_RET_OK) {
        printf("Error in %s: %d\n", what, (int)rc);
        // In a real app you might try to recover; for now we just stop.
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// =======================
// Application entry point
// =======================
//
// This is called by the micro-ROS FreeRTOS integration as the main task.
void appMain(void *arg)
{
    (void)arg;

    // 1) Configure transport (serial/Wi-Fi) for micro-ROS
    micro_ros_platform_configure();

    // 2) Initialize micro-ROS support and node
    g_allocator = rcl_get_default_allocator();
    check_ret(
        rclc_support_init(&g_support, 0, NULL, &g_allocator),
        "rclc_support_init"
    );

    check_ret(
        rclc_node_init_default(
            &g_node,
            "mmi_radio_esp32_node",
            "",
            &g_support),
        "rclc_node_init_default"
    );

    // 3) Create publishers
    // Volume topic
    check_ret(
        rclc_publisher_init_default(
            &g_volume_pub,
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/volume_input_from_knob"),
        "rclc_publisher_init_default(volume)"
    );

    // Frequency topic
    check_ret(
        rclc_publisher_init_default(
            &g_frequency_pub,
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/frequency_input_from_knob"),
        "rclc_publisher_init_default(frequency)"
    );

    // 4) Create queues
    //
    // Queue length = 1 because we only care about the latest value;
    // potentiometer_reader uses xQueueOverwrite().
    g_volume_queue = xQueueCreate(1, sizeof(int32_t));
    g_frequency_queue = xQueueCreate(1, sizeof(int32_t));

    if (g_volume_queue == NULL || g_frequency_queue == NULL) {
        printf("Failed to create queues.\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // 5) Start potentiometer readers
    //
    // These are reusable: you can configure different ADC channels and queues.
    // Adjust ADC1_CHANNEL_X to match your wiring.

    // --- Volume potentiometer ---
    potentiometer_reader_config_t vol_pot_cfg = {
        .adc_channel     = ADC1_CHANNEL_0,          // e.g., GPIO1
        .output_queue    = g_volume_queue,
        .sample_period   = pdMS_TO_TICKS(50),       // 20 Hz
        .min_raw         = 0,
        .max_raw         = 4095,
        .task_name       = "pot_volume",
        .task_stack_size = 2048,                    // words, not bytes
        .task_priority   = 5
    };
    check_ret(
        knob_reader_start(&vol_pot_cfg, NULL) == pdPASS ? RCL_RET_OK : RCL_RET_ERROR,
        "potentiometer_reader_start(volume)"
    );

    // --- Frequency potentiometer ---
    potentiometer_reader_config_t freq_pot_cfg = {
        .adc_channel     = ADC1_CHANNEL_3,          // e.g., GPIO4 (adjust to your board)
        .output_queue    = g_frequency_queue,
        .sample_period   = pdMS_TO_TICKS(50),
        .min_raw         = 0,
        .max_raw         = 4095,
        .task_name       = "pot_frequency",
        .task_stack_size = 2048,
        .task_priority   = 5
    };
    check_ret(
        knob_reader_start(&freq_pot_cfg, NULL) == pdPASS ? RCL_RET_OK : RCL_RET_ERROR,
        "potentiometer_reader_start(freq)"
    );

    // 6) Start publisher tasks
    //
    // volume_publisher is generic "queue → Int32 publisher", so we can reuse
    // it for both volume and frequency.

    // --- Volume publisher task ---
    volume_publisher_config_t vol_pub_cfg = {
        .input_queue     = g_volume_queue,
        .publisher       = &g_volume_pub,
        .msg             = &g_volume_msg,
        .task_name       = "volume_publisher",
        .task_stack_size = 4096,
        .task_priority   = 5
    };
    check_ret(
        volume_publisher_start(&vol_pub_cfg, NULL) == pdPASS ? RCL_RET_OK : RCL_RET_ERROR,
        "volume_publisher_start(volume)"
    );

    // --- Frequency publisher task ---
    volume_publisher_config_t freq_pub_cfg = {
        .input_queue     = g_frequency_queue,
        .publisher       = &g_frequency_pub,
        .msg             = &g_frequency_msg,
        .task_name       = "frequency_publisher",
        .task_stack_size = 4096,
        .task_priority   = 5
    };
    check_ret(
        volume_publisher_start(&freq_pub_cfg, NULL) == pdPASS ? RCL_RET_OK : RCL_RET_ERROR,
        "volume_publisher_start(freq)"
    );

    printf("mmi_radio_esp32_node started: publishing volume + frequency.\n");

    // 7) Main loop
    //
    // We have no timers/subscriptions here, just publishers in their own
    // FreeRTOS tasks, so we don't need an rclc_executor.
    // We just keep this task alive.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // (Normally never reached)
    // Cleanup would go here if you ever broke out of the loop.
}