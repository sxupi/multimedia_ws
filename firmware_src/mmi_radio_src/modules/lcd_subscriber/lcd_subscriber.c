#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "../rcchecker.c"
#include "./lcd1602_i2c.c" // your LCD driver implementation

#define LCD_SUBSCRIBER_TAG "LCD_SUBSCRIBER"
#define LCD_SUBSCRIBER_NODE_TAG "mmi_lcd_subscriber"
#define LCD_FIRST_SUBSCRIBER_TOPIC "lcd_display/first_string"
#define LCD_SECOND_SUBSCRIBER_TOPIC "lcd_display/second_string"

#define LCD_I2C_PORT I2C_NUM_0
#define LCD_I2C_SDA_GPIO 21
#define LCD_I2C_SCL_GPIO 22
#define LCD_I2C_FREQ_HZ 100000

rcl_node_t lcd_sub_node;

rcl_subscription_t lcd_first_subscriber;
rcl_subscription_t lcd_second_subscriber;
std_msgs__msg__String lcd_first_sub_msg;
std_msgs__msg__String lcd_second_sub_msg;

void first_subscription_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    const char *txt = (msg && msg->data.data) ? msg->data.data : "";
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "First line: '%s'", txt);

    // Write the text to the first row (row 0), padded to 16 chars
    lcd_write_line_padded(0, txt);
}

void second_subscription_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    const char *txt = (msg && msg->data.data) ? msg->data.data : "";
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Second line: '%s'", txt);

    // Write the text to the second row (row 1), padded to 16 chars
    lcd_write_line_padded(1, txt);
}

void lcd_subscriber_init(rclc_support_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Initializing node %s", LCD_SUBSCRIBER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &lcd_sub_node,
        LCD_SUBSCRIBER_NODE_TAG,
        "",
        support));
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Node (%s) initialized", LCD_SUBSCRIBER_NODE_TAG);

    lcd_write_line_padded(1, "Test 12345");

    // Initialize message structs
    std_msgs__msg__String__init(&lcd_first_sub_msg);
    std_msgs__msg__String__init(&lcd_second_sub_msg);

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Initializing first subscription");
    RCCHECK(rclc_subscription_init_default(
        &lcd_first_subscriber,
        &lcd_sub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        LCD_FIRST_SUBSCRIBER_TOPIC));
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "First subscription initialized");

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Initializing second subscription");
    RCCHECK(rclc_subscription_init_default(
        &lcd_second_subscriber,
        &lcd_sub_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        LCD_SECOND_SUBSCRIBER_TOPIC));
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Second subscription initialized");

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Adding subscribers to executor");
    RCCHECK(rclc_executor_add_subscription(
        executor,
        &lcd_first_subscriber,
        &lcd_first_sub_msg,
        &first_subscription_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        executor,
        &lcd_second_subscriber,
        &lcd_second_sub_msg,
        &second_subscription_callback,
        ON_NEW_DATA));
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Subscribers added to executor");

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Initializing the LCD screen");
    // Init your LCD using the driver you provided
    esp_err_t err = lcd_init(
        LCD_I2C_PORT,
        LCD_I2C_SDA_GPIO,
        LCD_I2C_SCL_GPIO,
        LCD_I2C_FREQ_HZ);
    if (err != ESP_OK)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "lcd_init failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(LCD_SUBSCRIBER_TAG, "LCD screen initialized");
        // Optional: show a startup message
        lcd_write_line_padded(0, "micro-ROS ready");
        lcd_write_line_padded(1, "Waiting topics...");
    }
}

void lcd_subscriber_cleanup(void)
{
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Cleaning up LCD subscriber");

    RCCHECK(rcl_subscription_fini(&lcd_first_subscriber, &lcd_sub_node));
    RCCHECK(rcl_subscription_fini(&lcd_second_subscriber, &lcd_sub_node));
    RCCHECK(rcl_node_fini(&lcd_sub_node));

    std_msgs__msg__String__fini(&lcd_first_sub_msg);
    std_msgs__msg__String__fini(&lcd_second_sub_msg);

    // You could also optionally: i2c_driver_delete(LCD_I2C_PORT);
}
