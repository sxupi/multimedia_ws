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
#include "smbus.h"
#include "i2c-lcd1602.h"

#define LCD_SUBSCRIBER_TAG "LCD_SUBSCRIBER"
#define LCD_SUBSCRIBER_NODE_TAG "mmi_lcd_subscriber"
#define LCD_FIRST_SUBSCRIBER_TOPIC "lcd_display/first_string"
#define LCD_SECOND_SUBSCRIBER_TOPIC "lcd_display/second_string"

// ----- I2C + LCD configuration -----
#define LCD_I2C_PORT I2C_NUM_0
#define LCD_SDA_GPIO GPIO_NUM_21
#define LCD_SCL_GPIO GPIO_NUM_22
#define LCD_I2C_ADDR 0x27 // from your i2cdetect on the Pi
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLS 16

// ----- micro-ROS globals -----
rcl_node_t lcd_sub_node;

rcl_subscription_t lcd_first_subscriber;
rcl_subscription_t lcd_second_subscriber;
std_msgs__msg__String lcd_first_sub_msg;
std_msgs__msg__String lcd_second_sub_msg;

// ----- LCD globals -----
static smbus_info_t *lcd_smbus = NULL;
static i2c_lcd1602_info_t *lcd_info = NULL;

// ================= LCD helper functions =================

static void lcd_write_line(uint8_t row, const char *text);

// Initialize I2C + SMBus + LCD driver
static void lcd_init_hw(void)
{
    esp_err_t err;

    // I2C config
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LCD_SDA_GPIO,
        .scl_io_num = LCD_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Configuring I2C for LCD on SDA=%d, SCL=%d",
             LCD_SDA_GPIO, LCD_SCL_GPIO);
    err = i2c_param_config(LCD_I2C_PORT, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "i2c_param_config failed: %d", err);
        return;
    }

    err = i2c_driver_install(LCD_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        // INVALID_STATE means driver already installed; we can ignore that.
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "i2c_driver_install failed: %d", err);
        return;
    }

    // SMBus + LCD structures
    lcd_smbus = smbus_malloc();
    if (!lcd_smbus)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "smbus_malloc failed");
        return;
    }

    err = smbus_init(lcd_smbus, LCD_I2C_PORT, LCD_I2C_ADDR);
    if (err != ESP_OK)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "smbus_init failed: %d", err);
        return;
    }

    lcd_info = i2c_lcd1602_malloc();
    if (!lcd_info)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "i2c_lcd1602_malloc failed");
        return;
    }

    err = i2c_lcd1602_init(
        lcd_info,
        lcd_smbus,
        true, // backlight on
        LCD_NUM_ROWS,
        LCD_NUM_COLS,
        LCD_NUM_COLS // visible columns
    );
    if (err != ESP_OK)
    {
        ESP_LOGE(LCD_SUBSCRIBER_TAG, "i2c_lcd1602_init failed: %d", err);
        return;
    }

    ESP_LOGI(LCD_SUBSCRIBER_TAG, "LCD initialized (addr=0x%02X, %dx%d)",
             LCD_I2C_ADDR, LCD_NUM_COLS, LCD_NUM_ROWS);

    i2c_lcd1602_clear(lcd_info);
}

// Write exactly one line, padded with spaces
static void lcd_write_line(uint8_t row, const char *text)
{
    if (!lcd_info || !text)
        return;
    if (row >= LCD_NUM_ROWS)
        row = LCD_NUM_ROWS - 1;

    char buf[LCD_NUM_COLS + 1];
    size_t len = strlen(text);
    if (len > LCD_NUM_COLS)
        len = LCD_NUM_COLS;

    memset(buf, ' ', LCD_NUM_COLS);
    memcpy(buf, text, len);
    buf[LCD_NUM_COLS] = '\0';

    // Move cursor to start of row and write line
    i2c_lcd1602_move_cursor(lcd_info, 0, row);
    i2c_lcd1602_write_string(lcd_info, buf);
}

// ============== micro-ROS callbacks ======================

void first_subscription_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    const char *txt = msg->data.data ? msg->data.data : "";
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "First line: '%s'", txt);

    lcd_write_line(0, txt); // first row
}

void second_subscription_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    const char *txt = msg->data.data ? msg->data.data : "";
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Second line: '%s'", txt);

    lcd_write_line(1, txt); // second row
}

// ============== micro-ROS init / cleanup =================

void lcd_subscriber_init(rclc_support_t *support, rclc_executor_t *executor)
{
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Initializing node %s", LCD_SUBSCRIBER_NODE_TAG);
    RCCHECK(rclc_node_init_default(
        &lcd_sub_node,
        LCD_SUBSCRIBER_NODE_TAG,
        "",
        support));
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Node (%s) initialized", LCD_SUBSCRIBER_NODE_TAG);

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
    lcd_init_hw();
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "LCD screen initialized");
}

void lcd_subscriber_cleanup(void)
{
    ESP_LOGI(LCD_SUBSCRIBER_TAG, "Cleaning up LCD subscriber");

    RCCHECK(rcl_subscription_fini(&lcd_first_subscriber, &lcd_sub_node));
    RCCHECK(rcl_subscription_fini(&lcd_second_subscriber, &lcd_sub_node));
    RCCHECK(rcl_node_fini(&lcd_sub_node));

    std_msgs__msg__String__fini(&lcd_first_sub_msg);
    std_msgs__msg__String__fini(&lcd_second_sub_msg);

    if (lcd_info)
    {
        i2c_lcd1602_free(&lcd_info);
        lcd_info = NULL;
    }
    if (lcd_smbus)
    {
        smbus_free(&lcd_smbus); // if your smbus lib provides this; if not, omit
        lcd_smbus = NULL;
    }

    // Optionally delete I2C driver if you don't use it elsewhere
    i2c_driver_delete(LCD_I2C_PORT);
}
