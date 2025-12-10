#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stddef.h>

#define LCD1602_TAG "LCD1602_I2C"

// Change this if your I2C address is different (common ones: 0x27, 0x3F)
#define LCD_I2C_ADDR 0x27

// LCD geometry
#define LCD_COLS 16
#define LCD_ROWS 2

// I2C configuration
static int s_i2c_port = I2C_NUM_0;

// PCF8574 pin bit definitions (may vary slightly by backpack, but this is common)
#define LCD_RS (1 << 0)
#define LCD_RW (1 << 1)
#define LCD_EN (1 << 2)
#define LCD_BACKLIGHT (1 << 3)

// LCD commands
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_DDRAM_ADDR 0x80

// Flags for display entry mode
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_OFF 0x00

// Flags for function set
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

static uint8_t s_backlight = LCD_BACKLIGHT;

static esp_err_t i2c_write_byte(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t lcd_pulse_enable(uint8_t data)
{
    esp_err_t ret;

    // Enable high
    ret = i2c_write_byte(data | LCD_EN);
    if (ret != ESP_OK)
        return ret;
    ets_delay_us(1); // enable pulse must be >450ns

    // Enable low
    ret = i2c_write_byte(data & ~LCD_EN);
    if (ret != ESP_OK)
        return ret;
    ets_delay_us(50); // commands need >37us to settle

    return ESP_OK;
}

static esp_err_t lcd_write4bits(uint8_t value)
{
    uint8_t data = value | s_backlight;
    esp_err_t ret = i2c_write_byte(data);
    if (ret != ESP_OK)
        return ret;

    return lcd_pulse_enable(data);
}

static esp_err_t lcd_send(uint8_t value, uint8_t mode)
{
    // mode = 0 for command, LCD_RS for data
    uint8_t high_nibble = (value & 0xF0) | mode;
    uint8_t low_nibble = ((value << 4) & 0xF0) | mode;

    esp_err_t ret = lcd_write4bits(high_nibble);
    if (ret != ESP_OK)
        return ret;

    ret = lcd_write4bits(low_nibble);
    return ret;
}

static esp_err_t lcd_command(uint8_t cmd)
{
    return lcd_send(cmd, 0x00); // RS = 0
}

static esp_err_t lcd_write(uint8_t data)
{
    return lcd_send(data, LCD_RS); // RS = 1
}

esp_err_t lcd_init(int i2c_port, int sda_gpio, int scl_gpio, uint32_t i2c_freq_hz)
{
    s_i2c_port = i2c_port;

    // I2C config
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = i2c_freq_hz};

    ESP_ERROR_CHECK(i2c_param_config(s_i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(s_i2c_port, conf.mode, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(50)); // wait for LCD power-up

    // Initialize into 4-bit mode (standard sequence)
    // We send 0x30 three times, then 0x20 to set 4-bit interface
    lcd_write4bits(0x30 | s_backlight);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30 | s_backlight);
    ets_delay_us(150);
    lcd_write4bits(0x30 | s_backlight);
    ets_delay_us(150);
    lcd_write4bits(0x20 | s_backlight); // 4-bit mode
    ets_delay_us(150);

    // Function set: 4-bit, 2 lines, 5x8 font
    ESP_ERROR_CHECK(lcd_command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS));

    // Display on, cursor off, blink off
    ESP_ERROR_CHECK(lcd_command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF));

    // Clear
    ESP_ERROR_CHECK(lcd_command(LCD_CLEAR_DISPLAY));
    vTaskDelay(pdMS_TO_TICKS(2));

    // Entry mode: left to right, no shift
    ESP_ERROR_CHECK(lcd_command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT));

    ESP_LOGI(LCD1602_TAG, "LCD 16x2 initialized on I2C port %d", s_i2c_port);
    return ESP_OK;
}

esp_err_t lcd_clear(void)
{
    esp_err_t ret = lcd_command(LCD_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ret;
}

esp_err_t lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= LCD_ROWS)
        row = LCD_ROWS - 1;
    return lcd_command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

esp_err_t lcd_write_char(char c)
{
    return lcd_write((uint8_t)c);
}

esp_err_t lcd_write_string(const char *str)
{
    while (*str)
    {
        esp_err_t ret = lcd_write_char(*str++);
        if (ret != ESP_OK)
            return ret;
    }
    return ESP_OK;
}

esp_err_t lcd_write_line_padded(uint8_t row, const char *text)
{
    esp_err_t ret = lcd_set_cursor(0, row);
    if (ret != ESP_OK)
        return ret;

    // Write up to 16 characters, pad with spaces
    size_t i = 0;
    for (; i < LCD_COLS; ++i)
    {
        char c = text[i];
        if (c == '\0')
            break;
        ret = lcd_write_char(c);
        if (ret != ESP_OK)
            return ret;
    }
    // Pad remaining cells with spaces
    for (; i < LCD_COLS; ++i)
    {
        ret = lcd_write_char(' ');
        if (ret != ESP_OK)
            return ret;
    }
    return ESP_OK;
}
