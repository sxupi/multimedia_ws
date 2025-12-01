// modules/volume_publisher/volume_publisher.c
#include "volume_publisher.h"

#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>

static const char *TAG = "VOL_PUB";

typedef struct
{
    rcl_publisher_t *publisher;
    QueueHandle_t input_queue;
    uint32_t max_adc_value;
    bool log_raw_values;
} volume_publisher_task_ctx_t;

static void volume_publisher_task(void *pvParameters)
{
    volume_publisher_task_ctx_t *ctx =
        (volume_publisher_task_ctx_t *)pvParameters;

    if (ctx->publisher == NULL || ctx->input_queue == NULL)
    {
        ESP_LOGE(TAG, "Invalid volume publisher context");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Volume publisher task started (max_adc=%u)",
             (unsigned)ctx->max_adc_value);

    std_msgs__msg__Float32 msg;
    msg.data = 0.0f;

    while (1)
    {
        uint32_t raw = 0;
        if (xQueueReceive(ctx->input_queue, &raw, portMAX_DELAY))
        {
            if (ctx->max_adc_value == 0)
            {
                ctx->max_adc_value = 4095; // avoid division by zero
            }

            float norm = (float)raw / (float)ctx->max_adc_value;
            if (norm < 0.0f)
                norm = 0.0f;
            if (norm > 1.0f)
                norm = 1.0f;

            msg.data = norm;

            if (ctx->log_raw_values)
            {
                ESP_LOGI(TAG, "ADC raw=%u -> volume=%.3f", (unsigned)raw, (double)norm);
            }

            rcl_ret_t rc = rcl_publish(ctx->publisher, &msg, NULL);
            if (rc != RCL_RET_OK)
            {
                ESP_LOGW(TAG, "rcl_publish failed: %d", (int)rc);
            }
        }
    }

    // Never reached
    vTaskDelete(NULL);
}

void volume_publisher_start(const volume_publisher_config_t *config)
{
    if (config == NULL || config->publisher == NULL || config->input_queue == NULL)
    {
        ESP_LOGE(TAG, "Invalid volume_publisher_config_t");
        return;
    }

    volume_publisher_task_ctx_t *ctx =
        (volume_publisher_task_ctx_t *)pvPortMalloc(sizeof(volume_publisher_task_ctx_t));
    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate volume_publisher_task_ctx_t");
        return;
    }

    ctx->publisher = config->publisher;
    ctx->input_queue = config->input_queue;
    ctx->max_adc_value = config->max_adc_value;
    ctx->log_raw_values = config->log_raw_values;

    const char *task_name = (config->task_name != NULL) ? config->task_name : "volume_pub_task";

    BaseType_t ret = xTaskCreate(
        volume_publisher_task,
        task_name,
        config->task_stack_size,
        ctx,
        config->task_priority,
        NULL);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create volume publisher task");
        vPortFree(ctx);
    }
}
