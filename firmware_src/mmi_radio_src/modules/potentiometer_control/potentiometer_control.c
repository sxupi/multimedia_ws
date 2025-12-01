// modules/potentiometer_control/potentiometer_control.c
#include "potentiometer_control.h"

#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"

static const char *TAG = "POT_CTRL";

typedef struct
{
    adc1_channel_t adc_channel;
    QueueHandle_t output_queue;
    uint32_t sample_period_ms;
} potentiometer_task_ctx_t;

static void potentiometer_task(void *pvParameters)
{
    potentiometer_task_ctx_t *ctx = (potentiometer_task_ctx_t *)pvParameters;

    ESP_LOGI(TAG, "Potentiometer task started (channel=%d, period=%u ms)",
             (int)ctx->adc_channel, (unsigned)ctx->sample_period_ms);

    // Configure ADC1 width and attenuation (global for ADC1)
    adc1_config_width(ADC_WIDTH_BIT_12);                          // 0..4095
    adc1_config_channel_atten(ctx->adc_channel, ADC_ATTEN_DB_11); // 0–3.3V approx.

    while (1)
    {
        uint32_t raw = (uint32_t)adc1_get_raw(ctx->adc_channel);

        // Send new sample to queue (overwrite oldest if full)
        if (ctx->output_queue != NULL)
        {
            BaseType_t ok = xQueueOverwrite(ctx->output_queue, &raw);
            if (ok != pdTRUE)
            {
                // Fallback: try normal send with timeout
                xQueueSend(ctx->output_queue, &raw, pdMS_TO_TICKS(10));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ctx->sample_period_ms));
    }

    // We never actually get here
    vTaskDelete(NULL);
}

void potentiometer_control_start(const potentiometer_config_t *config)
{
    if (config == NULL || config->output_queue == NULL)
    {
        ESP_LOGE(TAG, "Invalid potentiometer_config_t");
        return;
    }

    potentiometer_task_ctx_t *ctx =
        (potentiometer_task_ctx_t *)pvPortMalloc(sizeof(potentiometer_task_ctx_t));
    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate potentiometer_task_ctx_t");
        return;
    }

    ctx->adc_channel = config->adc_channel;
    ctx->output_queue = config->output_queue;
    ctx->sample_period_ms = config->sample_period_ms;

    const char *task_name = (config->task_name != NULL) ? config->task_name : "pot_task";

    BaseType_t ret = xTaskCreate(
        potentiometer_task,
        task_name,
        config->task_stack_size,
        ctx,
        config->task_priority,
        NULL);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create potentiometer task");
        vPortFree(ctx);
    }
}
