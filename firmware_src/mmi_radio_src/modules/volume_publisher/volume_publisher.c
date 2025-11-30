#include "volume_publisher.h"
#include <stdio.h>

static void volume_publisher_task(void *pvParameters)
{
    volume_publisher_config_t cfg = *(volume_publisher_config_t *)pvParameters;
    vPortFree(pvParameters); // free copied config

    int32_t value = 0;

    for (;;) {
        if (xQueueReceive(cfg.input_queue, &value, portMAX_DELAY) == pdTRUE) {
            cfg.msg->data = value;
            rcl_ret_t rc = rcl_publish(cfg.publisher, cfg.msg, NULL);

            if (rc != RCL_RET_OK) {
                printf("volume_publisher: rcl_publish error %d\n", (int)rc);
            }
        }
    }

    vTaskDelete(NULL);
}

BaseType_t volume_publisher_start(const volume_publisher_config_t *config,
                                  TaskHandle_t *out_task_handle)
{
    if (config == NULL || config->input_queue == NULL ||
        config->publisher == NULL || config->msg == NULL) {
        return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    volume_publisher_config_t *cfg_copy =
        pvPortMalloc(sizeof(volume_publisher_config_t));
    if (cfg_copy == NULL) {
        return errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    }

    *cfg_copy = *config; // shallow copy is enough here

    const char *name = config->task_name ? config->task_name : "volume_publisher";

    TaskHandle_t handle = NULL;
    BaseType_t res = xTaskCreate(
        volume_publisher_task,
        name,
        config->task_stack_size > 0 ? config->task_stack_size : 4096,
        cfg_copy,
        config->task_priority,
        &handle
    );

    if (res != pdPASS) {
        vPortFree(cfg_copy);
        return res;
    }

    if (out_task_handle != NULL) {
        *out_task_handle = handle;
    }

    return pdPASS;
}