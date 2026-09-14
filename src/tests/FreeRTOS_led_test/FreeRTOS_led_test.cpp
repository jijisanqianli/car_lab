#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_framework.h"

static const char* TAG = "FreeRTOS_led";

extern void button_input_task(void *pvParameter);
extern void led_control_task(void *pvParameter);

extern void button_input_teardown(); //清理函数

void FreeRTOS_led_test()
{
    // 创建一个队列用于通信
    QueueHandle_t led_queue = xQueueCreate(4, sizeof(uint32_t));
    // 创建按钮输入任务
    TaskHandle_t button_input_task_t = nullptr;
    xTaskCreate(button_input_task, "button_input_task", 4096, &led_queue, 5, &button_input_task_t);
    // 创建LED控制任务
    TaskHandle_t led_control_task_t = nullptr;
    xTaskCreate(led_control_task, "led_control_task", 4096, &led_queue, 4, &led_control_task_t);
    while (!test_should_abort()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // 清理阶段
    vTaskDelete(button_input_task_t);
    vTaskDelete(led_control_task_t);
    vQueueDelete(led_queue);
    button_input_teardown();
    ESP_LOGI(TAG, "任务结束");
}
