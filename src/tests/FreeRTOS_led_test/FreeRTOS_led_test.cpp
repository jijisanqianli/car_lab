#include <driver/gpio.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "FreeRTOS_led";

constexpr int subtask_num = 2; //子任务数量

extern void button_input_task(void *pvParameter);
extern void led_control_task(void *pvParameter);


struct task_arg_t {
    QueueHandle_t queue;
    std::atomic<int> deleted;
};

void FreeRTOS_led_test()
{
    // 创建一个队列用于通信
    QueueHandle_t led_queue = xQueueCreate(4, sizeof(TickType_t));
    // 构造任务创建参数
    task_arg_t task_args = {};
    task_args.queue = led_queue;
    task_args.deleted = 0;
    // 创建按钮输入任务
    xTaskCreate(button_input_task, "button_input_task", 4096, &task_args, 5, nullptr);
    // 创建LED控制任务
    xTaskCreate(led_control_task, "led_control_task", 4096, &task_args, 4, nullptr);
    // 等待子任务结束
    while (task_args.deleted != subtask_num) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // 清理阶段
    vQueueDelete(led_queue);
    ESP_LOGI(TAG, "任务结束");
}
