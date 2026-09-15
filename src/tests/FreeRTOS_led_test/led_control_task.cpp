#include <driver/gpio.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct task_arg_t {
    QueueHandle_t queue;
    std::atomic<int> deleted;
};

extern bool test_should_abort();

static gpio_num_t led_pin = GPIO_NUM_13;

void led_control_task(void *pvParameter) {
    // 任务初始化
    task_arg_t* task_args = static_cast<task_arg_t*>(pvParameter);
    QueueHandle_t led_queue = task_args->queue; // 获取led间传输队列
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << led_pin; // 设置位掩码,pin_bit_mask是一个二进制数，哪个引脚被绑定就0变1
    io_conf.mode = GPIO_MODE_OUTPUT;            // 设置为输出模式
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;   // 输出模式下关闭上拉
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 关闭下拉
    io_conf.intr_type = GPIO_INTR_DISABLE;     // 禁用中断
    gpio_config(&io_conf);                     // 将配置写入寄存器生效

    TickType_t currentPeriod = 1000;
    TickType_t newPeriod = 0;
    while (!test_should_abort()) {
        gpio_set_level(led_pin, 1);
        if (xQueueReceive(led_queue, &newPeriod, pdMS_TO_TICKS(currentPeriod/2))) {
            currentPeriod = newPeriod;
            continue;
        }
        gpio_set_level(led_pin, 0);
        if (xQueueReceive(led_queue, &newPeriod, pdMS_TO_TICKS(currentPeriod/2))) {
            currentPeriod = newPeriod;
            continue;
        }
    }
    // 清理阶段
    gpio_set_level(led_pin,0);
    task_args->deleted.fetch_add(1);
    vTaskDelete(nullptr);
}

