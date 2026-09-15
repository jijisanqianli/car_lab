#include <driver/gpio.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct task_arg_t {
    QueueHandle_t queue;
    std::atomic<int> deleted;
};

extern bool test_should_abort();

static QueueHandle_t isr_queue = nullptr;
static TickType_t periods[4] = {500, 200, 100, 1000};
static gpio_num_t button_pin = GPIO_NUM_0;

static void gpio_isr_handler(void *arg) {
    uint32_t gpio_num = reinterpret_cast<uint32_t>(arg);                 //哪个引脚触发
    xQueueOverwriteFromISR(isr_queue, &gpio_num, nullptr);  //从中断里向队列覆盖一个数据
}

static void button_input_teardown() {
    gpio_isr_handler_remove(button_pin);
    if (isr_queue) { vQueueDelete(isr_queue); isr_queue = nullptr; }
}

//专门检测按键输入
void button_input_task(void *pvParameter) {
    // 任务初始化
    // 队列初始化
    task_arg_t* task_args = static_cast<task_arg_t*>(pvParameter);
    QueueHandle_t led_queue = task_args->queue; // 获取led间传输队列
    isr_queue = xQueueCreate(1,sizeof(uint32_t));
    // 按键输入gpio配置
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << button_pin; // 设置位掩码,pin_bit_mask是一个二进制数，哪个引脚被绑定就0变1
    io_conf.mode = GPIO_MODE_INPUT;            // 设置为输入模式
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;   // 设置上拉电阻，此时默认为高电平，按下为低电平
    io_conf.intr_type = GPIO_INTR_POSEDGE;     // 设置为上升沿中断，即按钮抬起，从低电平变为高电平时中断
    gpio_config(&io_conf);                     // 将配置写入寄存器生效
    // 处理中断回调服务
    gpio_install_isr_service(0);   // 安装中断服务
    gpio_isr_handler_add(button_pin, gpio_isr_handler, reinterpret_cast<void *>(button_pin)); // 将中断服务和引脚绑定
    // 任务主体
    uint32_t gpio_num;
    int index = 0;
    while (!test_should_abort()) {
        if (xQueueReceive(isr_queue, &gpio_num, pdMS_TO_TICKS(100))) {
            vTaskDelay(pdMS_TO_TICKS(20));       // 防抖，跳过抖动期
            if (gpio_get_level(button_pin) == 1) { // 不是抖动
                index = index % 4;
                TickType_t period = periods[index++];
                xQueueSend(led_queue,&period,0);
            }
        }
    }
    // 清理阶段
    button_input_teardown();
    task_args->deleted.fetch_add(1);
    vTaskDelete(nullptr);
}
