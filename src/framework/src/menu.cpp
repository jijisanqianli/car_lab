#include "menu.h"
#include "framework_internal.h"
#include "console.h"

#include <cstdio>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "menu";

static void print_menu(const TestCase* tests, int count)
{
    printf("\n===== selftest 菜单 =====\n");
    for (int i = 0; i < count; ++i) {
        printf("  %d. %s%s\n", i + 1, tests[i].name,
               tests[i].need_confirm ? "  [需确认]" : "");
    }
    printf("选择编号开始测试; 测试运行中按 x 返回本菜单\n选择: ");
    fflush(stdout);
}

// 等待测试结束, 或用户按 x 主动中止 (给 3s 宽限, 超时强杀)
static void supervise_test(TaskHandle_t task)
{
    char c;
    while (true) {
        if (test_flags_done()) {
            ESP_LOGI(TAG, "测试结束, 返回菜单");
            return;
        }
        // 读取单字符
        if (console_poll_key(c, 50) && c == 'x') {
            // 设置终止标志，具体退出由测试任务自己控制
            test_flags_set_abort();
            printf("\n[x] 等待测试退出...\n");
            int waited_ms = 0;
            while (!test_flags_done() && waited_ms < 3000) {
                vTaskDelay(pdMS_TO_TICKS(50));
                waited_ms += 50;
            }
            if (!test_flags_done()) {
                vTaskDelete(task);
                ESP_LOGW(TAG, "测试未响应中止, 已强制终止(外设状态可能残留)");
            }
            return;
        }
    }
}

// 所有测试的统一跳板: xTaskCreate 要求 void f(void*) 签名, 这里桥接到 TestCase::run
static void test_trampoline(void* arg)
{
    const TestCase* tc = static_cast<const TestCase*>(arg);
    tc->run();
    test_flags_set_done();
    vTaskDelete(nullptr);
}

void menu_run(const TestCase* tests, int count)
{
    console_init();
    printf("\nselftest 固件就绪, 共 %d 个测试项\n", count);

    while (true) {
        print_menu(tests, count);
        char line[16];                       // 行式输入, 支持任意位数的编号
        console_read_line(line, sizeof(line));
        int idx = atoi(line) - 1;
        if (line[0] == '\0' || idx < 0 || idx >= count) {
            printf("无效选择\n");
            continue;
        }
        const TestCase& tc = tests[idx];
        if (tc.need_confirm) {
            printf("该项有风险(可能使小车运动). 确认安全后按 y 开始, 其他键取消: ");
            fflush(stdout);
            if (console_read_key() != 'y') {
                printf("已取消\n");
                continue;
            }
            printf("\n");
        }
        printf(">>> 运行 [%s], 按 x 返回菜单 <<<\n", tc.name);
        // 重置标记
        test_flags_reset();
        TaskHandle_t task = nullptr;
        // 创建任务
        if (xTaskCreate(test_trampoline, "test", 4096,
                        const_cast<TestCase*>(&tc), 5, &task) != pdTRUE) {
            printf("任务创建失败\n");
            continue;
        }
        supervise_test(task);
    }
}
