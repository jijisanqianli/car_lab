/* 空白测试模板: 复制本文件, 改函数名, 再到 main.cpp 注册表加一行即可 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_framework.h"

static const char* TAG = "template";

void test_template()
{
    int count = 0;
    while (!test_should_abort()) {
        ESP_LOGI(TAG, "running... %d", count++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
