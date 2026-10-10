#pragma once

/* 电机测试项共用工具
 * 供 motor_case_test.cpp / motor_hold_test.cpp 使用;
 * 写法与 motor_test.cpp 的 delay_chunked 保持一致(该文件保持原样不动)。
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "test_framework.h"

// 分片延时: 每 100ms 醒一次查 abort(100Hz tick 下 100ms=10tick, 粒度安全)
// 返回 false 表示等待期间被要求中止, 调用方应立即走清理退出
static inline bool delay_chunked(int ms)
{
    for (int elapsed = 0; elapsed < ms; elapsed += 100) {
        if (test_should_abort()) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return !test_should_abort();
}
