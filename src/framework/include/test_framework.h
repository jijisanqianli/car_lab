#pragma once

/* selftest 测试框架公共接口
 * 约定:
 * - 每个测试项是 void test_xxx(), 运行在独立 FreeRTOS 任务里
 * - 串口由菜单调度器独占, 测试项只用 ESP_LOGx 输出, 不要读串口
 * - 循环中周期性调用 test_should_abort(), 为 true 时尽快清理并返回
 * - 退出前释放外设资源, 保证同一测试项可重复进入
 */

struct TestCase {
    const char* name;    // 菜单里显示的名字
    void (*run)();       // 测试入口
    bool need_confirm;   // 危险项(如电机): 启动前需在串口按 y 确认
};

// 用户已按 x 要求中止当前测试
bool test_should_abort();
