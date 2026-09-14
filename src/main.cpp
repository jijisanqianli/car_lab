/* selftest 入口: 只负责登记测试项.
 * 框架实现在 framework/, 测试实现在 tests/, 本文件不含任何机制代码.
 * 新增测试项: tests/ 下写 void test_xxx() -> 下面 extern 声明 + 表里加一行
 */
#include "test_framework.h"
#include "menu.h"

extern void test_template();
extern void test_led();

static const TestCase kTests[] = {
    {"模板测试(计数心跳)", test_template, false},
    {"LED 心跳(WS2812)",   test_led,      false},
    // {"电机空载", test_motor, true},  // 危险项示例: need_confirm=true, 启动前需按 y
};
static constexpr int kTestCount = sizeof(kTests) / sizeof(kTests[0]);

extern "C" void app_main(void)
{
    menu_run(kTests, kTestCount);
}
