#pragma once
#include "test_framework.h"

/* 菜单调度器: 打印菜单 -> 选择 -> 独立任务运行测试 -> x 中止/自然结束回菜单
 * 框架不认识任何具体测试, 只消费调用方传入的注册表
 */
void menu_run(const TestCase* tests, int count);
