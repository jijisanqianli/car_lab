#pragma once
/* 框架内部状态接口: 仅框架实现文件(framework/src/)使用, 测试项不要 include */
void test_flags_reset();
void test_flags_set_abort();
void test_flags_set_done();
bool test_flags_done();
