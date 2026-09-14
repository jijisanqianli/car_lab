#pragma once

/* 控制台(UART0)封装: 由菜单调度器独占, 测试项不要使用 */

void console_init();

// 阻塞直到读到按键
char console_read_key();

// 最多等待 timeout_ms, 读到按键返回 true 并写入 out
bool console_poll_key(char& out, int timeout_ms);

// 阻塞读取一行(以回车结束), 带回显, 支持退格; PC 端 getline 的等价物
void console_read_line(char* buf, int len);
