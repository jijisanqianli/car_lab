#include "console.h"

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"

/**
 * @brief 初始化控制台串口 (UART0, 115200-8N1, 默认引脚)
 * @note  安装带 2048 字节接收缓冲的驱动, 之后才能用 uart_read_bytes 收键
 */
void console_init()
{
    uart_config_t cfg = {};                    // 先清零, 逐字段填配置
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(UART_NUM_0, 2048, 0, 0, nullptr, 0);
    uart_param_config(UART_NUM_0, &cfg);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief 阻塞读取单个字符
 * @note  使用 portMAX_DELAY 使任务进入无限期阻塞状态直到收到数据，
 *        期间不消耗 CPU 资源（让出给其他任务）。
 */
char console_read_key()
{
    uint8_t c = 0;
    // 第四个参数为等待超时时间，portMAX_DELAY 表示永久等待
    uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY);
    return static_cast<char>(c);
}

/**
 * @brief 带超时机制的单字符轮询读取
 * @param out 用于传出读取到的字符
 * @param timeout_ms 超时时间（毫秒）
 * @return true 成功读到字符；false 超时未读到
 */
bool console_poll_key(char& out, int timeout_ms)
{
    uint8_t c = 0;
    // 将毫秒转换为 FreeRTOS 的 Tick 数；若 timeout_ms 为 0 则不等待直接返回
    TickType_t ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : 0;

    if (uart_read_bytes(UART_NUM_0, &c, 1, ticks) == 1) {
        out = static_cast<char>(c);
        return true;
    }
    return false;
}

/**
 * @brief 带回显、退格处理和整行读取的控制台输入函数
 * @param buf 存储输入字符串的缓冲区
 * @param len 缓冲区最大长度
 */
void console_read_line(char* buf, int len)
{
    // 入参合法性检查
    if (buf == nullptr || len <= 0) return;

    int pos = 0; // 当前缓冲区内的字符索引计数器
    while (true) {
        uint8_t c = 0;

        // 阻塞等待串口输入一个字节（安全且不耗 CPU）
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;

        // 1. 检查是否为回车或换行符（用户按下 Enter 键，结束整行输入）
        if (c == '\r' || c == '\n') {
            // 行首的回车/换行不算输入, 丢弃:
            // pio monitor 按回车实际发送 \r\n 两个字节, \r 结束上一行后,
            // 残留的 \n 会在下一次读取时立刻到达, 不跳过会返回空行
            if (pos == 0) continue;
            break;
        }

        // 2. 检查是否为退格键 (Backspace 或 Delete)
        // 0x7F 对应 ASCII 的 DEL 键，'\b' 对应 0x08 回退键
        if (c == 0x7F || c == '\b') {
            if (pos > 0) {
                --pos; // 回退缓冲区指针
                // 终端交互艺术：向串口发送 "退格 -> 空格覆盖 -> 再退格"
                // 这样能真正在终端屏幕上擦除掉上一个已经打印的字符
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        // 3. 过滤掉非标准的 ASCII 可打印字符（范围：0x20 空格 到 0x7E '~'）
        // 防止不可控的控制字符污染缓冲区或导致终端乱码
        if (c < 0x20 || c > 0x7E) continue;

        // 4. 将合法的可打印字符存入缓冲区，并做防溢出保护 (留 1 位给字符串结束符 \0)
        if (pos < len - 1) {
            buf[pos++] = static_cast<char>(c);

            // 串口监控端（如 idf.py monitor）默认通常没有本地回显，
            // 所以我们需要把收到的字符再打印回去，让用户在屏幕上看到自己输了什么
            putchar(c);
            fflush(stdout); // 立即刷新标准输出缓冲区，保证字符实时显示
        }
    }

    // 5. 输入完成，补全字符串终止符
    buf[pos] = '\0';

    // 输入结束，在终端打印一个换行符，开启新的一行
    printf("\n");
    fflush(stdout);
}
