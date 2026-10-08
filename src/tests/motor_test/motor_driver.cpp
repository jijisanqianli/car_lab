 /* ============================================================================
 * TB6612FNG 电机驱动实现（第2课 · 靶标B 审计交付物）
 * ==========================================================================
 * 【TB6612 输出状态速查（真值表浓缩, STBY=1 前提下）】
 *   IN1 IN2 PWM     → 输出状态
 *   H   L   调制    → 正转(驱动)          ← motor_set(+duty)
 *   L   H   调制    → 反转(驱动)          ← motor_set(-duty)
 *   H   L   恒低    → 短接制动            ← motor_set(0) 实际落在这行
 *   L   L   X       → 短接制动            ← motor_stop_all
 *   H   H   X       → 高阻滑行            ← motor_coast
 *   X   X   X       → STBY=0 关断(高阻)   ← motor_disable / 退出安全姿态
 *
 * 【审计三条（手册§4-AI 靶标B 原文）】
 *   ① STBY 使能: motor_init 结尾 gpio_set_level(PIN_STBY, 1)
 *      依据: TB6612 真值表——STBY=0 时全部输出关断(高阻)，代码全对电机也纹丝不动
 *   ② update: ledc_set_duty 只写影子寄存器, ledc_update_duty 才生效
 *   ③ 限幅: duty 夹到 [-1,1]——算出 1.5×1023 溢出, 物理上也不存在
 *
 * 【接线契约】
 *   引脚已照官方 v1.0（《实验手册2》A1 HTML 原表）落到下方常量：
 *   PWMA=17 / PWMB=41 / AIN1=15 / AIN2=16 / BIN1=39 / BIN2=40 / STBY=42（39~42 系 JTAG 复用, DevKitC-1 默认走 USB-JTAG 可当普通 GPIO）。
 *   呼吸灯台架线仍在 GPIO13 —— 上板测电机前须把 PWMA 那根杜邦线从 13 改插到 17 再复跑一遍。
 *   接错方向脚 = 电机乱转; 共地不接 = "发了没反应"(手册前提第一条)。
 *
 * 【LEDC 参数(与呼吸灯一致, 已验证链路不许悄悄改动)】
 *   验算: 2^10 × 20000 = 20.48MHz ≤ 80MHz ✓
 *   一台 TIMER_0 带两个 CHANNEL(同频双通道, 省 timer 资源)
 * ========================================================================== */
#include "motor_driver.h"

#include <cmath>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char* TAG = "motor";

/* ---------------- 引脚表(官方 v1.0：《实验手册2》A1 HTML 原表，导出即冻结) ------ */
static constexpr gpio_num_t PIN_PWMA = GPIO_NUM_17;  // 左轮调速(LEDC_CH0)
static constexpr gpio_num_t PIN_PWMB = GPIO_NUM_41;  // 右轮调速(LEDC_CH1)
static constexpr gpio_num_t PIN_AIN1 = GPIO_NUM_15;  // 左轮方向1
static constexpr gpio_num_t PIN_AIN2 = GPIO_NUM_16;  // 左轮方向2
static constexpr gpio_num_t PIN_BIN1 = GPIO_NUM_39;  // 右轮方向1(39~42 为 JTAG 复用, DevKitC-1 默认 USB-JTAG, 可作普通 GPIO)
static constexpr gpio_num_t PIN_BIN2 = GPIO_NUM_40;  // 右轮方向2
static constexpr gpio_num_t PIN_STBY = GPIO_NUM_42;  // 总使能(10kΩ 上拉)

/* ---------------- LEDC 常量(与呼吸灯同款) ---------------- */
static constexpr ledc_mode_t      MOTOR_MODE   = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t     MOTOR_TIMER  = LEDC_TIMER_0;
static constexpr ledc_channel_t   MOTOR_CH_A   = LEDC_CHANNEL_0;
static constexpr ledc_channel_t   MOTOR_CH_B   = LEDC_CHANNEL_1;
static constexpr uint32_t         MOTOR_FREQ_HZ = 20000;
static constexpr ledc_timer_bit_t MOTOR_RES     = LEDC_TIMER_10_BIT;
static constexpr uint32_t         MOTOR_DUTY_MAX = (1u << MOTOR_RES) - 1;  // 1023

// 方向脚按"路"索引, 与 motor_set 的 motor 参数对齐
static constexpr gpio_num_t PIN_IN1[2] = {PIN_AIN1, PIN_BIN1};
static constexpr gpio_num_t PIN_IN2[2] = {PIN_AIN2, PIN_BIN2};
static constexpr ledc_channel_t CH_MOTOR[2] = {MOTOR_CH_A, MOTOR_CH_B};

/* -------------------------------------------------------------------------- */
void motor_init(void)
{
    // ---- 方向脚 + STBY: 推挽输出, 无上下拉(输出态不需要) ----
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2)
                    | (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2)
                    | (1ULL << PIN_STBY);
    io.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io));
    // 先立安全姿态再放使能: 方向脚全 0(短接制动), STBY=0(关断)
    gpio_set_level(PIN_AIN1, 0); gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0); gpio_set_level(PIN_BIN2, 0);
    gpio_set_level(PIN_STBY, 0);

    // ---- LEDC timer: 20kHz/10bit, 验算见文件头 ----
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = MOTOR_MODE;
    timer_conf.timer_num       = MOTOR_TIMER;
    timer_conf.duty_resolution = MOTOR_RES;
    timer_conf.freq_hz         = MOTOR_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // ---- 两个 channel 挂同一台 timer(同频双通道) ----
    struct { gpio_num_t pin; ledc_channel_t ch; } chans[2] = {
        {PIN_PWMA, MOTOR_CH_A}, {PIN_PWMB, MOTOR_CH_B},
    };
    for (auto& c : chans) {
        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num   = c.pin;
        ch_conf.speed_mode = MOTOR_MODE;      // 与 timer 同档(呼吸灯实验必查项④)
        ch_conf.channel    = c.ch;
        ch_conf.timer_sel  = MOTOR_TIMER;
        ch_conf.duty       = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    // ---- 靶标B必查项①: STBY 拉高, 总闸合上 ----
    // 真值表: STBY=0 → 全部输出关断。漏这行 = 代码全对、电机纹丝不动(头号翻车)。
    gpio_set_level(PIN_STBY, 1);
    ESP_LOGI(TAG, "TB6612 就绪: 20kHz/10bit, STBY 已使能");
}

void motor_set(int motor, float duty)
{
    if (motor < 0 || motor > 1) return;

    // ---- 靶标B必查项③: 夹限 [-1,1](饱和保护), 越界值物理不存在 ----
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;

    // 符号定方向: 正转 IN1=1/IN2=0, 反转反过来
    // 注: duty=0 时 PWM 输出恒低 → 真值表"短接制动"行(不是滑行!)
    //     需要高阻滑行请显式调 motor_coast()
    int in1 = (duty >= 0.0f) ? 1 : 0;
    int in2 = (duty >= 0.0f) ? 0 : 1;
    gpio_set_level(PIN_IN1[motor], in1);
    gpio_set_level(PIN_IN2[motor], in2);

    // 幅值定占空比: |duty| × 1023 → 10 位寄存器值
    uint32_t d = static_cast<uint32_t>(std::fabs(duty) * MOTOR_DUTY_MAX);

    // ---- 靶标B必查项②: set 写影子寄存器, update 才生效, 必须成对 ----
    ledc_set_duty(MOTOR_MODE, CH_MOTOR[motor], d);
    ledc_update_duty(MOTOR_MODE, CH_MOTOR[motor]);
}

void motor_stop_all(void)
{
    // IN1=IN2=0: TB6612 真值表=短接制动(电机绕组短接, 快速停, 有刹车感)
    gpio_set_level(PIN_AIN1, 0); gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0); gpio_set_level(PIN_BIN2, 0);
    ledc_set_duty(MOTOR_MODE, MOTOR_CH_A, 0);
    ledc_update_duty(MOTOR_MODE, MOTOR_CH_A);
    ledc_set_duty(MOTOR_MODE, MOTOR_CH_B, 0);
    ledc_update_duty(MOTOR_MODE, MOTOR_CH_B);
    // STBY 故意不动: 是否关总闸由调用方(测试退出/看门狗策略)决定
}

void motor_coast(int motor)
{
    if (motor < 0 || motor > 1) return;
    // 真值表: IN1=IN2=1 → 输出高阻(滑行), 与 PWM 电平无关
    gpio_set_level(PIN_IN1[motor], 1);
    gpio_set_level(PIN_IN2[motor], 1);
    // 占空比顺手归零: 高阻下 PWM 不起作用, 清零只为寄存器状态干净
    ledc_set_duty(MOTOR_MODE, CH_MOTOR[motor], 0);
    ledc_update_duty(MOTOR_MODE, CH_MOTOR[motor]);
}

void motor_disable(void)
{
    motor_stop_all();
    gpio_set_level(PIN_STBY, 0);   // 断总闸: 真值表 STBY=0 全输出关断
    ledc_stop(MOTOR_MODE, MOTOR_CH_A, 0);
    ledc_stop(MOTOR_MODE, MOTOR_CH_B, 0);
}
