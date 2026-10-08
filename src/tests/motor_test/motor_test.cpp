/* ============================================================================
 * 测试项: 双电机开环三动作（第2课 Step 5/6）
 * ==========================================================================
 * 【演示内容】前进 2s → 停 1s → 后退 2s → 停 1s → 原地左转 2s → 停 1s → 循环
 *   左右同占空比 = 前进/后退; 符号相反 = 原地旋转(体会"速度差即转向")
 *
 * 【安全前提（动手前逐条确认）】
 *   □ 电机架空！轮子离地(支架/泡沫块)——车会自己跑下桌是第一事故源
 *   □ 接线时电池开关断开; IN1/IN2/STBY 对照 motor_driver.cpp 顶部引脚表核对
 *   □ 电池负极 ↔ DevKit GND 共地（手册前提第一条）
 *   □ 本测试注册为"需确认"项: 菜单选中后要在串口按 y 才启动
 *
 * 【中止与清理】按 x → 循环退出 → 短接制动 → STBY 拉低(总闸断开) → 回菜单
 *   退出后轮子应立刻刹停、无余转; 再进测试从 motor_init 重新全流程初始化
 * ========================================================================== */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_framework.h"
#include "motor_driver.h"

static const char* TAG = "motor-test";

// 一步动作: 左右带符号占空比 + 持续时长 + 日志名
struct Step {
    float left;
    float right;
    int   duration_ms;
    const char* name;
};

static const Step kSteps[] = {
    { 0.6f,  0.6f, 2000, "前进" },
    { 0.0f,  0.0f, 1000, "停" },
    {-0.6f, -0.6f, 2000, "后退" },
    { 0.0f,  0.0f, 1000, "停" },
    {-0.5f,  0.5f, 2000, "原地左转" },   // 左倒右正 = 差速转向
    { 0.0f,  0.0f, 1000, "停" },
};

// 分片延时: 每 100ms 醒一次查 abort(100Hz tick 下 100ms=10tick, 粒度安全)
// 返回 false 表示等待期间被要求中止
static bool delay_chunked(int ms)
{
    for (int elapsed = 0; elapsed < ms; elapsed += 100) {
        if (test_should_abort()) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return !test_should_abort();
}

void test_motor()
{
    motor_init();
    ESP_LOGW(TAG, "电机测试运行中——确认轮子已架空! 按 x 立即停车退出");

    while (!test_should_abort()) {
        for (const Step& s : kSteps) {
            motor_set(0, s.left);
            motor_set(1, s.right);
            ESP_LOGI(TAG, "%s: L=%+.2f R=%+.2f (%dms)", s.name, s.left, s.right, s.duration_ms);
            if (!delay_chunked(s.duration_ms)) break;   // 动作中途按 x → 跳出
        }
    }

    // ---- 清理(可重入合同 + 安全姿态): 刹停 → 断总闸 → 停 PWM 输出 ----
    // 引脚/通道定义全部收在 motor_driver 里, 本文件不出现裸引脚号
    motor_disable();
    ESP_LOGI(TAG, "电机已停车并断开 STBY, 返回菜单");
}
