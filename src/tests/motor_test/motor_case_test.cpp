/* ============================================================================
 * 测试项: 电机八发用例（第2课 P11/P20 "八发用例表全中"）
 * ==========================================================================
 * 【演示内容】
 *   对 motor_set 做 8 条单元级用例验收，左轮 8 条跑完再跑右轮 8 条。
 *   每条用例日志一行给全: 输入 duty / 预期寄存器值 / 预期 IN1/IN2 / 预期现象，
 *   现场照抄进用例表；前后窗口标记行用于与慢动作视频时间戳对齐。
 *   预期寄存器值按 duty_reg = |duty|×1023 截断写死在表里（可复算）。
 *
 * 【安全前提】
 *   □ 电机必须架空！表内含满速用例（#4/#5 duty=±1.0，#6/#7 夹限后同为满速）
 *   □ 接线核对完毕（引脚见 motor_driver.cpp，本文件不出现任何裸引脚号）
 *   □ 本项注册为危险项: 菜单选中后需输入 y 回车确认
 *   □ #8 需要上手捏轮子感受短接制动，轮子离地才安全
 *
 * 【中止与清理】
 *   按 x → 分片延时在 100ms 内感知 → 跳出 → motor_disable()
 *   （短接制动 → STBY 拉低 → LEDC 停止），日志说明已回菜单；可重复进入。
 * ========================================================================== */
#include "esp_log.h"
#include "test_framework.h"
#include "motor_driver.h"
#include "motor_test_util.h"

static const char* TAG = "motor-case";

// 单条用例: 输入 + 预期（全部按需求单写死）+ 观察要点
struct MotorCase {
    float    duty;
    uint32_t exp_reg;    // 预期寄存器值 = |duty|×1023 截断（#6/#7 为夹限后）
    int      exp_in1;
    int      exp_in2;
    const char* expect;  // 预期现象
    const char* observe; // 观察要点（这条在测什么）
};

static const MotorCase kCases[] = {
    { +0.6f,  613, 1, 0, "正转·中速",        "作为基准转速" },
    { -0.6f,  613, 0, 1, "反转·中速",        "符号→方向映射" },
    { +0.3f,  306, 1, 0, "正转·低速",        "必须明显慢于 #1" },
    { +1.0f, 1023, 1, 0, "正转·满速",        "边界不溢出、不复位" },
    { -1.0f, 1023, 0, 1, "反转·满速",        "与 #4 同速反向" },
    { +1.5f, 1023, 1, 0, "等同 +1.0(夹限后)", "与 #4 必须同速; 疯转/异常=限幅缺失" },
    { -1.5f, 1023, 0, 1, "等同 -1.0(夹限后)", "与 #5 必须同速" },
    {  0.0f,    0, 1, 0, "停=短接制动",      "捏轮子反向拖动应明显吃力; 能自由滑行=语义不符" },
};
static constexpr int kCaseCount = sizeof(kCases) / sizeof(kCases[0]);

static constexpr int CASE_RUN_MS  = 2500;  // 每条用例观察时长
static constexpr int CASE_GAP_MS  = 500;   // 条间停车
static constexpr int HOLD_OBS_MS  = 3000;  // #8 加长观察(本身就是 0, 不再停车)

void test_motor_cases()
{
    motor_init();
    ESP_LOGW(TAG, "八发用例运行中(含满速)——确认轮子已架空! 按 x 立即刹停退出");

    bool abort_req = false;
    for (int m = 0; m < 2 && !abort_req; ++m) {
        const char side = (m == 0) ? 'L' : 'R';
        ESP_LOGI(TAG, "===== %c 轮 8 条用例 =====", side);

        for (int i = 0; i < kCaseCount; ++i) {
            const MotorCase& c = kCases[i];

            ESP_LOGI(TAG, ">>> 用例 %d 开始 [%c]", i + 1, side);
            ESP_LOGI(TAG, "[用例 %d/8][%c] duty=%+.2f 预期 reg=%lu IN1=%d IN2=%d → %s",
                     i + 1, side, c.duty, (unsigned long)c.exp_reg,
                     c.exp_in1, c.exp_in2, c.expect);
            ESP_LOGI(TAG, "    观察: %s", c.observe);
            if (i == 5) ESP_LOGI(TAG, "    对照 #4: 应同速");
            if (i == 6) ESP_LOGI(TAG, "    对照 #5: 应同速");
            if (i == 7) ESP_LOGI(TAG, "    现在: 用手捏轮子试反向拖动, 感受短接制动");

            motor_set(m, c.duty);
            const int dur = (i == kCaseCount - 1) ? HOLD_OBS_MS : CASE_RUN_MS;
            if (!delay_chunked(dur)) { abort_req = true; break; }
            ESP_LOGI(TAG, "<<< 用例 %d 结束 [%c]", i + 1, side);

            // 条间停车 500ms; #8 输入本身就是 0, 无需再停
            if (i != kCaseCount - 1) {
                motor_set(m, 0.0f);
                if (!delay_chunked(CASE_GAP_MS)) { abort_req = true; break; }
            }
        }
    }

    // 清理(可重入合同): 刹停 → STBY 拉低 → LEDC 停止
    motor_disable();
    // ESP_LOGI 的 format 位必须是字符串字面量(宏会拼接颜色前缀), 三元放参数位
    ESP_LOGI(TAG, "%s", abort_req ? "已中止: 刹停+STBY 拉低, 返回菜单"
                                  : "16 条用例全部完成, 返回菜单");
}
