/* ============================================================================
 * 测试项: 电机持续转·测转速差基线（第2课 实验 2-3）
 * ==========================================================================
 * 【演示内容】
 *   同占空比 0.5 下，左右轮各测两个 10 秒计数窗口（手机慢动作数轮面白标记圈数），
 *   产出"转速差基线"——第 8 课 PID 闭环后的对照组、第二学期噪声模型定参的原始数据。
 *   默认逐轮单独跑（视频里只数一只轮）；RUN_BOTH_WHEELS=true 切双轮同跑备用方案。
 *   窗口开始/结束均为 ESP_LOGI 标记行，日志时间戳(ms since boot)与视频时间轴对齐
 *   ——这是"数据可复算"证据链的一环。
 *
 * 【安全前提】
 *   □ 电机必须架空（持续转 10 秒，落地即飞车）
 *   □ 拍摄: 手机慢动作 120/240fps 对准轮面白标记条
 *   □ 电池电压现场用万用表实测并记录（电压影响转速，是复算四要素之一）
 *   □ 危险项: 菜单选中后需输入 y 回车确认
 *
 * 【中止与清理】
 *   按 x → 100ms 内感知 → motor_disable()（刹停→STBY 拉低→LEDC 停）→ 回菜单。
 *   任意窗口中途中止都算该窗口作废，重新进入从头开始。
 * ========================================================================== */
#include "esp_log.h"
#include "test_framework.h"
#include "motor_driver.h"
#include "motor_test_util.h"

static const char* TAG = "motor-hold";

// ---- 测量参数（需求单指定） ----
static constexpr float HOLD_DUTY  = 0.5f;   // 左右同占空比
static constexpr int   SETTLE_MS  = 3000;   // 起动稳定段, 不计入
static constexpr int   WINDOW_MS  = 10000;  // 计数窗口
static constexpr int   GAP_MS     = 2000;   // 两窗口间短暂停车(模拟第二次重新起测)
static constexpr int   SWITCH_MS  = 3000;   // 换轮前停车
static constexpr int   WINDOW_NUM = 2;      // 每轮窗口数

// ---- 模式开关: false=逐轮单独跑(默认)  true=双轮同跑(备用, 一次拍两只) ----
static constexpr bool  RUN_BOTH_WHEELS = true;

// 单个计数窗口: 开始/结束标记行 + 分片延时; 返回 false=期间被中止
static bool run_window(const char* label, int w)
{
    ESP_LOGI(TAG, ">>> [%s] 窗口%d/%d 开始 (%ds)", label, w, WINDOW_NUM, WINDOW_MS / 1000);
    if (!delay_chunked(WINDOW_MS)) {
        ESP_LOGW(TAG, "<<< [%s] 窗口%d/%d 中止(本窗口作废)", label, w, WINDOW_NUM);
        return false;
    }
    ESP_LOGI(TAG, "<<< [%s] 窗口%d/%d 结束, 圈数记 ___", label, w, WINDOW_NUM);
    return true;
}

void test_motor_hold()
{
    motor_init();
    ESP_LOGW(TAG, "持续转测转速——确认轮子已架空! 按 x 立即刹停退出");
    ESP_LOGI(TAG, "参数: duty=%.2f 稳定段%dms 窗口%ds×%d 间隔%dms 模式=%s",
             HOLD_DUTY, SETTLE_MS, WINDOW_MS / 1000, WINDOW_NUM, GAP_MS,
             RUN_BOTH_WHEELS ? "双轮同跑" : "逐轮单独");
    ESP_LOGI(TAG, "拍摄: 手机慢动作 120/240fps 对轮面白标记; 电压万用表实测记录");

    bool abort_req = false;

    if (RUN_BOTH_WHEELS) {
        // ---- 备用方案: 双轮同时 0.5, 一次拍两只 ----
        motor_set(0, HOLD_DUTY);
        motor_set(1, HOLD_DUTY);
        ESP_LOGI(TAG, "稳定段 %dms——别数", SETTLE_MS);
        abort_req = !delay_chunked(SETTLE_MS);
        for (int w = 1; w <= WINDOW_NUM && !abort_req; ++w) {
            abort_req = !run_window("双轮", w);
            if (!abort_req && w < WINDOW_NUM) {          // 窗口间短暂停车再重新起测
                motor_set(0, 0.0f); motor_set(1, 0.0f);
                abort_req = !delay_chunked(GAP_MS);
                if (!abort_req) { motor_set(0, HOLD_DUTY); motor_set(1, HOLD_DUTY); }
            }
        }
        motor_set(0, 0.0f); motor_set(1, 0.0f);
    } else {
        // ---- 默认: 逐轮单独跑, 左轮全部窗口完成后才轮到右轮 ----
        for (int m = 0; m < 2 && !abort_req; ++m) {
            const char* label = (m == 0) ? "左轮" : "右轮";
            motor_set(m, HOLD_DUTY);
            ESP_LOGI(TAG, "[%s] 稳定段 %dms——别数", label, SETTLE_MS);
            abort_req = !delay_chunked(SETTLE_MS);
            for (int w = 1; w <= WINDOW_NUM && !abort_req; ++w) {
                abort_req = !run_window(label, w);
                if (!abort_req && w < WINDOW_NUM) {
                    motor_set(m, 0.0f);
                    abort_req = !delay_chunked(GAP_MS);
                    if (!abort_req) motor_set(m, HOLD_DUTY);
                }
            }
            if (!abort_req) {                 // 本轮完成: 停车缓冲后换下一轮
                motor_set(m, 0.0f);
                abort_req = !delay_chunked(SWITCH_MS);
            }
        }
    }

    // 清理(可重入合同): 刹停 → STBY 拉低 → LEDC 停止
    motor_disable();
    // ESP_LOGI 的 format 位必须是字符串字面量(宏会拼接颜色前缀), 三元放参数位
    ESP_LOGI(TAG, "%s", abort_req ? "已中止: 刹停+STBY 拉低, 返回菜单"
                                  : "全部窗口完成, 圈数与电压记入台账, 返回菜单");
}
