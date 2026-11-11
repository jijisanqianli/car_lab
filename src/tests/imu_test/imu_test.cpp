/* ============================================================================
 * 测试项: IMU 驱动三步 + 零偏校准 + 互补滤波（第5课 实验 5-2）
 * ==========================================================================
 * 【演示内容】
 *   验身份 → 唤醒 → 静置 1000 次校准 → 100 Hz 互补滤波输出三轴角度。
 *   验收: WHO_AM_I=0x68; 三轴角度实时输出(倾斜跟随、静置回零); 校准前后
 *        yaw 行为差异可见; 换算系数与量程一致。
 *
 * 【台架要求】
 *   □ SDA→GPIO1, SCL→GPIO2, GND 必须与开发板共地
 *   □ 上拉只一处: 模块自带 2.2k（丝印222），代码内部上拉已关，不要再并联
 *   □ AD0: 本模块经 4.7k 板载下拉 → 默认 0x68（等效"悬空=低"）
 *   □ 校准与观察期间桌面不许碰 —— 手碰桌子 = 零偏里混进振动
 *   □ 上电先预热 3~5 分钟再校准: 零偏随温度漂, 冷机校出来的 bias 会一直偏
 *
 * 【坐标系三问（现场验收用）】
 *   车头抬起 → pitch 增大; 左侧抬起 → roll 增大; 俯视逆时针转 → yaw 增大。
 *   任一相反: 改 imu_driver.cpp 的 MAP_SIGN, 不动解算公式。
 *
 * 【退出】
 *   按 x → 主循环 10ms 内看到 → 跳出 → imu_deinit() 摘设备删总线 → 回菜单。
 *   初始化失败或校准失败也走同一条 imu_deinit(), 保证下一轮能重新建总线。
 * ========================================================================== */
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>   // sqrtf（诊断3 重力模长）
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_driver.h"
#include "test_framework.h"

static const char* TAG = "imu-test";

static constexpr int PERIOD_MS     = 10;    // 100 Hz = tick 率 100Hz 下的 1 tick
static constexpr int LOG_EVERY     = 10;    // 10 Hz 打印, 避免刷屏
static constexpr int CAL_SAMPLES   = 1000;
static constexpr int DRIFT_TOTAL_S = 600;   // 漂移会话总时长: 10 分钟
static constexpr int DRIFT_MARK_S  = 120;   // ★记录点间隔: 2 分钟

void test_imu_attitude()
{
    ESP_LOGI(TAG, "接线三查: 上拉(模块2.2k只一处) / 线长(尽量短) / AD0 板载下拉=0x68");

    if (!imu_init()) {
        ESP_LOGE(TAG, "初始化失败, 见上方日志");
        imu_deinit();          // 必须: 否则下一轮 i2c_new_master_bus 报 INVALID_STATE
        return;
    }

    // ---- 诊断三连（2026-11-11 增，对应采集单 §二【驱动验证】三格）----
    int who_fail = imu_whoami_sweep(100);
    ESP_LOGI(TAG, "[诊断1] WHO_AM_I 连读 100 次: 失败 %d 次（0 才算稳）", who_fail);

    uint8_t slp[2] = {0}, wake[2] = {0};
    if (imu_sleep_wake_demo(slp, wake)) {
        // 注: 本演示在 init 之后, 器件已醒过 → 睡前读到的是残值(与诊断3 ax 吻合即自洽)
        //     "全 0"只有冷上电首读才成立, 证据看 imu 标签的"唤醒前(写0x6B之前)"行
        ESP_LOGI(TAG, "[诊断2] 中途强制睡眠前/后 ACCEL_XOUT_H = 0x%02X 0x%02X / 0x%02X 0x%02X"
                      "（睡前读到残值=机制正常, 非故障）",
                 slp[0], slp[1], wake[0], wake[1]);
    } else {
        ESP_LOGE(TAG, "[诊断2] 睡眠/唤醒演示 I2C 出错");
    }

    ImuRaw g;
    if (imu_read_raw(g)) {
        float norm = sqrtf(g.ax * g.ax + g.ay * g.ay + g.az * g.az);
        ESP_LOGI(TAG, "[诊断3] 静置加速度 ax=%.3f ay=%.3f az=%.3f g → 模长=%.3f g（应≈1.00，>5%% 查系数）",
                 g.ax, g.ay, g.az, norm);
    }

    ESP_LOGI(TAG, "静置校准中(约 1.5s), 手离开桌面...");
    if (!imu_calibrate(CAL_SAMPLES)) {
        ESP_LOGE(TAG, "校准失败(采样期间 I2C 出错)");
        imu_deinit();
        return;
    }

    ESP_LOGI(TAG, "开始 100Hz 输出, 按 x 返回菜单");
    ESP_LOGI(TAG, "观察: 静置三轴≈0 | 倾斜跟随 | 松手回零 | yaw 缓慢单向增长(正常, 本课不修)");

    TickType_t last_wake = xTaskGetTickCount();
    int64_t    last_us   = esp_timer_get_time();
    uint32_t   n = 0;
    uint32_t   err_cnt = 0;
    float      a_peak = 0.0f;   // 1.17 猛拍桌面：静置后最大 |加速度|（g）

    while (!test_should_abort()) {
        // 真实间隔积分: 固定 dt 在任务被抢占时会让 yaw 累积错误
        const int64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.2f) dt = PERIOD_MS / 1000.0f;   // 异常值兜底

        if (imu_update(dt)) {
            const ImuRaw& r = imu_raw();
            float m = fmaxf(fmaxf(fabsf(r.ax), fabsf(r.ay)), fabsf(r.az));
            if (m > a_peak) a_peak = m;
        } else {
            ++err_cnt;
        }

        if (++n % LOG_EVERY == 0) {
            const ImuAttitude& a = imu_attitude();
            ESP_LOGI(TAG, "pitch=%6.1f  roll=%6.1f  yaw=%7.1f  dt=%.3fs  读错=%lu  峰值=%.2fg",
                     a.pitch, a.roll, a.yaw, dt, static_cast<unsigned long>(err_cnt), a_peak);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
    }

    imu_deinit();   // 摘设备 → 删总线 → 句柄置空
    ESP_LOGI(TAG, "已释放 I2C 总线, 返回菜单");
}

/* ============================================================================
 * 漂移测量会话（第 9/10 项共用）：10 分钟挂机，每 2 分钟一条★记录点，自动退出
 *   calibrate=false → 校准前组（零偏全额进积分，漂得快）
 *   calibrate=true  → 校准后组（残差+温漂，漂得慢——"校准值多少钱"的另一半）
 * 记录时刻以"进入循环"为 t0（校准后组即校准完成时刻），与第 8 项交互演示互不干扰。
 * ========================================================================== */
static void drift_session(const char* label, bool calibrate)
{
    if (!imu_init()) {
        ESP_LOGE(TAG, "初始化失败, 见上方日志");
        imu_deinit();
        return;
    }
    if (calibrate) {
        ESP_LOGI(TAG, "[%s] 静置校准中(约 1.5s), 手离开桌面...", label);
        if (!imu_calibrate(CAL_SAMPLES)) {
            ESP_LOGE(TAG, "[%s] 校准失败(采样期间 I2C 出错)", label);
            imu_deinit();
            return;
        }
    } else {
        ESP_LOGW(TAG, "[%s] 本项【不校准】，零偏直接进积分——yaw 漂是被测现象", label);
    }
    ESP_LOGI(TAG, "[%s] 平放挂机 %d 分钟，每 %d 分钟一条★记录点，到点自动退出",
             label, DRIFT_TOTAL_S / 60, DRIFT_MARK_S / 60);

    TickType_t last_wake = xTaskGetTickCount();
    const int64_t start_us = esp_timer_get_time();
    int64_t last_us = start_us;
    int next_mark = DRIFT_MARK_S;   // 普通局部变量: 每次进入从 2min 重新计

    while (!test_should_abort()) {
        const int64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.2f) dt = PERIOD_MS / 1000.0f;

        imu_update(dt);

        int elapsed_s = static_cast<int>((now_us - start_us) / 1000000);
        if (elapsed_s >= next_mark) {
            const ImuAttitude& a = imu_attitude();
            ESP_LOGI(TAG, "★ 记录点(%s) t=%ds: pitch=%.1f roll=%.1f yaw=%.1f",
                     label, next_mark, a.pitch, a.roll, a.yaw);
            next_mark += DRIFT_MARK_S;
        }
        if (elapsed_s >= DRIFT_TOTAL_S) {
            ESP_LOGI(TAG, "[%s] 10 分钟到，6 个记录点已齐", label);
            break;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
    }

    imu_deinit();
    ESP_LOGI(TAG, "[%s] 漂移测量结束，返回菜单", label);
}

void test_imu_drift()      { drift_session("校准前", false); }
void test_imu_drift_cal()  { drift_session("校准后", true); }
