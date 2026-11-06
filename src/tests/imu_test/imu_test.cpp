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
 *   □ 上拉只一处: 模块自带 4.7k 时不要再并联(内部上拉此时是临时手段)
 *   □ AD0 悬空(=0x68); 拉高会变 0x69
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_driver.h"
#include "test_framework.h"

static const char* TAG = "imu-test";

static constexpr int PERIOD_MS     = 10;    // 100 Hz = tick 率 100Hz 下的 1 tick
static constexpr int LOG_EVERY     = 10;    // 10 Hz 打印, 避免刷屏
static constexpr int CAL_SAMPLES   = 1000;

void test_imu_attitude()
{
    ESP_LOGI(TAG, "接线三查: 上拉(只一处) / 线长(尽量短) / AD0 悬空=0x68");

    if (!imu_init()) {
        ESP_LOGE(TAG, "初始化失败, 见上方日志");
        imu_deinit();          // 必须: 否则下一轮 i2c_new_master_bus 报 INVALID_STATE
        return;
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

    while (!test_should_abort()) {
        // 真实间隔积分: 固定 dt 在任务被抢占时会让 yaw 累积错误
        const int64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt <= 0.0f || dt > 0.2f) dt = PERIOD_MS / 1000.0f;   // 异常值兜底

        if (!imu_update(dt)) ++err_cnt;

        if (++n % LOG_EVERY == 0) {
            const ImuAttitude& a = imu_attitude();
            ESP_LOGI(TAG, "pitch=%6.1f  roll=%6.1f  yaw=%7.1f  dt=%.3fs  读错=%lu",
                     a.pitch, a.roll, a.yaw, dt, static_cast<unsigned long>(err_cnt));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
    }

    imu_deinit();   // 摘设备 → 删总线 → 句柄置空
    ESP_LOGI(TAG, "已释放 I2C 总线, 返回菜单");
}
