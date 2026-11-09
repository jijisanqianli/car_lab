#pragma once
#include <cstdint>

/* ============================================================================
 * IMU 纯算法层：不碰 I2C、不碰日志、不依赖 ESP-IDF
 * ==========================================================================
 * 存在的理由：这些是"拿到 14 字节之后"的全部数学 —— 最容易写错、错了又最难
 * 在硬件上查（现象永远是"数据在动"）。抽出来之后可以在 PC 上用 g++ 直接编译，
 * 喂合成数据验证；固件那边只是薄薄一层 I2C + 日志。
 *
 * 对应实验: 5-2（换算/滤波/校准）、5-3（漂移速率量化、α 三档对比）
 * ========================================================================== */

/* 一帧 14 字节解析后的物理量（已映射到车体坐标系） */
struct ImuRaw {
    float ax, ay, az;   // g
    float gx, gy, gz;   // °/s（未扣零偏）
    float temp_c;       // °C
};

/* 互补滤波输出的姿态, 单位 ° */
struct ImuAttitude {
    float pitch, roll, yaw;
};

/* 芯片轴 → 车体系映射: 车体第 k 轴 = sign[k] × 芯片第 src[k] 轴
 * 车体: +x 前、+y 左、+z 上；芯片同为右手系, 丝印 X 朝车头、Z 朝上时 Y 自动朝左,
 * 故默认恒等。装法不同只改这张表, 解算公式不动。 */
struct ImuAxisMap {
    int   src[3];
    float sign[3];
};

extern const ImuAxisMap IMU_AXIS_IDENTITY;   // {0,1,2} / {+1,+1,+1}

/* ---------------- 量程与系数（成对改, 见 imu_driver.cpp 注释） ---------------- */
static constexpr float IMU_ACC_LSB_PER_G     = 16384.0f;  // ±2 g
static constexpr float IMU_GYRO_LSB_PER_DPS  = 131.0f;    // ±250 °/s
static constexpr float IMU_TEMP_LSB_PER_C    = 340.0f;
static constexpr float IMU_TEMP_OFFSET_C     = 36.53f;
static constexpr float IMU_ALPHA             = 0.98f;     // 互补滤波权重

/* 解析 14 字节原始帧（加速度 6 + 温度 2 + 陀螺 6，大端）→ 物理量 + 轴映射 */
bool imu_parse_frame(const uint8_t raw[14], const ImuAxisMap& map, ImuRaw& out);

/* 静置校准累加器: 均值 = 零偏, 标准差 = 噪声底 */
struct ImuCalib {
    double sum[3];
    double sq[3];
    int    n;
};
void imu_calib_reset(ImuCalib& c);
void imu_calib_add(ImuCalib& c, const ImuRaw& r);
/* 输出零偏(°/s)与标准差(°/s); n==0 时全部置 0 并返回 false */
bool imu_calib_finish(const ImuCalib& c, float bias_out[3], float sigma_out[3]);

/* 互补滤波推进一步: 高频信陀螺, 低频信加速度计; yaw 无参考, 纯积分必漂。
 * alpha 显式传入, 便于 5-3 做 0.5 / 0.98 / 0.999 三档对比。 */
void imu_filter_step(const ImuRaw& r, const float bias[3], float dt_s, float alpha,
                     ImuAttitude& att);
