/* ============================================================================
 * IMU 纯算法层实现（无 IDF 依赖, 可在 PC 上编译测试）
 * ========================================================================== */
#include "imu_core.h"

#include <cmath>

const ImuAxisMap IMU_AXIS_IDENTITY = {{0, 1, 2}, {+1.0f, +1.0f, +1.0f}};

/* 大端拼接: 高字节在前 */
static inline int16_t be16(const uint8_t* p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

bool imu_parse_frame(const uint8_t raw[14], const ImuAxisMap& map, ImuRaw& out)
{
    if (raw == nullptr) return false;
    for (int k = 0; k < 3; ++k) {
        if (map.src[k] < 0 || map.src[k] > 2) return false;   // 映射表写错就别往下算
    }

    const int16_t ia[3] = {be16(&raw[0]),  be16(&raw[2]),  be16(&raw[4])};
    const int16_t it    = be16(&raw[6]);
    const int16_t ig[3] = {be16(&raw[8]),  be16(&raw[10]), be16(&raw[12])};

    // 先换算成物理量再映射, 保证滤波永远工作在车体坐标系里
    float ca[3], cg[3];
    for (int k = 0; k < 3; ++k) {
        ca[k] = ia[k] / IMU_ACC_LSB_PER_G;
        cg[k] = ig[k] / IMU_GYRO_LSB_PER_DPS;
    }

    float ba[3], bg[3];
    for (int k = 0; k < 3; ++k) {
        ba[k] = map.sign[k] * ca[map.src[k]];
        bg[k] = map.sign[k] * cg[map.src[k]];
    }

    out.ax = ba[0]; out.ay = ba[1]; out.az = ba[2];
    out.gx = bg[0]; out.gy = bg[1]; out.gz = bg[2];
    out.temp_c = it / IMU_TEMP_LSB_PER_C + IMU_TEMP_OFFSET_C;
    return true;
}

void imu_calib_reset(ImuCalib& c)
{
    for (int k = 0; k < 3; ++k) {
        c.sum[k] = 0.0;
        c.sq[k]  = 0.0;
    }
    c.n = 0;
}

void imu_calib_add(ImuCalib& c, const ImuRaw& r)
{
    const float g[3] = {r.gx, r.gy, r.gz};
    for (int k = 0; k < 3; ++k) {
        c.sum[k] += g[k];
        c.sq[k]  += static_cast<double>(g[k]) * g[k];
    }
    ++c.n;
}

bool imu_calib_finish(const ImuCalib& c, float bias_out[3], float sigma_out[3])
{
    for (int k = 0; k < 3; ++k) {
        bias_out[k]  = 0.0f;
        sigma_out[k] = 0.0f;
    }
    if (c.n <= 0) return false;
    for (int k = 0; k < 3; ++k) {
        const double mean = c.sum[k] / c.n;
        const double var  = c.sq[k] / c.n - mean * mean;
        bias_out[k]  = static_cast<float>(mean);
        sigma_out[k] = static_cast<float>(std::sqrt(var > 0.0 ? var : 0.0));
    }
    return true;
}

void imu_filter_step(const ImuRaw& r, const float bias[3], float dt_s, float alpha,
                     ImuAttitude& att)
{
    const float gx = r.gx - bias[0];   // °/s
    const float gy = r.gy - bias[1];
    const float gz = r.gz - bias[2];

    // 加速度计解算: 静态准, 怕振动与线加速度
    const float acc_pitch = std::atan2(-r.ax, std::sqrt(r.ay * r.ay + r.az * r.az)) * 57.29578f;
    const float acc_roll  = std::atan2(r.ay, r.az) * 57.29578f;

    att.pitch = alpha * (att.pitch + gy * dt_s) + (1.0f - alpha) * acc_pitch;
    att.roll  = alpha * (att.roll  + gx * dt_s) + (1.0f - alpha) * acc_roll;

    // yaw 无重力/磁参考: 纯积分, 零偏与噪声必然累积
    att.yaw += gz * dt_s;
}
