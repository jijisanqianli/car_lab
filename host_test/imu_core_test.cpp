/* ============================================================================
 * imu_core 的 PC 端测试（无硬件、无 ESP-IDF）
 * ==========================================================================
 * 编译: g++ -std=c++17 -O2 -I ../src/tests/imu_test imu_core_test.cpp \
 *            ../src/tests/imu_test/imu_core.cpp -o imu_core_test
 *
 * 验的是"拿到 14 字节之后"的每一步：解析换算、轴映射、校准统计、滤波收敛、
 * 漂移速率与 N 秒、α 三档权衡。真实传感器准不准不在这里的范围内。
 * ========================================================================== */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>

#include "imu_core.h"

static int g_fail = 0;

static void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

static void put_be16(uint8_t* p, int16_t v)
{
    p[0] = static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8);
    p[1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
}

/* 按 MPU6050 的量程把物理量编成 14 字节原始帧（与真实芯片同样的编码方式） */
static void encode(uint8_t f[14], float ax, float ay, float az, float temp,
                   float gx, float gy, float gz)
{
    put_be16(&f[0],  static_cast<int16_t>(std::lround(ax * IMU_ACC_LSB_PER_G)));
    put_be16(&f[2],  static_cast<int16_t>(std::lround(ay * IMU_ACC_LSB_PER_G)));
    put_be16(&f[4],  static_cast<int16_t>(std::lround(az * IMU_ACC_LSB_PER_G)));
    put_be16(&f[6],  static_cast<int16_t>(std::lround((temp - IMU_TEMP_OFFSET_C) * IMU_TEMP_LSB_PER_C)));
    put_be16(&f[8],  static_cast<int16_t>(std::lround(gx * IMU_GYRO_LSB_PER_DPS)));
    put_be16(&f[10], static_cast<int16_t>(std::lround(gy * IMU_GYRO_LSB_PER_DPS)));
    put_be16(&f[12], static_cast<int16_t>(std::lround(gz * IMU_GYRO_LSB_PER_DPS)));
}

/* ---------------- 场景 1: 解析与换算 ---------------- */
static void test_parse()
{
    std::printf("\n[1] 解析与换算（大端 + 量程系数）\n");
    uint8_t f[14];
    ImuRaw r;

    encode(f, 1.0f, 0.0f, 0.0f, 25.0f, 0.0f, 0.0f, 1.0f);
    check(imu_parse_frame(f, IMU_AXIS_IDENTITY, r), "解析成功");
    std::printf("      ax=%.4f g (期望 1.0)  gz=%.4f °/s (期望 1.0)  T=%.2f °C (期望 25)\n",
                r.ax, r.gz, r.temp_c);
    check(std::fabs(r.ax - 1.0f) < 1e-3f && std::fabs(r.gz - 1.0f) < 1e-3f &&
          std::fabs(r.temp_c - 25.0f) < 0.02f, "换算值落在量化误差内");

    // 满量程: int16 32767 → +1.9999 g（不是 2.000）
    put_be16(&f[0], 32767); put_be16(&f[2], 0); put_be16(&f[4], 0);
    put_be16(&f[6], 0); put_be16(&f[8], 0); put_be16(&f[10], 0); put_be16(&f[12], 0);
    imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
    std::printf("      32767 原始值 = %.4f g（满量程 ±2g 的上沿）\n", r.ax);
    check(r.ax > 1.999f && r.ax < 2.0f, "满量程换算正确");

    // 大端: 高字节在前，接反会读成 0x00FF 之类的怪值
    uint8_t g[14] = {0x40, 0x00};   // 0x4000 = 16384 = 1g
    for (int i = 2; i < 14; ++i) g[i] = 0;
    imu_parse_frame(g, IMU_AXIS_IDENTITY, r);
    std::printf("      0x4000 解析为 %.4f g\n", r.ax);
    check(std::fabs(r.ax - 1.0f) < 1e-3f, "大端字节序正确（接反会得 0.0038g）");
}

/* ---------------- 场景 2: 轴映射 ---------------- */
static void test_axis_map()
{
    std::printf("\n[2] 轴映射（装反了能救回来）\n");
    uint8_t f[14];
    ImuRaw r;
    encode(f, 0.0f, 1.0f, 0.0f, 25.0f, 0.0f, 3.0f, 0.0f);

    // 恒等: 芯片 +y 有 1g、gy 有 3°/s
    imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
    check(std::fabs(r.ay - 1.0f) < 1e-3f && std::fabs(r.gy - 3.0f) < 1e-3f, "恒等映射");

    // 芯片 y 实际朝车右: 车体 y = -芯片 y
    const ImuAxisMap flip_y = {{0, 1, 2}, {+1.0f, -1.0f, +1.0f}};
    imu_parse_frame(f, flip_y, r);
    std::printf("      翻转后 ay=%.3f g  gy=%.3f °/s\n", r.ay, r.gy);
    check(std::fabs(r.ay + 1.0f) < 1e-3f && std::fabs(r.gy + 3.0f) < 1e-3f, "Y 反号同时作用于加速度与陀螺");

    // 芯片平放但 X/Y 互换了安装方向
    const ImuAxisMap swap_xy = {{1, 0, 2}, {+1.0f, +1.0f, +1.0f}};
    imu_parse_frame(f, swap_xy, r);
    check(std::fabs(r.ax - 1.0f) < 1e-3f && std::fabs(r.ay) < 1e-3f, "X/Y 互换");

    // 非法映射表必须被拒
    const ImuAxisMap bad = {{0, 1, 9}, {+1.0f, +1.0f, +1.0f}};
    check(!imu_parse_frame(f, bad, r), "非法映射返回 false（防越界）");
}

/* ---------------- 场景 3: 校准统计 ---------------- */
static void test_calibration()
{
    std::printf("\n[3] 静置校准（均值=零偏, 标准差=噪声底）\n");
    const float true_bias[3] = {1.50f, -0.80f, 2.00f};
    const float noise_sigma = 0.15f;
    const int   N = 1000;

    std::mt19937 rng(20261030);
    std::normal_distribution<float> nd(0.0f, noise_sigma);

    ImuCalib c;
    imu_calib_reset(c);
    uint8_t f[14];
    ImuRaw r;
    for (int i = 0; i < N; ++i) {
        encode(f, 0.0f, 0.0f, 1.0f, 25.0f,
               true_bias[0] + nd(rng), true_bias[1] + nd(rng), true_bias[2] + nd(rng));
        imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
        imu_calib_add(c, r);
    }
    float bias[3], sigma[3];
    check(imu_calib_finish(c, bias, sigma), "统计完成");

    std::printf("      注入零偏 (%.2f, %.2f, %.2f)  实测 (%.3f, %.3f, %.3f) °/s\n",
                true_bias[0], true_bias[1], true_bias[2], bias[0], bias[1], bias[2]);
    std::printf("      注入噪声 σ=%.2f  实测 σ=(%.3f, %.3f, %.3f) °/s\n",
                noise_sigma, sigma[0], sigma[1], sigma[2]);

    // 均值误差上界约 3σ/√N = 3×0.15/31.6 ≈ 0.014，加量化误差留到 0.03
    bool bias_ok = true;
    for (int k = 0; k < 3; ++k) bias_ok = bias_ok && std::fabs(bias[k] - true_bias[k]) < 0.03f;
    check(bias_ok, "零偏估计误差 < 0.03 °/s");
    bool sig_ok = true;
    for (int k = 0; k < 3; ++k) sig_ok = sig_ok && std::fabs(sigma[k] - noise_sigma) < 0.03f;
    check(sig_ok, "噪声底估计误差 < 0.03 °/s");

    ImuCalib empty;
    imu_calib_reset(empty);
    check(!imu_calib_finish(empty, bias, sigma), "空样本返回 false（不产生 NaN）");
}

/* ---------------- 场景 4: 漂移速率与 N 秒 ---------------- */
static void test_drift()
{
    std::printf("\n[4] yaw 漂移速率与置信窗 N（实验 5-3 的算法验证）\n");
    const float dt = 0.01f;          // 100 Hz
    const int   secs = 60;
    const float injected = 2.0f;     // 注入的陀螺零偏 °/s

    auto run = [&](float bias_z, ImuAttitude& att) {
        att = {};
        uint8_t f[14];
        ImuRaw r;
        const float bias[3] = {0.0f, 0.0f, bias_z};
        for (int i = 0; i < secs * 100; ++i) {
            encode(f, 0.0f, 0.0f, 1.0f, 25.0f, 0.0f, 0.0f, injected);
            imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
            imu_filter_step(r, bias, dt, IMU_ALPHA, att);
        }
    };

    ImuAttitude before, after;
    run(0.0f, before);            // 未校准
    run(injected, after);         // 校准后

    const float rate_before = before.yaw / secs;                    // °/s
    const float per_min_before = rate_before * 60.0f;               // °/min
    const float N_before = 3.0f / (per_min_before / 60.0f);         // 误差超 3° 的秒数
    const float per_min_after = (after.yaw / secs) * 60.0f;

    std::printf("      60 s 内 yaw 变化: 校准前 %.2f°  →  速率 %.2f °/s = %.1f °/min\n",
                before.yaw, rate_before, per_min_before);
    std::printf("      N = 3° ÷ (%.1f/60) = %.1f s\n", per_min_before, N_before);
    std::printf("      校准后(扣掉 2.0 °/s) 速率 = %.3f °/min（残差来自量化）\n", per_min_after);

    check(std::fabs(per_min_before - 120.0f) < 0.5f, "注入 2 °/s → 算出 120 °/min");
    check(std::fabs(N_before - 1.5f) < 0.02f, "N 秒公式与 1.5 s 一致");
    check(std::fabs(per_min_after) < 3.0f, "扣零偏后残差 < 3 °/min（量化误差量级）");
}

/* ---------------- 场景 5: 互补滤波收敛 ---------------- */
static void test_convergence()
{
    std::printf("\n[5] 互补滤波收敛（静态倾斜 30°）\n");
    const float dt = 0.01f;
    const float tilt = 30.0f;
    const float rad  = tilt * 3.14159265f / 180.0f;

    uint8_t f[14];
    ImuRaw r;
    // 车头抬起 30°: ax = -sin30, az = cos30，陀螺静止
    encode(f, -std::sin(rad), 0.0f, std::cos(rad), 25.0f, 0.0f, 0.0f, 0.0f);
    imu_parse_frame(f, IMU_AXIS_IDENTITY, r);

    ImuAttitude att = {};
    const float bias[3] = {0, 0, 0};
    float t95 = -1.0f;
    for (int i = 1; i <= 500; ++i) {          // 5 s
        imu_filter_step(r, bias, dt, IMU_ALPHA, att);
        if (t95 < 0.0f && att.pitch >= 0.95f * tilt) t95 = i * dt;
    }
    std::printf("      3 s 后 pitch = %.3f°（期望 %.1f）\n", att.pitch, tilt);
    std::printf("      到达 95%% 用时 %.2f s，理论 3/(1-α)·dt = %.2f s\n", t95, 3.0f * dt / (1.0f - IMU_ALPHA));
    check(std::fabs(att.pitch - tilt) < 0.05f, "稳态收敛到注入角度");
    check(t95 > 1.0f && t95 < 2.0f, "收敛时间符合一阶惯性预期");
}

/* ---------------- 场景 6: α 三档权衡（实验 5-3 的表格） ---------------- */
static void test_alpha()
{
    std::printf("\n[6] α = 0.5 / 0.98 / 0.999 的权衡\n");
    const float dt = 0.01f;
    const float alphas[3] = {0.5f, 0.98f, 0.999f};
    std::mt19937 rng(7);
    std::normal_distribution<float> nd(0.0f, 0.5f);   // 加速度计 0.5 g 噪声，模拟振动

    std::printf("      %-6s %-16s %s\n", "α", "静置噪声峰峰值(°)", "30° 步进到 95% 用时(s)");
    for (float a : alphas) {
        // 噪声: 静态水平 + 加速度计噪声
        ImuAttitude att = {};
        const float bias[3] = {0, 0, 0};
        float lo = 1e9f, hi = -1e9f;
        uint8_t f[14];
        ImuRaw r;
        for (int i = 0; i < 1000; ++i) {              // 10 s
            encode(f, nd(rng), 0.0f, 1.0f, 25.0f, 0.0f, 0.0f, 0.0f);
            imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
            imu_filter_step(r, bias, dt, a, att);
            if (i > 100) {                            // 跳过起振
                lo = std::fmin(lo, att.pitch);
                hi = std::fmax(hi, att.pitch);
            }
        }
        // 步进: 从 0 到 30°
        const float rad = 30.0f * 3.14159265f / 180.0f;
        encode(f, -std::sin(rad), 0.0f, std::cos(rad), 25.0f, 0.0f, 0.0f, 0.0f);
        imu_parse_frame(f, IMU_AXIS_IDENTITY, r);
        ImuAttitude step = {};
        float t95 = -1.0f;
        for (int i = 1; i <= 6000; ++i) {             // 60 s 上限
            imu_filter_step(r, bias, dt, a, step);
            if (step.pitch >= 28.5f) { t95 = i * dt; break; }
        }
        std::printf("      %-6.3f %-16.3f %.2f\n", a, hi - lo, t95);
    }
    std::printf("      → α 越大越平滑、但跟随越慢；0.98 是两者的折中\n");
    check(true, "α 三档跑通（数值用于报告里的权衡表）");
}

int main()
{
    std::printf("imu_core 纯算法层测试（无硬件）\n");
    test_parse();
    test_axis_map();
    test_calibration();
    test_drift();
    test_convergence();
    test_alpha();

    std::printf("\n结果: %s（失败 %d 项）\n", g_fail == 0 ? "全部通过" : "有失败项", g_fail);
    return g_fail == 0 ? 0 : 1;
}
