/* ============================================================================
 * MPU6050 驱动实现（I2C 薄层；全部数学在 imu_core.cpp）
 * ==========================================================================
 * 【接线契约】
 *   SDA=GPIO1 / SCL=GPIO2（《实验手册2》A1 引脚表，导出即冻结）
 *   上拉只一处：模块自带 2.2 kΩ（丝印 222×2，2026-11-11 实拍确认），内部上拉已关闭
 *   ——并联第二处上拉会拉低高电平、加快边沿但改时序余量，评审口径按 2.2k 记
 *
 * 【两个常见症状的根因】
 *   读回 0x00 / 0xFF  → 接线或供电（不是"传感器坏了"）
 *   读回 0x69         → AD0 被拉高；AD0 悬空才是 0x68
 *   数据全零          → 漏写 PWR_MGMT_1（未退出睡眠）
 *
 * 【为什么校准不用 vTaskDelay(2ms)】
 *   本系统 CONFIG_FREERTOS_HZ=100 → 1 tick = 10 ms，pdMS_TO_TICKS(2) 截断为 0，
 *   vTaskDelay(0) 按文档"立即返回不让出 CPU" → 空转可能饿死 IDLE 触发看门狗。
 *   本文件改为「每采 10 个样本让出 1 tick」，既让出 CPU，又保持约 1.5 s 的校准时长。
 * ========================================================================== */
#include "imu_driver.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "imu";

/* ---------------- 接线与总线参数 ---------------- */
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_1;
static constexpr gpio_num_t PIN_SCL = GPIO_NUM_2;
static constexpr uint8_t    IMU_ADDR     = 0x68;     // AD0 悬空
static constexpr uint32_t   I2C_FREQ_HZ  = 400000;   // 400 kHz 快速模式
static constexpr int        I2C_TIMEOUT_MS = 100;    // 单次传输超时, 防总线挂死后永久阻塞

/* ---------------- 寄存器 ---------------- */
static constexpr uint8_t REG_WHO_AM_I   = 0x75;  // 期望 0x68
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;  // 写 0x00 退出睡眠
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr int     RAW_FRAME_LEN  = 14;    // ax ay az t gx gy gz

/* ---------------- 轴映射（装法不同只改这里，解算公式不动——1.22 纪律） ----------------
 * 台架物理约束：GY-521 排针在板边，芯片朝上放=引脚顶桌面，放不稳 → 实测只能芯片朝下。
 * 芯片朝下 = 相对"芯片朝上"绕 X 轴转了 180° → 车体系 y、z 两轴对芯片轴取反。
 * 上车载入 PCB 后，按实际安装朝向重新推导这张表（三问验证法见 imu_driver.h）。 */
static const ImuAxisMap AXIS_MAP = {{0, 1, 2}, {+1.0f, -1.0f, -1.0f}};

/* ---------------- 句柄与状态（生命周期 = imu_init ~ imu_deinit） ---------------- */
static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_dev = nullptr;

static ImuRaw      s_raw   = {};
static float       s_bias[3] = {0, 0, 0};   // °/s
static ImuAttitude s_att   = {};

static uint32_t    s_read_err = 0;  // 读失败计数(用于限流打印)

/* -------------------------------------------------------------------------- */
/* 私有工具: 单寄存器读 / 写                                                  */
/* -------------------------------------------------------------------------- */
static bool read_reg(uint8_t reg, uint8_t* buf, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        // 100Hz 主循环里持续失败会刷屏: 前 3 次必打, 之后每 100 次打一次
        if (s_read_err < 3 || (s_read_err % 100) == 0) {
            ESP_LOGE(TAG, "读 0x%02X 失败(%lu 次): %s（查接线/共地/上拉）",
                     reg, static_cast<unsigned long>(s_read_err), esp_err_to_name(err));
        }
        ++s_read_err;
        return false;
    }
    s_read_err = 0;   // 连续成功则清零, 下次故障重新从第 1 次开始打
    return true;
}

static bool write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写 0x%02X 失败: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* 初始化: 总线 → 设备 → 验身份 → 唤醒                                        */
/* 非幂等: 每次调用都新建, 端口被占用时 i2c_new_master_bus 返回 ESP_ERR_INVALID_STATE */
/* -------------------------------------------------------------------------- */
bool imu_init()
{
    // ---- ① 总线 ----
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port   = I2C_NUM_0;
    bus_cfg.sda_io_num = PIN_SDA;
    bus_cfg.scl_io_num = PIN_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;             // 典型值, 滤掉毛刺
    // 模块自带 2.2kΩ 上拉(丝印222, 2026-11-11 实拍确认), "上拉只一处"纪律 → 内部上拉关闭
    bus_cfg.flags.enable_internal_pullup = false;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        s_bus = nullptr;
        ESP_LOGE(TAG, "I2C 总线创建失败: %s"
                      "（ESP_ERR_INVALID_STATE 说明上一轮没 imu_deinit）",
                 esp_err_to_name(err));
        return false;
    }

    // ---- ② 设备 ----
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = IMU_ADDR;
    dev_cfg.scl_speed_hz    = I2C_FREQ_HZ;
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        s_dev = nullptr;
        ESP_LOGE(TAG, "I2C 设备挂载失败: %s", esp_err_to_name(err));
        return false;
    }

    // ---- ③ 验身份: 寄存器 0x75 ≠ 器件地址 0x68 ----
    uint8_t id = 0;
    if (!read_reg(REG_WHO_AM_I, &id, 1)) return false;
    if (id != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X, 期望 0x68"
                      "（0x69=AD0 被拉高; 0x00/0xFF=接线或供电）", id);
        return false;
    }

    // ---- 冷启动证据（2026-11-11 用户批准增）：唤醒前读一次加速度寄存器 ----
    // 上电后首跑：器件处于 POR 睡眠态，寄存器=复位值 → 应读到 0x00 0x00
    // 同一上电周期的第二次跑：器件已被上一轮唤醒过，读到的是残值（SLEEP 保持寄存器）
    uint8_t pre[2] = {0xFF, 0xFF};
    read_reg(REG_ACCEL_XOUT_H, pre, 2);
    ESP_LOGI(TAG, "唤醒前(写0x6B之前) ACCEL_XOUT_H = 0x%02X 0x%02X"
                  "（上电首跑应 0x00 0x00，重跑读到残值属正常）", pre[0], pre[1]);

    // ---- ④ 唤醒: 漏这步数据全零 ----
    if (!write_reg(REG_PWR_MGMT_1, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(50));   // 退出睡眠到第一帧有效数据

    s_raw = {};
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
    s_att = {};
    s_read_err = 0;
    ESP_LOGI(TAG, "MPU6050 就绪: WHO_AM_I=0x68, 400kHz, SDA=%d SCL=%d",
             PIN_SDA, PIN_SCL);
    return true;
}

void imu_deinit()
{
    // 先摘设备再删总线: i2c_del_master_bus 会连带清理设备, 顺序反了句柄就悬空
    if (s_dev != nullptr) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = nullptr;
    }
    if (s_bus != nullptr) {
        i2c_del_master_bus(s_bus);
        s_bus = nullptr;
    }
    s_raw  = {};
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
    s_att  = {};
}

/* -------------------------------------------------------------------------- */
/* 读一帧: 取原字节 → 交给 imu_core 解析换算映射                              */
/* -------------------------------------------------------------------------- */
bool imu_read_raw(ImuRaw& out)
{
    uint8_t buf[RAW_FRAME_LEN];
    if (!read_reg(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;
    return imu_parse_frame(buf, AXIS_MAP, out);
}

/* -------------------------------------------------------------------------- */
/* 静置校准: 采样与节奏在驱动层, 统计在 imu_core                              */
/* -------------------------------------------------------------------------- */
bool imu_calibrate(int samples)
{
    if (samples <= 0) return false;

    ImuCalib acc;
    imu_calib_reset(acc);
    ImuRaw r;

    for (int i = 0; i < samples; ++i) {
        if (!imu_read_raw(r)) return false;
        imu_calib_add(acc, r);
        // 每 10 个样本让出 1 tick: 2ms 会被 pdMS_TO_TICKS 截断成 0(见文件头)
        if ((i + 1) % 10 == 0) vTaskDelay(1);
    }

    float sigma[3];
    if (!imu_calib_finish(acc, s_bias, sigma)) return false;

    ESP_LOGI(TAG, "校准完成(%d 样本): 零偏=(%.2f, %.2f, %.2f) °/s, 噪声σ=(%.2f, %.2f, %.2f) °/s",
             samples, s_bias[0], s_bias[1], s_bias[2], sigma[0], sigma[1], sigma[2]);
    return true;
}

/* -------------------------------------------------------------------------- */
/* 采一帧 + 滤波推进（数学全在 imu_core）                                      */
/* -------------------------------------------------------------------------- */
bool imu_update(float dt_s)
{
    ImuRaw r;
    if (!imu_read_raw(r)) return false;
    s_raw = r;
    imu_filter_step(r, s_bias, dt_s, IMU_ALPHA, s_att);
    return true;
}

const ImuAttitude& imu_attitude() { return s_att; }

const ImuRaw& imu_raw() { return s_raw; }

void imu_get_bias(float out[3])
{
    out[0] = s_bias[0];
    out[1] = s_bias[1];
    out[2] = s_bias[2];
}

/* -------------------------------------------------------------------------- */
/* 诊断辅助（2026-11-11 增）：只调用既有 read/write，不改任何状态机            */
/* -------------------------------------------------------------------------- */
int imu_whoami_sweep(int n)
{
    if (n <= 0) return -1;
    int fail = 0;
    uint8_t id;
    for (int i = 0; i < n; ++i) {
        if (!read_reg(REG_WHO_AM_I, &id, 1) || id != 0x68) ++fail;
    }
    return fail;
}

bool imu_sleep_wake_demo(uint8_t out_sleep[2], uint8_t out_wake[2])
{
    if (!write_reg(REG_PWR_MGMT_1, 0x40)) return false;   // 强制睡眠
    vTaskDelay(pdMS_TO_TICKS(10));
    bool ok = read_reg(REG_ACCEL_XOUT_H, out_sleep, 2);
    if (!write_reg(REG_PWR_MGMT_1, 0x00)) return false;   // 唤醒
    vTaskDelay(pdMS_TO_TICKS(50));                        // 退出睡眠到首帧有效
    ok = ok && read_reg(REG_ACCEL_XOUT_H, out_wake, 2);
    return ok;
}
