/* ============================================================================
 * 实验 2-1 · LEDC 呼吸灯（课程 P9 P10）
 * ==========================================================================
 * 【做什么】
 *   配置 ESP32-S3 的 LEDC 外设（硬件 PWM 发生器），让 LED 亮度平滑地
 *   渐亮 → 渐暗 → 渐亮……循环，即"呼吸"。
 *
 * 【为什么先做它——故障解耦思想】
 *   第 2 课要用同一条 PWM 链路（timer→channel→引脚）驱动 TB6612 电机。
 *   LED 是低风险负载：接错不炸东西、现象肉眼可见。先用它把链路验证通，
 *   以后"电机不转"就能立刻排除"PWM 本身没配对"这个嫌疑，缩小排查范围。
 *   （对照手册 §10 故障树：电机不转的头号原因是 STBY，第二才是 PWM 链路）
 *
 * 【验收标准】
 *   □ 亮度平滑渐变、无台阶闪烁（1024 级 × 每步 2 级增量，肉眼无台阶）
 *   □ 按 x 退出后灯灭；再次进入从全暗重新开始（可重入）
 *   □ 量化证据：duty=512 时万用表直流档量 GPIO17 ≈ 3.3V×512/1024 ≈ 1.65V
 *
 * 【使用方法】
 *   1. 接线：LED 正极(长脚) → GPIO17；LED 负极(短脚) → 220Ω~1kΩ 电阻 → GND
 *      ⚠ 开发板与外部供电必须共地（手册重要前提第一条）
 *   2. 烧录：pio run -t upload（或 CLion 的 Upload 按钮）
 *   3. 看串口：pio device monitor → 菜单选 "4. LEDC 呼吸灯"
 *   4. 呼吸开始；按 x 停止并回菜单
 *   5. 调节奏：改下方 BREATH_STEP_MS / BREATH_DUTY_STEP 两个常量
 *      单程时长(ms) ≈ 1023 / DUTY_STEP × STEP_MS
 *      嫌慢：STEP_MS 不变、DUTY_STEP 调大（如 8 → 单程 1.3s）
 *      嫌糙：DUTY_STEP 调 1（单程 10.2s，最细腻）
 *      ⚠ STEP_MS 最低 10（tick 率 100Hz 的粒度下限，见"核心知识点三"）
 *
 * 【核心知识点一：参数有依据（必查项①，答辩要看这段注释）】
 *   LEDC 约束：2^分辨率 × PWM频率 ≤ 时钟源 80MHz
 *   本配置验算：2^10 × 20000 = 1024 × 20k = 20.48MHz ≤ 80MHz ✓
 *   贪心验证：14bit 则 2^14 × 20k ≈ 327MHz ✗ ——分辨率与频率是跷跷板
 *   为什么 20kHz：人耳可闻 20Hz~20kHz，5kHz 落在敏感区电机啸叫；
 *   20kHz 恰在听觉上限之外，安静且开关损耗可接受
 *
 * 【核心知识点二：set 之后必须 update（必查项②）】
 *   ledc_set_duty() 只是把新占空比写进"影子寄存器"（准备值）；
 *   ledc_update_duty() 才让它在下一个 PWM 周期生效。
 *   漏 update = 参数改了但永远不输出——AI 和初学者最常见遗漏，
 *   也是第 2 课"改了占空比没反应"的头号嫌疑。
 *
 * 【核心知识点三：延时粒度 = 1 tick（本实验真实翻车记录）】
 *   ESP-IDF 默认 CONFIG_FREERTOS_HZ=100 → 1 tick = 10ms。
 *   pdMS_TO_TICKS(5) 截断为 0，而 vTaskDelay(0) 按文档"立即返回不让出 CPU"
 *   → 循环全速空转 → IDLE 任务饿死 → 5 秒后 task watchdog 复位。
 *   教训：任何 <10ms 的"延时"在这个系统里都是空操作；要么参数 ≥10ms，
 *   要么加 max(1, ticks) 护栏（本文件两处都做了）。
 * ========================================================================== */

#include "driver/gpio.h"   // gpio_num_t 类型（IDF6 的 ledc.h 不再间接带入，必须显式包含）
#include "driver/ledc.h"   // LEDC 全部 API：timer/channel 配置、set/update/stop
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "test_framework.h" // selftest 框架：TestCase 约定 + test_should_abort()

static const char* TAG = "ledc-breath";  // 日志标签，串口按此过滤本测试的日志

/* ---------------------------------------------------------------------------
 * PWM 参数区（集中定义，一处改处处改）
 * 第 2 课 motor_init 将复用这套定义——已验证的链路参数不许悄悄改动，
 * 这是引脚表"法律效力"精神的延伸。
 * ------------------------------------------------------------------------- */

// 输出引脚：GPIO17 = 引脚分配表的 PWMA（TB6612 的 A 路调速脚）。
// 现在插 LED 验证链路，第 2 课把杜邦线从 LED 挪到 TB6612 的 PWMA 脚即可，代码零改动。
static constexpr gpio_num_t BREATH_PIN = GPIO_NUM_13;

// LEDC 分 LOW/HIGH 两组独立的 timer+channel。本实验全用 LOW_SPEED。
// ⚠ 必查项④：timer 和 channel 的 speed_mode 必须同档！一个 LOW 一个 HIGH
//   的后果：不报错、不崩溃，就是没输出——最隐蔽的坑。
// 不同mode直接对应不同的硬件，是完全独立的两套ledc
static constexpr ledc_mode_t BREATH_MODE = LEDC_LOW_SPEED_MODE;

// TIMER：硬件"节拍发生器"，决定 PWM 频率。一个 timer 可带多个 channel，
// 同频率的多个输出共用一个 timer 省资源（第 2 课 PWMA/PWMB 同 20kHz 即可共用）。
static constexpr ledc_timer_t BREATH_TIMER = LEDC_TIMER_0;

// CHANNEL：占空比发生器 + 引脚输出级。每个 channel 绑定一个 GPIO，
// 从所属 timer 借节拍、自己决定"高电平占多宽"。
static constexpr ledc_channel_t BREATH_CH = LEDC_CHANNEL_0;

// 开关进行打开关闭的频率，要保证够高使得灯不闪烁，电机无啸叫声音
static constexpr uint32_t BREATH_FREQ_HZ = 20000;                 // 20kHz，理由见文件头
// 就是档位有多少，越大调节的越细腻，渐变感更强，颗粒感更弱
static constexpr ledc_timer_bit_t BREATH_RES = LEDC_TIMER_10_BIT; // 10 位分辨率

// 占空比满量程：10bit → 取值范围 0~1023。写成 (1<<10)-1 而不是硬编码 1023，
// 将来分辨率改成 12bit 时这一行自动跟着变，不会漏改。
static constexpr uint32_t BREATH_DUTY_MAX = (1u << BREATH_RES) - 1;

/* ---------------------------------------------------------------------------
 * 呼吸节奏区：单程时长(ms) ≈ BREATH_DUTY_MAX / BREATH_DUTY_STEP × BREATH_STEP_MS
 * 当前：1023/4 × 10ms ≈ 2.5s 单程，全周期约 5s
 * ⚠ BREATH_STEP_MS 不得小于 10ms：本系统 tick 率 100Hz(1 tick=10ms)，
 *   更小的值会被 pdMS_TO_TICKS 截断成 0 → vTaskDelay(0) 不让出 CPU → 空转
 *   → 5 秒后 task watchdog 复位（本实验真实踩过，见下方循环内注释）
 * ------------------------------------------------------------------------- */
static constexpr uint32_t BREATH_STEP_MS   = 10; // 每步间隔(=1 tick 下限)；也决定 x 响应延迟
static constexpr uint32_t BREATH_DUTY_STEP = 4;  // 每步占空比增量（1=最细腻最慢, 8=快而略糙）

/* ---------------------------------------------------------------------------
 * P10 三步之①②：配置 timer（节拍）+ channel（引脚+占空比发生器）
 * 数据流：timer 产生 20kHz 节拍 → channel 按 duty/1024 决定每周期高电平宽度
 *        → GPIO 矩阵把信号路由到 BREATH_PIN → LED 看到的平均电压 = 3.3V × duty/1024
 *   （这就是万用表能量出 1.65V 的原理——表笔响应慢，读到的是 PWM 的平均值）
 * ------------------------------------------------------------------------- */
static void breath_pwm_init()
{
    // ---- 第一步 ledc_timer_config：定节拍发生器 ----
    ledc_timer_config_t timer_conf = {};   // 先清零：未填字段取 0/默认值，防栈上垃圾值
    timer_conf.speed_mode      = BREATH_MODE;
    timer_conf.timer_num       = BREATH_TIMER;
    timer_conf.duty_resolution = BREATH_RES;    // 10bit → 占空比取值 0~1023
    timer_conf.freq_hz         = BREATH_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK; // 让驱动自动选时钟源（80MHz APB 够用）
    // ESP_ERROR_CHECK：初始化类错误直接 abort 并打印原因——链路配不好就别往下走
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // ---- 第二步 ledc_channel_config：通道绑引脚 ----
    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num   = BREATH_PIN;
    ch_conf.speed_mode = BREATH_MODE;      // 与 timer 同档（必查项④，不一致=静默无输出）
    ch_conf.channel    = BREATH_CH;
    ch_conf.timer_sel  = BREATH_TIMER;     // 声明"我挂在哪台节拍器上"
    ch_conf.duty       = 0;                // 初始全暗，呼吸从亮起来
    // 注：IDF6 已废弃 intr_type 字段（中断由驱动内部处理），老教程里的这行别抄
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}

/* ---------------------------------------------------------------------------
 * 测试入口（selftest 框架约定的 void test_xxx() 签名）
 * 运行在框架创建的独立任务里；按 x 时框架置 abort 标志，本循环负责看到并退出。
 * ------------------------------------------------------------------------- */
void test_ledc_breathing()
{
    breath_pwm_init();
    ESP_LOGI(TAG, "呼吸灯启动 (GPIO%d, 20kHz/10bit), 按 x 退出", BREATH_PIN);

    // 档位
    int duty = 0;   // ⚠ 用有符号 int 而非 uint32_t：
                    //   无符号数在 duty=0/1 时执行 duty-2 会回绕成 ~42 亿，
                    //   下一行"越界夹紧"会把它钳到 1023——灯跳最亮而不是折返。
                    //   有符号数在 0 处减出负数，被 duty<=0 正常捕获折返。
    int dir = 1;    // 呼吸方向：+1 渐亮，-1 渐暗，到两端折返（三角波）

    while (!test_should_abort()) {   // 每步开头查中止标志（框架合同，5ms 内响应）

        // ---- 必查项②：set 写影子寄存器，update 才生效，两句必须成对出现 ----
        ledc_set_duty(BREATH_MODE, BREATH_CH, static_cast<uint32_t>(duty));
        ledc_update_duty(BREATH_MODE, BREATH_CH);

        // ---- 计算下一步位置并做两端折返 ----
        duty += static_cast<int>(dir * BREATH_DUTY_STEP);
        // 必查项③：越界先夹紧再换向，保证 duty 恒在 [0, 1023]
        //（不夹紧：负数/超 1023 的值喂给 ledc_set_duty 属越界，行为未定义）
        if (duty >= static_cast<int>(BREATH_DUTY_MAX)) {
            duty = static_cast<int>(BREATH_DUTY_MAX);  // 顶到头：钉在 1023
            dir = -1;                                  // 转为渐暗
        } else if (duty <= 0) {
            duty = 0;                                  // 底到头：钉在 0
            dir = 1;                                   // 转为渐亮
        }

        // ---- 阻塞让出 CPU：这是本任务唯一的"呼吸帧率"来源 ----
        // 护栏：tick 率 100Hz 下不足 10ms 会算成 0 tick，vTaskDelay(0) = 不让出
        // → 空转饿死 IDLE → task watchdog 5 秒复位。取 max(1, ticks) 兜底。
        TickType_t ticks = pdMS_TO_TICKS(BREATH_STEP_MS);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }

    // ---- 清理（可重入合同）：停掉通道并固定输出 0 = 灯灭 ----
    // 不做的后果：退出后引脚继续输出最后一次占空比的 PWM，
    // 灯半亮"卡住"，且下次进入测试时起点状态错乱。
    ledc_stop(BREATH_MODE, BREATH_CH, 0);
    ESP_LOGI(TAG, "呼吸灯已停止, 返回菜单");
}

/* ============================================================================
 * 延伸思考（做完本实验后的可选挑战，答辩加分素材）：
 * 1. 视觉非线性：人眼对亮度感知近似对数，duty 线性变化看起来"暗端快、
 *    亮端慢"。真·平滑要用指数/伽马曲线（如 duty = 1023 × pow(t, 2.2)）。
 *    代价：浮点运算——注意本任务栈只有框架给的 4096 字节。
 * 2. 为什么这里用 vTaskDelay 定节拍、上个实验 led_task 用"接收超时当节拍"？
 *    区别在"等待期间有没有别的事可干"：呼吸灯没有→delay 是对的；
 *    led_task 要等换挡消息→超时等待一石二鸟。
 * 3. 第 2 课预告：PWMB=GPIO41 将用 CHANNEL_1 挂同一台 TIMER_0（同频
 *    双通道），先在这里预留认知位置。
 * ========================================================================== */
