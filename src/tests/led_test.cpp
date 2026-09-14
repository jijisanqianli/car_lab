/* 测试项: 板载 WS2812 RGB 灯心跳
 * DevKitC-1 板载 RGB 灯引脚: 板卡 v1.0 = GPIO48, v1.1 = GPIO38, 按手上板子修改
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// led_strip 是纯 C 组件, 用 C++ 编译时用 extern "C" 包裹 include 最稳妥
extern "C" {
#include "led_strip.h"
}

#include "test_framework.h"

static const char* TAG = "led";
static constexpr int LED_GPIO = 48; /* 板卡 v1.1 改为 38 */
static constexpr int beat_delay_ms[4] = {100, 100, 100, 700}; /* 亮-灭-亮-长停 */

void test_led()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = LED_GPIO;
    strip_config.max_leds = 1;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; /* 10MHz RMT 时钟 */

    led_strip_handle_t strip = nullptr;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    led_strip_clear(strip);
    ESP_LOGI(TAG, "WS2812 heartbeat on GPIO%d", LED_GPIO);

    int step = 0;
    while (!test_should_abort()) {
        if (step == 0 || step == 2) {
            led_strip_set_pixel(strip, 0, 12, 0, 0); /* 暗红色 */
        } else {
            led_strip_clear(strip);
        }
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(beat_delay_ms[step]));
        step = (step + 1) % 4;
    }

    // 把led重置为关
    led_strip_clear(strip);

    led_strip_del(strip); /* 释放 RMT 资源, 保证测试可重复进入 */
    ESP_LOGI(TAG, "led test exited");
}
