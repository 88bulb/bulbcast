#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"

#include "driver/ledc.h"
#include "color.h"
#include "led.h"
#include "bulbcast.h"

#define TAG ("bulb light")

/**
 * LEDC_TIMER_13_BIT is preferred in offical example code
 * for which r,g,b value ranges over 0 to 8191
 */
#define DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define MAX_DUTY ((1 << DUTY_RESOLUTION) - 1)

#define USE_WARM_WHITE

#ifdef USE_WARM_WHITE
#define CHANNEL_NUM (5)
#else
#define CHANNEL_NUM (4)
#endif

static SemaphoreHandle_t counting_sem;
hsv_t curr_color = {0};
uint8_t curr_tint = 0;

static bool initialized = false;

static bool cb_ledc_fade_end_event(const ledc_cb_param_t *param,
                                   void *user_arg) {
    portBASE_TYPE taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t)user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

void led_init() {
    counting_sem = xSemaphoreCreateCounting(CHANNEL_NUM, 0);

    /* prepare timer */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = DUTY_RESOLUTION,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t channel_configs[CHANNEL_NUM] = {
        {.channel = LEDC_CHANNEL_0,
         .duty = 0,
         .gpio_num = 3,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_1,
         .duty = 0,
         .gpio_num = 0,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_2,
         .duty = 0,
         .gpio_num = 1,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
#ifdef USE_WARM_WHITE
        {.channel = LEDC_CHANNEL_3,
         .duty = 0,
         .gpio_num = 18,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
#endif
        {.channel = LEDC_CHANNEL_4,
         .duty = 0,
         .gpio_num = 19,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
    };

    for (int ch = 0; ch < CHANNEL_NUM; ch++) {
        ledc_channel_config(&channel_configs[ch]);
    }

    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
        .fade_cb = cb_ledc_fade_end_event,
    };

    for (int ch = 0; ch < CHANNEL_NUM; ch++) {
        ledc_cb_register(channel_configs[ch].speed_mode,
                         channel_configs[ch].channel, &callbacks,
                         (void *)counting_sem);
    }

    /*
     * there is a glitch for warm white comes slightly later than 
     * cold white, so we don't use warm white here
     */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    for (int i = 1; i < 5; i++) {
        uint32_t duty = (4 - i) * MAX_DUTY / 4;
        uint32_t max_fade_time_ms = 400 / i; 
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty,
                                max_fade_time_ms);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, LEDC_FADE_NO_WAIT);
        xSemaphoreTake(counting_sem, portMAX_DELAY);
    }

    vTaskDelay(400 / portTICK_PERIOD_MS);
}

void led_fade(uint32_t r, uint32_t g, uint32_t b, uint32_t w, uint32_t c,
              uint32_t max_fade_time_ms) {
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r,
                            max_fade_time_ms);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g,
                            max_fade_time_ms);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b,
                            max_fade_time_ms);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
#ifdef USB_WARM_WHITE
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, w,
                            max_fade_time_ms);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);
#endif
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, c,
                            max_fade_time_ms);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, LEDC_FADE_NO_WAIT);

    for (int i = 0; i < CHANNEL_NUM; i++) {
        xSemaphoreTake(counting_sem, portMAX_DELAY);
    }
}

void led_set_duty(uint32_t r, uint32_t g, uint32_t b, uint32_t w, uint32_t c) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
#ifdef USE_WARM_WHITE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void paint(hsv_t hsv, TickType_t ticks) {
    uint32_t r, g, b, c;
#ifdef USE_WARM_WHITE
    uint32_t w;
#endif

    if (!initialized) {
        led_init();
        initialized = true;
    }

    if (ticks <= 0) ticks = 1;

    if (hsv.h == curr_color.h && hsv.s == curr_color.s &&
        hsv.v == curr_color.v) {
        return vTaskDelay(ticks);
    }

    TickType_t start = xTaskGetTickCount();

    hsv_t tint = hsv;
    uint8_t unsat = 0xff - tint.s;
    c = ((uint32_t)unsat) / 8;
#ifdef USE_WARM_WHITE
    w = ((uint32_t)unsat) / 8;
#endif
    tint.s = 0xff;

    // if unsat = 0x00, then tint.v is unchanged
    // if unsat = 0xff, then tint.v is 0 and all brightness comes from
    // white led
    tint.v -= (uint8_t)(((uint32_t)unsat * (uint32_t)tint.v) / (uint32_t)0xff);

    rgb_t rgb = hsv2rgb_spectrum(tint);
    r = (uint32_t)rgb.r * 3;
    g = (uint32_t)rgb.g * 3;
    b = (uint32_t)rgb.b / 2;

    int time = ticks * portTICK_PERIOD_MS;

    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r, time);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g, time);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b, time);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
#ifdef USE_WARM_WHITE
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, w, time);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);
#endif
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, c, time);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, LEDC_FADE_NO_WAIT);

    for (int i = 0; i < CHANNEL_NUM; i++) {
        xSemaphoreTake(counting_sem, portMAX_DELAY);
    }

    TickType_t end = xTaskGetTickCount();
    ESP_LOGI(TAG,
             "\t[%08x-%08x] +%d, hsv: %02x %02x %02x, "
             "rgbc: %02x %02x %02x %02x",
             end, start, end - start, hsv.h, hsv.s, hsv.v, r, g, b, c);

    curr_color = hsv;
}

void draw(hsv_t hsv) {
    uint32_t r, g, b, c;
#ifdef USE_WARM_WHITE
    uint32_t w;
#endif

    if (!initialized) {
        led_init();
        initialized = true;
    }

    if (hsv.h == curr_color.h && hsv.s == curr_color.s &&
        hsv.v == curr_color.v) {
        return;
    }

    hsv_t tint = hsv;
    uint8_t unsat = 0xff - tint.s;
    c = ((uint32_t)unsat) / 8;
#ifdef USE_WARM_WHITE
    w = ((uint32_t)unsat) / 8;
#endif
    tint.s = 0xff;

    // if unsat = 0x00, then tint.v is unchanged
    // if unsat = 0xff, then tint.v is 0 and all brightness comes from
    // white led
    tint.v -= (uint8_t)(((uint32_t)unsat * (uint32_t)tint.v) / (uint32_t)0xff);

    rgb_t rgb = hsv2rgb_spectrum(tint);
    r = (uint32_t)rgb.r * 3;
    g = (uint32_t)rgb.g * 3;
    b = (uint32_t)rgb.b / 2;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
#ifdef USE_WARM_WHITE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    curr_color = hsv;
}

void flash(int ms) {
    if (ms > 10)
        ms = 10;

#ifdef USE_WARM_WHITE
    uint32_t w = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    uint32_t c = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    int64_t htime = esp_timer_get_time();
    while (esp_timer_get_time() - htime < 1000 * ms) {
    }

#ifdef USE_WARM_WHITE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void flash_max() {

    ESP_LOGI(TAG, "flash_max");

#ifdef USE_WARM_WHITE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void flash_min() {

    ESP_LOGI(TAG, "flash_max");

#ifdef USE_WARM_WHITE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}
