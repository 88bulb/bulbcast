#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "bootloader_common.h"
#include "bulbcast.h"
#include "color.h"

uint8_t bulbcode[16] = {0};

static const char *TAG = "bulbcode";

uint32_t rgbmap[256] = {
    0,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,
    2,    2,    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,
    4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5,    5,
    5,    6,    6,    6,    6,    7,    7,    7,    7,    7,    8,    8,
    8,    9,    9,    9,    10,   10,   10,   11,   11,   11,   12,   12,
    13,   13,   14,   14,   15,   15,   16,   16,   17,   18,   18,   19,
    19,   20,   21,   22,   22,   23,   24,   25,   26,   27,   28,   29,
    30,   31,   32,   33,   34,   35,   37,   38,   39,   41,   42,   44,
    45,   47,   49,   51,   52,   54,   56,   58,   60,   62,   65,   67,
    69,   72,   75,   77,   80,   83,   86,   89,   92,   95,   99,   102,
    106,  110,  114,  118,  122,  127,  131,  136,  141,  146,  151,  157,
    162,  168,  174,  180,  187,  193,  200,  208,  215,  223,  231,  239,
    248,  257,  266,  275,  285,  296,  306,  317,  329,  341,  353,  365,
    379,  392,  406,  421,  436,  452,  468,  485,  502,  520,  539,  559,
    579,  599,  621,  643,  666,  690,  715,  741,  768,  795,  824,  854,
    884,  916,  949,  983,  1018, 1055, 1093, 1132, 1173, 1215, 1259, 1304,
    1351, 1400, 1450, 1502, 1556, 1612, 1670, 1730, 1793, 1857, 1924, 1993,
    2065, 2139, 2216, 2296, 2378, 2464, 2552, 2644, 2739, 2838, 2940, 3046,
    3155, 3269, 3386, 3508, 3634, 3765, 3900, 4041, 4186, 4337, 4493, 4654,
    4822, 4995, 5175, 5361, 5554, 5753, 5960, 6175, 6397, 6627, 6865, 7112,
    7368, 7633, 7908, 8192};


uint32_t rgbmap2[256] = {
    0,    1,    17,   33,   49,   66,   82,   98,   114,  130,  146,  162,
    179,  195,  211,  227,  243,  259,  275,  292,  308,  324,  340,  356,
    372,  389,  405,  421,  437,  453,  469,  486,  502,  518,  534,  550,
    567,  583,  599,  615,  631,  647,  664,  680,  696,  712,  728,  745,
    761,  777,  793,  810,  826,  842,  858,  874,  891,  907,  923,  939,
    956,  972,  988,  1005, 1021, 1037, 1053, 1070, 1086, 1102, 1119, 1135,
    1151, 1168, 1184, 1200, 1217, 1233, 1250, 1266, 1282, 1299, 1315, 1332,
    1348, 1365, 1381, 1398, 1414, 1431, 1447, 1464, 1480, 1497, 1513, 1530,
    1547, 1563, 1580, 1597, 1613, 1630, 1647, 1664, 1680, 1697, 1714, 1731,
    1748, 1765, 1782, 1799, 1816, 1833, 1850, 1867, 1884, 1901, 1919, 1936,
    1953, 1970, 1988, 2005, 2023, 2040, 2058, 2076, 2093, 2111, 2129, 2147,
    2165, 2183, 2201, 2219, 2237, 2255, 2274, 2292, 2311, 2329, 2348, 2367,
    2386, 2405, 2424, 2443, 2462, 2482, 2501, 2521, 2541, 2561, 2581, 2601,
    2622, 2642, 2663, 2684, 2705, 2726, 2747, 2769, 2791, 2813, 2835, 2857,
    2880, 2903, 2926, 2949, 2973, 2997, 3021, 3045, 3070, 3095, 3121, 3147,
    3173, 3199, 3226, 3253, 3281, 3309, 3337, 3366, 3396, 3425, 3456, 3487,
    3518, 3550, 3582, 3616, 3649, 3684, 3719, 3754, 3791, 3828, 3866, 3905,
    3944, 3984, 4026, 4068, 4111, 4155, 4200, 4246, 4293, 4341, 4391, 4442,
    4493, 4547, 4601, 4657, 4714, 4773, 4834, 4896, 4959, 5025, 5092, 5161,
    5232, 5305, 5380, 5457, 5536, 5617, 5701, 5788, 5876, 5968, 6062, 6159,
    6259, 6362, 6468, 6578, 6690, 6807, 6926, 7050, 7178, 7309, 7445, 7585,
    7729, 7878, 8032, 8191};

esp_reset_reason_t rst_reason = ESP_RST_UNKNOWN;

uint8_t boot_params[6] = {0};
uint32_t my_bit_field;

// see examples/peripherals/ledc/ledc_fade/main/ledc_fade_example_main.c
// for a real-world example using callback for fading end.
// int ledc_fading_chan_num = 0;

// true if fading, when fading, fading_start, fading_end, prev_base_color_rgb,
// next_base_color_rgb, prev_base_color_hsv, next_base_color_hsv are meaningful.
// Otherwise, only curr_base_color_rgb and curr_base_color_hsv are meaningful

bool base_color_fading = false;
bool base_color_fading_ccw = false;

TickType_t base_color_fading_start = 0;
TickType_t base_color_fading_end = 0;

rgb_t prev_base_color_rgb = {0}, curr_base_color_rgb = {0},
      next_base_color_rgb = {0};
hsv_t prev_base_color_hsv = {0}, curr_base_color_hsv = {0},
      next_base_color_hsv = {0};

typedef struct base_color_hue_swing_t {
    TickType_t start;
    TickType_t end;
    uint8_t hue_base;
    uint8_t hue_peak;
} base_color_hue_swing_t;

base_color_hue_swing_t base_color_hue_swing;
base_color_hue_swing_t* base_color_hub_swing_handle = NULL;

typedef struct base_color_swing_config_t {
    uint8_t h_amp;
    uint8_t h_amp_var;
    uint8_t v_amp;
    uint8_t v_amp_var;
    TickType_t period;
    TickType_t period_var;
} base_color_swing_config_t;

base_color_hue_swing_config_t base_color_hue_swing_config;
base_color_hue_swing_config_t *base_color_hue_swing_config_handle = NULL;

static void set_duty(uint32_t r, uint32_t g, uint32_t b, uint32_t c,
                     uint32_t w) {
    /*
        r = rgbmap2[r];
        g = rgbmap2[g];
        b = rgbmap2[b];
    */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    if (rst_reason != ESP_RST_DEEPSLEEP) {
        ESP_LOGI(
            TAG,
            "set color, hsv: %u %u %u, rgb: %u %u %u, rgbcw: %u %u %u %u %u",
            curr_base_color_hsv.h, curr_base_color_hsv.s, curr_base_color_hsv.v,
            curr_base_color_rgb.r, curr_base_color_rgb.g, curr_base_color_rgb.b,
            r, g, b, c, w);
    }
}

static void fade(uint32_t r, uint32_t g, uint32_t b, uint32_t c, uint32_t w,
                 uint32_t duration) {
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, c, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, w, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, LEDC_FADE_NO_WAIT);
}

void led_init() {
    /* initialize led */
    ledc_timer_config_t ledc_timer = {
//        .duty_resolution = LEDC_TIMER_14_BIT, // resolution of PWM duty
//        .freq_hz = 2500,                      // frequency of PWM signal
        .duty_resolution = LEDC_TIMER_8_BIT,    // resolution of PWM duty
        .freq_hz = 20000,                       // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t lc[5] = {
        {.channel = LEDC_CHANNEL_0,
         .intr_type = LEDC_INTR_FADE_END,
         .duty = 0,
         .gpio_num = 3,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_1,
         .intr_type = LEDC_INTR_FADE_END,
         .duty = 0,
         .gpio_num = 0,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_2,
         .intr_type = LEDC_INTR_FADE_END,
         .duty = 0,
         .gpio_num = 1,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_3,
         .intr_type = LEDC_INTR_FADE_END,
         .duty = 0,
         .gpio_num = 18,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_4,
         .intr_type = LEDC_INTR_FADE_END,
         .duty = 0,
         .gpio_num = 19,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
    };

    for (int ch = 0; ch < 5; ch++) {
        ledc_channel_config(&lc[ch]);
    }

    ledc_fade_func_install(0);
    set_duty(0, 128, 0, 0, 0);
}

uint8_t interpolate(uint8_t prev, uint8_t next, TickType_t elapsed,
                    TickType_t duration) {
    if (prev == next)
        return prev;
    int p = (int)prev;
    int n = (int)next;
    int e = (int)elapsed;
    int d = (int)duration;
    return (uint8_t)(p + (n - p) * e / d);
}

uint8_t interpolate_cw(uint8_t prev, uint8_t next, TickType_t elapsed,
                       TickType_t duration) {
    if (prev == next)
        return prev;
    int p = (int)prev;
    int n = (int)next;
    int e = (int)elapsed;
    int d = (int)duration;
    if (n < p)
        n += 256;
    return (uint8_t)(p + (n - p) * e / d);
}

uint8_t interpolate_ccw(uint8_t prev, uint8_t next, TickType_t elapsed,
                        TickType_t duration) {
    if (prev == next)
        return prev;
    int p = (int)prev;
    int n = (int)next;
    int e = (int)elapsed;
    int d = (int)duration;
    if (p < n)
        p += 256;
    return (uint8_t)(p + (n - p) * e / d);
}

void fade_base_color() {
    if (!base_color_fading) return;

    TickType_t start = base_color_fading_start;
    TickType_t end = base_color_fading_end;
    TickType_t now = xTaskGetTickCount();

    if (now >= base_color_fading_end) {
        base_color_fading = false;
        curr_base_color_hsv = next_base_color_hsv;
        curr_base_color_rgb = next_base_color_rgb;
        set_duty(curr_base_color_rgb.r, curr_base_color_rgb.g,
                 curr_base_color_rgb.b, 0, 0);
    } else {
        if (base_color_fading_ccw) {
            curr_base_color_hsv.h =
                interpolate_ccw(prev_base_color_hsv.h, next_base_color_hsv.h,
                                now - start, end - start);
        } else {
            curr_base_color_hsv.h =
                interpolate_cw(prev_base_color_hsv.h, next_base_color_hsv.h,
                               now - start, end - start);
        }
        curr_base_color_hsv.s =
            interpolate(prev_base_color_hsv.s, next_base_color_hsv.s,
                        now - start, end - start);
        curr_base_color_hsv.v =
            interpolate(prev_base_color_hsv.v, next_base_color_hsv.v,
                        now - start, end - start);

        curr_base_color_rgb = hsv2rgb_spectrum(curr_base_color_hsv);

        set_duty(curr_base_color_rgb.r, curr_base_color_rgb.g,
                 curr_base_color_rgb.b, 0, 0);
    }
}

void swing_base_color () {

}

void handle_bulbcode() {
    uint16_t cmd = bulbcode[0] * 256 + bulbcode[1];
    if (cmd == 0)
        return;

    // erase bulbcode;
    bulbcode[0] = 0;
    bulbcode[1] = 0;

    switch (cmd) {
    case 0x0002: {
        uint8_t opt = bulbcode[2];
        uint8_t r = bulbcode[3];
        uint8_t g = bulbcode[4];
        uint8_t b = bulbcode[5];
        uint32_t dur_ms;
        TickType_t dur;

        if (opt & 0x01) {
            dur_ms = (bulbcode[6] * 256 + bulbcode[7]) * 1000;
        } else {
            dur_ms = (bulbcode[6] * 256 + bulbcode[7]) * 100;
        }

        if (opt & 0x02) {
            base_color_fading_ccw = true;
        } else {
            base_color_fading_ccw = false;
        }

        dur = dur_ms / portTICK_PERIOD_MS;

        ESP_LOGI(TAG,
                 "base color fade in, opt: %02x, r: %02x, g: %02x, b: "
                 "%02x, fade-in duration: %u ticks, or %ums",
                 opt, r, g, b, dur, dur_ms);

        rgb_t rgb;
        hsv_t hsv;

        if (opt & 0x80) {
            hsv.h = r;
            hsv.s = g;
            hsv.v = b;
            rgb = hsv2rgb_spectrum(hsv);
        } else {
            rgb.r = r;
            rgb.g = g;
            rgb.b = b;
            hsv = rgb2hsv_approximate(rgb);
        }

        if (dur == 0) {
            base_color_fading = false;
            curr_base_color_rgb = rgb;
            curr_base_color_hsv = hsv;
            set_duty(rgb.r, rgb.g, rgb.b, 0, 0);
        } else {
            base_color_fading = true;
            base_color_fading_start = xTaskGetTickCount();
            base_color_fading_end = base_color_fading_start + dur;
            prev_base_color_rgb = curr_base_color_rgb;
            prev_base_color_hsv = curr_base_color_hsv;
            next_base_color_rgb = rgb;
            next_base_color_hsv = hsv;
        }
    } break;
    case 0x0003: {
        // uint8_t opt = bulbcode[2];
        base_color_hue_swing_config.h_amp = bulbcode[3];
        base_color_hue_swing_config.h_amp_var = 0;
        base_color_hue_swing_config.v_amp = bulbcode[4];
        base_color_hue_swing_config.v_amp_var = 0;
        base_color_hue_swing_config.period = bulbcode[5] / portTICK_PERIOD_MS;
        base_color_hue_swing_config.period_var = 0;

        base_color_hue_swing_config_handle = &base_color_hue_swing_config;

        if (base_color_hue_swing_handle == NULL) {
            TickType_t now = xTaskGetTickCount();
            base_color_hue_swing.start = now;
            base_color_hue_swing.end = now + base_color_hue_swing_config.period;
            
        }
    } break;
    default:
        break;
    }
}

void app_main(void) {
    // TODO print
    rst_reason = esp_reset_reason();

    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    led_init();

    if (rst_reason == ESP_RST_DEEPSLEEP) {
        rtc_retain_mem_t *rtc_mem = bootloader_common_get_rtc_retain_mem();
        uint8_t *custom = rtc_mem->custom;
        boot_params[0] = custom[0];
        boot_params[1] = custom[1];
        boot_params[2] = custom[2];
        boot_params[3] = custom[3];
        boot_params[4] = custom[4];
        boot_params[5] = custom[5];
    } else {
        boot_params[0] = 0xa5;
        boot_params[1] = 0xa5;
        boot_params[2] = 0xa5;
        boot_params[3] = 0xa5;
        boot_params[4] = 0x02;
        boot_params[5] = 0x00;
    }

    my_bit_field = (1 << boot_params[5]);

    ESP_LOGI(TAG, "boot_params: %02x %02x %02x %02x %02x %02x", boot_params[0],
             boot_params[1], boot_params[2], boot_params[3], boot_params[4],
             boot_params[5]);
    ESP_LOGI(TAG, "my_bit_field: %08x", my_bit_field);
    ESP_LOGI(TAG, "portTICK_PERIOD_MS: %u", portTICK_PERIOD_MS);

    xTaskCreate(&ble_adv_scan, "ble", 4096, NULL, 6, NULL);

    while (1) {
        vTaskDelay(1);
        if (base_color_fading) {
            fade_base_color();
        } else {
            swing_base_color();
        } 

        handle_bulbcode();
    }
}
