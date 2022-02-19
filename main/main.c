#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "bootloader_common.h"

#include "bulbcast.h"
#include "led.h"

#include "color.h"
#include "stm.h"

#define DOUBLE(x) ((x) + ((x) << 4))

static int64_t play_noise_start = -1;

static int noise[] = {
    13,   94,   839,  2302, 909,  777,  6326, 5658, 7460, 9210, 8116, 6338,
    8053, 8113, 8084, 7407, 5345, 8063, 6946, 9219, 8872, 9618, 8109, 8517,
    9188, 3011, 6885, 7902, 8707, 7567, 6823, 8087, 8575, 7623, 8575, 6356,
    5889, 1500, 7459, 6475, 8105, 5133, 6026, 8644, 4705, 1267, 8591, 6687,
    780,  845,  4227, 1358, 5613, 4082, 8658, 8626, 8603, 8722, 8102, 9623,
    8057, 7535, 6223, 8525, 8119, 6489, 9683, 6856, 8526, 9734, 6891, 8061,
    8121, 8364, 7456, 5716, 4712, 920,  6020, 1478, 3631, 2511, 2867, 2967,
    2435, 3095, 5183, 2044, 2520, 6949, 4536, 5732, 3648, 952,  917,  6889,
    3085, 748,  930,  856,  863,  804,  2434, 1859, 5684, 7539, 2338, 846,
    904,  856,  751,  726,  860,  959,  851,  792,  678,  629,  365,  298,
    471,  293,  94,   95,   96,   95,   94,   95,   82,   97,   73,   95,
    96,   96,   97,   25};

static int noise_db[] = {
    11, 20, 29, 34, 30, 29, 38, 38, 39, 40, 39, 38, 39, 39, 39, 39, 37,
    39, 38, 40, 39, 40, 39, 39, 40, 35, 38, 39, 39, 39, 38, 39, 39, 39,
    39, 38, 38, 32, 39, 38, 39, 37, 38, 39, 37, 31, 39, 38, 29, 29, 36,
    31, 37, 36, 39, 39, 39, 39, 39, 40, 39, 39, 38, 39, 39, 38, 40, 38,
    39, 40, 38, 39, 39, 39, 39, 38, 37, 30, 38, 32, 36, 34, 35, 35, 34,
    35, 37, 33, 34, 38, 37, 38, 36, 30, 30, 38, 35, 29, 30, 29, 29, 29,
    34, 33, 38, 39, 34, 29, 30, 29, 29, 29, 29, 30, 29, 29, 28, 28, 26,
    25, 27, 25, 20, 20, 20, 20, 20, 20, 19, 20, 19, 20, 20, 20, 20, 14};

static int crest[] = {
    0,   9,   47,  107, 95,  127, 141, 151, 179, 283, 421, 232, 339, 336,
    369, 263, 233, 368, 229, 350, 317, 270, 319, 327, 250, 172, 247, 420,
    355, 256, 364, 265, 403, 334, 283, 193, 188, 183, 207, 167, 197, 181,
    142, 195, 180, 110, 179, 162, 81,  62,  120, 98,  155, 121, 219, 239,
    397, 313, 273, 295, 247, 345, 204, 332, 261, 279, 241, 210, 337, 350,
    306, 277, 294, 347, 186, 136, 168, 47,  161, 93,  110, 80,  123, 112,
    113, 98,  140, 108, 119, 185, 160, 173, 87,  108, 41,  188, 116, 38,
    25,  27,  39,  46,  92,  100, 132, 172, 119, 71,  59,  53,  50,  55,
    55,  52,  46,  26,  17,  14,  12,  11,  13,  10,  10,  7,   6,   5,
    6,   6,   3,   4,   3,   4,   3,   4,   4,   6,   0};

uint8_t boot_params[16] = {0};
uint32_t my_bit_field;
static const char *TAG = "bulbcode";

typedef enum {
    PRIMARY_COLOR,
    HSV_TRANSITION,
    RANDOM_TRAPEZOID,
    CYCLE_COLOR,
    FAST_FLASH,
} base_layer_type_t;

typedef struct {
    hsv_t from;
    hsv_t to;
    hsv_hue_dir_t dir;
} hsv_transition_t;

typedef struct {
    int64_t tmpl_cycle_min;
    int64_t tmpl_cycle_max;
    uint8_t tmpl_recycle_min;
    uint8_t tmpl_recycle_max;
    uint8_t tmpl_bottom_min;
    uint8_t tmpl_bottom_max;
    uint8_t tmpl_top_min;
    uint8_t tmpl_top_max;
    uint8_t tmpl_up_min;
    uint8_t tmpl_up_max;
    uint8_t tmpl_hue1_min;
    uint8_t tmpl_hue1_max;
    uint8_t tmpl_sat1_min;
    uint8_t tmpl_sat1_max;
    uint8_t tmpl_val1_min;
    uint8_t tmpl_val1_max;
    uint8_t tmpl_hue2_min;
    uint8_t tmpl_hue2_max;
    uint8_t tmpl_sat2_min;
    uint8_t tmpl_sat2_max;
    uint8_t tmpl_val2_min;
    uint8_t tmpl_val2_max;

    int countdown; // for recycle

    int64_t p0; // p0 <= x < p1 -> bottom
    int64_t p1; // p1 <= x < p2 -> up
    int64_t p2; // p2 <= x < p3 -> top
    int64_t p3; // p3 <= x < p4 -> down
    int64_t p4; // end
    hsv_t c0;
    hsv_t c1;
    hsv_t c2; // next c0
} random_trapezoid_t;

typedef struct {
    hsv_t color;
    int cycle_us;
} cycle_color_t;

typedef struct {
    int on_us;
    int tt_us; // tt for total
    int r;
    int g;
    int b;
    int c;
    int w;
} fast_flash_t;

typedef struct {
    base_layer_type_t type;
    int64_t start;
    int64_t end;
    hsv_t color;
    union {
        hsv_transition_t hsv_transition;
        random_trapezoid_t rand_trap;
        cycle_color_t cycle_color;
        fast_flash_t fast_flash;
    } data;
} base_layer_t;

const base_layer_t default_base = {
    .type = PRIMARY_COLOR,
    .color =
        {
            .h = 0x20,
            .s = 0x40,
            .v = 0xc0,
        },
};

static base_layer_t curr_base = default_base;
static base_layer_t next_base = default_base;
static esp_timer_handle_t periodic_timer = {0};
static bool periodic_timer_started = false;

/**
 * start timer
 */
static void start_periodic_timer() {
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));
    periodic_timer_started = true;
}

/**
 * stop timer, could be called if timer not startd
 */
static void stop_periodic_timer() {
    if (periodic_timer_started) {
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
        periodic_timer_started = false;
    }
}

static void shuffle_random_trapezoid(random_trapezoid_t *rt, int64_t now) {
    uint32_t rand = esp_random();
    int rand0 = (rand >> 0) & 0xff;
    int rand1 = (rand >> 1) & 0xff;
    int rand2 = (rand >> 2) & 0xff;
    int rand3 = (rand >> 3) & 0xff;
    int rand4 = (rand >> 4) & 0xff;
    int rand5 = (rand >> 5) & 0xff;
    int rand6 = (rand >> 6) & 0xff;

    int64_t cycle = rt->tmpl_cycle_min +
                    (rt->tmpl_cycle_max - rt->tmpl_cycle_min) * rand0 / 256;

    int64_t bottom_min = rt->tmpl_bottom_min * cycle / 16;
    int64_t bottom_max = rt->tmpl_bottom_max * cycle / 16;
    int64_t bottom = bottom_min + (bottom_max - bottom_min) * rand1 / 256;

    int64_t top_min = rt->tmpl_top_min * bottom / 16;
    int64_t top_max = rt->tmpl_top_max * bottom / 16;
    int64_t top = top_min + (top_max - top_min) * rand2 / 256;

    int64_t up_min = rt->tmpl_up_min * (bottom - top) / 16;
    int64_t up_max = rt->tmpl_up_max * (bottom - top) / 16;
    int64_t up = up_min + (up_max - up_min) * rand3 / 256;

    rt->p0 = now;
    rt->p1 = now + cycle - bottom;
    rt->p2 = rt->p1 + up;
    rt->p3 = rt->p2 + top;
    rt->p4 = now + cycle;
    /**
        ESP_LOGI(TAG, "cycle_min: %lld, cycle_max: %lld, cycle: %lld",
                 rt->tmpl_cycle_min / 1000000, rt->tmpl_cycle_max / 1000000,
                 cycle / 1000000);
        ESP_LOGI(TAG, "shuffled time pattern, %lld %lld %lld %lld %lld",
            rt->p0, rt->p1, rt->p2, rt->p3, rt->p4); */

    rt->c0 = rt->c2;

    hsv_t c1;
    c1.h = rt->tmpl_hue2_min +
           (rt->tmpl_hue2_max - rt->tmpl_hue2_min) * rand4 / 256;
    c1.s = rt->tmpl_sat2_min +
           (rt->tmpl_sat2_max - rt->tmpl_sat2_min) * rand5 / 256;
    c1.v = rt->tmpl_val2_min +
           (rt->tmpl_val2_max - rt->tmpl_val2_min) * rand6 / 256;
    rt->c1 = c1;

    hsv_t c2;
    c2.h = rt->tmpl_hue1_min +
           (rt->tmpl_hue1_max - rt->tmpl_hue1_min) * rand4 / 256;
    c2.s = rt->tmpl_sat1_min +
           (rt->tmpl_sat1_max - rt->tmpl_sat1_min) * rand5 / 256;
    c2.v = rt->tmpl_val1_min +
           (rt->tmpl_val1_max - rt->tmpl_val1_min) * rand6 / 256;
    rt->c2 = c2;
}

/**
 *
 */
static void render() {
    int64_t now = esp_timer_get_time();
    if (curr_base.end != 0 && curr_base.end < now) {
        curr_base = next_base;
        next_base = default_base;
    }

    switch (curr_base.type) {
    case PRIMARY_COLOR:
        break;
    case HSV_TRANSITION: {
        curr_base.color = interp_color(curr_base.data.hsv_transition.from,
                                       curr_base.data.hsv_transition.to,
                                       curr_base.data.hsv_transition.dir,
                                       curr_base.start, curr_base.end, now);
    } break;
    case RANDOM_TRAPEZOID: {
        random_trapezoid_t *p = &curr_base.data.rand_trap;
        if (now >= p->p0 && now < p->p1) {
            curr_base.color = p->c0;
        } else if (now >= p->p1 && now < p->p2) {
            curr_base.color =
                interp_color(p->c0, p->c1, INTERP_HUE_AUTO, p->p1, p->p2, now);
        } else if (now >= p->p2 && now < p->p3) {
            curr_base.color = p->c1;
        } else if (now >= p->p3 && now < p->p4) {
            curr_base.color =
                interp_color(p->c1, p->c2, INTERP_HUE_AUTO, p->p3, p->p4, now);
        } else {
            int64_t before_shuffle = esp_timer_get_time();
            shuffle_random_trapezoid(p, now);
            int64_t after_shuffle = esp_timer_get_time();
            ESP_LOGI(TAG, "shuffled in %lld micros",
                     after_shuffle - before_shuffle);
            curr_base.color = p->c0;
        }
    } break;

    case CYCLE_COLOR: {
        int64_t elapsed_time_us = now - curr_base.start;
        int64_t cycle_us = curr_base.data.cycle_color.cycle_us;
        int64_t modulo_us = elapsed_time_us % cycle_us;
        uint8_t delta_hue = (uint8_t)(modulo_us * 256 / cycle_us);

        curr_base.color = curr_base.data.cycle_color.color;
        curr_base.color.hue += delta_hue;
    } break;

    case FAST_FLASH: {
        fast_flash_t *p = &curr_base.data.fast_flash;
        int64_t elapsed_time_us = now - curr_base.start;
        int64_t modulo_us = elapsed_time_us % p->tt_us;
        if (modulo_us <= p->on_us) {
            direct_draw(p->r, p->g, p->b, p->c, p->w);
        } else {
            direct_draw(0, 0, 0, 0, 0);
        }
    } break;

    default:
        break;
    }

    if (curr_base.type != FAST_FLASH) {
        draw(curr_base.color);
    }
}

static void timer_callback(void *arg) {
    static int last_noise_index = -1;

    if (play_noise_start >= 0) {
        int64_t now = esp_timer_get_time();
        int elapsed = now - play_noise_start;
        int index = elapsed / (1024 * 1000000 / 44100);
        if (index == last_noise_index)
            return;

        ESP_LOGI(TAG, "now %lld, size %d, noise index %d", now,
                 sizeof(noise_db), index);

        if (index < sizeof(noise_db) / sizeof(int)) {
            hsv_t color;
            uint8_t x = noise_db[index] * 7 - 25;
            color.h = 0;
            color.s = x;
            color.v = x;
            draw(color);
            last_noise_index = index;
        } else {
            last_noise_index = -1;
            play_noise_start = -1;
            stop_periodic_timer();
            hsv_t color;
            color.h = 0;
            color.s = 0;
            color.v = 0;
            draw(color);
        }
        return;
    }

    const static int __default_countdown = 100;
    static int __index = 1;
    static int __countdown = __default_countdown;
    static int __sum = 0;
    static int __first = 0;
    static int __second = 0;
    static int __third = 0;

    int64_t __start = esp_timer_get_time();

    render();

    int64_t __end = esp_timer_get_time();
    int __duration = __end - __start;
    __sum += __duration;

    if (__first < __duration) {
        __third = __second;
        __second = __first;
        __first = __duration;
    } else if (__second < __duration) {
        __third = __second;
        __second = __duration;
    } else if (__third < __duration) {
        __third = __duration;
    }

    __countdown--;

    if (__countdown == 0) {
        ESP_LOGI("timer cb",
                 "%d times average (round: %d): %d micros, max: %d, second: "
                 "%d, third: %d",
                 __default_countdown, __index, __sum / __default_countdown,
                 __first, __second, __third);
        __countdown = __default_countdown;
        __sum = 0;
        __first = 0;
        __second = 0;
        __third = 0;
        __index++;
    }
}

void handle_bulbcode(uint16_t cmd, const uint8_t code[13]) {
    stop_periodic_timer();

    switch (cmd) {
    case 0x1002: {
        uint8_t opt = code[0];
        int64_t dur =
            ((opt & 0x01) ? 10000000 : 100000) * (code[1] * 256 + code[2]);

        hsv_t hsv;
        hsv.h = code[3];
        hsv.s = code[4];
        hsv.v = code[5];

        if (dur == 0) {
            curr_base = default_base;
            next_base = default_base;
            curr_base.color = hsv;
        } else {
            hsv_hue_dir_t dir;
            /* if set, cw or ccw is determined by next bit */
            if (opt & 0x02) {
                if (opt & 0x04) {
                    dir = INTERP_HUE_CCW;
                } else {
                    dir = INTERP_HUE_CW;
                }
            } else {
                dir = INTERP_HUE_AUTO;
            }

            int64_t now = esp_timer_get_time();
            /* curr_base.color unchanged */
            curr_base.type = HSV_TRANSITION;
            curr_base.start = now;
            curr_base.end = now + dur - 1;
            curr_base.data.hsv_transition.from = curr_base.color;
            curr_base.data.hsv_transition.to = hsv;
            curr_base.data.hsv_transition.dir = dir;

            next_base.type = PRIMARY_COLOR;
            next_base.start = 0;
            next_base.end = 0;
            next_base.color = hsv;
        }
    } break;

    /**
     *  option      3 byte
     *  cycle       1 byte (sec)
     *  bottom      1 byte (max/min)
     *  top         1 byte (max/min)
     *  up          1 byte (max/min)
     *  hue         1 byte (max/min)
     *  sat         1 byte (max/min)
     *  value       1 byte (max/min)
     *  hue         1 byte (max/min)
     *  sat         1 byte (max/min)
     *  value       1 byte (max/min)
     *  ----------------------------
     *  total      13 bytes
     */
    case 0x1003: {

        ESP_LOGI(TAG, "0x1003 (random trapezoid) received");

        int64_t now = esp_timer_get_time();
        base_layer_t *p = &curr_base;
        p->type = RANDOM_TRAPEZOID;
        p->start = now;
        p->end = 0;
        random_trapezoid_t *q = &p->data.rand_trap;
        q->tmpl_cycle_min = code[3] * 1000000;
        //        q->tmpl_cycle_max = code[2] * 1000000 * 4 / 3; // TODO
        q->tmpl_cycle_max = code[3] * 1000000; // TODO
        q->tmpl_recycle_min = 1;
        q->tmpl_recycle_max = 3;
        q->tmpl_bottom_min = code[4] & 0x0f;
        q->tmpl_bottom_max = code[4] >> 4;
        q->tmpl_top_min = code[5] & 0x0f;
        q->tmpl_top_max = code[5] >> 4;
        q->tmpl_up_min = code[6] & 0x0f;
        q->tmpl_up_max = code[6] >> 4;
        q->tmpl_hue1_min = DOUBLE(code[7] & 0x0f);
        q->tmpl_hue1_max = DOUBLE(code[7] >> 4);
        q->tmpl_sat1_min = DOUBLE(code[8] & 0x0f);
        q->tmpl_sat1_max = DOUBLE(code[8] >> 4);
        q->tmpl_val1_min = DOUBLE(code[9] & 0x0f);
        q->tmpl_val1_max = DOUBLE(code[9] >> 4);
        q->tmpl_hue2_min = DOUBLE(code[10] & 0x0f);
        q->tmpl_hue2_max = DOUBLE(code[10] >> 4);
        q->tmpl_sat2_min = DOUBLE(code[11] & 0x0f);
        q->tmpl_sat2_max = DOUBLE(code[11] >> 4);
        q->tmpl_val2_min = DOUBLE(code[12] & 0x0f);
        q->tmpl_val2_max = DOUBLE(code[12] >> 4);

        shuffle_random_trapezoid(q, now);
    } break;

    /**
     * option       1 byte
     * fade in dur  1 byte (max)
     * fade in dur  1 byte (min)
     * cycle        1 byte (max)
     * cycle        1 byte (min)
     * hue          1 byte
     * sat          1 byte
     * value        1 byte
     * padding      5 bytes (0000000000)
     * -------------------------
     * total       13 bytes
     *
     * example: 5-second cycle, start from red, max sat and brightness
     *
     * b01bc0de00a5a5a5a5ffff1005000000000400ffff0000000000
     */
    case 0x1004: {

        ESP_LOGI(TAG, "0x1004 received");

        // uint8_t opt = code[0];
        // uint8_t dur_max = code[1];
        // uint8_t dur_min = code[2];
        int64_t dur = 0;
        // int cycle_max = code[3];
        int cycle_min = code[4];
        int cycle = cycle_min;

        hsv_t hsv;
        hsv.h = code[5];
        hsv.s = code[6];
        hsv.v = code[7];

        if (dur == 0) {
            int64_t now = esp_timer_get_time();

            curr_base.type = CYCLE_COLOR;
            curr_base.start = now;
            curr_base.end = 0;

            cycle_color_t *q = &curr_base.data.cycle_color;
            q->color = hsv;
            q->cycle_us = (int64_t)cycle * 1000000;
        }
    } break;

    /**
     * on           1 byte (on time in msec)
     * off          2 bytes (off time in msec)
     * red          2 bytes
     * green        2 bytes
     * blue         2 bytes
     * cold white   2 bytes
     * warm white   2 bytes
     * -------------------------
     * total       13 bytes
     *
     * example: 20ms on, 200ms cycle,
     *
     * b01bc0de00a5a5a5a500ff10051400c8000000000000ffffffff
     */
    case 0x1005: {

        ESP_LOGI(TAG, "0x1005 receive");

        int on = code[0];
        int tt = code[1] * 256 + code[2];
        if (tt < on * 2)
            tt = on * 2;

        int r = (code[3] * 256 + code[4]) >> 4;
        int g = (code[5] * 256 + code[6]) >> 4;
        int b = (code[7] * 256 + code[8]) >> 4;
        int c = (code[9] * 256 + code[10]) >> 4;
        int w = (code[11] * 256 + code[12]) >> 4;

        int64_t now = esp_timer_get_time();

        // don't touch curr_base.color here
        curr_base.type = FAST_FLASH;
        curr_base.start = now;
        curr_base.end = 0;

        fast_flash_t *q = &curr_base.data.fast_flash;
        q->on_us = on * 1000;
        q->tt_us = tt * 1000;
        q->r = r;
        q->g = g;
        q->b = b;
        q->c = c;
        q->w = w;
    } break;

    default:
        break;
    }

    render();
    start_periodic_timer();
}

void app_main(void) {
    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_reset_reason_t rst_reason = esp_reset_reason();
    if (rst_reason == ESP_RST_DEEPSLEEP) {
        rtc_retain_mem_t *rtc_mem = bootloader_common_get_rtc_retain_mem();
        uint8_t *custom = rtc_mem->custom;
        boot_params[0] = custom[0];
        boot_params[1] = custom[1];
        boot_params[2] = custom[2];
        boot_params[3] = custom[3];
        boot_params[4] = custom[4];
        boot_params[5] = custom[5];
        boot_params[6] = custom[6];
        boot_params[7] = custom[7];
    } else {
        boot_params[0] = 0xa5;
        boot_params[1] = 0xa5;
        boot_params[2] = 0xa5;
        boot_params[3] = 0xa5;
        boot_params[4] = 0x02;
        boot_params[5] = 0x00;
        boot_params[6] = 0x80;
        boot_params[7] = 0x19;
    }

    my_bit_field = (1 << boot_params[5]);

    ESP_LOGI(TAG, "boot_params: %02x %02x %02x %02x %02x %02x", boot_params[0],
             boot_params[1], boot_params[2], boot_params[3], boot_params[4],
             boot_params[5]);
    ESP_LOGI(TAG, "my_bit_field: %08x", my_bit_field);
    ESP_LOGI(TAG, "portTICK_PERIOD_MS: %u", portTICK_PERIOD_MS);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &timer_callback,
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    for (int h = 0; h < 256; h++) {
        hsv_t hsv;
        hsv.h = (uint8_t)h;
        hsv.s = 0xff;
        hsv.v = 0xff;
        rgb_t rgb = hsv2rgb_spectrum(hsv);
        hue_max_rgb[h] = (rgb.r > rgb.g && rgb.r > rgb.b)
                             ? rgb.r
                             : (rgb.g > rgb.b) ? rgb.g : rgb.b;
    }

    fade(curr_base.color, 1000 / portTICK_PERIOD_MS);

    xTaskCreate(&ble_adv_scan, "ble", 4096, NULL, 6, NULL);

    //    vTaskDelay(3000 / portTICK_PERIOD_MS);
    //
    //    for (int i = 0; i < 10; i++) {
    //        play_noise_start = esp_timer_get_time();
    //        start_periodic_timer();
    //        vTaskDelay(8000 / portTICK_PERIOD_MS);
    //    }
    //
    vTaskDelay(portMAX_DELAY);
}
