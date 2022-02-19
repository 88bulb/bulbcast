#include <math.h>
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
#define DUTY_RESOLUTION LEDC_TIMER_12_BIT
#define MAX_DUTY ((1 << DUTY_RESOLUTION) - 1)

#define USE_WARM_WHITE

#ifdef USE_WARM_WHITE
#define CHANNEL_NUM (5)
#else
#define CHANNEL_NUM (4)
#endif

const int rgb4096[256] = {
    0,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,
    1,    1,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,
    2,    2,    2,    2,    2,    3,    3,    3,    3,    3,    3,    3,
    3,    3,    3,    3,    4,    4,    4,    4,    4,    4,    4,    5,
    5,    5,    5,    5,    5,    5,    6,    6,    6,    6,    6,    7,
    7,    7,    7,    8,    8,    8,    8,    9,    9,    9,    10,   10,
    10,   11,   11,   11,   12,   12,   12,   13,   13,   14,   14,   15,
    15,   16,   16,   17,   17,   18,   18,   19,   20,   20,   21,   22,
    22,   23,   24,   25,   26,   26,   27,   28,   29,   30,   31,   32,
    33,   34,   35,   37,   38,   39,   40,   42,   43,   45,   46,   48,
    49,   51,   53,   54,   56,   58,   60,   62,   64,   66,   68,   71,
    73,   75,   78,   80,   83,   86,   89,   92,   95,   98,   101,  105,
    108,  112,  115,  119,  123,  127,  132,  136,  140,  145,  150,  155,
    160,  165,  171,  177,  182,  189,  195,  201,  208,  215,  222,  229,
    237,  245,  253,  262,  270,  279,  289,  298,  308,  318,  329,  340,
    351,  363,  375,  388,  400,  414,  428,  442,  456,  472,  487,  504,
    520,  538,  556,  574,  593,  613,  633,  654,  676,  699,  722,  746,
    771,  796,  823,  850,  879,  908,  938,  969,  1002, 1035, 1069, 1105,
    1142, 1180, 1219, 1260, 1302, 1345, 1390, 1436, 1484, 1533, 1584, 1637,
    1692, 1748, 1806, 1866, 1928, 1992, 2059, 2127, 2198, 2271, 2347, 2425,
    2506, 2589, 2675, 2764, 2856, 2951, 3050, 3151, 3256, 3365, 3477, 3592,
    3712, 3835, 3963, 4095};

// =(C2-1)/254*30-30 for: 1 -> -30dB and 255 -> 0dB
// =10^(G2/10)*4095: -30dB -> 4 and 0dB -> 4095
const int grayscale_min_30db[256] = {
    0,    4,    4,    4,    4,    5,    5,    5,    5,    5,    5,    5,
    6,    6,    6,    6,    6,    6,    7,    7,    7,    7,    7,    7,
    8,    8,    8,    8,    9,    9,    9,    9,    10,   10,   10,   10,
    11,   11,   11,   12,   12,   12,   12,   13,   13,   14,   14,   14,
    15,   15,   16,   16,   16,   17,   17,   18,   18,   19,   19,   20,
    20,   21,   22,   22,   23,   23,   24,   25,   25,   26,   27,   27,
    28,   29,   30,   31,   31,   32,   33,   34,   35,   36,   37,   38,
    39,   40,   41,   42,   44,   45,   46,   47,   49,   50,   51,   53,
    54,   56,   57,   59,   60,   62,   64,   66,   67,   69,   71,   73,
    75,   77,   79,   82,   84,   86,   88,   91,   93,   96,   99,   101,
    104,  107,  110,  113,  116,  119,  123,  126,  129,  133,  137,  141,
    144,  148,  152,  157,  161,  165,  170,  175,  179,  184,  189,  195,
    200,  206,  211,  217,  223,  229,  236,  242,  249,  256,  263,  270,
    277,  285,  293,  301,  309,  318,  326,  335,  345,  354,  364,  374,
    384,  395,  406,  417,  428,  440,  452,  465,  478,  491,  504,  518,
    533,  547,  562,  578,  594,  610,  627,  644,  662,  680,  699,  718,
    738,  759,  779,  801,  823,  846,  869,  893,  918,  943,  969,  996,
    1023, 1051, 1080, 1110, 1141, 1172, 1204, 1238, 1272, 1307, 1343, 1380,
    1418, 1457, 1497, 1538, 1581, 1624, 1669, 1715, 1762, 1811, 1861, 1912,
    1965, 2019, 2075, 2132, 2191, 2251, 2313, 2377, 2443, 2510, 2579, 2650,
    2723, 2798, 2875, 2955, 3036, 3120, 3206, 3294, 3385, 3478, 3574, 3673,
    3774, 3878, 3985, 4095};

const int grayscale_0db_to_minus_27db[256] = {
    0,    8,    8,    9,    9,    9,    9,    9,    10,   10,   10,   10,
    11,   11,   11,   12,   12,   12,   12,   13,   13,   13,   14,   14,
    14,   15,   15,   15,   16,   16,   17,   17,   17,   18,   18,   19,
    19,   20,   20,   21,   21,   22,   22,   23,   23,   24,   25,   25,
    26,   26,   27,   28,   28,   29,   30,   31,   31,   32,   33,   34,
    35,   35,   36,   37,   38,   39,   40,   41,   42,   43,   44,   45,
    46,   48,   49,   50,   51,   52,   54,   55,   56,   58,   59,   61,
    62,   64,   65,   67,   69,   70,   72,   74,   76,   78,   80,   82,
    84,   86,   88,   90,   92,   94,   97,   99,   102,  104,  107,  109,
    112,  115,  118,  121,  124,  127,  130,  133,  136,  140,  143,  147,
    150,  154,  158,  162,  166,  170,  174,  178,  183,  187,  192,  197,
    202,  207,  212,  217,  222,  228,  234,  239,  245,  251,  258,  264,
    271,  277,  284,  291,  298,  306,  313,  321,  329,  337,  346,  354,
    363,  372,  381,  391,  400,  410,  420,  431,  442,  452,  464,  475,
    487,  499,  511,  524,  537,  550,  564,  578,  592,  607,  622,  637,
    653,  669,  686,  703,  720,  738,  756,  775,  794,  814,  834,  855,
    876,  898,  920,  943,  966,  990,  1015, 1040, 1066, 1092, 1119, 1147,
    1175, 1204, 1234, 1265, 1296, 1328, 1361, 1395, 1429, 1465, 1501, 1538,
    1576, 1616, 1656, 1697, 1739, 1782, 1826, 1871, 1917, 1965, 2014, 2064,
    2115, 2167, 2221, 2276, 2332, 2390, 2449, 2510, 2572, 2636, 2701, 2768,
    2837, 2907, 2979, 3053, 3128, 3206, 3285, 3367, 3450, 3536, 3623, 3713,
    3805, 3899, 3996, 4095};

const int grayscale_0db_to_minus_24db[256] = {
    0,    16,   17,   17,   17,   18,   18,   19,   19,   19,   20,   20,
    21,   21,   22,   22,   23,   23,   24,   24,   25,   25,   26,   26,
    27,   27,   28,   29,   29,   30,   31,   31,   32,   33,   33,   34,
    35,   36,   36,   37,   38,   39,   40,   41,   42,   42,   43,   44,
    45,   46,   47,   48,   49,   51,   52,   53,   54,   55,   56,   58,
    59,   60,   61,   63,   64,   66,   67,   69,   70,   72,   73,   75,
    76,   78,   80,   82,   83,   85,   87,   89,   91,   93,   95,   97,
    99,   101,  104,  106,  108,  111,  113,  116,  118,  121,  123,  126,
    129,  132,  135,  137,  141,  144,  147,  150,  153,  157,  160,  164,
    167,  171,  175,  178,  182,  186,  191,  195,  199,  203,  208,  212,
    217,  222,  227,  232,  237,  242,  247,  253,  258,  264,  270,  276,
    282,  288,  294,  301,  307,  314,  321,  328,  335,  343,  350,  358,
    366,  374,  382,  391,  399,  408,  417,  426,  436,  445,  455,  465,
    475,  486,  496,  507,  518,  530,  541,  553,  565,  578,  591,  604,
    617,  630,  644,  658,  673,  688,  703,  718,  734,  750,  767,  784,
    801,  819,  837,  855,  874,  893,  913,  933,  953,  974,  996,  1017,
    1040, 1063, 1086, 1110, 1134, 1159, 1185, 1211, 1238, 1265, 1293, 1321,
    1350, 1380, 1410, 1441, 1473, 1505, 1538, 1572, 1607, 1642, 1678, 1715,
    1753, 1791, 1831, 1871, 1912, 1954, 1997, 2041, 2086, 2132, 2179, 2227,
    2276, 2326, 2377, 2429, 2483, 2537, 2593, 2650, 2708, 2768, 2829, 2891,
    2955, 3020, 3086, 3154, 3223, 3294, 3367, 3441, 3517, 3594, 3673, 3754,
    3836, 3921, 4007, 4095};

int hue_max_rgb[256] = {0};

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

void fade(hsv_t hsv, TickType_t ticks) {
    uint32_t r, g, b, c;
#ifdef USE_WARM_WHITE
    uint32_t w;
#endif

    if (!initialized) {
        led_init();
        initialized = true;
    }

    if (ticks <= 0)
        ticks = 1;

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
    /** these are ideal values */
    uint32_t red = 0, green = 0, blue = 0, white = 0, total = 0;
    /** these are modified values */
    uint32_t r = 0, g = 0, b = 0, c = 0;
#ifdef USE_WARM_WHITE
    uint32_t w = 0;
#endif

    if (!initialized) {
        led_init();
        initialized = true;
    }

    if (hsv.h == curr_color.h && hsv.s == curr_color.s &&
        hsv.v == curr_color.v) {
        return;
    }

    total = grayscale_0db_to_minus_24db[hsv.v];

    white = total * (int)(255 - hsv.s) / (int)255;

    // TODO
    int h360 = (int)round((float)hsv.h * 360.0 / 256.0);

    if (h360 >= 0 && h360 < 60) {
        red = (total - white) * 60 / (60 + h360);
        green = (total - white) * h360 / (60 + h360);
        blue = 0;
    } else if (h360 >= 60 && h360 < 120) {
        red = (total - white) * (60 - (h360 - 60)) / ((60 - (h360 - 60)) + 60);
        green = (total - white) * 60 / ((60 - (h360 - 60)) + 60);
        blue = 0;
    } else if (h360 >= 120 && h360 < 180) {
        red = 0;
        green = (total - white) * 60 / (60 + (h360 - 120));
        blue = (total - white) * (h360 - 120) / (60 + (h360 - 120));
    } else if (h360 >= 180 && h360 < 240) {
        red = 0;
        green =
            (total - white) * (60 - (h360 - 180)) / ((60 - (h360 - 180)) + 60);
        blue = (total - white) * 60 / ((60 - (h360 - 180)) + 60);
    } else if (h360 >= 240 && h360 < 300) {
        red = (total - white) * (h360 - 240) / ((h360 - 240) + 60);
        green = 0;
        blue = (total - white) * 60 / ((h360 - 240) + 60);
    } else { /* h360 >= 300 && h360 < 360 */
        red = (total - white) * 60 / (60 + (60 - (h360 - 300)));
        green = 0;
        blue =
            (total - white) * (60 - (h360 - 300)) / (60 + (60 - (h360 - 300)));
    }

    r = red;
    g = green;
    b = blue;
#ifdef USE_WARM_WHITE
    c = white / 8 / 2;
    w = c;
#elif
    c = white / 8;
#endif

    /*
        hsv_t tint = hsv;
        uint8_t unsat = 0xff - tint.s;
        uint8_t white = (uint8_t)(((uint32_t)unsat * (uint32_t)tint.v) / 0xff);
        c = rgb4096[white];
    #ifdef USE_WARM_WHITE
        w = c;
    #endif
        tint.s = 0xff;
        tint.v -= white;
        rgb_t rgb = hsv2rgb_spectrum(tint);
    */

    /**
        r = (uint32_t)rgb.r * 3;
        g = (uint32_t)rgb.g * 3;
        b = (uint32_t)rgb.b / 2;
    */

    /**
        r = (rgb4096[rgb.r] * 2 + rgb.r * 16) / 3;
        g = (rgb4096[rgb.g] * 2 + rgb.g * 16) / 3;
        b = (rgb4096[rgb.b] * 2 + rgb.b * 8) / 3;
    */

    // normalize the largest one
    /*
        int max = (rgb.r > rgb.g && rgb.r >= rgb.b)
                      ? rgb.r
                      : (rgb.g >= rgb.b) ? rgb.g : rgb.b;

        int normalized = rgb4096[max * 255 / hue_max_rgb[hsv.h]];
        r = rgb.r * normalized / max;
        g = rgb.g * normalized / max;
        b = rgb.b * normalized / max / 2;
    */
    /*
        r = rgb4096[(int)rgb.r * 255 / hue_max_rgb[hsv.h]];
        g = rgb4096[(int)rgb.g * 255 / hue_max_rgb[hsv.h]];
        b = rgb4096[(int)rgb.b * 255 / hue_max_rgb[hsv.h]];
    */

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

void direct_draw(uint32_t r, uint32_t g, uint32_t b, uint32_t c, uint32_t w) {
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

    curr_color.v = 0;
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
