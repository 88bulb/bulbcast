#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "bootloader_common.h"

#include "bulbcast.h"
#include "led.h"

#include "color.h"

typedef enum {
    nilEvent = 0
} event_enum_t;

typedef struct {
    event_enum_t type;
    union{
    } data;
} event_type;

uint8_t bulbcode[16] = {0};

static const char *TAG = "bulbcode";

esp_reset_reason_t rst_reason = ESP_RST_UNKNOWN;

uint8_t boot_params[16] = {0};
uint32_t my_bit_field;

bool base_color_transitioning = false;
bool base_color_transition_mode = false; // TODO
bool base_color_transition_ccw = false;
TickType_t base_color_transition_start = 0;
TickType_t base_color_transition_end = 0;

rgb_t prev_base_color_rgb = {0}, curr_base_color_rgb = {0},
      next_base_color_rgb = {0};
hsv_t prev_base_color_hsv = {0}, curr_base_color_hsv = {0},
      next_base_color_hsv = {0};

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

void fade_base_color(TickType_t start, TickType_t middle, TickType_t end) {
    if (middle >= end) {
        base_color_transitioning = false;
        curr_base_color_hsv = next_base_color_hsv;
        curr_base_color_rgb = next_base_color_rgb;
    } else {
        if (base_color_transition_ccw) {
            curr_base_color_hsv.h =
                interpolate_ccw(prev_base_color_hsv.h, next_base_color_hsv.h,
                                middle - start, end - start);
        } else {
            curr_base_color_hsv.h =
                interpolate_cw(prev_base_color_hsv.h, next_base_color_hsv.h,
                               middle - start, end - start);
        }
        curr_base_color_hsv.s =
            interpolate(prev_base_color_hsv.s, next_base_color_hsv.s,
                        middle - start, end - start);
        curr_base_color_hsv.v =
            interpolate(prev_base_color_hsv.v, next_base_color_hsv.v,
                        middle - start, end - start);
        curr_base_color_rgb = hsv2rgb_spectrum(curr_base_color_hsv);
    }
}

hsv_t insert_color(hsv_t c1, hsv_t c2, TickType_t t1, TickType_t t2,
                            TickType_t t) {

    if (t >= t2) {
        base_color_transitioning = false;
        return c2;
    } else {
        hsv_t c;
        if (base_color_transition_ccw) {
            c.h = interpolate_ccw(c1.h, c2.h, t - t1, t2 - t1);
        } else {
            c.h = interpolate_cw(c1.h, c2.h, t - t1, t2 - t1);
        }
        c.s = interpolate(c1.s, c2.s, t - t1, t2 - t1);
        c.v = interpolate(c1.v, c2.v, t - t1, t2 - t1);
        return c;
    }
}

void handle_bulbcode() {
    uint16_t cmd = bulbcode[0] * 256 + bulbcode[1];
    if (cmd == 0)
        return;

    // erase bulbcode;
    bulbcode[0] = 0;
    bulbcode[1] = 0;

    switch (cmd) {
    case 0x1002: {
        uint8_t opt = bulbcode[2];

        TickType_t dur = ((opt & 0x01) ? 1000 : 100) *
                         (bulbcode[3] * 256 + bulbcode[4]) / portTICK_PERIOD_MS;

        hsv_t hsv;
        hsv.h = bulbcode[5];
        hsv.s = bulbcode[6];
        hsv.v = bulbcode[7];

        rgb_t rgb = hsv2rgb_rainbow(hsv);

        if (dur == 0) {
            // may be stopping
            base_color_transitioning = false;
            curr_base_color_hsv = hsv;
        } else {
            if (opt & 0x02) {
                base_color_transition_ccw = !!(opt & 0x04);
            } else {
                int hue1 = (int)prev_base_color_hsv.h;
                int hue2 = (int)next_base_color_hsv.h;
                int cw_dist = hue2 > hue1 ? hue2 - hue1 : (hue2 + 256) - hue1;
                int ccw_dist = hue1 > hue2 ? hue1 - hue2 : (hue1 + 256) - hue2;
                base_color_transition_ccw = (ccw_dist < cw_dist);
            }

            // may be restarting
            base_color_transitioning = true;
            base_color_transition_start = xTaskGetTickCount();
            base_color_transition_end = base_color_transition_start + dur;
            prev_base_color_rgb = curr_base_color_rgb;
            prev_base_color_hsv = curr_base_color_hsv;
            next_base_color_rgb = rgb;
            next_base_color_hsv = hsv;
        }
    } break;
    default:
        break;
    }
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

    rst_reason = esp_reset_reason();
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

    xTaskCreate(&ble_adv_scan, "ble", 4096, NULL, 6, NULL);

    curr_base_color_hsv.h = 0x20;
    curr_base_color_hsv.s = 0xc0;
    curr_base_color_hsv.v = 0x80;

    paint(curr_base_color_hsv, 1); 

    while (1) {
        handle_bulbcode();
        if (base_color_transitioning) {
            TickType_t t1 = base_color_transition_start;
            TickType_t t2 = base_color_transition_end;
            TickType_t now = xTaskGetTickCount();
            hsv_t c1 = prev_base_color_hsv;
            hsv_t c2 = next_base_color_hsv;
            hsv_t c = insert_color(c1, c2, t1, t2, now + 1);
            curr_base_color_hsv = c;
        }
        paint(curr_base_color_hsv, 1);
    }
}
