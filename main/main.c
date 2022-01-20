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

uint8_t boot_params[16] = {0};
uint32_t my_bit_field;

typedef enum { nilEvent = 0 } event_enum_t;

typedef struct {
    event_enum_t type;
    union {
    } data;
} event_type;

static const char *TAG = "bulbcode";

static esp_timer_handle_t periodic_timer = {0};
static bool periodic_timer_started = false;

bool base_color_transitioning = false;

hsv_hue_dir_t base_color_transition_dir;
int64_t base_color_transition_start = 0;
int64_t base_color_transition_end = 0;

hsv_t prev_base_color_hsv = {0}, curr_base_color_hsv = {0},
      next_base_color_hsv = {0};

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

/**
 *
 */
static void render() {
    if (base_color_transitioning) {
        int64_t t1 = base_color_transition_start;
        int64_t t2 = base_color_transition_end;
        int64_t now = esp_timer_get_time();
        hsv_t c1 = prev_base_color_hsv;
        hsv_t c2 = next_base_color_hsv;
        hsv_hue_dir_t dir = base_color_transition_dir;
        curr_base_color_hsv = interp_color(c1, c2, dir, t1, t2, now);
        if (now >= t2)
            base_color_transitioning = false;
    }

    draw(curr_base_color_hsv);
}

static void timer_callback(void* arg) {
    int64_t __start = esp_timer_get_time();

    render();

    int64_t __end = esp_timer_get_time();
    if (__end - __start > 100) {
        ESP_LOGI("hardtick", "prio: %u, micros: %lld", uxTaskPriorityGet(NULL),
                 esp_timer_get_time() - __start);
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
            // may be stopping
            base_color_transitioning = false;
            curr_base_color_hsv = hsv;
        } else {
            /* if set, cw or ccw is determined by next bit */
            if (opt & 0x02) {
                if (opt & 0x04) {
                    base_color_transition_dir = INTERP_HUE_CCW;
                } else {
                    base_color_transition_dir = INTERP_HUE_CW;
                }
            } else {
                base_color_transition_dir = INTERP_HUE_AUTO;
            }

            // may be restarting
            base_color_transitioning = true;
            base_color_transition_start = esp_timer_get_time();
            base_color_transition_end = base_color_transition_start + dur;
            prev_base_color_hsv = curr_base_color_hsv;
            next_base_color_hsv = hsv;
        }
    } break;

    /**
     *  option      2 byte
     *  delay       1 byte  delay is used for 
     *  cycle min   1 byte
     *  bottom      1 byte
     *  top         1 byte
     *  
     */
    case 0x1003: {
         
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

    curr_base_color_hsv.h = 0x20;
    curr_base_color_hsv.s = 0xff;
    curr_base_color_hsv.v = 0x80;
    fade(curr_base_color_hsv, 1);

    xTaskCreate(&ble_adv_scan, "ble", 4096, NULL, 6, NULL);

    vTaskDelay(portMAX_DELAY);
}

