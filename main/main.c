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

typedef enum { nilEvent = 0 } event_enum_t;

typedef struct {
    event_enum_t type;
    union {
    } data;
} event_type;

uint8_t bulbcode[16] = {0};

static const char *TAG = "bulbcode";

esp_reset_reason_t rst_reason = ESP_RST_UNKNOWN;

uint8_t boot_params[16] = {0};
uint32_t my_bit_field;

bool base_color_transitioning = false;

hsv_hue_dir_t base_color_transition_dir;
TickType_t base_color_transition_start = 0;
TickType_t base_color_transition_end = 0;

hsv_t prev_base_color_hsv = {0}, curr_base_color_hsv = {0},
      next_base_color_hsv = {0};

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
            base_color_transition_start = xTaskGetTickCount();
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
        // uint8_t = bulbcode[2];
        // uint8_t* payload = &bulbcode[2]; 
         
    } break;
    default:
        break;
    }
}

static void hard_tick(void* arg) {
    int64_t start = esp_timer_get_time();
    flash_max();
    usleep(10000);
    flash_min();
    ESP_LOGI("htimer", "%lld", esp_timer_get_time() - start);
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

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &hard_tick,
        .name = "hard_tick"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000000));

    xTaskCreate(&ble_adv_scan, "ble", 4096, NULL, 6, NULL);

    curr_base_color_hsv.h = 0x20;
    curr_base_color_hsv.s = 0xff;
    curr_base_color_hsv.v = 0x20;

    paint(curr_base_color_hsv, 1);

    vTaskDelay(portMAX_DELAY);

    for (int i = 0; i < 100; i++) {
        flash(10);
        vTaskDelay(1);
    }

    while (1) {
        handle_bulbcode();
        if (base_color_transitioning) {
            TickType_t t1 = base_color_transition_start;
            TickType_t t2 = base_color_transition_end;
            TickType_t curr_tick = xTaskGetTickCount();
            hsv_t c1 = prev_base_color_hsv;
            hsv_t c2 = next_base_color_hsv;
            hsv_hue_dir_t dir = base_color_transition_dir; 
            curr_base_color_hsv = interp_color(c1, c2, dir, t1, t2, curr_tick);
            if (curr_tick >= t2) {
               base_color_transitioning = false; 
            }
        }
        // paint(curr_base_color_hsv, 1);
        draw(curr_base_color_hsv);
        vTaskDelay(1);
    }
}

