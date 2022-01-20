#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "bulbcast.h"

static const char *TAG = "bulbcode";

EventGroupHandle_t evt_handle;

/**
 *  b0 1b c0 de         magic
 *  00                  seq number
 *  00 00 00 00         group id
 *  00 00               bit mask
 *  00 00               command number (0xff00 ~ 0xffff are reserved)
 */
static void handle_mfr_data(uint8_t *bda, uint8_t *data, size_t data_len) {
    if (data_len != 26) return;

    /* magic must match */
    if (data[0] != 0xb0 || data[1] != 0x1b || data[2] != 0xc0 ||
        data[3] != 0xde)
        return;

    /* match group id */
    if (data[5] != boot_params[0] || data[6] != boot_params[1] ||
        data[7] != boot_params[2] || data[8] != boot_params[3])
        return;

    /* bit mask */
    if (!((data[9] * 256 + data[10]) & my_bit_field))
        return;

    handle_bulbcode(&data[11]);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    // priority 19, lower than that of esp_timer callback (22)
    // ESP_LOGI(TAG, "prio: %u", uxTaskPriorityGet(NULL)); 
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x0C00, // 1920ms
            .adv_int_max = 0x1000, // 2560ms
            .adv_type = ADV_TYPE_NONCONN_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    } break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
        ESP_LOGI(TAG, "ble adv started");
    } break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble adv stopped");
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble scan started");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            uint8_t mfr_data_len;
            uint8_t *mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);
            handle_mfr_data(scan_result->scan_rst.bda, mfr_data, mfr_data_len);
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble scan stopped");
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void ble_adv_scan(void *pvParameters) {
    uint8_t mfr[13] = {0xb0, 0x1b, 0xca, 0x57, 0x10, 0x07, 0x02};
    mfr[7] = boot_params[0];
    mfr[8] = boot_params[1];
    mfr[9] = boot_params[2];
    mfr[10] = boot_params[3];
    mfr[11] = boot_params[4];
    mfr[12] = boot_params[5];

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x50,
        .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,
        .include_txpower = false,
        .min_interval = 0x0000,
        .max_interval = 0x0000,
        .appearance = 0x00,
        .manufacturer_len = 13,
        .p_manufacturer_data = mfr,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    vTaskDelay(portMAX_DELAY);
}
