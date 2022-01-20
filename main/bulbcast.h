#ifndef _BULBCAST_H
#define _BULBCAST_H

#include "esp_timer.h"

extern uint8_t boot_params[16];
extern uint32_t my_bit_field;

void handle_bulbcode(uint16_t cmd, const uint8_t code[13]);

void ble_adv_scan(void *pvParams);

#endif
