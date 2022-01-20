#ifndef _BULBCAST_H
#define _BULBCAST_H

#include "esp_timer.h"

extern uint8_t boot_params[16];
extern uint32_t my_bit_field;
extern esp_timer_handle_t periodic_timer;

void handle_bulbcode();

void ble_adv_scan(void *pvParams);

#endif
