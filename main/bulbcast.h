#ifndef _BULBCAST_H
#define _BULBCAST_H

extern uint8_t bulbcode[16];
extern uint8_t boot_params[6];
extern uint32_t my_bit_field;

void ble_adv_scan(void *pvParams);

#endif
