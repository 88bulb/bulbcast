#ifndef _LED_H_
#define _LED_H_

#include "freertos/FreeRTOS.h"
#include "color.h"

extern int hue_max_rgb[256];

void led_init();
void led_fade(uint32_t r, uint32_t g, uint32_t b, uint32_t c, uint32_t w,
              uint32_t duration);
void led_set_duty(uint32_t r, uint32_t g, uint32_t b, uint32_t c, uint32_t w);

/**
 * This is a blocking function
 */
void fade(hsv_t hsv, TickType_t ticks); 

void draw(hsv_t hsv);
void direct_draw(uint32_t r, uint32_t g, uint32_t b, uint32_t c, uint32_t w);
void flash(int ms);
void flash_max();
void flash_min();

#endif // _LED_H_
