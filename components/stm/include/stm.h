#ifndef _88BULB_STM_H
#define _88BULB_STM_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "color.h"

typedef enum {
    INTERP_HUE_CW,
    INTERP_HUE_CCW,
    INTERP_HUE_AUTO,
    INTERP_HUE_MAX
} hsv_hue_dir_t;

/**
 * interpolate v1 and v2 by (t-t1)/(t2-t1)
 * both v1 and v2 are signed integers.
 * no assumption on which one is bigger than the other.
 */
uint8_t interp(int v1, int v2, int64_t t1, int64_t t2, int64_t t);

/**
 * interpolate hue (0~255), clockwise
 */
uint8_t interp_hue_cw(uint8_t h1, uint8_t h2, int64_t t1, int64_t t2,
                      int64_t t);

/**
 * interpolate hue (0~255), counter-clockwise
 */
uint8_t interp_hue_ccw(uint8_t h1, uint8_t h2, int64_t t1, int64_t t2,
                       int64_t t);

/**
 * interpolate hsv color
 */
hsv_t interp_color(hsv_t c1, hsv_t c2, hsv_hue_dir_t dir, int64_t t1,
                   int64_t t2, int64_t t);
#endif
