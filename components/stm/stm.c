#include <stdio.h>
#include "stm.h"

uint8_t interp(int v1, int v2, TickType_t t1, TickType_t t2, TickType_t t) {
    return (v1 == v2) ? v1
                      : (uint8_t)(v1 + (v2 - v1) * ((int)t - (int)t1) /
                                           ((int)t2 - (int)t1));
}

uint8_t interp_hue_cw(uint8_t h1, uint8_t h2, TickType_t t1, TickType_t t2,
                      TickType_t t) {
    // guarantee h2 >= h1
    int h = (h2 < h1) ? (int)h2 + 256 : (int)h2;
    return interp(h1, h, t1, t2, t);
}

uint8_t interp_hue_ccw(uint8_t h1, uint8_t h2, TickType_t t1, TickType_t t2,
                       TickType_t t) {
    // guarantee h2 <= h1
    int h = (h1 < h2) ? (int)h1 + 256 : (int)h1;
    return interp(h, h2, t1, t2, t);
}

/** TODO unit testing **/
hsv_hue_dir_t short_path_dir(uint8_t h1, uint8_t h2) {
    if (h1 < h2) {
        if (h2 - h1 < 128)
            return INTERP_HUE_CW;
        else if (h2 - h1 > 128)
            return INTERP_HUE_CCW;
        else 
            return INTERP_HUE_CW;
    } else if (h1 > h2) {
        if (h1 - h2 < 128)
            return INTERP_HUE_CCW;
        else if (h1 - h2 > 128)
            return INTERP_HUE_CW;
        else
            return INTERP_HUE_CW;
    } else {
        return INTERP_HUE_CW;
    }
}

hsv_t interp_color(hsv_t c1, hsv_t c2, hsv_hue_dir_t dir,
                   TickType_t t1, TickType_t t2, TickType_t t) {
    hsv_t c;

    if (t >= t2)
        return c2;

    if (dir == INTERP_HUE_AUTO) 
        dir = short_path_dir(c1.h, c2.h);

    switch (dir) {
    case INTERP_HUE_CW:
        c.h = interp_hue_cw(c1.h, c2.h, t1, t2, t);
        break;
    case INTERP_HUE_CCW:
        c.h = interp_hue_ccw(c1.h, c2.h, t1, t2, t);
        break;
    default:
        break;
    }

    c.s = interp((int)c1.s, (int)c2.s, t1, t2, t);
    c.v = interp((int)c1.v, (int)c2.v, t1, t2, t);
    return c;
}
