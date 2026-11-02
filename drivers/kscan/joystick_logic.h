/*
 * Analog stick -> 8-way digital direction logic.
 * No Zephyr dependency, so tests/test_joystick_logic.c can run it on a PC.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Kscan column of each direction. */
enum {
    JL_BIT_UP = 0,
    JL_BIT_LEFT = 1,
    JL_BIT_DOWN = 2,
    JL_BIT_RIGHT = 3,
    JL_DIRECTIONS = 4,
};

#define JL_MASK(bit) ((uint8_t)(1u << (bit)))
#define JL_UP JL_MASK(JL_BIT_UP)
#define JL_LEFT JL_MASK(JL_BIT_LEFT)
#define JL_DOWN JL_MASK(JL_BIT_DOWN)
#define JL_RIGHT JL_MASK(JL_BIT_RIGHT)

struct jl_config {
    /* Radial distance from centre in ADC counts: keys engage at press, drop at release. */
    int32_t press;
    int32_t release;
    /*
     * Minor/major axis ratio in permille deciding the 8-way sectors. An axis turns on when
     * |axis| * 1000 >= on_ratio * |other axis| and stays on down to off_ratio.
     * on_ratio must be <= 1000 so at least one axis is always active.
     */
    int32_t on_ratio;
    int32_t off_ratio;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
    /* Consecutive samples near centre before the centre starts tracking drift. 0 disables. */
    uint32_t recenter_after;
};

struct jl_state {
    /* Fixed point with 4 fractional bits. */
    int32_t center_x;
    int32_t center_y;
    int32_t filt_x;
    int32_t filt_y;
    uint32_t rest_samples;
    uint8_t mask;
};

/* Sets the centre from a calibration average and clears all directions. */
void jl_init(struct jl_state *state, int32_t center_x, int32_t center_y);

/* Feeds one raw sample and returns the bitmask of active directions. */
uint8_t jl_update(struct jl_state *state, const struct jl_config *cfg, int32_t raw_x,
                  int32_t raw_y);
