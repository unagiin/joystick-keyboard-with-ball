/*
 * SPDX-License-Identifier: MIT
 */

#include "joystick_logic.h"

#define JL_FRAC_BITS 4

static int32_t jl_abs(int32_t value) { return value < 0 ? -value : value; }

void jl_init(struct jl_state *state, int32_t center_x, int32_t center_y) {
    state->center_x = center_x << JL_FRAC_BITS;
    state->center_y = center_y << JL_FRAC_BITS;
    state->filt_x = state->center_x;
    state->filt_y = state->center_y;
    state->rest_samples = 0;
    state->mask = 0;
}

static bool axis_active(int32_t major, int32_t minor, bool was_active, const struct jl_config *cfg) {
    int32_t ratio = was_active ? cfg->off_ratio : cfg->on_ratio;

    return major * 1000 >= ratio * minor;
}

uint8_t jl_update(struct jl_state *state, const struct jl_config *cfg, int32_t raw_x,
                  int32_t raw_y) {
    /* One pole IIR at 1/2: removes single sample ADC noise for one sample of latency. */
    state->filt_x += ((raw_x << JL_FRAC_BITS) - state->filt_x) / 2;
    state->filt_y += ((raw_y << JL_FRAC_BITS) - state->filt_y) / 2;

    int32_t dx = (state->filt_x - state->center_x) >> JL_FRAC_BITS;
    int32_t dy = (state->filt_y - state->center_y) >> JL_FRAC_BITS;

    if (cfg->swap_xy) {
        int32_t tmp = dx;
        dx = dy;
        dy = tmp;
    }
    if (cfg->invert_x) {
        dx = -dx;
    }
    if (cfg->invert_y) {
        dy = -dy;
    }

    int32_t r2 = dx * dx + dy * dy;
    int32_t threshold = state->mask ? cfg->release : cfg->press;

    if (r2 < threshold * threshold) {
        state->mask = 0;

        /* Only follow drift when clearly resting, so slow tilts are not swallowed. */
        int32_t rest = cfg->release / 2;
        if (cfg->recenter_after && r2 < rest * rest) {
            if (state->rest_samples < cfg->recenter_after) {
                state->rest_samples++;
            } else {
                state->center_x += (state->filt_x > state->center_x) -
                                   (state->filt_x < state->center_x);
                state->center_y += (state->filt_y > state->center_y) -
                                   (state->filt_y < state->center_y);
            }
        } else {
            state->rest_samples = 0;
        }
        return 0;
    }

    state->rest_samples = 0;

    int32_t ax = jl_abs(dx);
    int32_t ay = jl_abs(dy);
    uint8_t next = 0;

    uint8_t vertical = dy >= 0 ? JL_UP : JL_DOWN;
    uint8_t horizontal = dx >= 0 ? JL_RIGHT : JL_LEFT;

    if (axis_active(ay, ax, state->mask & vertical, cfg)) {
        next |= vertical;
    }
    if (axis_active(ax, ay, state->mask & horizontal, cfg)) {
        next |= horizontal;
    }

    state->mask = next;
    return next;
}
