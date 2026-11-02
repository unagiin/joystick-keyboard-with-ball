/*
 * Host test for the stick logic. Build and run with any C compiler:
 *   cc -I../drivers/kscan test_joystick_logic.c ../drivers/kscan/joystick_logic.c -lm -o t && ./t
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "joystick_logic.h"

#define CENTER 1500
#define SPAN 900
#define PI 3.14159265358979

static const struct jl_config cfg = {
    .press = 340,
    .release = 240,
    .on_ratio = 520,
    .off_ratio = 330,
    .recenter_after = 400,
};

static int failures;

static void expect(const char *name, unsigned got, unsigned want) {
    if (got != want) {
        printf("FAIL %-44s got %02x want %02x\n", name, got, want);
        failures++;
    } else {
        printf("ok   %s\n", name);
    }
}

/* Holds a position long enough for the filter to converge. */
static uint8_t hold(struct jl_state *s, const struct jl_config *c, int dx, int dy) {
    uint8_t mask = 0;
    for (int i = 0; i < 12; i++) {
        mask = jl_update(s, c, CENTER + dx, CENTER + dy);
    }
    return mask;
}

static uint8_t hold_polar(struct jl_state *s, double degrees, double radius) {
    double a = degrees * PI / 180.0;
    return hold(s, &cfg, (int)lround(radius * cos(a)), (int)lround(radius * sin(a)));
}

static void test_directions(void) {
    static const struct {
        const char *name;
        double degrees;
        uint8_t want;
    } cases[] = {
        {"0 deg is D", 0, JL_RIGHT},          {"45 deg is W+D", 45, JL_UP | JL_RIGHT},
        {"90 deg is W", 90, JL_UP},           {"135 deg is W+A", 135, JL_UP | JL_LEFT},
        {"180 deg is A", 180, JL_LEFT},       {"225 deg is S+A", 225, JL_DOWN | JL_LEFT},
        {"270 deg is S", 270, JL_DOWN},       {"315 deg is S+D", 315, JL_DOWN | JL_RIGHT},
        {"20 deg stays D only", 20, JL_RIGHT}, {"70 deg stays W only", 70, JL_UP},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        struct jl_state s;
        jl_init(&s, CENTER, CENTER);
        expect(cases[i].name, hold_polar(&s, cases[i].degrees, SPAN), cases[i].want);
    }
}

static void test_radial_hysteresis(void) {
    struct jl_state s;
    jl_init(&s, CENTER, CENTER);
    expect("below press stays off", hold(&s, &cfg, 0, 300), 0);
    expect("above press turns on", hold(&s, &cfg, 0, 360), JL_UP);
    expect("between thresholds stays on", hold(&s, &cfg, 0, 280), JL_UP);
    expect("below release turns off", hold(&s, &cfg, 0, 200), 0);
    expect("between thresholds stays off", hold(&s, &cfg, 0, 280), 0);
}

static void test_angular_hysteresis(void) {
    struct jl_state s;
    jl_init(&s, CENTER, CENTER);
    expect("start at W", hold_polar(&s, 90, SPAN), JL_UP);
    /* tan(25 deg) = 0.466: below on-ratio, so D must not join yet. */
    expect("65 deg does not add D", hold_polar(&s, 65, SPAN), JL_UP);
    expect("55 deg adds D", hold_polar(&s, 55, SPAN), JL_UP | JL_RIGHT);
    /* Coming back, D holds until tan(angle from W) falls below off-ratio 0.33 (18 deg). */
    expect("65 deg keeps D", hold_polar(&s, 65, SPAN), JL_UP | JL_RIGHT);
    expect("80 deg drops D", hold_polar(&s, 80, SPAN), JL_UP);
}

static void test_no_chatter_on_noise(void) {
    struct jl_state s;
    jl_init(&s, CENTER, CENTER);
    hold(&s, &cfg, 0, 300 + 40);
    unsigned transitions = 0;
    uint8_t previous = s.mask;
    for (int i = 0; i < 2000; i++) {
        int noise = (i * 7919) % 41 - 20;
        uint8_t mask = jl_update(&s, &cfg, CENTER + noise, CENTER + 300 + noise);
        transitions += mask != previous;
        previous = mask;
    }
    expect("noise near threshold makes no transitions", transitions, 0);
}

static void test_reversal(void) {
    struct jl_state s;
    jl_init(&s, CENTER, CENTER);
    hold(&s, &cfg, 0, SPAN);
    /* A snap through centre must never report both opposing keys. */
    int both = 0;
    for (int i = 0; i < 20; i++) {
        uint8_t mask = jl_update(&s, &cfg, CENTER, CENTER - SPAN);
        both |= (mask & JL_UP) && (mask & JL_DOWN);
    }
    expect("reversal never holds W and S together", both, 0);
    expect("reversal ends on S", s.mask, JL_DOWN);
}

static void test_all_angles_valid(void) {
    int bad = 0;
    for (int degrees = 0; degrees < 3600; degrees++) {
        struct jl_state s;
        jl_init(&s, CENTER, CENTER);
        uint8_t mask = hold_polar(&s, degrees / 10.0, SPAN);
        bad += mask == 0;
        bad += (mask & JL_UP) && (mask & JL_DOWN);
        bad += (mask & JL_LEFT) && (mask & JL_RIGHT);
    }
    expect("every angle gives one valid key set", bad, 0);
}

static void test_transforms(void) {
    struct jl_config c = cfg;
    struct jl_state s;
    c.invert_x = true;
    c.invert_y = true;
    jl_init(&s, CENTER, CENTER);
    expect("inverted +x is A", hold(&s, &c, SPAN, 0), JL_LEFT);
    jl_init(&s, CENTER, CENTER);
    expect("inverted +y is S", hold(&s, &c, 0, SPAN), JL_DOWN);
    c = cfg;
    c.swap_xy = true;
    jl_init(&s, CENTER, CENTER);
    expect("swapped +x is W", hold(&s, &c, SPAN, 0), JL_UP);
}

static void test_recenter(void) {
    struct jl_state s;
    jl_init(&s, CENTER, CENTER);
    /* Rest 60 counts off centre, for far longer than recenter_after. */
    for (int i = 0; i < 400 + 16 * 60 + 16; i++) {
        jl_update(&s, &cfg, CENTER + 60, CENTER);
    }
    /* The fixed point filter stops one LSB short, hence the tolerance of one count. */
    expect("centre converges on drift", abs((s.center_x >> 4) - (CENTER + 60)) <= 1, 1);

    jl_init(&s, CENTER, CENTER);
    hold(&s, &cfg, 0, SPAN);
    for (int i = 0; i < 5000; i++) {
        jl_update(&s, &cfg, CENTER, CENTER + SPAN);
    }
    expect("centre does not move while pushed", (s.center_y >> 4) == CENTER, 1);
}

int main(void) {
    test_directions();
    test_radial_hysteresis();
    test_angular_hysteresis();
    test_no_chatter_on_noise();
    test_reversal();
    test_all_angles_valid();
    test_transforms();
    test_recenter();
    printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures != 0;
}
