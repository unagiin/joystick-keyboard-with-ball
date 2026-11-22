/*
 * Streams the voltage on J3 S2 while only the internal pull-up (about 13 kOhm) feeds it.
 *
 * Safety: nothing on J3 is driven. The 1.8 V LDO is held off, so S1 is dead, and any two pins
 * the user joins see at most VDD / 13 kOhm. All interpretation is in tools/stick_probe.html.
 *
 * Line format: "JBP1 <0..4095>\n", or "JBPE <errno>\n" on failure.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PROBE_NODE DT_PATH(zephyr_user)
#define PERIOD_MS 25
#define READS_PER_LINE 16

static const struct adc_dt_spec channel = ADC_DT_SPEC_GET(PROBE_NODE);
static const struct gpio_dt_spec probe = GPIO_DT_SPEC_GET(PROBE_NODE, probe_gpios);
static const struct gpio_dt_spec ldo_enable = GPIO_DT_SPEC_GET(PROBE_NODE, ldo_en_gpios);
static const struct device *const port = DEVICE_DT_GET(DT_NODELABEL(probe_uart));

static int setup(void) {
    if (!gpio_is_ready_dt(&probe) || !gpio_is_ready_dt(&ldo_enable) || !adc_is_ready_dt(&channel)) {
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&ldo_enable, GPIO_OUTPUT_INACTIVE);
    if (err) {
        return err;
    }

    err = adc_channel_setup_dt(&channel);
    if (err) {
        return err;
    }

    return gpio_pin_configure_dt(&probe, GPIO_INPUT | GPIO_PULL_UP);
}

static int measure(bool calibrate, int *result) {
    int16_t raw;
    int32_t sum = 0;

    for (int i = 0; i < READS_PER_LINE; i++) {
        struct adc_sequence sequence = {
            .buffer = &raw,
            .buffer_size = sizeof(raw),
            .calibrate = calibrate && i == 0,
        };

        int err = adc_sequence_init_dt(&channel, &sequence);
        if (!err) {
            err = adc_read(channel.dev, &sequence);
        }
        if (err) {
            return err;
        }

        sum += CLAMP(raw, 0, 4095);
    }

    *result = sum / READS_PER_LINE;
    return 0;
}

static void send(const char *text) {
    while (*text) {
        uart_poll_out(port, *text++);
    }
}

static void probe_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    int setup_err = setup();
    bool calibrate = true;

    while (true) {
        uint32_t listening = 0;

        if (device_is_ready(port) &&
            uart_line_ctrl_get(port, UART_LINE_CTRL_DTR, &listening) == 0 && listening) {
            char line[24];
            int value = 0;
            int err = setup_err ? setup_err : measure(calibrate, &value);

            calibrate = false;
            if (err) {
                snprintk(line, sizeof(line), "JBPE %d\n", err);
            } else {
                snprintk(line, sizeof(line), "JBP1 %d\n", value);
            }
            send(line);
        }

        k_msleep(PERIOD_MS);
    }
}

K_THREAD_DEFINE(stick_probe_thread, 2048, probe_thread, NULL, NULL, NULL, 7, 0, 0);
