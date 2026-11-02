/*
 * Presents an analog stick as four virtual keys of a kscan matrix.
 *
 * Because the output is ordinary kscan events, the directions get key positions in the keymap
 * and reach the central over the normal split link. No custom BLE characteristic is needed.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_joystick

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "joystick_logic.h"

LOG_MODULE_REGISTER(kscan_joystick, CONFIG_ZMK_KSCAN_JOYSTICK_LOG_LEVEL);

/*
 * adc_read() blocks, and the nRF52840 SAADC is shared with the battery driver, which has been
 * reported to hang reads when it oversamples. On a private queue a stuck read loses the stick
 * only; on the system queue it would take the whole keyboard with it.
 */
#define JOYSTICK_STACK_SIZE 1024
#define JOYSTICK_PRIORITY 5
K_THREAD_STACK_DEFINE(joystick_stack, JOYSTICK_STACK_SIZE);
static struct k_work_q joystick_queue;

enum phase {
    PHASE_OFF,
    PHASE_SETTLING,
    PHASE_CALIBRATING,
    PHASE_RUNNING,
};

struct joystick_config {
    struct adc_dt_spec x;
    struct adc_dt_spec y;
    struct gpio_dt_spec enable;
    struct jl_config logic;
    uint16_t active_period_ms;
    uint16_t idle_period_ms;
    uint16_t idle_after_samples;
    uint16_t settle_ms;
    uint16_t calibration_samples;
    uint16_t calibration_retry_ms;
    int16_t center_min;
    int16_t center_max;
    int16_t calibration_max_spread;
};

struct joystick_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    struct jl_state state;
    enum phase phase;
    bool wanted;
    uint16_t idle_samples;
    uint16_t calibration_count;
    int32_t sum_x;
    int32_t sum_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
};

static int read_channel(const struct adc_dt_spec *spec, int16_t *out) {
    struct adc_sequence sequence = {
        .buffer = out,
        .buffer_size = sizeof(*out),
    };
    int err = adc_sequence_init_dt(spec, &sequence);

    return err ? err : adc_read_dt(spec, &sequence);
}

static void report(const struct device *dev, uint8_t previous, uint8_t next) {
    struct joystick_data *data = dev->data;
    uint8_t changed = previous ^ next;

    if (!data->callback) {
        return;
    }
    for (uint32_t bit = 0; bit < JL_DIRECTIONS; bit++) {
        if (changed & JL_MASK(bit)) {
            data->callback(dev, 0, bit, (next & JL_MASK(bit)) != 0);
        }
    }
}

static void begin_calibration(struct joystick_data *data) {
    data->phase = PHASE_CALIBRATING;
    data->calibration_count = 0;
    data->sum_x = 0;
    data->sum_y = 0;
    data->min_x = INT16_MAX;
    data->max_x = INT16_MIN;
    data->min_y = INT16_MAX;
    data->max_y = INT16_MIN;
}

static bool in_range(int32_t value, const struct joystick_config *cfg) {
    return value >= cfg->center_min && value <= cfg->center_max;
}

static void calibration_step(const struct device *dev, int16_t x, int16_t y) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;

    data->sum_x += x;
    data->sum_y += y;
    data->min_x = MIN(data->min_x, x);
    data->max_x = MAX(data->max_x, x);
    data->min_y = MIN(data->min_y, y);
    data->max_y = MAX(data->max_y, y);

    if (++data->calibration_count < cfg->calibration_samples) {
        k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(cfg->active_period_ms));
        return;
    }

    int32_t center_x = data->sum_x / data->calibration_count;
    int32_t center_y = data->sum_y / data->calibration_count;
    int32_t spread = MAX(data->max_x - data->min_x, data->max_y - data->min_y);

    /*
     * A missing stick leaves the inputs floating. Refusing to run here is what keeps a loose
     * ribbon from holding W forever; a thumb on the stick during boot lands here as well.
     */
    if (!in_range(center_x, cfg) || !in_range(center_y, cfg) ||
        spread > cfg->calibration_max_spread) {
        LOG_WRN("Calibration rejected: x=%d y=%d spread=%d", center_x, center_y, spread);
        begin_calibration(data);
        k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(cfg->calibration_retry_ms));
        return;
    }

    LOG_INF("Calibrated: x=%d y=%d spread=%d", center_x, center_y, spread);
    jl_init(&data->state, center_x, center_y);
    data->idle_samples = 0;
    data->phase = PHASE_RUNNING;
    k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(cfg->active_period_ms));
}

static void run_step(const struct device *dev, int16_t x, int16_t y) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;
    uint8_t previous = data->state.mask;
    uint8_t next = jl_update(&data->state, &cfg->logic, x, y);

    if (next != previous) {
        LOG_DBG("Directions %02x -> %02x (x=%d y=%d)", previous, next, x, y);
        report(dev, previous, next);
    }

    if (next) {
        data->idle_samples = 0;
    } else if (data->idle_samples < cfg->idle_after_samples) {
        data->idle_samples++;
    }

    uint16_t period = data->idle_samples >= cfg->idle_after_samples ? cfg->idle_period_ms
                                                                    : cfg->active_period_ms;
    k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(period));
}

static void joystick_work(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct joystick_data *data = CONTAINER_OF(delayable, struct joystick_data, work);
    const struct device *dev = data->dev;
    const struct joystick_config *cfg = dev->config;
    int16_t x, y;

    if (data->phase == PHASE_OFF) {
        return;
    }
    if (data->phase == PHASE_SETTLING) {
        begin_calibration(data);
    }

    int err = read_channel(&cfg->x, &x);
    if (!err) {
        err = read_channel(&cfg->y, &y);
    }
    if (err) {
        LOG_ERR("ADC read failed: %d", err);
        k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(cfg->calibration_retry_ms));
        return;
    }

    if (data->phase == PHASE_CALIBRATING) {
        calibration_step(dev, x, y);
    } else {
        run_step(dev, x, y);
    }
}

static void power_up(const struct device *dev) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;

    gpio_pin_set_dt(&cfg->enable, 1);
    data->phase = PHASE_SETTLING;
    k_work_reschedule_for_queue(&joystick_queue, &data->work, K_MSEC(cfg->settle_ms));
}

static void power_down(const struct device *dev) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;

    data->phase = PHASE_OFF;
    k_work_cancel_delayable(&data->work);

    /* A held direction must not stay stuck on the host across a suspend or disable. */
    report(dev, data->state.mask, 0);
    data->state.mask = 0;

    gpio_pin_set_dt(&cfg->enable, 0);
}

static int joystick_configure(const struct device *dev, kscan_callback_t callback) {
    struct joystick_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int joystick_enable(const struct device *dev) {
    struct joystick_data *data = dev->data;

    data->wanted = true;
    if (data->phase == PHASE_OFF) {
        power_up(dev);
    }
    return 0;
}

static int joystick_disable(const struct device *dev) {
    struct joystick_data *data = dev->data;

    data->wanted = false;
    power_down(dev);
    return 0;
}

static int joystick_init(const struct device *dev) {
    const struct joystick_config *cfg = dev->config;
    struct joystick_data *data = dev->data;

    if (!adc_is_ready_dt(&cfg->x) || !adc_is_ready_dt(&cfg->y) ||
        !gpio_is_ready_dt(&cfg->enable)) {
        LOG_ERR("ADC or enable GPIO not ready");
        return -ENODEV;
    }

    int err = adc_channel_setup_dt(&cfg->x);
    if (!err) {
        err = adc_channel_setup_dt(&cfg->y);
    }
    if (!err) {
        err = gpio_pin_configure_dt(&cfg->enable, GPIO_OUTPUT_INACTIVE);
    }
    if (err) {
        LOG_ERR("Setup failed: %d", err);
        return err;
    }

    k_work_queue_start(&joystick_queue, joystick_stack, K_THREAD_STACK_SIZEOF(joystick_stack),
                       JOYSTICK_PRIORITY, NULL);

    data->dev = dev;
    data->phase = PHASE_OFF;
    k_work_init_delayable(&data->work, joystick_work);
    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int joystick_pm_action(const struct device *dev, enum pm_device_action action) {
    struct joystick_data *data = dev->data;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        power_down(dev);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        if (data->wanted) {
            power_up(dev);
        }
        return 0;
    default:
        return -ENOTSUP;
    }
}
#endif

static const struct kscan_driver_api joystick_api = {
    .config = joystick_configure,
    .enable_callback = joystick_enable,
    .disable_callback = joystick_disable,
};

#define JOYSTICK_DEFINE(n)                                                                         \
    BUILD_ASSERT(DT_INST_PROP(n, on_ratio_permille) <= 1000,                                       \
                 "on-ratio-permille above 1000 can leave no axis active");                         \
    BUILD_ASSERT(DT_INST_PROP(n, release_threshold) < DT_INST_PROP(n, press_threshold),            \
                 "release-threshold must be below press-threshold");                               \
    static struct joystick_data joystick_data_##n;                                                 \
    static const struct joystick_config joystick_config_##n = {                                    \
        .x = ADC_DT_SPEC_STRUCT(DT_IO_CHANNELS_CTLR_BY_NAME(DT_DRV_INST(n), x),                    \
                                DT_IO_CHANNELS_INPUT_BY_NAME(DT_DRV_INST(n), x)),                  \
        .y = ADC_DT_SPEC_STRUCT(DT_IO_CHANNELS_CTLR_BY_NAME(DT_DRV_INST(n), y),                    \
                                DT_IO_CHANNELS_INPUT_BY_NAME(DT_DRV_INST(n), y)),                  \                                        \
        .enable = GPIO_DT_SPEC_INST_GET(n, enable_gpios),                                          \
        .logic =                                                                                   \
            {                                                                                      \
                .press = DT_INST_PROP(n, press_threshold),                                         \
                .release = DT_INST_PROP(n, release_threshold),                                     \
                .on_ratio = DT_INST_PROP(n, on_ratio_permille),                                    \
                .off_ratio = DT_INST_PROP(n, off_ratio_permille),                                  \
                .swap_xy = DT_INST_PROP(n, swap_xy),                                               \
                .invert_x = DT_INST_PROP(n, invert_x),                                             \
                .invert_y = DT_INST_PROP(n, invert_y),                                             \
                .recenter_after = DT_INST_PROP(n, recenter_after_samples),                         \
            },                                                                                     \
        .active_period_ms = DT_INST_PROP(n, active_period_ms),                                     \
        .idle_period_ms = DT_INST_PROP(n, idle_period_ms),                                         \
        .idle_after_samples = DT_INST_PROP(n, idle_after_samples),                                 \
        .settle_ms = DT_INST_PROP(n, settle_ms),                                                   \
        .calibration_samples = DT_INST_PROP(n, calibration_samples),                               \
        .calibration_retry_ms = DT_INST_PROP(n, calibration_retry_ms),                             \
        .center_min = DT_INST_PROP(n, center_min),                                                 \
        .center_max = DT_INST_PROP(n, center_max),                                                 \
        .calibration_max_spread = DT_INST_PROP(n, calibration_max_spread),                         \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, joystick_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, joystick_init, PM_DEVICE_DT_INST_GET(n), &joystick_data_##n,          \
                          &joystick_config_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,           \
                          &joystick_api);

DT_INST_FOREACH_STATUS_OKAY(JOYSTICK_DEFINE)
