/*
 * Copyright (c) 2025 The roBa contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_accel_scaler

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct accel_scaler_config {
    uint8_t type;
    size_t codes_len;
    const uint16_t *codes;
    uint16_t min_factor;
    uint16_t max_factor;
    uint32_t fast_ms;
    uint32_t slow_ms;
};

struct accel_scaler_data {
    int64_t last_event_time;
};

// factor は 10 倍した固定小数点(10 = 1.0倍)。
static uint16_t factor_for_interval(const struct accel_scaler_config *cfg, int64_t dt_ms) {
    if (dt_ms <= cfg->fast_ms) {
        return cfg->max_factor;
    }

    if (dt_ms >= cfg->slow_ms) {
        return cfg->min_factor;
    }

    // fast_ms と slow_ms の間を線形補間する。
    int32_t span = (int32_t)(cfg->slow_ms - cfg->fast_ms);
    int32_t pos = (int32_t)(dt_ms - cfg->fast_ms);
    int32_t factor_span = (int32_t)cfg->max_factor - (int32_t)cfg->min_factor;

    return (uint16_t)(cfg->max_factor - (factor_span * pos) / span);
}

static int accel_scaler_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    const struct accel_scaler_config *cfg = dev->config;
    struct accel_scaler_data *data = dev->data;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool matches = false;
    for (int i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            matches = true;
            break;
        }
    }

    if (!matches) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t now = k_uptime_get();
    int64_t dt_ms = (data->last_event_time == 0) ? cfg->slow_ms : (now - data->last_event_time);
    data->last_event_time = now;

    if (dt_ms < 0) {
        dt_ms = 0;
    }

    uint16_t factor = factor_for_interval(cfg, dt_ms);

    int32_t value_scaled = (int32_t)event->value * (int32_t)factor;

    if (state && state->remainder) {
        value_scaled += *state->remainder;
    }

    int32_t result = value_scaled / 10;

    if (state && state->remainder) {
        *state->remainder = (int16_t)(value_scaled - (result * 10));
    }

    LOG_DBG("accel-scaled %d (dt=%lld ms, factor=%u.%u) to %d", event->value, dt_ms, factor / 10,
            factor % 10, result);

    event->value = (int16_t)result;

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api accel_scaler_driver_api = {
    .handle_event = accel_scaler_handle_event,
};

#define ACCEL_SCALER_INST(n)                                                                       \
    static const uint16_t accel_scaler_codes_##n[] = DT_INST_PROP(n, codes);                       \
    static const struct accel_scaler_config accel_scaler_config_##n = {                            \
        .type = DT_INST_PROP(n, type),                                                             \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = accel_scaler_codes_##n,                                                           \
        .min_factor = DT_INST_PROP(n, min_factor),                                                 \
        .max_factor = DT_INST_PROP(n, max_factor),                                                 \
        .fast_ms = DT_INST_PROP(n, fast_ms),                                                        \
        .slow_ms = DT_INST_PROP(n, slow_ms),                                                        \
    };                                                                                             \
    static struct accel_scaler_data accel_scaler_data_##n;                                         \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &accel_scaler_data_##n, &accel_scaler_config_##n,          \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                         \
                          &accel_scaler_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_SCALER_INST)
