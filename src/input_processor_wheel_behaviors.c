/*
 * Copyright (c) 2026 The roBa contributors
 *
 * SPDX-License-Identifier: MIT
 */

// ホイール入力(REL_WHEEL など)を、回転方向に応じたビヘイビアに置き換える
// 自作 input-processor。
// 入力リスナーのレイヤー別設定と組み合わせて、「NUMレイヤーのときだけ
// エンコーダを回すと音量が変わる」ようにするために使う。
// 標準の zmk,input-processor-behaviors は押下/離しのキー入力向けで、
// ホイールの正負を区別できないため自作している。

#define DT_DRV_COMPAT zmk_input_processor_wheel_behaviors

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>

// 1回のイベントで出すタップ数の上限(暴走防止)
#define WHEEL_BEHAVIORS_MAX_TAPS 8

// 標準の behaviors processor と仮想キー位置が重ならないようにずらす
#define WHEEL_BEHAVIORS_INDEX_OFFSET 16

struct wheel_behaviors_config {
    uint8_t index;
    uint8_t type;
    size_t codes_len;
    const uint16_t *codes;
    uint32_t tap_ms;
    const struct zmk_behavior_binding *bindings; // [0]=正方向, [1]=負方向
};

static int wheel_behaviors_handle_event(const struct device *dev, struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    const struct wheel_behaviors_config *cfg = dev->config;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool matches = false;
    for (size_t i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == event->code) {
            matches = true;
            break;
        }
    }

    if (!matches) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int taps = abs(event->value);
    if (taps == 0) {
        return ZMK_INPUT_PROC_STOP;
    }
    if (taps > WHEEL_BEHAVIORS_MAX_TAPS) {
        taps = WHEEL_BEHAVIORS_MAX_TAPS;
    }

    const struct zmk_behavior_binding *binding =
        (event->value > 0) ? &cfg->bindings[0] : &cfg->bindings[1];

    struct zmk_behavior_binding_event behavior_event = {
        .layer = 0,
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state ? state->input_device_index : 0, WHEEL_BEHAVIORS_INDEX_OFFSET + cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    LOG_DBG("wheel %d -> %d taps of %s", event->value, taps, binding->behavior_dev);

    for (int i = 0; i < taps; i++) {
        zmk_behavior_queue_add(&behavior_event, *binding, true, cfg->tap_ms);
        zmk_behavior_queue_add(&behavior_event, *binding, false, 0);
    }

    // ホイール入力そのものは PC に送らない
    event->value = 0;
    return ZMK_INPUT_PROC_STOP;
}

static struct zmk_input_processor_driver_api wheel_behaviors_driver_api = {
    .handle_event = wheel_behaviors_handle_event,
};

#define WHEEL_BEHAVIORS_INST(n)                                                                    \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == 2,                                               \
                 "bindings must have exactly 2 entries: <positive>, <negative>");                  \
    static const uint16_t wheel_behaviors_codes_##n[] = DT_INST_PROP(n, codes);                    \
    static const struct zmk_behavior_binding wheel_behaviors_bindings_##n[] = {                    \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    static const struct wheel_behaviors_config wheel_behaviors_config_##n = {                      \
        .index = n,                                                                                \
        .type = DT_INST_PROP(n, type),                                                             \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = wheel_behaviors_codes_##n,                                                        \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                         \
        .bindings = wheel_behaviors_bindings_##n,                                                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &wheel_behaviors_config_##n, POST_KERNEL,           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &wheel_behaviors_driver_api);

DT_INST_FOREACH_STATUS_OKAY(WHEEL_BEHAVIORS_INST)
