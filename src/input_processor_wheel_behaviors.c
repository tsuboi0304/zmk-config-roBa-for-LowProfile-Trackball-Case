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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>

// たまっていられるタップ数の上限(暴走防止)
#define WHEEL_BEHAVIORS_MAX_PENDING 10

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

// 入力スレッドではビヘイビアを実行しない。
// 入力スレッドはスタックが小さく、そこでキー入力→HID送信まで走らせると
// スタック溢れで固まる(音量キーが押されたまま止まる)ため、
// 回転量だけ数えておき、実際の押下/離しはシステムワークキューで1回ずつ行う。
struct wheel_behaviors_data {
    const struct device *dev;
    struct k_work_delayable work;
    atomic_t pending;   // 正=正方向のタップ待ち、負=負方向のタップ待ち
    bool pressed;       // 今どちらかのビヘイビアを押下中か
    uint8_t pressed_idx; // 押下中のビヘイビア(0 or 1)
    uint8_t input_device_index;
};

static struct zmk_behavior_binding_event make_event(const struct wheel_behaviors_config *cfg,
                                                    const struct wheel_behaviors_data *data) {
    struct zmk_behavior_binding_event event = {
        .layer = 0,
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            data->input_device_index, WHEEL_BEHAVIORS_INDEX_OFFSET + cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };
    return event;
}

static void wheel_behaviors_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct wheel_behaviors_data *data = CONTAINER_OF(dwork, struct wheel_behaviors_data, work);
    const struct wheel_behaviors_config *cfg = data->dev->config;

    if (data->pressed) {
        // 押下中なら離す。まだ残りがあれば間隔をあけて次へ
        zmk_behavior_invoke_binding(&cfg->bindings[data->pressed_idx], make_event(cfg, data),
                                    false);
        data->pressed = false;
        if (atomic_get(&data->pending) != 0) {
            k_work_schedule(&data->work, K_MSEC(cfg->tap_ms));
        }
        return;
    }

    atomic_val_t pending = atomic_get(&data->pending);
    if (pending == 0) {
        return;
    }

    if (pending > 0) {
        atomic_dec(&data->pending);
        data->pressed_idx = 0;
    } else {
        atomic_inc(&data->pending);
        data->pressed_idx = 1;
    }

    zmk_behavior_invoke_binding(&cfg->bindings[data->pressed_idx], make_event(cfg, data), true);
    data->pressed = true;
    k_work_schedule(&data->work, K_MSEC(cfg->tap_ms));
}

static int wheel_behaviors_handle_event(const struct device *dev, struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    const struct wheel_behaviors_config *cfg = dev->config;
    struct wheel_behaviors_data *data = dev->data;

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

    if (event->value != 0) {
        if (state) {
            data->input_device_index = state->input_device_index;
        }

        atomic_val_t next = atomic_get(&data->pending) + event->value;
        if (next > WHEEL_BEHAVIORS_MAX_PENDING) {
            next = WHEEL_BEHAVIORS_MAX_PENDING;
        } else if (next < -WHEEL_BEHAVIORS_MAX_PENDING) {
            next = -WHEEL_BEHAVIORS_MAX_PENDING;
        }
        atomic_set(&data->pending, next);

        LOG_DBG("wheel %d -> pending %ld", event->value, (long)next);

        // 待機中でなければすぐ処理を始める(実行はシステムワークキュー側)
        k_work_schedule(&data->work, K_NO_WAIT);
    }

    // ホイール入力そのものは PC に送らない
    event->value = 0;
    return ZMK_INPUT_PROC_STOP;
}

static int wheel_behaviors_init(const struct device *dev) {
    struct wheel_behaviors_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->work, wheel_behaviors_work_cb);
    return 0;
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
    static struct wheel_behaviors_data wheel_behaviors_data_##n;                                   \
    DEVICE_DT_INST_DEFINE(n, &wheel_behaviors_init, NULL, &wheel_behaviors_data_##n,               \
                          &wheel_behaviors_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &wheel_behaviors_driver_api);

DT_INST_FOREACH_STATUS_OKAY(WHEEL_BEHAVIORS_INST)
