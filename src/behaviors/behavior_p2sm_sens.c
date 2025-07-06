/*
* Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include "drivers/p2sm_runtime.h"
#include "dt-bindings/zmk/p2sm.h"

#define CORRECTION_THRES 10
#define DT_DRV_COMPAT zmk_behavior_p2sm_sens
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_p2sm_sens_config {
    uint32_t step;
};

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    LOG_ERR("Press");

    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(binding->behavior_dev)->config;
    const bool direction = binding->param1 | P2SM_INC;
    const int8_t steps = (int32_t) binding->param2;
    const float current = p2sm_get_move_coef();

    if ((int16_t) (current * 10) % cfg->step > CORRECTION_THRES) {
        LOG_WRN("Sensitivity value drift detected! Correction applied.");
        p2sm_set_move_coef(1.0f);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    float new_val = current + (float) cfg->step * (float) steps / 10.0f * (direction ? 1.0f : -1.0f);
    if (new_val > 1.0f) {
        new_val = new_val - 1.0f;
    }
    if (new_val < 0.0f) {
        new_val = new_val + 1.0f;
    }

    if (direction) {
        LOG_INF("Sensitivity increased by %d step(s)", steps);
    } else {
        LOG_INF("Sensitivity decreased by %d step(s)", steps);
    }

    p2sm_set_move_coef(new_val);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    LOG_ERR("Release");
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define RST_INST(n)                                                          \
    static const struct behavior_p2sm_sens_config p2sm_config_##n = {        \
        .step = DT_INST_PROP(0, step),                                       \
    };                                                                       \
    BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, &p2sm_config_##n,           \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api);

DT_INST_FOREACH_STATUS_OKAY(RST_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
