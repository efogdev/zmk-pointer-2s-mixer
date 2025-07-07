/*
* Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include "drivers/p2sm_runtime.h"
#include "dt-bindings/zmk/p2sm.h"
#include "zmk/event_manager.h"
#include "zmk/events/position_state_changed.h"

#define CORRECTION_THRES 3 // %
#define DT_DRV_COMPAT zmk_behavior_p2sm_sens
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_p2sm_sens_config {
    uint32_t step;
};

static int p2sm_detect_drift(const struct zmk_behavior_binding *binding) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(binding->behavior_dev)->config;
    const float current = p2sm_get_move_coef();

    if ((int16_t) (current * 1000) % cfg->step > CORRECTION_THRES * 10) {
        const int steps_count = (int)(current * 1000 / cfg->step + 0.5f);
        const float closest = (float)(steps_count * cfg->step) / 1000.0f;
        LOG_WRN("Sensitivity drift detected! Setting to the closest correct value: %d%%", (int) (closest * 1000.0f));
        p2sm_set_move_coef(closest);
        return 1;
    }

    return 0;
}

static int on_p2sm_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(binding->behavior_dev)->config;
    const bool direction = binding->param1 & P2SM_INC;
    const int8_t steps = (int32_t) binding->param2;
    const float current = p2sm_get_move_coef();

    if (p2sm_detect_drift(binding)) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    float new_val = current + (float) cfg->step * (float) steps / 1000.0f * (direction ? 1.0f : -1.0f);
    if (new_val > 1.0f || new_val < 0.0f) {
        new_val = new_val - floor(new_val);
        if (new_val < 0.0f) {
            new_val = new_val + 1.0f;
        }
        LOG_WRN("Sensitivity wrapped around to stay in range!");
    }

    if (direction) {
        LOG_INF("Sensitivity increased by %d step(s), currently at %d%%", steps, (int) (new_val * 1000.0f));
    } else {
        LOG_INF("Sensitivity decreased by %d step(s), currently at %d%%", steps, (int) (new_val * 1000.0f));
    }

    p2sm_set_move_coef(new_val);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_p2sm_sens_init(const struct device *dev) {
    p2sm_detect_drift(zmk_behavior_get_binding(dev->name)->config);
    return 0;
}

static const struct behavior_driver_api behavior_p2sm_sens_driver_api = {
    .binding_pressed = on_p2sm_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

static const struct behavior_p2sm_sens_config behavior_p2sm_sens_config = {
    .step = DT_INST_PROP(0, step),
};
BEHAVIOR_DT_INST_DEFINE(0, behavior_p2sm_sens_init, NULL, NULL, &behavior_p2sm_sens_config,
    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_sens_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
