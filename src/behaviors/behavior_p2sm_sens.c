#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "drivers/behavior.h"
#include "drivers/p2sm_runtime.h"
#include "dt-bindings/zmk/p2sm.h"
#include "zephyr/logging/log.h"
#include "zephyr/settings/settings.h"
#include "zmk/behavior.h"

#define MAX_DEVICES 8
#define DT_DRV_COMPAT zmk_behavior_p2sm_sens
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static bool initialized = false;
static uint8_t g_dev_num = 0;
static const char* g_devices[MAX_DEVICES] = { NULL };
static float g_from_settings[2] = { -1, -1 };

struct behavior_p2sm_sens_config {
    uint32_t step;
    uint32_t min_step, max_step;
    uint32_t max_multiplier;
    bool scroll;
};

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata mtd_param1_values[] = {
    {
        .display_name = "Increase",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = P2SM_INC,
    },
    {
        .display_name = "Decrease",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = P2SM_DEC,
    },
};

static const struct behavior_parameter_value_metadata mtd_param2_values[] = {
    {
        .display_name = "Steps",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
        .range = {.min = 1, .max = 1000},
    },
};

static const struct behavior_parameter_metadata_set profile_index_metadata_set = {
    .param1_values = mtd_param1_values,
    .param1_values_len = ARRAY_SIZE(mtd_param1_values),
    .param2_values = mtd_param2_values,
    .param2_values_len = ARRAY_SIZE(mtd_param2_values),
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {profile_index_metadata_set};
static const struct behavior_parameter_metadata metadata = { .sets_len = ARRAY_SIZE(metadata_sets), .sets = metadata_sets};
#endif

static char log_buf[64];
static void loose_log_percents(const char* prefix, const float num, const bool debug) {
    const int32_t int_part = (int32_t) (num * 100);
    const int32_t frac_part = ((int32_t) (num * 100 * 100)) % 100;

    if (frac_part != 0) {
        sprintf(log_buf, "%s~%d.%d%%", prefix, int_part, frac_part);
    } else {
        sprintf(log_buf, "%s%d%%", prefix, int_part);
    }

    if (debug) {
        LOG_DBG("%s", log_buf);
    } else {
        LOG_INF("%s", log_buf);
    }
}

static int p2sm_detect_drift(const char* dev_name, const float min) {
    const struct device *dev = zmk_behavior_get_binding(dev_name);
    if (dev == NULL) {
        LOG_ERR("Device not found!");
        return -1;
    }

    const struct behavior_p2sm_sens_config *cfg = dev->config;
    if (cfg == NULL || cfg->step == 0 || cfg->min_step == 0 || cfg->max_step == 0) {
        LOG_ERR("Invalid configuration!");
        return -1;
    }

    const float one_step = (float) cfg->step / 1000.0f;
    if (one_step * 100.0f < ((float) CONFIG_POINTER_2S_MIXER_SENS_DRIFT_CORRECTION) / 10.0f) {
        LOG_WRN("Drift correction is not possible, consider bigger steps");
        return 0;
    }

    const float current = cfg->scroll ? p2sm_get_yaw_coef() : p2sm_get_move_coef();
    const int steps_count = (int) (current * 1000.0f / cfg->step - 0.5f);
    const uint16_t d_drift = fabs(current - (float) steps_count * one_step) * 1000.0f;

    loose_log_percents("  > Now: ", current, true);
    LOG_DBG("  > Current step: %d", steps_count);
    LOG_DBG("  > Step size: %d/%d", cfg->step, cfg->max_multiplier * 1000);
    LOG_DBG("  > Drift: %d", cfg->step - d_drift);

    if (cfg->step - d_drift > CONFIG_POINTER_2S_MIXER_SENS_DRIFT_CORRECTION) {
        float closest = (float) (steps_count * cfg->step) / 1000.0f;
        if (closest < min) {
            closest = min;
        }

        LOG_WRN("Sensitivity drift detected!");
        loose_log_percents("Setting to the closest correct value: ", closest, false);

        if (cfg->scroll) {
            p2sm_set_yaw_coef(closest);
        } else {
            p2sm_set_move_coef(closest);
        }

        return 1;
    }

    return 0;
}

static float find_min_value(const char* dev_name) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(dev_name)->config;
    return (float) cfg->min_step * (float) cfg->step / 1000.0f;
}

static float find_max_value(const char* dev_name) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(dev_name)->config;
    float max_value = (float) cfg->max_step * (float) cfg->step / 1000.0f;
    if (!cfg->scroll && max_value > 1.0f) {
        max_value = 1.0f;
    }
    if (max_value > (float) cfg->max_multiplier) {
        max_value = (float) cfg->max_multiplier;
    }

    return max_value;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
static int on_p2sm_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(binding->behavior_dev)->config;
    const float min_value = find_min_value(binding->behavior_dev);
    const float max_value = find_max_value(binding->behavior_dev);
    const bool direction = binding->param1 & P2SM_INC;
    const float current = cfg->scroll ?  p2sm_get_yaw_coef() : p2sm_get_move_coef();
    const int8_t steps = (int32_t) binding->param2;

    if (p2sm_detect_drift(binding->behavior_dev, min_value)) {
        LOG_DBG("Cycling despite drift…");
    }

    float new_val = current + (float) cfg->step * (float) steps / 1000.0f * (direction ? 1.0f : -1.0f);
    if (new_val > max_value || new_val > cfg->max_multiplier * 1.0f) {
        new_val = min_value;
        LOG_DBG("Sensitivity wrapped around to minimum value");
    } else if (new_val < min_value) {
        new_val = max_value;
        LOG_DBG("Sensitivity wrapped around to maximum value");
    }

    if (direction) {
        LOG_DBG("Sensitivity increased by %d step(s)", steps);
    } else {
        LOG_DBG("Sensitivity decreased by %d step(s)", steps);
    }

    loose_log_percents("New sensitivity: ", new_val, false);

    if (cfg->scroll) {
        p2sm_set_yaw_coef(new_val);
    } else {
        p2sm_set_move_coef(new_val);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

void p2sm_sens_driver_init() {
    if (initialized) {
        LOG_ERR("Sensitivity driver already initialized!");
        return;
    }

    LOG_INF("Initializing sensitivity cycling driver…");
    if (g_from_settings[0] != -1 && g_from_settings[1] != -1) {
        p2sm_set_move_coef(g_from_settings[0]);
        p2sm_set_yaw_coef(g_from_settings[1]);
    } else {
        LOG_WRN("Sensitivity values not found in settings");
    }

    for (int i = 0; i < MAX_DEVICES; i++) {
        if (g_devices[i] != NULL) {
            p2sm_detect_drift(g_devices[i], find_min_value(g_devices[i]));
        }
    }

    initialized = true;
}

static int behavior_p2sm_sens_init(const struct device *dev) {
    const struct behavior_p2sm_sens_config *cfg = zmk_behavior_get_binding(dev->name)->config;
    if (cfg->step == 0 || cfg->max_multiplier == 0 || cfg->min_step == 0 || cfg->max_step == 0) {
        LOG_ERR("Invalid configuration: 0 is not a valid parameter");
        return -1;
    }

    if (cfg->min_step >= cfg->max_step) {
        LOG_ERR("Invalid configuration: max_step ≤ min_step");
        return -1;
    }

    g_devices[g_dev_num++] = dev->name;
    return 0;
}

static const struct behavior_driver_api behavior_p2sm_sens_driver_api = {
    .binding_pressed = on_p2sm_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define P2SM_INST(n)                                                                                \
    static const struct behavior_p2sm_sens_config behavior_p2sm_sens_config_##n = {                 \
        .step = DT_INST_PROP(n, step),                                                              \
        .max_multiplier = DT_INST_PROP_OR(n, max_multiplier, 1),                                    \
        .min_step = DT_INST_PROP_OR(n, min_step, 1),                                                \
        .max_step = DT_INST_PROP_OR(n, max_step, 1000),                                             \
        .scroll = DT_INST_PROP_OR(n, scroll, false),                                                \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_p2sm_sens_init, NULL, NULL, &behavior_p2sm_sens_config_##n, \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_sens_driver_api);

DT_INST_FOREACH_STATUS_OKAY(P2SM_INST)

#if IS_ENABLED(CONFIG_SETTINGS)
// ReSharper disable once CppParameterMayBeConst
static int p2sm_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const int err = read_cb(cb_arg, g_from_settings, sizeof(g_from_settings));
    if (err < 0) {
        LOG_ERR("Failed to load settings (err = %d)", err);
    }

    if (initialized) {
        LOG_WRN("Unexpected: settings loaded after driver initialization, drift will not be detected!");
        p2sm_set_move_coef(g_from_settings[0]);
        p2sm_set_yaw_coef(g_from_settings[1]);
    }

    return err;
}

SETTINGS_STATIC_HANDLER_DEFINE(sensor_attr_cycle, P2SM_SETTINGS_PREFIX, NULL, p2sm_settings_load_cb, NULL, NULL);
#endif

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
