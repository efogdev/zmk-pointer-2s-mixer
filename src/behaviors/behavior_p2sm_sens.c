#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "drivers/behavior.h"
#include "drivers/p2sm_runtime.h"
#include "dt-bindings/zmk/p2sm.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/logging/log.h"
#include "zephyr/settings/settings.h"
#include "zmk/behavior.h"

#define DT_DRV_COMPAT zmk_behavior_p2sm_sens
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static bool initialized = false;
static uint8_t g_dev_num = 0;
static const char* g_devices[CONFIG_POINTER_2S_MIXER_SENS_MAX_DEVICES] = { NULL };
static float g_from_settings[2] = { -1, -1 };

struct behavior_p2sm_sens_config {
    const bool scroll;
    const bool wrap;

    const uint32_t step;
    const uint32_t min_step, max_step;
    const uint32_t max_multiplier;

    const struct gpio_dt_spec feedback_gpios;
    const uint32_t feedback_duration;
};

struct behavior_p2sm_sens_data {
    const struct device *dev;
    struct k_work_delayable feedback_off_work;
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

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SENS_LOG_EN)
static char log_buf[64];
static void log_sensitivity(const char* prefix, const float num, const bool debug) {
    const int32_t int_part = (int32_t) (num * 100);
    const int32_t frac_part = (int32_t) (num * 100 * 100) % 100;

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
#endif

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
    if (one_step * 200.0f < ((float) CONFIG_POINTER_2S_MIXER_SENS_DRIFT_CORRECTION) / 10.0f) {
        LOG_WRN("Drift correction is not possible, consider bigger steps");
        return 0;
    }

    const float current = cfg->scroll ? p2sm_get_twist_coef() : p2sm_get_move_coef();
    const int steps_count = (int) (current * 1000.0f / cfg->step - .5f);
    const uint16_t d_drift = fabs(current - (float) steps_count * one_step) * 1000.0f;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SENS_LOG_EN)
    log_sensitivity("  > Now: ", current, true);
    LOG_DBG("  > Current step: %d", steps_count + 1);
    LOG_DBG("  > Step size: %d/%d", cfg->step, cfg->max_multiplier * 1000);
    LOG_DBG("  > Drift: %d", cfg->step - d_drift);
#endif

    if (cfg->step - d_drift > CONFIG_POINTER_2S_MIXER_SENS_DRIFT_CORRECTION) {
        float closest = (float) (steps_count * cfg->step) / 1000.0f;
        if (closest < min) {
            closest = min;
        }

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SENS_LOG_EN)
        LOG_WRN("Sensitivity drift detected!");
        log_sensitivity("Setting to the closest correct value: ", closest, false);
#endif

        if (cfg->scroll) {
            p2sm_set_twist_coef(closest);
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
    const struct device* dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_p2sm_sens_config *cfg = dev->config;
    struct behavior_p2sm_sens_data *data = dev->data;

    const float min_value = find_min_value(binding->behavior_dev);
    const float max_value = find_max_value(binding->behavior_dev);
    const bool direction = binding->param1 & P2SM_INC;
    const int8_t steps = binding->param2 != 0 ? (int32_t) binding->param2 : 1;
    float current = cfg->scroll ? p2sm_get_twist_coef() : p2sm_get_move_coef();

    if (p2sm_detect_drift(binding->behavior_dev, min_value)) {
        LOG_DBG("Cycling despite drift…");
        current = cfg->scroll ? p2sm_get_twist_coef() : p2sm_get_move_coef();
    }

    float new_val = current + (float) cfg->step * (float) steps / 1000.0f * (direction ? 1.0f : -1.0f);
    if (cfg->wrap) {
        if (new_val > max_value || new_val > (float) cfg->max_multiplier) {
            LOG_DBG("Sensitivity wrapped around");
            new_val = min_value;

            // specifically for toggle, because... reasons
            if (fabs(current - new_val) <= 1e-6) {
                new_val = max_value;
            }
        } else if (new_val < min_value) {
            LOG_DBG("Sensitivity wrapped around");
            new_val = max_value;
        }
    } else {
        if (direction && (new_val > max_value || new_val > (float) cfg->max_multiplier)) {
            new_val = max_value;
        } else if (!direction && new_val < min_value) {
            new_val = min_value;
        }
    }

    LOG_DBG("Sensitivity %s by %d step(s)", direction ? "increased" : "decreased", steps);
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SENS_LOG_EN)
    log_sensitivity(cfg->scroll ? "Scroll sensitivity: " : "Pointer sensitivity: ", new_val, false);
#endif

    if (cfg->scroll) {
        p2sm_set_twist_coef(new_val);
    } else {
        p2sm_set_move_coef(new_val);
    }

    if (cfg->feedback_duration > 0 && cfg->feedback_gpios.port != NULL) {
        if (gpio_pin_set_dt(&cfg->feedback_gpios, 1) == 0) {
            LOG_DBG("Feedback turned on");
            k_work_reschedule(&data->feedback_off_work, K_MSEC(cfg->feedback_duration));
        } else {
            LOG_ERR("Failed to enable the feedback");
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static void feedback_off_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct behavior_p2sm_sens_data *data = CONTAINER_OF(dwork, struct behavior_p2sm_sens_data, feedback_off_work);
    const struct device *dev = data->dev;
    const struct behavior_p2sm_sens_config *config = dev->config;

    gpio_pin_set_dt(&config->feedback_gpios, 0);
    LOG_DBG("Feedback turned off");
}

void p2sm_sens_driver_init() {
    if (initialized) {
        LOG_ERR("Sensitivity driver already initialized!");
        return;
    }

    LOG_INF("Initializing sensitivity cycling driver…");
    if (g_from_settings[0] != -1 && g_from_settings[1] != -1) {
        p2sm_set_move_coef(g_from_settings[0]);
        p2sm_set_twist_coef(g_from_settings[1]);
    } else {
        LOG_WRN("Sensitivity values not found in settings");
    }

    for (int i = 0; i < CONFIG_POINTER_2S_MIXER_SENS_MAX_DEVICES; i++) {
        if (g_devices[i] != NULL) {
            p2sm_detect_drift(g_devices[i], find_min_value(g_devices[i]));
        }
    }

    initialized = true;
}

static int behavior_p2sm_sens_init(const struct device *dev) {
    const struct behavior_p2sm_sens_config *cfg = dev->config;
    struct behavior_p2sm_sens_data *data = dev->data;

    if (cfg->step == 0 || cfg->max_multiplier == 0 || cfg->min_step == 0 || cfg->max_step == 0) {
        LOG_ERR("Invalid configuration: 0 is not a valid parameter");
        return -1;
    }

    if (cfg->min_step >= cfg->max_step) {
        LOG_ERR("Invalid configuration: max_step ≤ min_step");
        return -1;
    }

    const float max_value = (cfg->scroll ? (float) cfg->max_multiplier : 1.0f) / 1000.0f * (float) cfg->step * (float) cfg->max_step;
    if (max_value > (cfg->scroll ? (float) cfg->max_multiplier : 1.0f)) {
        LOG_WRN("Warning: max_step is unreachable");
    }

    if (cfg->feedback_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&cfg->feedback_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure sensitivity feedback GPIO");
        } else {
            LOG_DBG("Sensitivity feedback GPIO configured");
        }

        k_work_init_delayable(&data->feedback_off_work, feedback_off_work_cb);
    } else {
        LOG_DBG("No feedback set up for sensitivity cycling");
    }

    data->dev = dev;
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
    static struct behavior_p2sm_sens_data behavior_p2sm_sens_data_##n = {};                         \
    static const struct behavior_p2sm_sens_config behavior_p2sm_sens_config_##n = {                 \
        .step = DT_INST_PROP(n, step),                                                              \
        .wrap = DT_INST_PROP_OR(n, wrap, true),                                                     \
        .max_multiplier = DT_INST_PROP_OR(n, max_multiplier, 1),                                    \
        .min_step = DT_INST_PROP_OR(n, min_step, 1),                                                \
        .max_step = DT_INST_PROP_OR(n, max_step, 1000),                                             \
        .scroll = DT_INST_PROP_OR(n, scroll, false),                                                \
        .feedback_gpios = GPIO_DT_SPEC_INST_GET_OR(n, feedback_gpios, { .port = NULL }),            \
        .feedback_duration = DT_INST_PROP_OR(n, feedback_duration, 0),                              \
    };                                                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_p2sm_sens_init, NULL, &behavior_p2sm_sens_data_##n,         \
        &behavior_p2sm_sens_config_##n, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_sens_driver_api);

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
        p2sm_set_twist_coef(g_from_settings[1]);
    }

    return err;
}

SETTINGS_STATIC_HANDLER_DEFINE(sensor_attr_cycle, P2SM_SETTINGS_PREFIX, NULL, p2sm_settings_load_cb, NULL, NULL);
#endif

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
