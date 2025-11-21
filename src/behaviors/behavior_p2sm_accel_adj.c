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

#define DT_DRV_COMPAT zmk_behavior_p2sm_accel_adj
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static bool initialized = false;
static uint8_t g_dev_num = 0;
static const char* g_devices[CONFIG_POINTER_2S_MIXER_SENS_MAX_DEVICES] = { NULL };
static float g_from_settings[2] = { -1, -1 };

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
struct behavior_p2sm_accel_adj_config {
    const bool wrap;

    const uint16_t step;
    const uint16_t min_step, max_step;
    const uint8_t max_multiplier;

    const struct gpio_dt_spec feedback_gpios;
    const struct gpio_dt_spec feedback_extra_gpios;
    const uint16_t feedback_duration;
    const uint8_t feedback_wrap_pattern_len;
    const int feedback_wrap_pattern[CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_ARR_VALUES];
};

struct behavior_p2sm_accel_adj_data {
    const struct device *dev;
    struct k_work_delayable feedback_off_work;
    struct k_work_delayable feedback_pattern_work;
    int previous_feedback_extra_state;
    uint8_t current_pattern_index;
    bool pattern_active;
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
static void log_accel_value(const char* prefix, const float num, const bool debug) {
    const int32_t int_part = (int32_t) (num * 100);
    const int32_t frac_part = (int32_t) (num * 100 * 100) % 100;

    if (frac_part != 0) {
        sprintf(log_buf, "%s~%d.%02d", prefix, int_part, frac_part);
    } else {
        sprintf(log_buf, "%s%d", prefix, int_part);
    }

    if (debug) {
        LOG_DBG("%s", log_buf);
    } else {
        LOG_INF("%s", log_buf);
    }
}
#endif

static float find_min_accel_value(const char* dev_name) {
    const struct behavior_p2sm_accel_adj_config *cfg = zmk_behavior_get_binding(dev_name)->config;
    return (float) cfg->min_step * (float) cfg->step / 1000.0f;
}

static float find_max_accel_value(const char* dev_name) {
    const struct behavior_p2sm_accel_adj_config *cfg = zmk_behavior_get_binding(dev_name)->config;
    float max_value = (float) cfg->max_step * (float) cfg->step / 1000.0f;
    if (max_value > (float) cfg->max_multiplier) {
        max_value = (float) cfg->max_multiplier;
    }

    return max_value;
}

static int on_p2sm_accel_adj_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct device* dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_p2sm_accel_adj_config *cfg = dev->config;
    struct behavior_p2sm_accel_adj_data *data = dev->data;

    const float min_value = find_min_accel_value(binding->behavior_dev);
    const float max_value = find_max_accel_value(binding->behavior_dev);
    const bool direction = binding->param1 & P2SM_INC;
    const int8_t steps = binding->param2 != 0 ? (int32_t) binding->param2 : 1;
    const float current = p2sm_get_twist_accel_value();

    bool wrapped = false;
    float new_val = current + (float) cfg->step * (float) steps / 1000.0f * (direction ? 1.0f : -1.0f);
    if (cfg->wrap) {
        if (new_val > max_value || new_val > (float) cfg->max_multiplier) {
            LOG_DBG("Acceleration value wrapped around");
            new_val = min_value;
            wrapped = true;

            if (fabsf(current - new_val) <= 1e-6f) {
                new_val = max_value;
            }
        } else if (new_val < min_value) {
            LOG_DBG("Acceleration value wrapped around");
            new_val = max_value;
            wrapped = true;
        }
    } else {
        if (direction && (new_val > max_value || new_val > (float) cfg->max_multiplier)) {
            new_val = max_value;
            wrapped = true;
        } else if (!direction && new_val < min_value) {
            new_val = min_value;
            wrapped = true;
        }
    }

    LOG_DBG("Acceleration %s by %d step(s)", direction ? "increased" : "decreased", steps);
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SENS_LOG_EN)
    log_accel_value("Scroll acceleration: ", fabsf(new_val / 100), false);
#endif

    p2sm_set_twist_accel_value(new_val);

    if (cfg->feedback_duration > 0 && cfg->feedback_gpios.port != NULL) {
        if (cfg->feedback_extra_gpios.port != NULL) {
            data->previous_feedback_extra_state = gpio_pin_get_dt(&cfg->feedback_extra_gpios);
            gpio_pin_set_dt(&cfg->feedback_extra_gpios, 1);
        }

        if (wrapped && cfg->feedback_wrap_pattern_len > 0) {
            data->pattern_active = true;
            data->current_pattern_index = 0;
            
            if (cfg->feedback_wrap_pattern_len > 0) {
                const int pattern_duration = cfg->feedback_wrap_pattern[0];
                if (gpio_pin_set_dt(&cfg->feedback_gpios, 1) == 0) {
                    data->current_pattern_index = 1;
                    k_work_reschedule(&data->feedback_pattern_work, K_MSEC(pattern_duration));
                    LOG_DBG("Starting feedback wrap pattern: duration=%d",
                            pattern_duration);
                } else {
                    LOG_ERR("Failed to enable the feedback pattern");
                    data->pattern_active = false;
                }
            }
        } else {
            if (gpio_pin_set_dt(&cfg->feedback_gpios, 1) == 0) {
                k_work_reschedule(&data->feedback_off_work, K_MSEC(cfg->feedback_duration));
            } else {
                LOG_ERR("Failed to enable the feedback");
            }
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static void feedback_off_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct behavior_p2sm_accel_adj_data *data = CONTAINER_OF(dwork, struct behavior_p2sm_accel_adj_data, feedback_off_work);
    const struct device *dev = data->dev;
    const struct behavior_p2sm_accel_adj_config *config = dev->config;

    if (data->pattern_active) {
        if (config->feedback_gpios.port != NULL) {
            gpio_pin_set_dt(&config->feedback_gpios, 0);
        }

        LOG_DBG("Feedback pattern step completed");
        return;
    }

    if (config->feedback_extra_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
    }

    if (config->feedback_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_gpios, 0);
    }

    LOG_DBG("Feedback turned off");
}

static void feedback_pattern_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_p2sm_accel_adj_data *data = CONTAINER_OF(dwork, struct behavior_p2sm_accel_adj_data, feedback_pattern_work);
    const struct device *dev = data->dev;
    const struct behavior_p2sm_accel_adj_config *config = dev->config;

    if (!data->pattern_active) {
        return;
    }

    if (data->current_pattern_index >= config->feedback_wrap_pattern_len) {
        data->pattern_active = false;
        
        if (config->feedback_extra_gpios.port != NULL) {
            gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
        }
        
        if (config->feedback_gpios.port != NULL) {
            gpio_pin_set_dt(&config->feedback_gpios, 0);
        }
        
        LOG_DBG("Feedback pattern completed");
        return;
    }

    const int pattern_duration = config->feedback_wrap_pattern[data->current_pattern_index];
    const int pin_state = (data->current_pattern_index % 2 == 1) ? 0 : 1;
    
    if (config->feedback_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_gpios, pin_state);
    }
    
    LOG_DBG("Feedback pattern step %d: state=%d, duration=%d",
            data->current_pattern_index, pin_state, pattern_duration);
    
    data->current_pattern_index++;
    k_work_reschedule(&data->feedback_pattern_work, K_MSEC(pattern_duration));
}

static int behavior_p2sm_accel_adj_init(const struct device *dev) {
    const struct behavior_p2sm_accel_adj_config *cfg = dev->config;
    struct behavior_p2sm_accel_adj_data *data = dev->data;

    if (cfg->step == 0 || cfg->max_multiplier == 0 || cfg->min_step == 0 || cfg->max_step == 0) {
        LOG_ERR("Invalid configuration: 0 is not a valid parameter");
        return -1;
    }

    if (cfg->min_step >= cfg->max_step) {
        LOG_ERR("Invalid configuration: max_step ≤ min_step");
        return -1;
    }

    const float max_value = (float) cfg->max_multiplier / 1000.0f * (float) cfg->step * (float) cfg->max_step;
    if (max_value > (float) cfg->max_multiplier) {
        LOG_WRN("Warning: max_step is unreachable");
    }

    if (cfg->feedback_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&cfg->feedback_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure acceleration adjustment feedback GPIO");
        } else {
            LOG_DBG("Acceleration adjustment feedback GPIO configured");
        }

        k_work_init_delayable(&data->feedback_off_work, feedback_off_work_cb);
        k_work_init_delayable(&data->feedback_pattern_work, feedback_pattern_work_cb);
    } else {
        LOG_DBG("No feedback set up for acceleration adjustment");
    }

    if (cfg->feedback_extra_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&cfg->feedback_extra_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure acceleration adjustment extra feedback GPIO");
        } else {
            LOG_DBG("Acceleration adjustment extra feedback GPIO configured");
        }
    } else {
        LOG_DBG("No extra feedback set up for acceleration adjustment");
    }

    g_devices[g_dev_num++] = dev->name;
    data->dev = dev;
    return 0;
}

static const struct behavior_driver_api behavior_p2sm_accel_adj_driver_api = {
    .binding_pressed = on_p2sm_accel_adj_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define P2SM_ACCEL_ADJ_INST(n)                                                                              \
    static struct behavior_p2sm_accel_adj_data behavior_p2sm_accel_adj_data_##n = {};                       \
    static const struct behavior_p2sm_accel_adj_config behavior_p2sm_accel_adj_config_##n = {                   \
        .step = DT_INST_PROP(n, step),                                                                \
        .wrap = DT_INST_PROP_OR(n, wrap, true),                                                       \
        .max_multiplier = DT_INST_PROP_OR(n, max_multiplier, 1),                                      \
        .min_step = DT_INST_PROP_OR(n, min_step, 1),                                                  \
        .max_step = DT_INST_PROP_OR(n, max_step, 1000),                                               \
        .feedback_gpios = GPIO_DT_SPEC_INST_GET_OR(n, feedback_gpios, { .port = NULL }),              \
        .feedback_extra_gpios = GPIO_DT_SPEC_INST_GET_OR(n, feedback_extra_gpios, { .port = NULL }),  \
        .feedback_duration = DT_INST_PROP_OR(n, feedback_duration, 0),                                \
        .feedback_wrap_pattern_len = DT_INST_PROP_LEN_OR(n, feedback_wrap_pattern, 0),                \
        .feedback_wrap_pattern = DT_INST_PROP_OR(n, feedback_wrap_pattern, { 0 }),                    \
    };                                                                                                \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_p2sm_accel_adj_init, NULL, &behavior_p2sm_accel_adj_data_##n,   \
        &behavior_p2sm_accel_adj_config_##n, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_accel_adj_driver_api);

DT_INST_FOREACH_STATUS_OKAY(P2SM_ACCEL_ADJ_INST)

#if IS_ENABLED(CONFIG_SETTINGS)
// ReSharper disable once CppParameterMayBeConst
static int p2sm_accel_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const int err = read_cb(cb_arg, g_from_settings, sizeof(g_from_settings));
    if (err < 0) {
        LOG_ERR("Failed to load settings (err = %d)", err);
    }

    if (initialized) {
        p2sm_set_twist_accel_enabled(g_from_settings[0] > 0.5f);
        p2sm_set_twist_accel_value(g_from_settings[1]);
    }

    return err;
}

SETTINGS_STATIC_HANDLER_DEFINE(twist_accel_cycle, P2SM_ACCEL_SETTINGS_PREFIX, NULL, p2sm_accel_settings_load_cb, NULL, NULL);
#endif

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

void p2sm_accel_driver_init() {
    if (initialized) {
        LOG_ERR("Acceleration driver already initialized!");
        return;
    }

    LOG_DBG("Initializing acceleration cycling driver…");
    if (g_from_settings[0] != -1 && g_from_settings[1] != -1) {
        p2sm_set_twist_accel_enabled(g_from_settings[0] > 0.5f);
        p2sm_set_twist_accel_value(g_from_settings[1]);
    } else {
        LOG_DBG("Acceleration values not found in settings");
    }

    initialized = true;
}
