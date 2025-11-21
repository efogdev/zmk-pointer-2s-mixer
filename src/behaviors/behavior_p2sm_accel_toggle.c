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
#include "zmk/behavior.h"

#define DT_DRV_COMPAT zmk_behavior_p2sm_accel_toggle
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_p2sm_accel_toggle_config {
    const struct gpio_dt_spec feedback_gpios;
    const struct gpio_dt_spec feedback_extra_gpios;
    const uint16_t feedback_duration;
};

struct behavior_p2sm_accel_toggle_data {
    const struct device *dev;
    struct k_work_delayable feedback_off_work;
    int previous_feedback_extra_state;
};

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata mtd_param1_values[] = {
    {
        .display_name = "Enable",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = P2SM_ACCEL_EN,
    },
    {
        .display_name = "Disable",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = P2SM_ACCEL_DIS,
    },
    {
        .display_name = "Toggle",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = P2SM_ACCEL_TOGGLE,
    },
};

static const struct behavior_parameter_metadata_set profile_index_metadata_set = {
    .param1_values = mtd_param1_values,
    .param1_values_len = ARRAY_SIZE(mtd_param1_values),
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {profile_index_metadata_set};
static const struct behavior_parameter_metadata metadata = { .sets_len = ARRAY_SIZE(metadata_sets), .sets = metadata_sets};
#endif

static int on_p2sm_accel_toggle_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct device* dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_p2sm_accel_toggle_config *cfg = dev->config;
    struct behavior_p2sm_accel_toggle_data *data = dev->data;

    const uint8_t action = binding->param1;
    const bool current_state = p2sm_get_twist_accel_enabled();

    bool new_state;
    switch (action) {
        case P2SM_ACCEL_DIS:
            new_state = false;
            break;
        case P2SM_ACCEL_EN:
            new_state = true;
            break;
        case P2SM_ACCEL_TOGGLE:
            new_state = !current_state;
            break;
        default:
            new_state = !current_state;
            break;
    }

    p2sm_set_twist_accel_enabled(new_state);

    const char* action_str = (action == 0) ? "disabled" : (action == 1) ? "enabled" : (new_state ? "enabled" : "disabled");
    LOG_INF("Scroll acceleration %s", action_str);

    if (cfg->feedback_duration > 0 && cfg->feedback_gpios.port != NULL) {
        if (cfg->feedback_extra_gpios.port != NULL) {
            data->previous_feedback_extra_state = gpio_pin_get_dt(&cfg->feedback_extra_gpios);
            gpio_pin_set_dt(&cfg->feedback_extra_gpios, 1);
        }

        if (gpio_pin_set_dt(&cfg->feedback_gpios, 1) == 0) {
            k_work_reschedule(&data->feedback_off_work, K_MSEC(cfg->feedback_duration));
        } else {
            LOG_ERR("Failed to enable the feedback");
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static void feedback_off_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct behavior_p2sm_accel_toggle_data *data = CONTAINER_OF(dwork, struct behavior_p2sm_accel_toggle_data, feedback_off_work);
    const struct device *dev = data->dev;
    const struct behavior_p2sm_accel_toggle_config *config = dev->config;

    if (config->feedback_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_gpios, 0);
    }

    if (config->feedback_extra_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
    }

    LOG_DBG("Feedback turned off");
}

static int behavior_p2sm_accel_toggle_init(const struct device *dev) {
    const struct behavior_p2sm_accel_toggle_config *cfg = dev->config;
    struct behavior_p2sm_accel_toggle_data *data = dev->data;

    if (cfg->feedback_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&cfg->feedback_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure acceleration toggle feedback GPIO");
        } else {
            LOG_DBG("Acceleration toggle feedback GPIO configured");
        }

        k_work_init_delayable(&data->feedback_off_work, feedback_off_work_cb);
    } else {
        LOG_DBG("No feedback set up for acceleration toggle");
    }

    data->dev = dev;
    return 0;
}

static const struct behavior_driver_api behavior_p2sm_accel_toggle_driver_api = {
    .binding_pressed = on_p2sm_accel_toggle_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define P2SM_ACCEL_TOGGLE_INST(n)                                                                              \
    static struct behavior_p2sm_accel_toggle_data behavior_p2sm_accel_toggle_data_##n = {};                       \
    static const struct behavior_p2sm_accel_toggle_config behavior_p2sm_accel_toggle_config_##n = {               \
        .feedback_gpios = GPIO_DT_SPEC_INST_GET_OR(n, feedback_gpios, { .port = NULL }),                      \
        .feedback_extra_gpios = GPIO_DT_SPEC_INST_GET_OR(n, feedback_extra_gpios, { .port = NULL }),                      \
        .feedback_duration = DT_INST_PROP_OR(n, feedback_duration, 0),                                        \
    };                                                                                                      \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_p2sm_accel_toggle_init, NULL, &behavior_p2sm_accel_toggle_data_##n,   \
        &behavior_p2sm_accel_toggle_config_##n, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_accel_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(P2SM_ACCEL_TOGGLE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
