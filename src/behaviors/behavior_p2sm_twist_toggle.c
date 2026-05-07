#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "drivers/behavior.h"
#include "drivers/p2sm_runtime.h"
#include "dt-bindings/zmk/p2sm.h"
#include "zephyr/logging/log.h"
#include "zmk/behavior.h"
#if IS_ENABLED(CONFIG_ZMK_FEEDBACK_COMMON)
#include <zmk/feedback_common/feedback_gpio.h>
#endif

#define DT_DRV_COMPAT zmk_behavior_p2sm_twist_toggle
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_p2sm_twist_toggle_config {
    const uint16_t feedback_duration;
};

static int on_p2sm_twist_toggle_binding_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct device* dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_p2sm_twist_toggle_config *cfg = dev->config;
    p2sm_toggle_twist();

#if IS_ENABLED(CONFIG_ZMK_FEEDBACK_COMMON)
    if (cfg->feedback_duration > 0) {
        fbc_trigger(cfg->feedback_duration);
    }
#else
    ARG_UNUSED(cfg);
#endif

    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_p2sm_twist_toggle_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct behavior_driver_api behavior_p2sm_twist_toggle_driver_api = {
    .binding_pressed = on_p2sm_twist_toggle_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define P2SM_TWIST_TOGGLE_INST(n)                                                                              \
    static const struct behavior_p2sm_twist_toggle_config behavior_p2sm_twist_toggle_config_##n = {               \
        .feedback_duration = DT_INST_PROP_OR(n, feedback_duration, 0),                                        \
    };                                                                                                      \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_p2sm_twist_toggle_init, NULL, NULL,                                   \
        &behavior_p2sm_twist_toggle_config_##n, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_p2sm_twist_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(P2SM_TWIST_TOGGLE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
