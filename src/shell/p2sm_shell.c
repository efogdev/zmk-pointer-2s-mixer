#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>

#include "drivers/p2sm_runtime.h"
#include "zmk/keymap.h"
#include "zmk/matrix.h"
#include "zmk/studio/core.h"

#define DT_DRV_COMPAT zmk_p2sm_shell
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_SHELL) && IS_ENABLED(CONFIG_ZMK_P2SM_SHELL)
#define shprint(_sh, _fmt, ...) \
do { \
  if ((_sh) != NULL) \
    shell_print((_sh), _fmt, ##__VA_ARGS__); \
} while (0)

static char* ftoi(const float num) {
    const int32_t int_part = (int32_t) (num * 100);
    const int32_t frac_part = (int32_t) (num * 100 * 100) % 100;
    char* log_buf = malloc(12 * sizeof(char));
    if (frac_part != 0) {
        sprintf(log_buf, "~%d.%02d%%", int_part, frac_part);
    } else {
        sprintf(log_buf, "%d%%", int_part);
    }

    return log_buf;
}

static int cmd_sens(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 3) {
        shprint(sh, "Usage: p2sm sens <pointer|twist> <get|set> [value]\n");
        return -EINVAL;
    }

    bool is_pointer = false;
    if (strcmp(argv[1], "pointer") == 0) {
        is_pointer = true;
    } else if (strcmp(argv[1], "twist") == 0) {} else {
        shprint(sh, "Usage: p2sm sens <pointer|twist> <get|set> [value]\n");
        return -EINVAL;
    }

    if (strcmp(argv[2], "get") == 0) {
        const float val = is_pointer ? p2sm_get_move_coef() : p2sm_get_twist_coef();
        shprint(sh, "%d (%s)", (int) (val * 1000), ftoi(val));
    } else if (strcmp(argv[2], "set") == 0) {
        if (argc < 4) {
            shprint(sh, "Usage: p2sm sens <pointer|twist> <get|set> [value]\n");
            return -EINVAL;
        }

        char *endptr;
        const uint16_t parsed = strtoul(argv[3], &endptr, 10);

        if (is_pointer) {
            p2sm_set_move_coef((float) parsed / 1000);
        } else {
            p2sm_set_twist_coef((float) parsed / 1000);
        }

        const float val = is_pointer ? p2sm_get_move_coef() : p2sm_get_twist_coef();
        shprint(sh, "Set: %d (%s)", (int) (val * 1000), ftoi(val));
    } else {
        shprint(sh, "Usage: p2sm sens <pointer|twist> <get|set> [value]\n");
        return -EINVAL;
    }

    return 0;
}

static int cmd_twist(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 2) {
        shprint(sh, "Usage: p2sm twist <on|off|toggle|reverse>\n");
        return -EINVAL;
    }

    const bool en = p2sm_twist_enabled();
    if (strcmp(argv[1], "on") == 0) {
        if (!en) p2sm_toggle_twist();
    } else if (strcmp(argv[1], "off") == 0) {
        if (en) p2sm_toggle_twist();
    } else if (strcmp(argv[1], "toggle") == 0) {
        p2sm_toggle_twist();
    } else if (strcmp(argv[1], "reverse") == 0) {
        p2sm_toggle_twist_reverse();
    } else {
        shprint(sh, "Usage: p2sm twist <on|off|toggle|reverse>\n");
        return -EINVAL;
    }

    return 0;
}

static int cmd_status(const struct shell *sh, const size_t argc, char **argv) {
    shprint(sh, "----- General -----");
    shprint(sh, "Twist scroll: %s", p2sm_twist_enabled() ? "enabled" : "disabled");
    shprint(sh, "Twist reversed: %s", p2sm_twist_is_reversed() ? "yes" : "no");
    shprint(sh, "");

    shprint(sh, "----- Sensitivity -----");
    shprint(sh, "Pointer: %s", ftoi(p2sm_get_move_coef()));
    shprint(sh, "Twist scroll: %s", ftoi(p2sm_get_twist_coef()));
    shprint(sh, "");

    shprint(sh, "----- Behaviors -----");
    const uint8_t num_behaviors = p2sm_sens_num_behaviors();
    shprint(sh, "Number of behaviors: %d", num_behaviors);
    
    for (uint8_t i = 0; i < num_behaviors; i++) {
        const struct p2sm_sens_behavior_config cfg = p2sm_sens_behavior_get_config(i);
        shprint(sh, "");
        shprint(sh, "[ID %d] %s%s", i, cfg.display_name, cfg.scroll ? " [scroll]" : "");
        shprint(sh, "  step: %d", cfg.step);
        shprint(sh, "  min_step: %d, max_step: %d", cfg.min_step, cfg.max_step);
        shprint(sh, "  max_multiplier: %d", cfg.max_multiplier);
        shprint(sh, "  wrap: %s", cfg.wrap ? "true" : "false");
        shprint(sh, "  feedback_on_limit: %s", cfg.feedback_on_limit ? "true" : "false");
        shprint(sh, "  feedback_duration: %d", cfg.feedback_duration);
        if (cfg.feedback_wrap_pattern_len > 0 && (cfg.wrap || cfg.feedback_on_limit)) {
            char pattern_str[128] = {0};
            int offset = 0;
            offset += snprintf(pattern_str + offset, sizeof(pattern_str) - offset, "[");
            for (uint8_t j = 0; j < cfg.feedback_wrap_pattern_len; j++) {
                if (j > 0) {
                    offset += snprintf(pattern_str + offset, sizeof(pattern_str) - offset, ", ");
                }
                offset += snprintf(pattern_str + offset, sizeof(pattern_str) - offset, "%d", cfg.feedback_wrap_pattern[j]);
            }
            snprintf(pattern_str + offset, sizeof(pattern_str) - offset, "]");
            shprint(sh, "  feedback_wrap_pattern: %s", pattern_str);
        }
    }

    return 0;
}

static int cmd_behavior_set(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 10) {
        shprint(sh, "Usage: p2sm behavior set <id> <step> <min_step> <max_step> <max_mult> <wrap> <fb_on_limit> <fb_duration> <fb_pattern_len> [pattern_values...]");
        shprint(sh, "  id: behavior index (0-%d)", p2sm_sens_num_behaviors() - 1);
        shprint(sh, "  step: step size (1-1000)");
        shprint(sh, "  min_step: minimum step");
        shprint(sh, "  max_step: maximum step");
        shprint(sh, "  max_mult: maximum multiplier");
        shprint(sh, "  wrap: wrap around (0/1)");
        shprint(sh, "  fb_on_limit: feedback on limit (0/1)");
        shprint(sh, "  fb_duration: feedback duration in ms");
        shprint(sh, "  fb_pattern_len: feedback pattern length");
        shprint(sh, "  pattern_values: pattern values (if pattern_len > 0)");
        return -EINVAL;
    }

    char *endptr;
    const uint8_t id = strtoul(argv[1], &endptr, 10);
    
    if (id >= p2sm_sens_num_behaviors()) {
        shprint(sh, "Error: Invalid behavior id %d (max: %d)", id, p2sm_sens_num_behaviors() - 1);
        return -EINVAL;
    }

    const struct p2sm_sens_behavior_config orig_cfg = p2sm_sens_behavior_get_config(id);

    struct p2sm_sens_behavior_config cfg = {
        .step = (uint16_t)strtoul(argv[2], &endptr, 10),
        .min_step = (uint16_t)strtoul(argv[3], &endptr, 10),
        .max_step = (uint16_t)strtoul(argv[4], &endptr, 10),
        .max_multiplier = (uint8_t)strtoul(argv[5], &endptr, 10),
        .wrap = (bool)strtoul(argv[6], &endptr, 10),
        .feedback_on_limit = (bool)strtoul(argv[7], &endptr, 10),
        .feedback_duration = (uint16_t)strtoul(argv[8], &endptr, 10),
        .feedback_wrap_pattern_len = (uint8_t)strtoul(argv[9], &endptr, 10),
        .scroll = orig_cfg.scroll,
        .display_name = orig_cfg.display_name,
    };

    memset(cfg.feedback_wrap_pattern, 0, sizeof(cfg.feedback_wrap_pattern));
    if (cfg.feedback_wrap_pattern_len > 0) {
        if (argc < 10 + cfg.feedback_wrap_pattern_len) {
            shprint(sh, "Error: Not enough pattern values. Expected %d, got %d", 
                    cfg.feedback_wrap_pattern_len, argc - 10);
            return -EINVAL;
        }

        if (cfg.feedback_wrap_pattern_len > CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_ARR_VALUES) {
            shprint(sh, "Error: Pattern length %d exceeds max %d", 
                    cfg.feedback_wrap_pattern_len, CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_ARR_VALUES);
            return -EINVAL;
        }

        for (uint8_t i = 0; i < cfg.feedback_wrap_pattern_len; i++) {
            cfg.feedback_wrap_pattern[i] = strtol(argv[10 + i], &endptr, 10);
        }
    } else {
        for (uint8_t i = 0; i < CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_ARR_VALUES; i++) {
            cfg.feedback_wrap_pattern[i] = 0;
        }
    }

    const int ret = p2sm_sens_behavior_set_config(id, cfg);
    if (ret == 0) {
        shprint(sh, "Behavior %d configuration updated successfully", id);
    } else {
        shprint(sh, "Failed to update behavior %d configuration (error: %d)", id, ret);
    }

    return ret;
}

static int cmd_behavior_save(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 2) {
        shprint(sh, "Usage: p2sm behavior save <all|id>");
        shprint(sh, "  all: save all behaviors");
        shprint(sh, "  id: save specific behavior (0-%d)", p2sm_sens_num_behaviors() - 1);
        return -EINVAL;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    if (strcmp(argv[1], "all") == 0) {
        p2sm_sens_behaviors_save_all();
    } else {
        char *endptr;
        const uint8_t id = strtoul(argv[1], &endptr, 10);
        
        if (id >= p2sm_sens_num_behaviors()) {
            shprint(sh, "Error: Invalid behavior id %d (max: %d)", id, p2sm_sens_num_behaviors() - 1);
            return -EINVAL;
        }
        
        p2sm_sens_behaviors_save_all();
    }

    shprint(sh, "Done.");
    return 0;
#else
    shprint(sh, "Error: Settings support not enabled");
    return -ENOTSUP;
#endif
}

static int cmd_behavior_load(const struct shell *sh, size_t argc, char **argv) {
    p2sm_sens_load_and_apply_behaviors_config();
    shprint(sh, "Done.");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_behavior,
    SHELL_CMD(set, NULL, "Set behavior configuration", cmd_behavior_set),
    SHELL_CMD(save, NULL, "Save behavior configuration", cmd_behavior_save),
    SHELL_CMD(load, NULL, "Load behavior configuration", cmd_behavior_load),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_p2sm,
    SHELL_CMD(status, NULL, "Show current configuration", cmd_status),
    SHELL_CMD(twist, NULL, "Change status of twist scroll", cmd_twist),
    SHELL_CMD(sens, NULL, "Change sensitivity", cmd_sens),
    SHELL_CMD(behavior, &sub_behavior, "Manage behaviors", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(p2sm, &sub_p2sm, "Sensor mixer configuration", NULL);
#endif
