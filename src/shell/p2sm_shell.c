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
        shprint(sh, "Usage: p2sm twist <on|off|toggle>\n");
        return -EINVAL;
    }

    const bool en = p2sm_twist_enabled();
    if (strcmp(argv[1], "on") == 0) {
        if (!en) p2sm_toggle_twist();
    } else if (strcmp(argv[1], "off") == 0) {
        if (en) p2sm_toggle_twist();
    } else if (strcmp(argv[1], "toggle") == 0) {
        p2sm_toggle_twist();
    } else {
        shprint(sh, "Usage: p2sm twist <on|off|toggle>\n");
        return -EINVAL;
    }

    return 0;
}

static int cmd_status(const struct shell *sh, const size_t argc, char **argv) {
    shprint(sh, "----- General -----");
    shprint(sh, "Twist scroll: %s", p2sm_twist_enabled() ? "enabled" : "disabled");
    shprint(sh, "");

    shprint(sh, "----- Sensitivity -----");
    shprint(sh, "Pointer: %s", ftoi(p2sm_get_move_coef()));
    shprint(sh, "Twist scroll: %s", ftoi(p2sm_get_twist_coef()));

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_p2sm,
    SHELL_CMD(status, NULL, "Show current configuration", cmd_status),
    SHELL_CMD(twist, NULL, "Change status of twist scroll", cmd_twist),
    SHELL_CMD(sens, NULL, "Change sensitivity", cmd_sens),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(p2sm, &sub_p2sm, "Sensor mixer configuration", NULL);
#endif
