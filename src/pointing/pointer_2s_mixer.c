#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <math.h>
#include <dt-bindings/zmk/input_mixer.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>

#define DT_DRV_COMPAT zmk_pointer_2s_mixer
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct zip_pointer_2s_mixer_config {
    uint32_t sync_report_ms;
    uint32_t sync_report_yaw_ms;
    uint32_t yaw_div;

    uint32_t ball_radius;
    uint8_t sensor1_pos[3];
    uint8_t sensor2_pos[3];
};

struct zip_pointer_2s_mixer_data {
    const struct device *dev;

    int64_t last_rpt_time;
    int64_t last_rpt_time_yaw;

    int16_t rpt_x;
    int16_t rpt_y;
    int16_t rpt_yaw;

    float sensor1_surface_x;
    float sensor1_surface_y;
    float sensor1_surface_z;

    float sensor2_surface_x;
    float sensor2_surface_y;
    float sensor2_surface_z;

    int16_t sensor1_x;
    int16_t sensor2_x;

    int16_t sensor1_y;
    int16_t sensor2_y;
};

int line_sphere_intersection(float r, float x, float y, float z, float intersection[3]) {
    float distance = sqrt(x*x + y*y + z*z);
    if (distance < 1e-10) {
        return 0;
    }

    float scale = r / distance;
    intersection[0] = x * scale;
    intersection[1] = y * scale;
    intersection[2] = z * scale;
    return 1;
}

static int sy_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {

    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    if (param1 & INPUT_MIXER_SENSOR1) {
        if (event->code == INPUT_REL_X) {
            data->sensor1_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->sensor1_y += event->value;
        }
    }
    
    if (param1 & INPUT_MIXER_SENSOR2) {
        if (event->code == INPUT_REL_X) {
            data->sensor2_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->sensor2_y += event->value;
        }
    }

    event->value = 0;
    event->sync = false;

    int64_t now = k_uptime_get();
    if (now - data->last_rpt_time > config->sync_report_ms) {
        // ToDo
        
        bool have_x = data->rpt_x != 0;
        bool have_y = data->rpt_y != 0;

        if (have_x || have_y) {
            data->last_rpt_time = now;

            if (have_x) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->rpt_x, !have_y, K_NO_WAIT);
            }
            if (have_y) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->rpt_y, true, K_NO_WAIT);
            }
            data->rpt_x = data->rpt_y = 0;
        }
    }

    if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
        int16_t yaw = data->rpt_yaw / config->yaw_div;
        
        if (yaw) {
            data->last_rpt_time_yaw = now;
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, yaw, true, K_NO_WAIT);
            data->rpt_yaw = 0;
        }
    }

    return 0;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static int sy_init(const struct device *dev) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    data->dev = dev;

    int a = *(uint8_t*)(NULL);
    a = 5;

    float radius = (float) config->ball_radius;
    float surface_p1[3];
    float surface_p2[3];

    if (!line_sphere_intersection(radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 0)) - radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 1)) - radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 2)) - radius,
        surface_p1)) {
        LOG_ERR("Failed to get surface position for sensor 1");
        return 1;
    }

    if (!line_sphere_intersection(radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 0)) - radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 1)) - radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 2)) - radius,
        surface_p2)) {
        LOG_ERR("Failed to get surface position for sensor 1");
        return 1;
    }

    data->sensor1_surface_x = surface_p1[0];
    data->sensor1_surface_y = surface_p1[1];
    data->sensor1_surface_z = surface_p1[2];

    data->sensor2_surface_x = surface_p2[0];
    data->sensor2_surface_y = surface_p2[1];
    data->sensor2_surface_z = surface_p2[2];

    LOG_INF("Sensor mixer driver initialized");
    LOG_INF("  Sensor 1: X=%d, Y=%d, Z=%d", (int) data->sensor1_surface_x, (int) data->sensor1_surface_y, (int) data->sensor1_surface_z);
    LOG_INF("  Sensor 2: X=%d, Y=%d, Z=%d", (int) data->sensor2_surface_x, (int) data->sensor2_surface_y, (int) data->sensor2_surface_z);
    return 0;
}

#define TL_INST(n)                                                                                 \
    static struct zip_pointer_2s_mixer_data data_##n = {};                                         \
    static struct zip_pointer_2s_mixer_config config_##n = {                                       \
        .sync_report_ms = DT_INST_PROP(n, sync_report_ms),                                         \
        .sync_report_yaw_ms = DT_INST_PROP(n, sync_report_yaw_ms),                                 \
        .yaw_div = DT_INST_PROP(n, yaw_div),                                                       \
        .sensor1_pos = DT_INST_PROP(n, sensor1_pos),                                               \
        .sensor2_pos = DT_INST_PROP(n, sensor2_pos),                                               \
        .ball_radius = DT_INST_PROP(n, ball_radius),                                               \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
