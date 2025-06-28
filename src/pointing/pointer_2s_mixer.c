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

    // zero = down left bottom
    uint8_t sensor1_pos[3];
    uint8_t sensor2_pos[3];
};

#define LATENCY_BUFFER_SIZE 100
struct latency_event {
    int64_t timestamp;
    bool is_sensor1;
};

// zero = ball origin
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    bool initialized;

    int64_t last_rpt_time;
    int64_t last_rpt_time_yaw;

    int16_t rpt_x;
    int16_t rpt_y;
    float rpt_x_remainder;
    float rpt_y_remainder;
    int16_t rpt_yaw;

    float sensor1_surface_x;
    float sensor1_surface_y;
    float sensor1_surface_z;

    float sensor2_surface_x;
    float sensor2_surface_y;
    float sensor2_surface_z;

    float sensor1_acc_x;
    float sensor1_acc_y;
    float sensor2_acc_x;
    float sensor2_acc_y;

    float sensor1_basis_x[3];
    float sensor1_basis_y[3];
    float sensor2_basis_x[3];
    float sensor2_basis_y[3];
};

static int data_init(const struct device *dev);
static int line_sphere_intersection(float r, float x, float y, float z, float intersection[3]);
static void calculate_tangent_basis(float px, float py, float pz, float basis_x[3], float basis_y[3]);
static void map_accumulated_movements_to_reference(const struct device *dev);

static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    if (!data->initialized) {
        if (!data_init(dev)) {
            LOG_ERR("Failed to initialize mixer driver data!");
            return -1;
        }
    }

    if (event->code == INPUT_REL_X) {
        if (param1 & INPUT_MIXER_SENSOR1) {
            data->sensor1_acc_x += (float)event->value;
        } else {
            data->sensor2_acc_x += (float)event->value;
        }
    } else if (event->code == INPUT_REL_Y) {
        if (param1 & INPUT_MIXER_SENSOR1) {
            data->sensor1_acc_y += (float)event->value;
        } else {
            data->sensor2_acc_y += (float)event->value;
        }
    }

    event->value = 0;
    event->sync = false;

    const int64_t now = k_uptime_get();
    if (now - data->last_rpt_time > config->sync_report_ms) {
        map_accumulated_movements_to_reference(dev);

        const bool have_x = data->rpt_x != 0;
        const bool have_y = data->rpt_y != 0;

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

    // if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
    //     const int16_t yaw = data->rpt_yaw / config->yaw_div;
    //
    //     if (yaw) {
    //         data->last_rpt_time_yaw = now;
    //         input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, yaw, true, K_NO_WAIT);
    //         data->rpt_yaw = 0;
    //     }
    // }

    return 0;
}

static int line_sphere_intersection(const float r, const float x, const float y, const float z, float intersection[3]) {
    const float distance = sqrtf(x*x + y*y + z*z);
    if (distance < 1e-10) {
        return 0;
    }

    const float scale = r / distance;
    intersection[0] = x * scale;
    intersection[1] = y * scale;
    intersection[2] = z * scale;
    return 1;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static int sy_init(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    data->dev = dev;
    return 0;
}

static int data_init(const struct device *dev) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    const float radius = (float) config->ball_radius;
    float surface_p1[3];
    float surface_p2[3];

    if (!line_sphere_intersection(radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 0)) - radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 1)) - radius,
        (float) (*(uint8_t*)(config->sensor1_pos + 2)) - radius,
        surface_p1)) {
        LOG_ERR("Failed to get surface position for sensor 1");
        return 0;
    }

    if (!line_sphere_intersection(radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 0)) - radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 1)) - radius,
        (float) (*(uint8_t*)(config->sensor2_pos + 2)) - radius,
        surface_p2)) {
        LOG_ERR("Failed to get surface position for sensor 1");
        return 0;
    }

    data->sensor1_surface_x = surface_p1[0];
    data->sensor1_surface_y = surface_p1[1];
    data->sensor1_surface_z = surface_p1[2];

    data->sensor2_surface_x = surface_p2[0];
    data->sensor2_surface_y = surface_p2[1];
    data->sensor2_surface_z = surface_p2[2];

    if (surface_p1[0] == surface_p2[0] && surface_p1[1] == surface_p2[1] && surface_p1[2] == surface_p2[2]) {
        LOG_ERR("Unexpected: both trackpoints have same coordinates!");
        return 0;
    }

    calculate_tangent_basis(data->sensor1_surface_x, data->sensor1_surface_y, data->sensor1_surface_z, data->sensor1_basis_x, data->sensor1_basis_y);
    calculate_tangent_basis(data->sensor2_surface_x, data->sensor2_surface_y, data->sensor2_surface_z, data->sensor2_basis_x, data->sensor2_basis_y);

    LOG_INF("Sensor mixer driver initialized");
    LOG_INF("  Ball radius: %d", (int) config->ball_radius);
    LOG_INF("  Surface trackpoint 1 at (%d, %d, %d)", (int) data->sensor1_surface_x, (int) data->sensor1_surface_y, (int) data->sensor1_surface_z);
    LOG_INF("  Surface trackpoint 2 at (%d, %d, %d)", (int) data->sensor2_surface_x, (int) data->sensor2_surface_y, (int) data->sensor2_surface_z);

    data->initialized = true;
    return 1;
}

static void calculate_tangent_basis(const float px, const float py, const float pz, float basis_x[3], float basis_y[3]) {
    float normal[3] = {px, py, pz};
    float norm = sqrtf(px*px + py*py + pz*pz);

    normal[0] /= norm;
    normal[1] /= norm;
    normal[2] /= norm;

    float ref[3];
    if (fabsf(normal[2]) > 0.9f) {
        ref[0] = 1.0f; ref[1] = 0.0f; ref[2] = 0.0f;
    } else {
        ref[0] = 0.0f; ref[1] = 0.0f; ref[2] = 1.0f;
    }

    basis_x[0] = normal[1] * ref[2] - normal[2] * ref[1];
    basis_x[1] = normal[2] * ref[0] - normal[0] * ref[2];
    basis_x[2] = normal[0] * ref[1] - normal[1] * ref[0];

    norm = sqrtf(basis_x[0]*basis_x[0] + basis_x[1]*basis_x[1] + basis_x[2]*basis_x[2]);
    basis_x[0] /= norm;
    basis_x[1] /= norm;
    basis_x[2] /= norm;

    basis_y[0] = normal[1] * basis_x[2] - normal[2] * basis_x[1];
    basis_y[1] = normal[2] * basis_x[0] - normal[0] * basis_x[2];
    basis_y[2] = normal[0] * basis_x[1] - normal[1] * basis_x[0];
}

static void map_accumulated_movements_to_reference(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;

    if (data->sensor1_acc_x == 0 && data->sensor1_acc_y == 0 &&
        data->sensor2_acc_x == 0 && data->sensor2_acc_y == 0) {
        return;
    }

    float total_movement_x = data->rpt_x_remainder;
    float total_movement_y = data->rpt_y_remainder;

    if (data->sensor1_acc_x != 0 || data->sensor1_acc_y != 0) {
        float movement_vector[3];
        movement_vector[0] = data->sensor1_acc_x * data->sensor1_basis_x[0] + data->sensor1_acc_y * data->sensor1_basis_y[0];
        movement_vector[1] = data->sensor1_acc_x * data->sensor1_basis_x[1] + data->sensor1_acc_y * data->sensor1_basis_y[1];
        movement_vector[2] = data->sensor1_acc_x * data->sensor1_basis_x[2] + data->sensor1_acc_y * data->sensor1_basis_y[2];

        total_movement_x += movement_vector[0];
        total_movement_y += movement_vector[1];

        data->sensor1_acc_x = 0;
        data->sensor1_acc_y = 0;
    }

    if (data->sensor2_acc_x != 0 || data->sensor2_acc_y != 0) {
        float movement_vector[3];
        movement_vector[0] = data->sensor2_acc_x * data->sensor2_basis_x[0] + data->sensor2_acc_y * data->sensor2_basis_y[0];
        movement_vector[1] = data->sensor2_acc_x * data->sensor2_basis_x[1] + data->sensor2_acc_y * data->sensor2_basis_y[1];
        movement_vector[2] = data->sensor2_acc_x * data->sensor2_basis_x[2] + data->sensor2_acc_y * data->sensor2_basis_y[2];

        total_movement_x += movement_vector[0];
        total_movement_y += movement_vector[1];

        data->sensor2_acc_x = 0;
        data->sensor2_acc_y = 0;
    }

    const int16_t int_movement_x = (int16_t)total_movement_x;
    const int16_t int_movement_y = (int16_t)total_movement_y;

    data->rpt_x += int_movement_x;
    data->rpt_y += int_movement_y;

    data->rpt_x_remainder = total_movement_x - (float)int_movement_x;
    data->rpt_y_remainder = total_movement_y - (float)int_movement_y;
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
