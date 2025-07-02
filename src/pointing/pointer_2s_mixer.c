#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <math.h>
#include <dt-bindings/zmk/input_mixer.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>

#define  M_PI 3.14159265358979323846
#define DT_DRV_COMPAT zmk_pointer_2s_mixer
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct zip_pointer_2s_mixer_config {
    uint32_t sync_report_ms;
    uint32_t sync_report_yaw_ms;
    uint32_t yaw_div, yaw_mul;
    uint32_t yaw_interference_thres;
    uint32_t ball_radius;

    // zero = down left bottom, not the ball origin
    uint8_t sensor1_pos[3], sensor2_pos[3];
};

// zero = ball origin
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    bool initialized;

    int64_t last_rpt_time, last_rpt_time_yaw;
    int16_t rpt_x, rpt_y, rpt_yaw;
    float rpt_x_remainder, rpt_y_remainder;

    int16_t s1_x, s1_y, s2_x, s2_y;
    float sensor1_surface_x, sensor1_surface_y, sensor1_surface_z;
    float sensor2_surface_x, sensor2_surface_y, sensor2_surface_z;
    float sensor1_rotation_matrix[3][3], sensor2_rotation_matrix[3][3];
    float sensor1_2d_rotation_matrix[2][2], sensor2_2d_rotation_matrix[2][2];
    float radius;
};

static int data_init(const struct device *dev);
static int line_sphere_intersection(float r, float x, float y, float z, float intersection[3]);
static void create_rotation_matrix(float surface_x, float surface_y, float surface_z, float radius, float matrix[3][3]);
static void transform_sensor_input(float dx, float dy, float matrix[3][3], float *out_x, float *out_y, float *out_z);
static void map_tangent_to_sphere(float dx, float dy, float radius, float rotation_matrix[3][3], float *out_x, float *out_y, float *out_z);

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
            data->s1_x += event->value;
        } else {
            data->s2_x += event->value;
        }
    } else if (event->code == INPUT_REL_Y) {
        if (param1 & INPUT_MIXER_SENSOR1) {
            data->s1_y += event->value;
        } else {
            data->s2_y += event->value;
        }
    }

    event->value = 0;
    event->sync = false;

    const int64_t now = k_uptime_get();
    if (now - data->last_rpt_time > config->sync_report_ms) {
        float s1_x = 0, s1_y = 0, s1_z = 0;
        map_tangent_to_sphere(data->s1_x, data->s1_y, data->radius, data->sensor1_rotation_matrix, &s1_x, &s1_y, &s1_z);

        float s2_x = 0, s2_y = 0, s2_z = 0;
        map_tangent_to_sphere(data->s2_x, data->s2_y, data->radius, data->sensor2_rotation_matrix, &s2_x, &s2_y, &s2_z);

        const float final_s1_x = s1_x * data->sensor1_2d_rotation_matrix[0][0] + s1_y * data->sensor1_2d_rotation_matrix[0][1];
        const float final_s1_y = s1_x * data->sensor1_2d_rotation_matrix[1][0] + s1_y * data->sensor1_2d_rotation_matrix[1][1];
        const float final_s2_x = s2_x * data->sensor2_2d_rotation_matrix[0][0] + s2_y * data->sensor2_2d_rotation_matrix[0][1];
        const float final_s2_y = s2_x * data->sensor2_2d_rotation_matrix[1][0] + s2_y * data->sensor2_2d_rotation_matrix[1][1];
        const float x_mov = final_s1_x + final_s2_x + data->rpt_x_remainder;
        const float y_mov = final_s1_y + final_s2_y + data->rpt_y_remainder;

        data->rpt_x = x_mov;
        data->rpt_y = y_mov;
        data->rpt_x_remainder = x_mov - data->rpt_x;
        data->rpt_y_remainder = y_mov - data->rpt_y;
        data->rpt_yaw += s1_z + s2_z;
        data->s1_x = data->s1_y = data->s2_x = data->s2_y = 0;

        const bool have_x = data->rpt_x != 0;
        const bool have_y = data->rpt_y != 0;

        if (have_x || have_y) {
            data->last_rpt_time = now;

            if (have_x) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->rpt_x, !have_y, K_NO_WAIT);
                data->rpt_x = 0;
            }
            if (have_y) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->rpt_y, true, K_NO_WAIT);
                data->rpt_y = 0;
            }
        }
    }

    if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
        const int16_t yaw = data->rpt_yaw / (float) config->yaw_div * (float) config->yaw_mul;

        if (yaw) {
            data->last_rpt_time_yaw = now;
            // input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, yaw, true, K_NO_WAIT);
            data->rpt_yaw = 0;
        }
    }

    return 0;
}

static void create_rotation_matrix(const float surface_x, const float surface_y, const float surface_z, const float radius, float matrix[3][3]) {
    const float normal[3] = {surface_x / radius, surface_y / radius, surface_z / radius};
    const float global_x[3] = {1, 0, 0};
    const float dot_x_normal = global_x[0] * normal[0] + global_x[1] * normal[1] + global_x[2] * normal[2];

    float local_x[3] = {
        global_x[0] - dot_x_normal * normal[0],
        global_x[1] - dot_x_normal * normal[1],
        global_x[2] - dot_x_normal * normal[2]
    };

    float local_x_length = sqrtf(local_x[0] * local_x[0] + local_x[1] * local_x[1] + local_x[2] * local_x[2]);
    if (local_x_length < 1e-6) {
        const float global_y[3] = {0, 1, 0};
        const float dot_y_normal = global_y[0] * normal[0] + global_y[1] * normal[1] + global_y[2] * normal[2];
        local_x[0] = global_y[0] - dot_y_normal * normal[0];
        local_x[1] = global_y[1] - dot_y_normal * normal[1];
        local_x[2] = global_y[2] - dot_y_normal * normal[2];
        local_x_length = sqrtf(local_x[0] * local_x[0] + local_x[1] * local_x[1] + local_x[2] * local_x[2]);
    }

    local_x[0] /= local_x_length;
    local_x[1] /= local_x_length;
    local_x[2] /= local_x_length;

    const float local_y[3] = {
        normal[1] * local_x[2] - normal[2] * local_x[1],
        normal[2] * local_x[0] - normal[0] * local_x[2],
        normal[0] * local_x[1] - normal[1] * local_x[0]
    };

    matrix[0][0] = local_x[0];
    matrix[1][0] = local_x[1];
    matrix[2][0] = local_x[2];

    matrix[0][1] = local_y[0];
    matrix[1][1] = local_y[1];
    matrix[2][1] = local_y[2];

    matrix[0][2] = normal[0];
    matrix[1][2] = normal[1];
    matrix[2][2] = normal[2];
}

static void transform_sensor_input(const float dx, const float dy, float matrix[3][3], float *out_x, float *out_y, float *out_z) {
    *out_x = matrix[0][0] * dx + matrix[0][1] * dy;
    *out_y = matrix[1][0] * dx + matrix[1][1] * dy;
    *out_z = matrix[2][0] * dx + matrix[2][1] * dy;
}

static void map_tangent_to_sphere(const float dx, const float dy, const float radius, float rotation_matrix[3][3], float *out_x, float *out_y, float *out_z) {
    float tangent_x, tangent_y, tangent_z;
    transform_sensor_input(dx, dy, rotation_matrix, &tangent_x, &tangent_y, &tangent_z);

    const float scale_factor = radius;
    const float tangent_length = sqrtf(tangent_x*tangent_x + tangent_y*tangent_y);

    if (tangent_length > 0) {
        const float angle = tangent_length / radius;
        const float arc_scale = sinf(angle) / tangent_length;

        *out_x = tangent_x * arc_scale * scale_factor;
        *out_y = tangent_y * arc_scale * scale_factor;
        *out_z = tangent_z * (1.0f - cosf(angle)) * scale_factor;
    } else {
        *out_x = 0;
        *out_y = 0;
        *out_z = 0;
    }
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
    data->radius = radius;
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
        LOG_ERR("Failed to get surface position for sensor 2");
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

    create_rotation_matrix(surface_p1[0], surface_p1[1], surface_p1[2], radius, data->sensor1_rotation_matrix);
    create_rotation_matrix(surface_p2[0], surface_p2[1], surface_p2[2], radius, data->sensor2_rotation_matrix);

    const float sensor1_angle_rad = -atan2f(surface_p1[1], surface_p1[0]);
    data->sensor1_2d_rotation_matrix[0][0] = cosf(sensor1_angle_rad);
    data->sensor1_2d_rotation_matrix[0][1] = -sinf(sensor1_angle_rad);
    data->sensor1_2d_rotation_matrix[1][0] = sinf(sensor1_angle_rad);
    data->sensor1_2d_rotation_matrix[1][1] = cosf(sensor1_angle_rad);

    const float sensor2_angle_rad = -atan2f(surface_p2[1], surface_p2[0]);
    data->sensor2_2d_rotation_matrix[0][0] = cosf(sensor2_angle_rad);
    data->sensor2_2d_rotation_matrix[0][1] = -sinf(sensor2_angle_rad);
    data->sensor2_2d_rotation_matrix[1][0] = sinf(sensor2_angle_rad);
    data->sensor2_2d_rotation_matrix[1][1] = cosf(sensor2_angle_rad);

    LOG_INF("Sensor mixer driver initialized");
    LOG_DBG("  Ball radius: %d", (int) config->ball_radius);
    LOG_DBG("== Sensor 1 ==");
    LOG_DBG("  > Surface trackpoint 1 ≈ (%d, %d, %d)", (int) data->sensor1_surface_x, (int) data->sensor1_surface_y, (int) data->sensor1_surface_z);
    LOG_DBG("  > Rotation angle ≈ %d°", (int) (sensor1_angle_rad * 180.0f / M_PI));
    LOG_DBG("== Sensor 2 ==");
    LOG_DBG("  > Surface trackpoint 2 ≈ (%d, %d, %d)", (int) data->sensor2_surface_x, (int) data->sensor2_surface_y, (int) data->sensor2_surface_z);
    LOG_DBG("  > Rotation angle ≈ %d°", (int) (sensor2_angle_rad * 180.0f / M_PI));

    data->initialized = true;
    return 1;
}

#define TL_INST(n)                                                                                 \
    static struct zip_pointer_2s_mixer_data data_##n = {};                                         \
    static struct zip_pointer_2s_mixer_config config_##n = {                                       \
        .sync_report_ms = DT_INST_PROP(n, sync_report_ms),                                         \
        .sync_report_yaw_ms = DT_INST_PROP(n, sync_report_yaw_ms),                                 \
        .yaw_div = DT_INST_PROP(n, yaw_div),                                                       \
        .yaw_mul = DT_INST_PROP(n, yaw_mul),                                                       \
        .yaw_interference_thres = DT_INST_PROP(n, yaw_interference_thres),                         \
        .sensor1_pos = DT_INST_PROP(n, sensor1_pos),                                               \
        .sensor2_pos = DT_INST_PROP(n, sensor2_pos),                                               \
        .ball_radius = DT_INST_PROP(n, ball_radius),                                               \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
