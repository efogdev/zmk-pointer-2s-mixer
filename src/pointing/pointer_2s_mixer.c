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

// zero = ball origin
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    bool initialized;

    int64_t last_rpt_time;
    int64_t last_rpt_time_yaw;

    int16_t rpt_x;
    int16_t rpt_y;
    int16_t rpt_yaw;

    float sensor1_surface_x;
    float sensor1_surface_y;
    float sensor1_surface_z;
    int16_t sensor1_x;
    int16_t sensor1_y;

    float sensor2_surface_x;
    float sensor2_surface_y;
    float sensor2_surface_z;
    int16_t sensor2_x;
    int16_t sensor2_y;

    float rotation_matrix_1[3][3];  // sensor 1 to (-R, 0, 0)
    float rotation_matrix_2[3][3];  // sensor 2 to (-R, R, 0)
};

static int data_init(const struct device *dev);
static int line_sphere_intersection(float r, float x, float y, float z, float intersection[3]);

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
            data->sensor1_x += event->value;
        } else {
            data->sensor2_x += event->value;
        }
    } else if (event->code == INPUT_REL_Y) {
        if (param1 & INPUT_MIXER_SENSOR1) {
            data->sensor1_y += event->value;
        } else {
            data->sensor2_y += event->value;
        }
    }

    event->value = 0;
    event->sync = false;

    const int64_t now = k_uptime_get();
    if (now - data->last_rpt_time > config->sync_report_ms) {
        // ToDo:


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

    if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
        const int16_t yaw = data->rpt_yaw / config->yaw_div;

        if (yaw) {
            data->last_rpt_time_yaw = now;
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, yaw, true, K_NO_WAIT);
            data->rpt_yaw = 0;
        }
    }

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

// calculates rotation matrix to rotate point (X,Y,Z) on sphere to (-R,0,0)
static void calculate_rotation_matrix_1(const float X, const float Y, const float Z, const float R, float matrix[3][3]) {
    if (fabsf(Y) < 1e-6 && fabsf(Z) < 1e-6) {
        if (X > 0) {
            // point is at (R,0,0), need 180° rotation around any axis perpendicular to X
            matrix[0][0] = -1.0f; matrix[0][1] = 0.0f;  matrix[0][2] = 0.0f;
            matrix[1][0] = 0.0f;  matrix[1][1] = -1.0f; matrix[1][2] = 0.0f;
            matrix[2][0] = 0.0f;  matrix[2][1] = 0.0f;  matrix[2][2] = 1.0f;
        } else {
            // point is already at (-R,0,0), no rotation needed
            matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f;
            matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f;
            matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f;
        }
        return;
    }

    const float n = sqrtf(Y*Y + Z*Z);  // length of (Y,Z) component
    const float c = -X/R;              // cosine of rotation angle
    const float s = n/R;               // sine of rotation angle

    matrix[0][0] = c;
    matrix[0][1] = 0.0f;
    matrix[0][2] = 0.0f;

    matrix[1][0] = 0.0f;
    matrix[1][1] = (Z*Z)/(n*n) + c*(Y*Y)/(n*n);
    matrix[1][2] = -s*(Y/n);

    matrix[2][0] = 0.0f;
    matrix[2][1] = s*(Y/n);
    matrix[2][2] = (Y*Y)/(n*n) + c*(Z*Z)/(n*n);
}

// calculates rotation matrix to rotate point (X,Y,Z) on sphere to (-R,R,0)
static void calculate_rotation_matrix_2(const float X, const float Y, const float Z, const float R, float matrix[3][3]) {
    const float sqrt2 = 1.4142135623730951f; // √2

    const float m = sqrtf(Z*Z + (X+Y)*(X+Y)/2.0f);
    const float c = (-X+Y)/(sqrt2*R);  // cosine of rotation angle

    if (m < 1e-6) {
        if (fabsf(X + R/sqrt2) < 1e-6 && fabsf(Y - R/sqrt2) < 1e-6) {
            // point is already at (-R,R,0), no rotation needed
            matrix[0][0] = 1.0f; matrix[0][1] = 0.0f; matrix[0][2] = 0.0f;
            matrix[1][0] = 0.0f; matrix[1][1] = 1.0f; matrix[1][2] = 0.0f;
            matrix[2][0] = 0.0f; matrix[2][1] = 0.0f; matrix[2][2] = 1.0f;
        } else {
            // point is at the opposite position, need 180° rotation
            // around an axis perpendicular to both (X,Y,Z) and (-R,R,0)
            matrix[0][0] = -1.0f; matrix[0][1] = 0.0f;  matrix[0][2] = 0.0f;
            matrix[1][0] = 0.0f;  matrix[1][1] = -1.0f; matrix[1][2] = 0.0f;
            matrix[2][0] = 0.0f;  matrix[2][1] = 0.0f;  matrix[2][2] = 1.0f;
        }
        return;
    }

    const float s = m/R;
    const float ux = (Z/sqrt2)/m;
    const float uy = (Z/sqrt2)/m;
    const float uz = (-(X+Y)/sqrt2)/m;

    // calculate rotation matrix using Rodriguez' formula
    // R = I + sin(θ)[u]× + (1-cos(θ))[u]×²

    // calculate [u]× (skew-symmetric matrix of u)
    const float ux_cross[3][3] = {
        {0.0f, -uz, uy},
        {uz, 0.0f, -ux},
        {-uy, ux, 0.0f}
    };

    // calculate [u]×²
    float ux_cross_squared[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ux_cross_squared[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                ux_cross_squared[i][j] += ux_cross[i][k] * ux_cross[k][j];
            }
        }
    }

    // calculate final rotation matrix: R = I + sin(θ)[u]× + (1-cos(θ))[u]×²
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            matrix[i][j] = (i == j ? 1.0f : 0.0f) + s * ux_cross[i][j] + (1.0f - c) * ux_cross_squared[i][j];
        }
    }
}

static void float_to_str(const float f, char *buf, size_t len) {
    const int whole = (int)f;
    int frac = (int)((f - whole) * 1000);
    if (frac < 0) frac = -frac;

    snprintk(buf, len, "%d.%03d", whole, frac);
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

    calculate_rotation_matrix_1(data->sensor1_surface_x, data->sensor1_surface_y, data->sensor1_surface_z, radius, data->rotation_matrix_1);
    calculate_rotation_matrix_2(data->sensor2_surface_x, data->sensor2_surface_y, data->sensor2_surface_z, radius, data->rotation_matrix_2);

    LOG_INF("Sensor mixer driver initialized");
    LOG_INF("  Ball radius: %d", (int) config->ball_radius);
    LOG_INF("  Surface trackpoint 1 at (%d, %d, %d)", (int) data->sensor1_surface_x, (int) data->sensor1_surface_y, (int) data->sensor1_surface_z);
    LOG_INF("  Surface trackpoint 2 at (%d, %d, %d)", (int) data->sensor2_surface_x, (int) data->sensor2_surface_y, (int) data->sensor2_surface_z);

    char float_str[16];
    LOG_INF("  Rotation matrix 1:");
    for (int i = 0; i < 3; i++) {
        float_to_str(data->rotation_matrix_1[i][0], float_str, sizeof(float_str));
        LOG_INF("    [ %s, ", float_str);
        float_to_str(data->rotation_matrix_1[i][1], float_str, sizeof(float_str));
        LOG_INF("      %s, ", float_str);
        float_to_str(data->rotation_matrix_1[i][2], float_str, sizeof(float_str));
        LOG_INF("      %s ]", float_str);
    }

    LOG_INF("  Rotation matrix 2:");
    for (int i = 0; i < 3; i++) {
        float_to_str(data->rotation_matrix_2[i][0], float_str, sizeof(float_str));
        LOG_INF("    [ %s, ", float_str);
        float_to_str(data->rotation_matrix_2[i][1], float_str, sizeof(float_str));
        LOG_INF("      %s, ", float_str);
        float_to_str(data->rotation_matrix_2[i][2], float_str, sizeof(float_str));
        LOG_INF("      %s ]", float_str);
    }

    data->initialized = true;
    return 1;
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
