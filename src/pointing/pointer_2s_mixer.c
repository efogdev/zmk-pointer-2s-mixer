#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <math.h>
#include <dt-bindings/zmk/p2sm.h>
#include <zephyr/logging/log.h>
#include <zmk/keymap.h>
#include "drivers/p2sm_runtime.h"

#if IS_ENABLED(CONFIG_SETTINGS)
#ifndef CONFIG_SETTINGS_RUNTIME
#define CONFIG_SETTINGS_RUNTIME true
#endif
#include <zephyr/settings/settings.h>
#endif

#define DT_DRV_COMPAT zmk_pointer_2s_mixer
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct device *g_dev = NULL;
static void twist_filter_cleanup_work_cb(struct k_work *work);

#if CONFIG_LOG_MAX_LEVEL >= 4
static char log_buf[32];
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
struct k_work_delayable save_work;
static void save_work_cb(struct k_work *work);
#endif

struct zip_pointer_2s_mixer_config {
    uint32_t sync_report_ms, sync_report_yaw_ms;
    uint32_t yaw_interference_thres; // in sensor points, CPI dependent
    uint32_t yaw_thres; // in sensor points, CPI dependent
    uint32_t ball_radius;

    // zero (origin) = down left bottom, not the ball center
    uint8_t sensor1_pos[3], sensor2_pos[3];
};

struct p2sm_dataframe {
    int16_t s1_x, s1_y, s2_x, s2_y;
};

// origin = ball center
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    struct k_work_delayable twist_filter_cleanup_work;

    bool initialized;
    int64_t last_rpt_time, last_rpt_time_yaw;
    int16_t rpt_x, rpt_y;
    float rpt_x_remainder, rpt_y_remainder, rpt_yaw_remainder;
    float move_coef, yaw_coef;

    // accumulates NON-transformed X, Y movements
    struct p2sm_dataframe values;

    // accumulates transformed X, Y movements
    struct p2sm_dataframe yaw_values;

    // pre-calculated
    float sensor1_surface_x, sensor1_surface_y, sensor1_surface_z;
    float sensor2_surface_x, sensor2_surface_y, sensor2_surface_z;
    float rotation_matrix1[3][3], rotation_matrix2[3][3];

    int64_t last_sensor1_report, last_sensor2_report;
    int64_t last_twist; // to filter out single events as they are probably accidental
    int8_t last_twist_direction; // to filter out first event in the opposite direction
};

struct twist_detection {
    int16_t val1;
    int16_t val2;
    bool direction;
};

static int data_init(const struct device *dev);
static void apply_rotation(float matrix[3][3], float dx, float dy, float *out_x, float *out_y);
static void apply_coef(float coef, float *x, float *y);

#if CONFIG_LOG_MAX_LEVEL >= 4
static int float_to_str(float value, int precision, char *buffer);
#endif

static int process_and_report(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();
    const int64_t dt = now - data->last_rpt_time;
    float rotated_x = 0, rotated_y = 0;

    if (data->values.s1_x != 0 || data->values.s1_y != 0) {
        apply_rotation(data->rotation_matrix1, data->values.s1_x, data->values.s1_y, &rotated_x, &rotated_y);
        apply_coef(data->move_coef, &rotated_x, &rotated_y);

        if (dt > CONFIG_POINTER_2S_MIXER_REMAINDER_TTL) {
            data->rpt_x_remainder = rotated_x;
            data->rpt_y_remainder = rotated_y;
        } else {
            data->rpt_x_remainder += rotated_x;
            data->rpt_y_remainder += rotated_y;
        }

        data->yaw_values.s1_x += rotated_x / data->move_coef;
        data->yaw_values.s1_y += rotated_y / data->move_coef;
        data->values.s1_x = 0;
        data->values.s1_y = 0;
    }

    if (data->values.s2_x != 0 || data->values.s2_y != 0) {
        apply_rotation(data->rotation_matrix2, data->values.s2_x, data->values.s2_y, &rotated_x, &rotated_y);
        apply_coef(data->move_coef, &rotated_x, &rotated_y);

        if (dt > CONFIG_POINTER_2S_MIXER_REMAINDER_TTL) {
            data->rpt_x_remainder = rotated_x;
            data->rpt_y_remainder = rotated_y;
        } else {
            data->rpt_x_remainder += rotated_x;
            data->rpt_y_remainder += rotated_y;
        }

        data->yaw_values.s2_x += rotated_x / data->move_coef;
        data->yaw_values.s2_y += rotated_y / data->move_coef;
        data->values.s2_x = 0;
        data->values.s2_y = 0;
    }

    data->rpt_x = (int16_t) data->rpt_x_remainder;
    data->rpt_y = (int16_t) data->rpt_y_remainder;
    data->rpt_x_remainder -= data->rpt_x;
    data->rpt_y_remainder -= data->rpt_y;

    const bool have_x = data->rpt_x != 0;
    const bool have_y = data->rpt_y != 0;
    if (have_x || have_y) {
        if (have_x) {
            input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->rpt_x, !have_y, K_NO_WAIT);
            data->rpt_x = 0;
        }
        if (have_y) {
            input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->rpt_y, true, K_NO_WAIT);
            data->rpt_y = 0;
        }
    }

    data->last_rpt_time = now;
    return 0;
}

static void calculate_rotation_matrix(float from_x, float from_y, float from_z, float to_x, float to_y, float to_z, float matrix[3][3]) {
    const float from_len = sqrtf(from_x*from_x + from_y*from_y + from_z*from_z);
    from_x /= from_len;
    from_y /= from_len;
    from_z /= from_len;

    const float to_len = sqrtf(to_x*to_x + to_y*to_y + to_z*to_z);
    to_x /= to_len;
    to_y /= to_len;
    to_z /= to_len;

    float axis_x = from_y * to_z - from_z * to_y;
    float axis_y = from_z * to_x - from_x * to_z;
    float axis_z = from_x * to_y - from_y * to_x;

    const float axis_len = sqrtf(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);
    if (axis_len < 1e-6) {
        LOG_ERR("Unexpected edge case. Consider repositioning one of the sensors.");
        return;
    }

    axis_x /= axis_len;
    axis_y /= axis_len;
    axis_z /= axis_len;

    const float cos_angle = from_x*to_x + from_y*to_y + from_z*to_z;
    const float sin_angle = sqrtf(1.0f - cos_angle*cos_angle);

    matrix[0][0] = cos_angle + axis_x*axis_x*(1-cos_angle);
    matrix[0][1] = axis_x*axis_y*(1-cos_angle) - axis_z*sin_angle;
    matrix[0][2] = axis_x*axis_z*(1-cos_angle) + axis_y*sin_angle;

    matrix[1][0] = axis_y*axis_x*(1-cos_angle) + axis_z*sin_angle;
    matrix[1][1] = cos_angle + axis_y*axis_y*(1-cos_angle);
    matrix[1][2] = axis_y*axis_z*(1-cos_angle) - axis_x*sin_angle;

    matrix[2][0] = axis_z*axis_x*(1-cos_angle) - axis_y*sin_angle;
    matrix[2][1] = axis_z*axis_y*(1-cos_angle) + axis_x*sin_angle;
    matrix[2][2] = cos_angle + axis_z*axis_z*(1-cos_angle);
}

static void apply_rotation(float matrix[3][3], const float dx, const float dy, float *out_x, float *out_y) {
    *out_x = matrix[0][0] * dx + matrix[0][1] * dy;
    *out_y = matrix[1][0] * dx + matrix[1][1] * dy;
}

static void apply_coef(const float coef, float *x, float *y) {
    *x *= coef;
    *y *= coef;
}

static float calculate_twist(const struct device *dev) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();
    const int64_t passed = now - data->last_twist;
    const int16_t s1_x = data->yaw_values.s1_x;
    const int16_t s1_y = data->yaw_values.s1_y;
    const int16_t s2_x = data->yaw_values.s2_x;
    const int16_t s2_y = data->yaw_values.s2_y;
    memset(&data->yaw_values, 0, sizeof(data->yaw_values));

    if (s1_x == 0 && s1_y == 0 && s2_x == 0 && s2_y == 0) {
        return 0;
    }

    uint8_t total_under_thres = 0;
    total_under_thres += abs(s1_x) < config->yaw_thres;
    total_under_thres += abs(s1_y) < config->yaw_thres;
    total_under_thres += abs(s2_x) < config->yaw_thres;
    total_under_thres += abs(s2_y) < config->yaw_thres;
    
    LOG_DBG("Analyzing movement: (%d, %d) (%d, %d)", s1_x, s1_y, s2_x, s2_y);
    if (total_under_thres == 4 && passed > CONFIG_POINTER_2S_MIXER_TWIST_NO_FILTER_THRES) {
        LOG_DBG("Discarded movement (reason = yaw_thres)");
        return 0;
    }

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_34_FILTER_EN)
    if (total_under_thres == 3 && passed > CONFIG_POINTER_2S_MIXER_TWIST_NO_FILTER_THRES) {
        LOG_DBG("Discarded movement (reason = 3_of_4_below_yaw_thres)");
        return 0;
    }
#endif

    const int16_t x = abs(s1_x + s2_x);
    const int16_t y = abs(s1_y + s2_y);
    if (x > config->yaw_interference_thres * CONFIG_POINTER_2S_MIXER_SIGNIFICANT_MOVEMENT_MUL ||
        y > config->yaw_interference_thres * CONFIG_POINTER_2S_MIXER_SIGNIFICANT_MOVEMENT_MUL) {
        data->last_twist_direction = -1;
        data->last_twist = 0;
        LOG_DBG("Discarded movement (reason = significant_translation), filters reapplied");
        return 0;
    }

    if ((x > config->yaw_interference_thres || y > config->yaw_interference_thres) &&
        passed > CONFIG_POINTER_2S_MIXER_TWIST_NO_FILTER_THRES) {
        LOG_DBG("Discarded twist (reason = yaw_interference_thres)");
        return 0;
    }

    if (passed > CONFIG_POINTER_2S_MIXER_TWIST_FILTER_TTL) {
        LOG_DBG("Discarded twist (reason = time_filter)");
        data->last_twist = now;
        return 0;
    }

    // here should go the angle calculation
    // but the solution below seems to work more reliable
    const struct twist_detection twist_scenarios[] = {
        {s1_x, s2_x, true},   // s1_x > 0, s2_x < 0
        {s1_x, s2_x, false},  // s1_x < 0, s2_x > 0
        {s1_y, s2_y, true},   // s1_y > 0, s2_y < 0
        {s1_y, s2_y, false}   // s1_y < 0, s2_y > 0
    };

    for (int i = 0; i < 4; i++) {
        const struct twist_detection *t = &twist_scenarios[i];
        const int16_t threshold = config->yaw_interference_thres;

        const bool condition = (i % 2 == 0)
            ? (t->val1 > threshold && t->val2 < -threshold)
            : (t->val1 < -threshold && t->val2 > threshold);

        if (!condition) {
            continue;
        }

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
        if (data->last_twist_direction != t->direction) {
            data->last_twist_direction = t->direction;
            LOG_DBG("Discarded twist (reason = direction_filter)");
            return 0;
        }
#endif

        const float result = t->direction ? -(float)(t->val1 - t->val2) : (float)(t->val2 - t->val1);
        LOG_DBG("Scroll value calculated: %d", (int) result);

        data->last_twist = now;
        data->last_twist_direction = t->direction;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
        k_work_reschedule(&data->twist_filter_cleanup_work, K_MSEC(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_TTL));
#endif

        return result;
    }

    return 0;
}

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
static void twist_filter_cleanup_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct zip_pointer_2s_mixer_data *dwork_data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_filter_cleanup_work);
    const struct device *dev = dwork_data->dev;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    data->last_twist_direction = -1;
    LOG_DBG("Direction filter data discarded (timeout)");
}
#endif

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

static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();

    if (!data->initialized) {
        if (!data_init(dev)) {
            LOG_ERR("Failed to initialize mixer driver data!");
            return -1;
        }
    }

    if (param1 & INPUT_MIXER_SENSOR1) {
        data->last_sensor1_report = now;

        if (event->code == INPUT_REL_X) {
            data->values.s1_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->values.s1_y += event->value;
        }
    } else {
        data->last_sensor2_report = now;

        if (event->code == INPUT_REL_X) {
            data->values.s2_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->values.s2_y += event->value;
        }
    }

    event->value = 0;
    event->sync = false;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    if (abs((int32_t) (data->last_sensor1_report - data->last_sensor2_report)) > CONFIG_POINTER_2S_MIXER_SYNC_WINDOW_MS) {
        memset(&data->values, 0, sizeof(struct p2sm_dataframe));
        memset(&data->yaw_values, 0, sizeof(struct p2sm_dataframe));
        return 0;
    }
#endif

    if (now - data->last_rpt_time > config->sync_report_ms) {
        process_and_report(dev);
    }

    if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
        const float yaw_float = calculate_twist(dev) * data->yaw_coef;
        if (now - data->last_rpt_time_yaw > CONFIG_POINTER_2S_MIXER_TWIST_REMAINDER_TTL) {
            data->rpt_yaw_remainder = yaw_float;
        } else {
            data->rpt_yaw_remainder += yaw_float;
        }

        const int16_t yaw_int = (int16_t)data->rpt_yaw_remainder;
        data->rpt_yaw_remainder -= yaw_int;
        if (yaw_int != 0) {
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, yaw_int, true, K_NO_WAIT);
        }

        data->last_rpt_time_yaw = now;
    }

    return 0;
}

static int sy_init(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    data->dev = dev;

#if !IS_ENABLED(CONFIG_POINTER_2S_MIXER_LAZY_INIT)
    if (!data_init(dev)) {
        LOG_ERR("Failed to initialize mixer driver data!");
        return -1;
    }
#endif

    return 0;
}

static int data_init(const struct device *dev) {
    if (g_dev != NULL) {
        LOG_ERR("Only one mixer instance is supported at the moment");
        return 0;
    }

    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const float radius = (float) config->ball_radius;
    float surface_p1[3], surface_p2[3];

    if (radius > 127) {
        LOG_ERR("Invalid configuration: radius must be less than 127");
        return 0;
    }

    if (!line_sphere_intersection(radius,
        (float) *(uint8_t*)(config->sensor1_pos + 0) - 127.f,
        (float) *(uint8_t*)(config->sensor1_pos + 1) - 127.f,
        (float) *(uint8_t*)(config->sensor1_pos + 2) - 127.f,
        surface_p1)) {
        LOG_ERR("Failed to get surface position for sensor 1!");
        return 0;
    }

    if (!line_sphere_intersection(radius,
        (float) *(uint8_t*)(config->sensor2_pos + 0) - 127.f,
        (float) *(uint8_t*)(config->sensor2_pos + 1) - 127.f,
        (float) *(uint8_t*)(config->sensor2_pos + 2) - 127.f,
        surface_p2)) {
        LOG_ERR("Failed to get surface position for sensor 2!");
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

    calculate_rotation_matrix(data->sensor1_surface_x, data->sensor1_surface_y, data->sensor1_surface_z,
        0, 0, -radius, data->rotation_matrix1);
    calculate_rotation_matrix(data->sensor2_surface_x, data->sensor2_surface_y, data->sensor2_surface_z,
        0, 0, -radius, data->rotation_matrix2);

    data->last_twist_direction = -1;
    data->move_coef = 1.0f;
    data->yaw_coef = 1.0f;

    // going >1 means losing precision
    // acceptable for scroll but not movement
    if (data->move_coef > 1.0f) {
        data->move_coef = 1.0f;
    }

    LOG_INF("Sensor mixer driver initialized");
    LOG_DBG("  > Ball radius: %d", (int) config->ball_radius);
    LOG_DBG("  > Surface trackpoint 1 ≈ (%d, %d, %d)", (int) data->sensor1_surface_x, (int) data->sensor1_surface_y, (int) data->sensor1_surface_z);
    LOG_DBG("  > Surface trackpoint 2 ≈ (%d, %d, %d)", (int) data->sensor2_surface_x, (int) data->sensor2_surface_y, (int) data->sensor2_surface_z);

#if CONFIG_LOG_MAX_LEVEL >= 4
    float_to_str(data->move_coef * 100, 2, log_buf);
    LOG_DBG("  > Pointer sensitivity: %s%%", log_buf);

    float_to_str(data->yaw_coef * 100, 2, log_buf);
    LOG_DBG("  > Scroll sensitivity: %s%%", log_buf);
#endif

    g_dev = (struct device *) dev;
    data->initialized = true;

    p2sm_sens_driver_init();

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    k_work_init_delayable(&data->twist_filter_cleanup_work, twist_filter_cleanup_work_cb);
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&save_work, save_work_cb);
#endif

    return 1;
}

#if CONFIG_LOG_MAX_LEVEL >= 4
static int float_to_str(const float value, int precision, char *buffer) {
    int int_part = (int)value;
    float decimal_part = value - int_part;

    if (value < 0 && decimal_part != 0) {
        int_part = -int_part;
        decimal_part = -decimal_part;
        *buffer++ = '-';
    }

    int len = snprintf(buffer, 16, "%d", int_part);
    buffer += len;
    *buffer++ = '.';

    for (int i = 0; i < precision; i++) {
        decimal_part *= 10;
        const int digit = (int) decimal_part;
        *buffer++ = '0' + digit;
        decimal_part -= digit;
    }

    *buffer = '\0';
    return len + precision + 1;
}
#endif

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static void save_work_cb(struct k_work *work) {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    const float values[2] = { data->move_coef, data->yaw_coef };

    const int err = settings_save_one(P2SM_SETTINGS_PREFIX, values, sizeof(values));
    if (err < 0) {
        LOG_ERR("Failed to save settings %d", err);
    } else {
        LOG_DBG("Sensitivity settings saved");
    }
}

static void p2sm_save_sensitivity() {
    k_work_reschedule(&save_work, K_MSEC(CONFIG_POINTER_2S_MIXER_SETTINGS_SAVE_DELAY));
}
#endif

float p2sm_get_move_coef() {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return 0;
    }

    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    return data->move_coef;
}

float p2sm_get_yaw_coef() {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return 0;
    }

    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    return data->yaw_coef;
}

void p2sm_set_move_coef(const float coef) {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->move_coef = coef;

#if IS_ENABLED(CONFIG_SETTINGS)
    p2sm_save_sensitivity();
#endif
}

void p2sm_set_yaw_coef(const float coef) {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->yaw_coef = coef;

#if IS_ENABLED(CONFIG_SETTINGS)
    p2sm_save_sensitivity();
#endif
}

static struct zip_pointer_2s_mixer_data data = {};
static struct zip_pointer_2s_mixer_config config = {
    .sync_report_ms = DT_INST_PROP(0, sync_report_ms),
    .sync_report_yaw_ms = DT_INST_PROP(0, sync_report_yaw_ms),
    .yaw_interference_thres = DT_INST_PROP(0, yaw_interference_thres),
    .yaw_thres = DT_INST_PROP(0, yaw_thres),
    .sensor1_pos = DT_INST_PROP(0, sensor1_pos),
    .sensor2_pos = DT_INST_PROP(0, sensor2_pos),
    .ball_radius = DT_INST_PROP(0, ball_radius),
};                                                                                             
DEVICE_DT_INST_DEFINE(0, &sy_init, NULL, &data, &config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);
