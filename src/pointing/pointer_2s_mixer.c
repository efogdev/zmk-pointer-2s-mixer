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
#include "zephyr/drivers/gpio.h"

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

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
static void twist_feedback_off_work_cb(struct k_work *work);
static void twist_feedback_extra_delay_work_cb(struct k_work *work);
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
struct k_work_delayable p2sm_save_work;
static void p2sm_save_work_cb(struct k_work *work);
#endif

struct zip_pointer_2s_mixer_config {
    const uint32_t sync_report_ms, sync_scroll_report_ms;

    // CPI and sync window dependent
    const uint16_t twist_thres, twist_interference_thres, twist_interference_window;

    // zero (origin) = down left bottom, not the ball center
    const uint8_t sensor1_pos[3], sensor2_pos[3];
    const uint8_t ball_radius; // up to 127

    // feedback (i.e. vibration)
    const struct gpio_dt_spec feedback_gpios;
    const struct gpio_dt_spec feedback_extra_gpios;
    const uint16_t twist_feedback_duration, twist_feedback_threshold, twist_feedback_delay;
};

struct p2sm_dataframe {
    int16_t s1_x, s1_y, s2_x, s2_y;
};

struct dataframe_history_entry {
    int64_t timestamp;
    struct p2sm_dataframe dataframe;
    float calculated_scroll;
    struct dataframe_history_entry *next;
};

// origin = ball center
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    struct k_work_delayable twist_filter_cleanup_work;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    struct k_work_delayable twist_feedback_off_work;
    struct k_work_delayable twist_feedback_extra_delay_work;
    int previous_feedback_extra_state;
#endif

    bool initialized;
    int64_t last_rpt_time, last_rpt_time_twist;
    int16_t rpt_x, rpt_y;
    float rpt_x_remainder, rpt_y_remainder, rpt_twist_remainder;
    float move_coef, twist_coef;

    // accumulates NON-transformed X, Y movements
    struct p2sm_dataframe values;

    // accumulates transformed X, Y movements
    struct p2sm_dataframe twist_values;

    // pre-calculated
    float rotation_matrix1[3][3], rotation_matrix2[3][3];

    int64_t last_twist; // to filter out single events as they are probably accidental
    int8_t last_twist_direction; // to filter out first event in the opposite direction

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    uint16_t twist_accumulator; // for feedback
#endif

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    int64_t last_sensor1_report, last_sensor2_report;
#endif

    struct dataframe_history_entry *history_head;
    struct dataframe_history_entry *history_pool;
    int16_t interference_accumulator;
    
    uint8_t max_history_entries;
    struct dataframe_history_entry *history_buffer;
    uint8_t history_buffer_index;
};

struct twist_detection {
    const int16_t val1;
    const int16_t val2;
    const bool direction;
};

static int data_init(const struct device *dev);
static void apply_rotation(float matrix[3][3], float dx, float dy, float *out_x, float *out_y);
static void apply_coef(float coef, float *x, float *y);
static struct dataframe_history_entry* dataframe_history_add(const struct device *dev, const struct p2sm_dataframe *dataframe);
static void dataframe_history_cleanup(const struct device *dev, int64_t cutoff_time);

static bool scroll_percentage_filter(const struct device *dev) {
    const struct zip_pointer_2s_mixer_data *data = dev->data;
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    const int64_t now = k_uptime_get();
    const int64_t cutoff_time = now - config->twist_interference_window;
    const struct dataframe_history_entry *current = data->history_head;
    int total_entries = 0, scroll_entries = 0;

    while (current != NULL) {
        if (current->timestamp >= cutoff_time) {
            total_entries++;
            if (fabs(current->calculated_scroll) > config->twist_thres) {
                scroll_entries++;
            }
        }
        current = current->next;
    }
    
    if (total_entries == 0) {
        return false;
    }
    
    const int scroll_percentage = (scroll_entries * 100) / total_entries;
    LOG_WRN("Scroll %: %d", scroll_percentage);
    return scroll_percentage >= CONFIG_POINTER_2S_MIXER_SCROLL_PERCENTAGE_THRESHOLD;
}

static int process_and_report(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();
    int64_t dt = now - data->last_rpt_time;
    float rotated_x = 0, rotated_y = 0;

    if (data->values.s1_x != 0 || data->values.s1_y != 0) {
        apply_rotation(data->rotation_matrix1, data->values.s1_x, data->values.s1_y, &rotated_x, &rotated_y);
        data->twist_values.s1_x += rotated_x;
        data->twist_values.s1_y += rotated_y;

        apply_coef(data->move_coef, &rotated_x, &rotated_y);
        if (dt > CONFIG_POINTER_2S_MIXER_REMAINDER_TTL) {
            data->rpt_x_remainder = rotated_x;
            data->rpt_y_remainder = rotated_y;
        } else {
            data->rpt_x_remainder += rotated_x;
            data->rpt_y_remainder += rotated_y;
        }

        data->values.s1_x = 0;
        data->values.s1_y = 0;
        dt = 0;
    }

    if (data->values.s2_x != 0 || data->values.s2_y != 0) {
        apply_rotation(data->rotation_matrix2, data->values.s2_x, data->values.s2_y, &rotated_x, &rotated_y);
        data->twist_values.s2_x += rotated_x;
        data->twist_values.s2_y += rotated_y;

        apply_coef(data->move_coef, &rotated_x, &rotated_y);
        if (dt > CONFIG_POINTER_2S_MIXER_REMAINDER_TTL) {
            data->rpt_x_remainder = rotated_x;
            data->rpt_y_remainder = rotated_y;
        } else {
            data->rpt_x_remainder += rotated_x;
            data->rpt_y_remainder += rotated_y;
        }

        data->values.s2_x = 0;
        data->values.s2_y = 0;
    }

    data->rpt_x = (int16_t) data->rpt_x_remainder;
    data->rpt_y = (int16_t) data->rpt_y_remainder;
    data->rpt_x_remainder -= data->rpt_x;
    data->rpt_y_remainder -= data->rpt_y;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_SCROLL_DISABLES_POINTER)
    if (now - data->last_rpt_time_twist < CONFIG_POINTER_2S_MIXER_POINTER_AFTER_SCROLL_ACTIVATION) {
        data->last_rpt_time = now;
        data->rpt_x = 0;
        data->rpt_y = 0;
        return 0;
    }
#endif

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

static struct dataframe_history_entry* dataframe_history_add(const struct device *dev, const struct p2sm_dataframe *dataframe) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();
    
    struct dataframe_history_entry *entry = data->history_pool;
    if (entry != NULL) {
        data->history_pool = entry->next;
    } else if (data->history_buffer_index < data->max_history_entries) {
        entry = &data->history_buffer[data->history_buffer_index++];
    } else {
        entry = malloc(sizeof(struct dataframe_history_entry));
        if (entry == NULL) {
            LOG_ERR("Failed to allocate dataframe history entry");
            return NULL;
        }
    }
    
    entry->timestamp = now;
    entry->dataframe = *dataframe;
    entry->next = data->history_head;
    data->history_head = entry;
    
    const int16_t movement = dataframe->s1_x + dataframe->s1_y + dataframe->s2_x + dataframe->s2_y;
    data->interference_accumulator += movement;
    return entry;
}

static void dataframe_history_cleanup(const struct device *dev, const int64_t cutoff_time) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    struct dataframe_history_entry **current = &data->history_head;
    
    while (*current != NULL) {
        if ((*current)->timestamp < cutoff_time) {
            struct dataframe_history_entry *to_remove = *current;
            *current = to_remove->next;
            
            const int16_t movement = to_remove->dataframe.s1_x + to_remove->dataframe.s1_y + 
                                   to_remove->dataframe.s2_x + to_remove->dataframe.s2_y;
            data->interference_accumulator -= movement;
            to_remove->next = data->history_pool;
            data->history_pool = to_remove;
        } else {
            current = &(*current)->next;
        }
    }
}

static float calculate_twist(const struct device *dev) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();
    const int64_t passed = now - data->last_twist;
    const int16_t s1_x = data->twist_values.s1_x;
    const int16_t s1_y = data->twist_values.s1_y;
    const int16_t s2_x = data->twist_values.s2_x;
    const int16_t s2_y = data->twist_values.s2_y;

    memset(&data->twist_values, 0, sizeof(data->twist_values));
    if (s1_x == 0 && s1_y == 0 && s2_x == 0 && s2_y == 0) {
        return 0;
    }

    const struct p2sm_dataframe current_dataframe = {s1_x, s1_y, s2_x, s2_y};
    const int64_t cutoff_time = now - config->twist_interference_window;
    struct dataframe_history_entry *history_entry = dataframe_history_add(dev, &current_dataframe);
    dataframe_history_cleanup(dev, cutoff_time);

    LOG_DBG("Interference: %d", abs(data->interference_accumulator));
    if (abs(data->interference_accumulator) > config->twist_interference_thres) {
        LOG_DBG("Discarded twist (reason = interference_threshold)");
        return 0;
    }

    uint8_t total_under_thres = 0;
    total_under_thres += abs(s1_x) < config->twist_thres;
    total_under_thres += abs(s1_y) < config->twist_thres;
    total_under_thres += abs(s2_x) < config->twist_thres;
    total_under_thres += abs(s2_y) < config->twist_thres;
    
    LOG_DBG("Analyzing movement: (%d, %d) (%d, %d)", s1_x, s1_y, s2_x, s2_y);
    if (total_under_thres == 4) {
        LOG_DBG("Discarded movement (reason = twist_thres)");
        return 0;
    }

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_34_FILTER_EN)
    if (total_under_thres == 3) {
        LOG_DBG("Discarded movement (reason = 3_of_4_below_twist_thres)");
        return 0;
    }
#endif

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
        const int16_t threshold = config->twist_thres;

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

        const float result = (t->direction ? -(float)(t->val1 - t->val2) : (float)(t->val2 - t->val1)) - (float)(s1_y + s2_y);
        history_entry->calculated_scroll = result;

        data->last_twist = now;
        data->last_twist_direction = t->direction;
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
        k_work_reschedule(&data->twist_filter_cleanup_work, K_MSEC(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_TTL));
#endif

        LOG_DBG("Scroll value calculated: %d", (int) result);
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

static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t p1,
                           const uint32_t p2, struct zmk_input_processor_state *s) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const int64_t now = k_uptime_get();

    if (unlikely(!data->initialized)) {
        if (!data_init(dev)) {
            LOG_ERR("Failed to initialize mixer driver data!");
            return -1;
        }
    }

    if (p1 & INPUT_MIXER_SENSOR1) {
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
        data->last_sensor1_report = now;
#endif

        if (event->code == INPUT_REL_X) {
            data->values.s1_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->values.s1_y += event->value;
        }
    } else if (p1 & INPUT_MIXER_SENSOR2) {
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
        data->last_sensor2_report = now;
#endif

        if (event->code == INPUT_REL_X) {
            data->values.s2_x += event->value;
        } else if (event->code == INPUT_REL_Y) {
            data->values.s2_y += event->value;
        }
    }

    event->value = 0;
    event->sync = false;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    if (unlikely(abs((int32_t) (data->last_sensor1_report - data->last_sensor2_report)) > CONFIG_POINTER_2S_MIXER_SYNC_WINDOW_MS)) {
        memset(&data->values, 0, sizeof(struct p2sm_dataframe));
        memset(&data->twist_values, 0, sizeof(struct p2sm_dataframe));
        return 0;
    }
#endif

    if (now - data->last_rpt_time > config->sync_report_ms) {
        process_and_report(dev);
    }

    if (now - data->last_rpt_time_twist > config->sync_scroll_report_ms) {
        const float twist_float = calculate_twist(dev) * data->twist_coef;
        if (now - data->last_rpt_time_twist > CONFIG_POINTER_2S_MIXER_TWIST_REMAINDER_TTL) {
            data->rpt_twist_remainder = twist_float;
        } else {
            data->rpt_twist_remainder += twist_float;
        }

        const int16_t twist_int = (int16_t)data->rpt_twist_remainder;
        data->rpt_twist_remainder -= twist_int;

        if (twist_int != 0) {
            if (scroll_percentage_filter(dev)) {
                data->last_rpt_time_twist = now;
                input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, twist_int, true, K_NO_WAIT);
            } 
        }

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
        data->twist_accumulator += abs(twist_int);

        if (config->feedback_gpios.port != NULL &&
            data->twist_accumulator >= config->twist_feedback_threshold &&
            config->twist_feedback_threshold > 0) {
            data->twist_accumulator = 0;

            if (config->feedback_extra_gpios.port != NULL) {
                data->previous_feedback_extra_state = gpio_pin_get_dt(&config->feedback_extra_gpios);
                if (gpio_pin_set_dt(&config->feedback_extra_gpios, 1) != 0) {
                    LOG_ERR("Failed to set twist feedback extra GPIO");
                }
            }

            if (config->twist_feedback_delay > 0) {
                k_work_reschedule(&data->twist_feedback_extra_delay_work, K_MSEC(config->twist_feedback_delay));
                LOG_DBG("Twist feedback extra GPIO activated, scheduling main feedback after %d ms delay", config->twist_feedback_delay);
            } else {
                if (gpio_pin_set_dt(&config->feedback_gpios, 1) == 0) {
                    k_work_reschedule(&data->twist_feedback_off_work, K_MSEC(config->twist_feedback_duration));
                    LOG_DBG("Twist feedback activated immediately (accumulator: %d)", data->twist_accumulator);
                } else {
                    LOG_ERR("Failed to set twist feedback GPIO");
                }
            }
        }
#endif
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
    const float radius = config->ball_radius;
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

    if (surface_p1[0] == surface_p2[0] && surface_p1[1] == surface_p2[1] && surface_p1[2] == surface_p2[2]) {
        LOG_ERR("Unexpected: both trackpoints have same coordinates!");
        return 0;
    }

    calculate_rotation_matrix(surface_p1[0], surface_p1[1], surface_p1[2], 0, 0, -radius, data->rotation_matrix1);
    calculate_rotation_matrix(surface_p2[0], surface_p2[1], surface_p2[2], 0, 0, -radius, data->rotation_matrix2);

    data->last_twist_direction = -1;
    data->move_coef = 1.0f;
    data->twist_coef = 1.0f;
    
    data->history_head = NULL;
    data->history_pool = NULL;
    data->interference_accumulator = 0;
    
    data->max_history_entries = (config->twist_interference_window / config->sync_scroll_report_ms) + 2;
    data->history_buffer = malloc(data->max_history_entries * sizeof(struct dataframe_history_entry));
    data->history_buffer_index = 0;
    
    if (data->history_buffer == NULL) {
        LOG_ERR("Failed to allocate history buffer");
        data->max_history_entries = 0;
    }

    // going >1 means losing precision
    // acceptable for scroll but not movement
    if (data->move_coef > 1.0f) {
        data->move_coef = 1.0f;
    }

    LOG_DBG("Sensor mixer driver initialized");
    LOG_DBG("  > Ball radius: %d", (int) config->ball_radius);
    LOG_DBG("  > Surface trackpoint 1 ≈ (%d, %d, %d)", (int) surface_p1[0], (int) surface_p1[1], (int) surface_p1[2]);
    LOG_DBG("  > Surface trackpoint 2 ≈ (%d, %d, %d)", (int) surface_p2[0], (int) surface_p2[1], (int) surface_p2[2]);

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    if (config->feedback_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&config->feedback_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure twist feedback GPIO");
        } else {
            LOG_DBG("Twist feedback GPIO configured");
            k_work_init_delayable(&data->twist_feedback_off_work, twist_feedback_off_work_cb);
        }
    } else {
        LOG_DBG("No feedback set up for twist");
    }

    if (config->feedback_extra_gpios.port != NULL) {
        if (gpio_pin_configure_dt(&config->feedback_extra_gpios, GPIO_OUTPUT) != 0) {
            LOG_WRN("Failed to configure twist feedback extra GPIO");
        } else {
            LOG_DBG("Twist feedback extra GPIO configured");
            k_work_init_delayable(&data->twist_feedback_extra_delay_work, twist_feedback_extra_delay_work_cb);
        }
    } else {
        LOG_DBG("No extra feedback set up for twist");
    }
#endif

    g_dev = (struct device *) dev;
    data->initialized = true;

    p2sm_sens_driver_init();

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    k_work_init_delayable(&data->twist_filter_cleanup_work, twist_filter_cleanup_work_cb);
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&p2sm_save_work, p2sm_save_work_cb);
#endif

    return 1;
}

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
static void twist_feedback_off_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct zip_pointer_2s_mixer_data *data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_feedback_off_work);
    const struct device *dev = data->dev;
    const struct zip_pointer_2s_mixer_config *config = dev->config;

    gpio_pin_set_dt(&config->feedback_gpios, 0);
    
    if (config->feedback_extra_gpios.port != NULL) {
        gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
    }
    
    LOG_DBG("Twist feedback turned off");
}

static void twist_feedback_extra_delay_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zip_pointer_2s_mixer_data *data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_feedback_extra_delay_work);
    const struct device *dev = data->dev;
    const struct zip_pointer_2s_mixer_config *config = dev->config;

    if (gpio_pin_set_dt(&config->feedback_gpios, 1) == 0) {
        k_work_reschedule(&data->twist_feedback_off_work, K_MSEC(config->twist_feedback_duration));
        LOG_DBG("Twist feedback activated after extra delay");
    } else {
        LOG_ERR("Failed to set twist feedback GPIO after extra delay");
    }
}
#endif

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static void p2sm_save_work_cb(struct k_work *work) {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    const float values[2] = { data->move_coef, data->twist_coef };

    const int err = settings_save_one(P2SM_SETTINGS_PREFIX, values, sizeof(values));
    if (err < 0) {
        LOG_ERR("Failed to save settings %d", err);
    } else {
        LOG_DBG("Sensitivity settings saved");
    }
}

static void p2sm_save_sensitivity() {
    k_work_reschedule(&p2sm_save_work, K_MSEC(CONFIG_POINTER_2S_MIXER_SETTINGS_SAVE_DELAY));
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

float p2sm_get_twist_coef() {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return 0;
    }

    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    return data->twist_coef;
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

void p2sm_set_twist_coef(const float coef) {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->twist_coef = coef;

#if IS_ENABLED(CONFIG_SETTINGS)
    p2sm_save_sensitivity();
#endif
}

static struct zip_pointer_2s_mixer_data data = {};
static struct zip_pointer_2s_mixer_config config = {
    .sync_report_ms = DT_INST_PROP(0, sync_report_ms),
    .sync_scroll_report_ms = DT_INST_PROP(0, sync_scroll_report_ms),
    .twist_interference_thres = DT_INST_PROP(0, twist_interference_thres),
    .twist_interference_window = DT_INST_PROP(0, twist_interference_window),
    .twist_thres = DT_INST_PROP(0, twist_thres),
    .sensor1_pos = DT_INST_PROP(0, sensor1_pos),
    .sensor2_pos = DT_INST_PROP(0, sensor2_pos),
    .ball_radius = DT_INST_PROP(0, ball_radius),
    .feedback_gpios = GPIO_DT_SPEC_INST_GET_OR(0, feedback_gpios, { .port = NULL }),
    .feedback_extra_gpios = GPIO_DT_SPEC_INST_GET_OR(0, feedback_extra_gpios, { .port = NULL }),
    .twist_feedback_duration = DT_INST_PROP_OR(0, twist_feedback_duration, 0),
    .twist_feedback_threshold = DT_INST_PROP_OR(0, twist_feedback_threshold, 0),
    .twist_feedback_delay = DT_INST_PROP_OR(0, twist_feedback_delay, 0),
};
DEVICE_DT_INST_DEFINE(0, &sy_init, NULL, &data, &config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);
