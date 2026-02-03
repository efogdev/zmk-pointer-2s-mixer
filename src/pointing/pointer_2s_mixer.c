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
static void twist_feedback_cooldown_work_cb(struct k_work *work);
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
static float g_from_settings[2] = { -1, -1 };
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
    // ToDo refactor to accept any behavior
    const struct gpio_dt_spec feedback_gpios;
    const struct gpio_dt_spec feedback_extra_gpios;
    const uint16_t twist_feedback_duration, twist_feedback_threshold, twist_feedback_delay;
};

struct p2sm_dataframe {
    int16_t s1_x, s1_y, s2_x, s2_y;
};

// ToDo get rid of it completely?
struct dataframe_history_entry {
    uint32_t timestamp;
};

// origin = ball center
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    struct k_work_delayable twist_filter_cleanup_work, twist_history_cleanup_work;

    bool initialized, twist_enabled, twist_reversed;
    uint32_t last_rpt_time, last_rpt_time_twist;
    int16_t rpt_x, rpt_y;
    float rpt_x_remainder, rpt_y_remainder, rpt_twist_remainder;
    float move_coef, twist_coef;

    struct dataframe_history_entry *history_buffer;
    uint8_t max_history_entries;
    uint8_t history_head_index;
    uint8_t history_count;

    // accumulates NON-transformed X, Y movements
    struct p2sm_dataframe values;

    // accumulates transformed X, Y movements
    struct p2sm_dataframe twist_values;

    // pre-calculated
    float rotation_matrix1[3][3], rotation_matrix2[3][3];

    uint32_t last_twist, debounce_start; // to filter out single events as they are probably accidental
    int8_t last_twist_direction; // to filter out first event in the opposite direction

    float ema_delta_y, ema_translation;
    bool ema_initialized;

    uint32_t last_significant_movement;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    uint32_t last_sensor1_report, last_sensor2_report;
#endif

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    uint16_t twist_accumulator;
    int8_t twist_feedback_direction;
    struct k_work_delayable twist_feedback_off_work;
    struct k_work_delayable twist_feedback_extra_delay_work;
    struct k_work_delayable twist_feedback_cooldown_work;
    int previous_feedback_extra_state;
    uint32_t feedback_start_time;
    uint32_t feedback_cooldown_until;
    bool feedback_is_in_cooldown;
#endif
};

static int data_init(const struct device *dev);
static void apply_rotation(float matrix[3][3], float dx, float dy, float *out_x, float *out_y);
static void apply_coef(float coef, float *x, float *y);
static struct dataframe_history_entry* dataframe_history_add(const struct device *dev, const struct p2sm_dataframe *dataframe);
static bool dataframe_history_cleanup(const struct device *dev, uint32_t cutoff_time);

static int process_and_report(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const uint32_t now = (uint32_t) k_uptime_get();
    uint32_t dt = now - data->last_rpt_time;
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
        if (abs(data->rpt_x) > CONFIG_POINTER_2S_MIXER_STEADY_THRES || abs(data->rpt_y) > CONFIG_POINTER_2S_MIXER_STEADY_THRES) {
            data->last_significant_movement = now;
        }

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
    if (axis_len < (float) 1e-6) {
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
    const uint32_t now = (uint32_t) k_uptime_get();

    if (data->history_buffer == NULL) {
        LOG_WRN("History buffer not allocated");
        data->history_buffer = malloc(data->max_history_entries * sizeof(struct dataframe_history_entry));
        if (data->history_buffer == NULL) {
            LOG_ERR("Failed to allocate history buffer");
        } else {
            data->history_head_index = 0;
            data->history_count = 0;
            memset(data->history_buffer, 0, data->max_history_entries * sizeof(struct dataframe_history_entry));
            LOG_DBG("Circular history buffer allocated: %d entries", data->max_history_entries);
        }

        return NULL;
    }

    struct dataframe_history_entry *entry = &data->history_buffer[data->history_head_index];
    entry->timestamp = (uint32_t) now;

    data->history_head_index = (data->history_head_index + 1) % data->max_history_entries;
    if (data->history_count < data->max_history_entries) {
        data->history_count++;
    }

    return entry;
}

static bool dataframe_history_cleanup(const struct device *dev, const uint32_t cutoff_time) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    const struct zip_pointer_2s_mixer_data *data = dev->data;

    if (config->sync_scroll_report_ms == 0) {
        return true;
    }

    if (data->history_count == 0) {
        return false;
    }

    uint8_t valid_entries = 0;
    const uint8_t oldest_index = (data->history_head_index - data->history_count + data->max_history_entries) % data->max_history_entries;
    for (uint8_t i = 0; i < data->history_count; i++) {
        const uint8_t index = (oldest_index + i) % data->max_history_entries;
        if (data->history_buffer[index].timestamp >= cutoff_time) {
            valid_entries++;
        }
    }

    return valid_entries >= (config->twist_interference_window / config->sync_scroll_report_ms);
}

static float calculate_twist(const struct device *dev) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const uint32_t now = (uint32_t) k_uptime_get();
    const uint32_t passed = now - data->last_twist;
    const int16_t s1_x = data->twist_values.s1_x;
    const int16_t s1_y = data->twist_values.s1_y;
    const int16_t s2_x = data->twist_values.s2_x;
    const int16_t s2_y = data->twist_values.s2_y;

    memset(&data->twist_values, 0, sizeof(data->twist_values));
    if (s1_x == 0 && s1_y == 0 && s2_x == 0 && s2_y == 0) {
        return 0;
    }

    if (abs(s1_y) < config->twist_thres || abs(s2_y) < config->twist_thres) {
        LOG_DBG("Discarded movement (reason = twist_thres)");
        return 0;
    }

    const uint8_t translation_allowed = config->twist_interference_thres * CONFIG_POINTER_2S_MIXER_SIGNIFICANT_MOVEMENT_MUL;
    if (abs(s1_x+ s2_x) > translation_allowed || abs(s1_y + s2_y) > translation_allowed) {
        LOG_DBG("Discarded movement (reason = significant_translation)");
        return 0;
    }

    const bool direction = s1_y < s2_y;
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    if (data->last_twist_direction != direction) {
        data->last_twist_direction = direction;
        data->last_twist = now;
        data->debounce_start = now;
        data->ema_initialized = false;
        data->history_head_index = 0;
        data->history_count = 0;
        memset(data->history_buffer, 0, data->max_history_entries * sizeof(struct dataframe_history_entry));
        LOG_DBG("Discarded twist (reason = direction_filter)");
        return 0;
    }
#endif

    const struct p2sm_dataframe current_dataframe = {s1_x, s1_y, s2_x, s2_y};
    const struct dataframe_history_entry *history_entry = dataframe_history_add(dev, &current_dataframe);
    if (history_entry == NULL) {
        LOG_ERR("Failed to write twist history");
        return 0;
    }

    const uint32_t cutoff = now - config->twist_interference_window;
    const bool enough_entries = dataframe_history_cleanup(dev, cutoff);
    if (!enough_entries) {
        LOG_DBG("Discarded movement (reason = history_not_full)");
        return 0;
    }

    const float delta_y = (float) abs(direction ? s2_y - s1_y : s1_y - s2_y);
    const float translation = abs(s1_x + s2_x) + abs(s1_y + s2_y);
    if (!data->ema_initialized) {
        data->ema_translation = translation;
        data->ema_delta_y = delta_y;
        data->ema_initialized = true;
    } else {
        const float alpha = (float) CONFIG_POINTER_2S_MIXER_EMA_ALPHA / 100;
        data->ema_translation = alpha * translation + (1.0f - alpha) * data->ema_translation;
        data->ema_delta_y = alpha * delta_y + (1.0f - alpha) * data->ema_delta_y;
    }

    const uint16_t avg_translation = (uint16_t) data->ema_translation;
    const uint16_t avg_delta_y = (uint16_t) data->ema_delta_y;
    const uint16_t max_mag = avg_translation * CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_MUL / CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_DIV;
    const float result = ((avg_delta_y - config->twist_thres) > max_mag ? avg_delta_y - avg_translation : 0) * (s1_y > s2_y ? -1 : 1);
    const int int_result = abs((int) result);

    // LOG_INF("timestamp: %d, twist data: delta_y=%d, translation=%d", (int) now, avg_delta_y, avg_translation);

    if (avg_translation > translation_allowed) {
        LOG_DBG("Discarded twist (reason = significant_translation)");
        data->ema_initialized = false;
        data->history_head_index = 0;
        data->history_count = 0;
        memset(data->history_buffer, 0, data->max_history_entries * sizeof(struct dataframe_history_entry));
        return 0;
    }

    if (int_result < config->twist_thres || int_result > CONFIG_POINTER_2S_MIXER_TWIST_MAX_VALUE) {
        LOG_DBG("Discarded twist (reason = twist_thres)");
        return 0;
    }

    if (avg_translation > config->twist_interference_thres) {
        LOG_DBG("Discarded twist (reason = interference)");
        return 0;
    }

    if (now - data->debounce_start < CONFIG_POINTER_2S_MIXER_TWIST_FILTER_DEBOUNCE) {
        LOG_DBG("Discarded twist (reason = debounce)");
        data->last_twist = now;
        return 0;
    }

    if (passed > CONFIG_POINTER_2S_MIXER_TWIST_FILTER_TTL) {
        LOG_DBG("Discarded twist (reason = time_filter)");
        data->debounce_start = now;
        data->last_twist = now;
        return 0;
    }

    if (data->last_significant_movement - now < CONFIG_POINTER_2S_MIXER_STEADY_COOLDOWN) {
        LOG_DBG("Discarded twist (reason = steady_cooldown)");
        data->debounce_start = now;
        data->last_twist = now;
        return 0;
    }

    data->last_twist = now;
    data->last_twist_direction = direction;

    k_work_reschedule(&data->twist_history_cleanup_work, K_MSEC(config->twist_interference_window));
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN) || IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    k_work_reschedule(&data->twist_filter_cleanup_work, K_MSEC(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_TTL));
#endif

    LOG_DBG("Scroll value calculated: %d", (int) result);
    return result;
}

static void twist_filter_cleanup_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct zip_pointer_2s_mixer_data *dwork_data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_filter_cleanup_work);
    const struct device *dev = dwork_data->dev;
    struct zip_pointer_2s_mixer_data *data = dev->data;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
    data->twist_feedback_direction = -1;
#endif

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    data->last_twist_direction = -1;
#endif

    LOG_DBG("Direction filter data discarded (timeout)");
}

static void twist_history_cleanup_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct zip_pointer_2s_mixer_data *dwork_data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_history_cleanup_work);
    const struct device *dev = dwork_data->dev;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    data->history_head_index = 0;
    data->history_count = 0;
    memset(data->history_buffer, 0, data->max_history_entries * sizeof(struct dataframe_history_entry));

    LOG_DBG("Twist history discarded (timeout)");
}

static int line_sphere_intersection(const float r, const float x, const float y, const float z, float intersection[3]) {
    const float distance = sqrtf(x*x + y*y + z*z);
    if (distance < (float) 1e-6) {
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
    const uint32_t now = (uint32_t) k_uptime_get();

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

    if (data->twist_enabled && now - data->last_rpt_time_twist > config->sync_scroll_report_ms) {
        const float twist_float = calculate_twist(dev) * data->twist_coef;
        if (now - data->last_twist > CONFIG_POINTER_2S_MIXER_TWIST_REMAINDER_TTL) {
            data->rpt_twist_remainder = twist_float;
        } else {
            data->rpt_twist_remainder += twist_float;
        }

        const int16_t twist_int = (int16_t) data->rpt_twist_remainder;
        if (twist_int != 0) {
            data->last_rpt_time_twist = now;
            data->rpt_twist_remainder -= twist_int;
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, data->twist_reversed ? -twist_int : twist_int, true, K_NO_WAIT);

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN)
            data->twist_accumulator += abs(twist_int);

            const bool direction = twist_float > 0;
            if (config->feedback_gpios.port != NULL &&
                (data->twist_accumulator >= config->twist_feedback_threshold || data->twist_feedback_direction != direction) &&
                config->twist_feedback_threshold > 0) {
                data->twist_accumulator = 0;

                if (data->feedback_is_in_cooldown && now < data->feedback_cooldown_until) {
                    LOG_DBG("Twist feedback skipped (in cooldown until %d)", data->feedback_cooldown_until - now);
                    data->twist_feedback_direction = direction;
                    return 0;
                }

                if (data->feedback_start_time > 0 && (now - data->feedback_start_time) >= CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_CONTINUOUS) {
                    k_work_cancel_delayable(&data->twist_feedback_off_work);
                    k_work_cancel_delayable(&data->twist_feedback_extra_delay_work);

                    gpio_pin_set_dt(&config->feedback_gpios, 0);
                    if (config->feedback_extra_gpios.port != NULL) {
                        gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
                    }

                    data->feedback_start_time = 0;
                    data->feedback_is_in_cooldown = true;
                    data->feedback_cooldown_until = now + CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN;
                    k_work_reschedule(&data->twist_feedback_cooldown_work, K_MSEC(CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN));

                    LOG_DBG("Twist feedback forced off after max continuous duration, cooldown for %d ms", CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN);
                    data->twist_feedback_direction = direction;
                    return 0;
                }

                if (data->feedback_start_time == 0) {
                    data->feedback_start_time = now;
                }

                if (config->feedback_extra_gpios.port != NULL) {
                    data->previous_feedback_extra_state = gpio_pin_get_dt(&config->feedback_extra_gpios);
                    if (gpio_pin_set_dt(&config->feedback_extra_gpios, 1) != 0) {
                        LOG_ERR("Failed to set twist feedback extra GPIO");
                    }
                }

                k_work_reschedule(&data->twist_feedback_extra_delay_work, K_MSEC(MAX(1, config->twist_feedback_delay)));
            }

            data->twist_feedback_direction = direction;
#endif
        }
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
    data->move_coef = (float) CONFIG_POINTER_2S_MIXER_DEFAULT_MOVE_COEF / 100;
    data->twist_coef = (float) CONFIG_POINTER_2S_MIXER_DEFAULT_TWIST_COEF / 100;

    if (config->sync_scroll_report_ms != 0) {
        data->max_history_entries = (config->twist_interference_window / config->sync_scroll_report_ms) + 1;
    } else {
        data->max_history_entries = 1;
    }

    data->history_head_index = 0;
    data->history_count = 0;
    
    data->ema_delta_y = 0.0f;
    data->ema_translation = 0.0f;
    data->ema_initialized = false;

    data->twist_enabled = true;

    // going >1 means losing precision
    // acceptable for scroll but not movement
    if (data->move_coef > 1.0f) {
        data->move_coef = 1.0f;
    }

    data->history_buffer = malloc(data->max_history_entries * sizeof(struct dataframe_history_entry));
    if (data->history_buffer == NULL) {
        LOG_ERR("Failed to allocate history buffer");
    } else {
        memset(data->history_buffer, 0, data->max_history_entries * sizeof(struct dataframe_history_entry));
        LOG_INF("Circular history buffer allocated: %d entries", data->max_history_entries);
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

    data->feedback_start_time = 0;
    data->feedback_cooldown_until = 0;
    data->feedback_is_in_cooldown = false;
    k_work_init_delayable(&data->twist_feedback_cooldown_work, twist_feedback_cooldown_work_cb);
#endif

    g_dev = (struct device *) dev;
    data->initialized = true;

    p2sm_sens_driver_init();

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&p2sm_save_work, p2sm_save_work_cb);
#endif

    k_work_init_delayable(&data->twist_filter_cleanup_work, twist_filter_cleanup_work_cb);
    k_work_init_delayable(&data->twist_history_cleanup_work, twist_history_cleanup_work_cb);
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
    const uint32_t now = (uint32_t) k_uptime_get();
    const uint16_t elapsed = data->feedback_start_time > 0 ? now - data->feedback_start_time : 0;
    const uint16_t remaining_duration = CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_CONTINUOUS > elapsed
        ? CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_CONTINUOUS - elapsed  : 0;
    const uint16_t feedback_duration = config->twist_feedback_duration < remaining_duration
        ? config->twist_feedback_duration : remaining_duration;

    if (feedback_duration > 0) {
        gpio_pin_set_dt(&config->feedback_gpios, 1);
        k_work_reschedule(&data->twist_feedback_off_work, K_MSEC(feedback_duration));
        LOG_DBG("Twist feedback activated after extra delay for %d ms (remaining: %d ms)", feedback_duration, remaining_duration);
    } else {
        k_work_cancel_delayable(&data->twist_feedback_off_work);
        k_work_cancel_delayable(&data->twist_feedback_cooldown_work);
        gpio_pin_set_dt(&config->feedback_gpios, 0);
        data->feedback_start_time = 0;
        data->feedback_is_in_cooldown = true;
        data->feedback_cooldown_until = now + CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN;
        k_work_reschedule(&data->twist_feedback_cooldown_work, K_MSEC(CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN));
        LOG_DBG("Twist feedback after delay immediately off, max duration reached, cooldown for %d ms", CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN);
    }
}

static void twist_feedback_cooldown_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zip_pointer_2s_mixer_data *data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_feedback_cooldown_work);

    data->feedback_is_in_cooldown = false;
    data->feedback_cooldown_until = 0;
    LOG_DBG("Twist feedback cooldown period ended");
}
#endif

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static void p2sm_save_work_cb(struct k_work *work) {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    const float values[2] = { data->move_coef, data->twist_coef };

    char key[24];
    sprintf(key, "%s/global", P2SM_SETTINGS_PREFIX);
    int err = settings_save_one(key, values, sizeof(values));
    if (err < 0) {
        LOG_ERR("Failed to save settings %d", err);
    } else {
        LOG_DBG("Sensitivity settings saved");
    }

    sprintf(key, "%s/twist_reversed", P2SM_SETTINGS_PREFIX);
    err = settings_save_one(key, &data->twist_reversed, sizeof(data->twist_reversed));
    if (err < 0) {
        LOG_ERR("Failed to save settings %d", err);
    }
}

static void p2sm_save_config() {
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
    p2sm_save_config();
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
    p2sm_save_config();
#endif
}

bool p2sm_twist_enabled() {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    return data->twist_enabled;
}

bool p2sm_twist_is_reversed() {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    return data->twist_reversed;
}

void p2sm_toggle_twist_reverse() {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->twist_reversed = !data->twist_reversed;

#if IS_ENABLED(CONFIG_SETTINGS)
    p2sm_save_config();
#endif
}

static void p2sm_toggle_twist_set_reversed(const bool reversed) {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->twist_reversed = reversed;
}

void p2sm_toggle_twist() {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return;
    }

    struct zip_pointer_2s_mixer_data *data = g_dev->data;
    data->twist_enabled = !data->twist_enabled;
}

#if IS_ENABLED(CONFIG_SETTINGS)
// ReSharper disable once CppParameterMayBeConst
static int p2sm_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (settings_name_steq(name, "twist_reversed", NULL)) {
        bool reverse = false;
        const int err = read_cb(cb_arg, &reverse, sizeof(reverse));
        p2sm_toggle_twist_set_reversed(reverse);
        return err;
    }

    if (!settings_name_steq(name, "global", NULL)) {
        if (settings_name_steq(name, "", NULL)) {
            LOG_WRN("Loading old values for backward compatibility");
        } else {
            return 0;
        }
    }

    const int err = read_cb(cb_arg, g_from_settings, sizeof(g_from_settings));
    if (err < 0) {
        LOG_ERR("Failed to load settings (err = %d)", err);
    }

    p2sm_set_move_coef(g_from_settings[0]);
    p2sm_set_twist_coef(g_from_settings[1]);
    return err;
}

SETTINGS_STATIC_HANDLER_DEFINE(sensor_attr_cycle, P2SM_SETTINGS_PREFIX, NULL, p2sm_settings_load_cb, NULL, NULL);
#endif

static struct zip_pointer_2s_mixer_data data = {};
static struct zip_pointer_2s_mixer_config config = {
    .sync_report_ms = DT_INST_PROP(0, sync_report_ms),
    .sync_scroll_report_ms = DT_INST_PROP(0, sync_scroll_report_ms),
    .twist_interference_thres = DT_INST_PROP(0, twist_interference_thres),
    .twist_interference_window = DT_INST_PROP_OR(0, twist_interference_window, 0),
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
