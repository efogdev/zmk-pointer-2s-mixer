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

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#include <zmk_runtime_config/runtime_config.h>
#else
#define ZRC_GET(key, default_val) (default_val)
#endif

#define DT_DRV_COMPAT zmk_pointer_2s_mixer
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct device *g_dev = NULL;

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
static uint32_t g_zrc_cache_last_refresh = 0;
static bool     g_zrc_cache_initialized  = false;
#endif

/* pointer path */
static bool     g_zrc_frame_sync       = (bool)     IS_ENABLED(CONFIG_POINTER_2S_MIXER_FRAME_SYNC);
static bool     g_zrc_scroll_dis_ptr   = (bool)     IS_ENABLED(CONFIG_POINTER_2S_MIXER_SCROLL_DISABLES_POINTER);
static uint32_t g_zrc_ptr_after_scroll = (uint32_t) CONFIG_POINTER_2S_MIXER_POINTER_AFTER_SCROLL_ACTIVATION;
static uint32_t g_zrc_steady_thres     = (uint32_t) CONFIG_POINTER_2S_MIXER_STEADY_THRES;

/* twist/scroll path */
static bool     g_zrc_twist_global_en  = (bool)     IS_ENABLED(CONFIG_POINTER_2S_MIXER_TWIST_EN);
static uint32_t g_zrc_twist_ttl        = (uint32_t) CONFIG_POINTER_2S_MIXER_TWIST_FILTER_TTL;
static bool     g_zrc_twist_hyst_en    = (bool)     IS_ENABLED(CONFIG_POINTER_2S_MIXER_TWIST_HYST_EN);
static uint16_t g_zrc_twist_hyst_thres = (uint16_t) CONFIG_POINTER_2S_MIXER_TWIST_HYST_THRES;
static uint16_t g_zrc_twist_thres      = (uint16_t) CONFIG_POINTER_2S_MIXER_TWIST_THRES;
static uint16_t g_zrc_twist_hyst_mul   = (uint16_t) CONFIG_POINTER_2S_MIXER_TWIST_HYST_MUL;
static uint16_t g_zrc_dy_mag_mul       = (uint16_t) CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_MUL;
static uint16_t g_zrc_twist_hyst_div   = (uint16_t) CONFIG_POINTER_2S_MIXER_TWIST_HYST_DIV;
static uint16_t g_zrc_dy_mag_div       = (uint16_t) CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_DIV;
static uint8_t  g_zrc_ema_alpha        = (uint8_t)  CONFIG_POINTER_2S_MIXER_EMA_ALPHA;
static uint32_t g_zrc_twist_deb        = (uint32_t) CONFIG_POINTER_2S_MIXER_TWIST_FILTER_DEBOUNCE;
static uint32_t g_zrc_steady_cd        = (uint32_t) CONFIG_POINTER_2S_MIXER_STEADY_COOLDOWN;
static bool     g_zrc_feedback_en      = (bool)     IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN);
static uint16_t g_zrc_fb_thres         = (uint16_t) CONFIG_POINTER_2S_MIXER_TWIST_FEEDBACK_THRESHOLD;
static uint32_t g_zrc_fb_max_cont      = (uint32_t) CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_CONTINUOUS;
static int32_t  g_zrc_fb_cooldown      = (int32_t)  CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN;
static uint32_t g_zrc_fb_dur           = (uint32_t) CONFIG_POINTER_2S_MIXER_TWIST_FEEDBACK_DURATION;

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#define ZRC_REFRESH_YIELD()                                          \
    do {                                                             \
        if (CONFIG_POINTER_2S_MIXER_ZRC_REFRESH_YIELD_US > 0) {      \
            k_usleep(CONFIG_POINTER_2S_MIXER_ZRC_REFRESH_YIELD_US);  \
        }                                                            \
    } while (0)
#endif

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
static const struct zrc_cache_entry {
    const char *key;
    void *dst;
    uint8_t size;
} zrc_cache_tbl[] = {
    { "p2sm/frame_sync",       &g_zrc_frame_sync,       sizeof(g_zrc_frame_sync)       },
    { "p2sm/scroll_dis_ptr",   &g_zrc_scroll_dis_ptr,   sizeof(g_zrc_scroll_dis_ptr)   },
    { "p2sm/ptr_after_scroll", &g_zrc_ptr_after_scroll, sizeof(g_zrc_ptr_after_scroll) },
    { "p2sm/steady_thres",     &g_zrc_steady_thres,     sizeof(g_zrc_steady_thres)     },
    { "p2sm/twist_global_en",  &g_zrc_twist_global_en,  sizeof(g_zrc_twist_global_en)  },
    { "p2sm/twist_ttl",        &g_zrc_twist_ttl,        sizeof(g_zrc_twist_ttl)        },
    { "p2sm/twist_hyst_en",    &g_zrc_twist_hyst_en,    sizeof(g_zrc_twist_hyst_en)    },
    { "p2sm/twist_hyst_thres", &g_zrc_twist_hyst_thres, sizeof(g_zrc_twist_hyst_thres) },
    { "p2sm/twist_thres",      &g_zrc_twist_thres,      sizeof(g_zrc_twist_thres)      },
    { "p2sm/twist_hyst_mul",   &g_zrc_twist_hyst_mul,   sizeof(g_zrc_twist_hyst_mul)   },
    { "p2sm/twist_dy_mag_mul", &g_zrc_dy_mag_mul,       sizeof(g_zrc_dy_mag_mul)       },
    { "p2sm/twist_hyst_div",   &g_zrc_twist_hyst_div,   sizeof(g_zrc_twist_hyst_div)   },
    { "p2sm/twist_dy_mag_div", &g_zrc_dy_mag_div,       sizeof(g_zrc_dy_mag_div)       },
    { "p2sm/ema_alpha",        &g_zrc_ema_alpha,        sizeof(g_zrc_ema_alpha)        },
    { "p2sm/twist_deb",        &g_zrc_twist_deb,        sizeof(g_zrc_twist_deb)        },
    { "p2sm/steady_cd",        &g_zrc_steady_cd,        sizeof(g_zrc_steady_cd)        },
    { "p2sm/feedback_en",      &g_zrc_feedback_en,      sizeof(g_zrc_feedback_en)      },
    { "p2sm/fb_thres",         &g_zrc_fb_thres,         sizeof(g_zrc_fb_thres)         },
    { "p2sm/fb_max_cont",      &g_zrc_fb_max_cont,      sizeof(g_zrc_fb_max_cont)      },
    { "p2sm/fb_cooldown",      &g_zrc_fb_cooldown,      sizeof(g_zrc_fb_cooldown)      },
    { "p2sm/fb_dur",           &g_zrc_fb_dur,           sizeof(g_zrc_fb_dur)           },
};
#endif

// even though ZRC_GET is very cheap, it's not free.
// local cache with polling helps to avoid thousands of reads per sec
static __attribute__((noinline)) void zrc_cache_refresh_if_due(const uint32_t now) {
#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
    if (likely(g_zrc_cache_initialized) &&
        (now - g_zrc_cache_last_refresh) < CONFIG_POINTER_2S_MIXER_ZRC_POLL_MS) {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(zrc_cache_tbl); i++) {
        const struct zrc_cache_entry *e = &zrc_cache_tbl[i];
        const int32_t v = zrc_get(e->key);
        memcpy(e->dst, &v, e->size);
        if (i + 1 < ARRAY_SIZE(zrc_cache_tbl)) {
            ZRC_REFRESH_YIELD();
        }
    }

    g_zrc_cache_last_refresh = now;
    g_zrc_cache_initialized  = true;
#else
    ARG_UNUSED(now);
#endif
}

static void twist_filter_cleanup_work_cb(struct k_work *work);

static void twist_feedback_off_work_cb(struct k_work *work);
static void twist_feedback_extra_delay_work_cb(struct k_work *work);
static void twist_feedback_cooldown_work_cb(struct k_work *work);

#if IS_ENABLED(CONFIG_SETTINGS)
static float g_from_settings[2] = { -1, -1 };
struct k_work_delayable p2sm_save_work;
static void p2sm_save_work_cb(struct k_work *work);
#endif

struct zip_pointer_2s_mixer_config {
    const uint32_t sync_report_ms, sync_scroll_report_ms;

    // CPI and sync window dependent
    const uint16_t twist_interference_thres, twist_interference_window;

    // zero (origin) = down left bottom, not the ball center
    const uint8_t sensor1_pos[3], sensor2_pos[3];
    const uint8_t ball_radius; // up to 127
    
    // feedback (i.e. vibration)
    // ToDo refactor to accept any behavior
    const struct gpio_dt_spec feedback_gpios;
    const struct gpio_dt_spec feedback_extra_gpios;
    const uint16_t twist_feedback_delay;
};

struct p2sm_dataframe {
    int16_t s1_x, s1_y, s2_x, s2_y;
};

// origin = ball center
struct zip_pointer_2s_mixer_data {
    const struct device *dev;
    struct k_work_delayable twist_filter_cleanup_work;

    bool initialized, twist_enabled, twist_reversed;
    bool s1_synced, s2_synced;
    uint32_t last_rpt_time, last_rpt_time_twist;
    int16_t rpt_x, rpt_y;
    float rpt_x_remainder, rpt_y_remainder, rpt_twist_remainder;
    float move_coef, twist_coef;

    struct p2sm_dataframe frame;
    float rotated_x[2], rotated_y[2];
    struct p2sm_dataframe twist_values;

    // pre-calculated
    float rotation_matrix1[3][3], rotation_matrix2[3][3];

    uint32_t last_twist, debounce_start; // to filter out single events as they are probably accidental
    int8_t last_twist_direction; // to filter out first event in the opposite direction

    float ema_delta_y, ema_translation;
    bool ema_initialized;
    bool sma_enabled;

    uint32_t last_sig_move;
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    uint32_t last_sensor1_report, last_sensor2_report;
#endif

    uint32_t twist_accumulator;
    int8_t twist_feedback_direction;
    struct k_work_delayable twist_feedback_off_work;
    struct k_work_delayable twist_feedback_extra_delay_work;
    struct k_work_delayable twist_feedback_cooldown_work;
    int previous_feedback_extra_state;
    uint32_t feedback_start_time;
    uint32_t feedback_cooldown_until;
    bool feedback_is_in_cooldown;

    float (*sma_buffer)[2];
    uint8_t sma_head_index;
    uint8_t sma_count;
    uint8_t sma_window_size;
    uint32_t last_sma_time;
};

static int data_init(const struct device *dev);
static void apply_rotation(float matrix[3][3], float dx, float dy, float *out_x, float *out_y);
static void apply_coef(float coef, float *x, float *y);

static void apply_sma(struct zip_pointer_2s_mixer_data *data, float *x, float *y) {
    if (data == NULL || x == NULL || y == NULL || data->sma_window_size < 2) {
        return;
    }

    if (data->sma_buffer == NULL) {
        data->sma_buffer = malloc(CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX * sizeof(*data->sma_buffer));
        if (data->sma_buffer == NULL) {
            LOG_ERR("SMA buffer allocation failed");
            return;
        }
        memset(data->sma_buffer, 0, CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX * sizeof(*data->sma_buffer));
        data->sma_head_index = 0;
        data->sma_count = 0;
    }

    const uint32_t now = (uint32_t) k_uptime_get();
    if (data->sma_count > 0 && now - data->last_sma_time > CONFIG_POINTER_2S_MIXER_SMA_TIMEOUT) {
        data->sma_head_index = 0;
        data->sma_count = 0;
        LOG_DBG("SMA history discarded (timeout)");
    }

    data->last_sma_time = now;
    data->sma_buffer[data->sma_head_index][0] = *x;
    data->sma_buffer[data->sma_head_index][1] = *y;
    data->sma_head_index = (data->sma_head_index + 1) % data->sma_window_size;
    if (data->sma_count < data->sma_window_size) {
        data->sma_count++;
    }

    if (data->sma_count == data->sma_window_size) {
        float sum_x = 0.0f, sum_y = 0.0f;
        for (uint8_t i = 0; i < data->sma_count; i++) {
            sum_x += data->sma_buffer[i][0];
            sum_y += data->sma_buffer[i][1];
        }

        *x = sum_x / (float) data->sma_window_size;
        *y = sum_y / (float) data->sma_window_size;
    }
}

static int process_and_report(const struct device *dev) {
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const uint32_t now = (uint32_t) k_uptime_get();
    uint32_t dt = now - data->last_rpt_time;

    int16_t *twist_x[2] = { &data->twist_values.s1_x, &data->twist_values.s2_x };
    int16_t *twist_y[2] = { &data->twist_values.s1_y, &data->twist_values.s2_y };
    for (uint8_t s = 0; s < 2; s++) {
        float rx = data->rotated_x[s];
        float ry = data->rotated_y[s];
        if (rx == 0 && ry == 0) {
            continue;
        }

        *twist_x[s] += (int16_t) rx;
        *twist_y[s] += (int16_t) ry;

        apply_coef(data->move_coef, &rx, &ry);
        if (dt > CONFIG_POINTER_2S_MIXER_REMAINDER_TTL) {
            data->rpt_x_remainder = rx;
            data->rpt_y_remainder = ry;
        } else {
            data->rpt_x_remainder += rx;
            data->rpt_y_remainder += ry;
        }

        data->rotated_x[s] = 0;
        data->rotated_y[s] = 0;
        dt = 0;
    }

    if (g_zrc_scroll_dis_ptr && now - data->last_rpt_time_twist < g_zrc_ptr_after_scroll) {
        data->last_rpt_time = now;
        data->rpt_x_remainder = 0;
        data->rpt_y_remainder = 0;
        data->rpt_x = 0;
        data->rpt_y = 0;
        return 0;
    }

    data->rpt_x = (int16_t) data->rpt_x_remainder;
    data->rpt_y = (int16_t) data->rpt_y_remainder;

    if (data->sma_enabled && (data->rpt_x || data->rpt_y)) {
        apply_sma(data, &data->rpt_x_remainder, &data->rpt_y_remainder);
        data->rpt_x = (int16_t) data->rpt_x_remainder;
        data->rpt_y = (int16_t) data->rpt_y_remainder;
    }

    data->rpt_x_remainder -= data->rpt_x;
    data->rpt_y_remainder -= data->rpt_y;

    const bool have_x = data->rpt_x != 0;
    const bool have_y = data->rpt_y != 0;
    if (have_x || have_y) {
        const int32_t steady_thres = (int32_t) g_zrc_steady_thres;
        if (abs(data->rpt_x) > steady_thres || abs(data->rpt_y) > steady_thres) {
            data->last_sig_move = now;
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

    const uint32_t filter_ttl = g_zrc_twist_ttl;
    const bool hyst_en = g_zrc_twist_hyst_en;
    const bool hyst_active = hyst_en && passed < filter_ttl;
    const uint16_t eff_thres = hyst_active ? g_zrc_twist_hyst_thres : g_zrc_twist_thres;
    const uint16_t eff_mul   = hyst_active ? g_zrc_twist_hyst_mul   : g_zrc_dy_mag_mul;
    const uint16_t eff_div   = hyst_active ? g_zrc_twist_hyst_div   : g_zrc_dy_mag_div;

    if (abs(s1_y) < eff_thres || abs(s2_y) < eff_thres) {
        LOG_DBG("Discarded movement (reason = twist_thres)");
        return 0;
    }

    if (config->twist_interference_thres != 0) {
        if (abs(s1_x+ s2_x) > config->twist_interference_thres || abs(s1_y + s2_y) > config->twist_interference_thres) {
            LOG_DBG("Discarded movement (reason = significant_translation)");
            return 0;
        }
    }

    const bool direction = s1_y < s2_y;
#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    if (data->last_twist_direction != direction) {
        data->last_twist_direction = direction;
        data->last_twist = now;
        data->debounce_start = now;
        data->ema_initialized = false;
        LOG_DBG("Discarded twist (reason = direction_filter)");
        return 0;
    }
#endif

    const float delta_y = (float) abs(direction ? s2_y - s1_y : s1_y - s2_y);
    const float translation = abs(s1_x + s2_x) + abs(s1_y + s2_y);
    if (!data->ema_initialized) {
        data->ema_translation = translation;
        data->ema_delta_y = delta_y;
        data->ema_initialized = true;
    } else {
        const float alpha = (float) g_zrc_ema_alpha / 100.0f;
        data->ema_translation = alpha * translation + (1.0f - alpha) * data->ema_translation;
        data->ema_delta_y = alpha * delta_y + (1.0f - alpha) * data->ema_delta_y;
    }

    const uint16_t avg_translation = (uint16_t) data->ema_translation;
    const uint16_t avg_delta_y = (uint16_t) data->ema_delta_y;
    const uint16_t max_mag = avg_translation * eff_mul / eff_div;
    const float result = ((avg_delta_y - eff_thres) > max_mag ? avg_delta_y - avg_translation : 0) * (s1_y > s2_y ? -1 : 1);
    const int int_result = abs((int) result);

    if (config->twist_interference_thres != 0 && avg_translation > config->twist_interference_thres) {
        LOG_DBG("Discarded twist (reason = significant_translation)");
        data->ema_initialized = false;
        return 0;
    }

    if (int_result == 0) {
        return 0;
    }

    if (config->twist_interference_thres > 0 && avg_translation > config->twist_interference_thres) {
        LOG_DBG("Discarded twist (reason = interference)");
        return 0;
    }

    if (now - data->debounce_start < g_zrc_twist_deb) {
        LOG_DBG("Discarded twist (reason = debounce)");
        data->last_twist = now;
        return 0;
    }

    if (passed > filter_ttl) {
        LOG_DBG("Discarded twist (reason = time_filter)");
        data->debounce_start = now;
        data->last_twist = now;
        return 0;
    }

    if (data->last_sig_move - now < g_zrc_steady_cd) {
        LOG_DBG("Discarded twist (reason = steady_cooldown)");
        data->debounce_start = now;
        data->last_twist = now;
        return 0;
    }

    data->last_twist = now;
    data->last_twist_direction = direction;

    if (IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN) || g_zrc_feedback_en) {
        k_work_reschedule(&data->twist_filter_cleanup_work, K_MSEC(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_TTL));
    }

    LOG_DBG("Scroll value calculated: %d", (int) result);
    return result;
}

static void twist_filter_cleanup_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct zip_pointer_2s_mixer_data *dwork_data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_filter_cleanup_work);
    const struct device *dev = dwork_data->dev;
    struct zip_pointer_2s_mixer_data *data = dev->data;

    data->twist_feedback_direction = -1;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_DIRECTION_FILTER_EN)
    data->last_twist_direction = -1;
#endif

    LOG_DBG("Direction filter data discarded (timeout)");
}

static int line_sphere_intersection(const float r, const float x, const float y, const float z, float intersection[3]);

static int sensor_surface_pos(const float radius, const uint8_t pos[3], float out[3]) {
    return line_sphere_intersection(radius, (float) pos[0] - 127.f, (float) pos[1] - 127.f, (float) pos[2] - 127.f, out);
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

static void on_sensor_event(struct zip_pointer_2s_mixer_data *data, const uint8_t s,
                            struct input_event *event, const bool frame_end, const uint32_t now) {
    int16_t *fx = (s == 0) ? &data->frame.s1_x : &data->frame.s2_x;
    int16_t *fy = (s == 0) ? &data->frame.s1_y : &data->frame.s2_y;
    bool *synced = (s == 0) ? &data->s1_synced : &data->s2_synced;
    float (*matrix)[3] = (s == 0) ? data->rotation_matrix1 : data->rotation_matrix2;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    uint32_t *last_report = (s == 0) ? &data->last_sensor1_report : &data->last_sensor2_report;
    *last_report = now;
#endif

    if (event->code == INPUT_REL_X) {
        *fx += event->value;
    } else if (event->code == INPUT_REL_Y) {
        *fy += event->value;
    }

    if (!frame_end) {
        return;
    }

    const int16_t dx = *fx;
    const int16_t dy = *fy;
    *fx = 0;
    *fy = 0;

    float rx, ry;
    apply_rotation(matrix, (float) dx, (float) dy, &rx, &ry);
    data->rotated_x[s] += rx;
    data->rotated_y[s] += ry;
    *synced = true;
}

static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t p1,
                           const uint32_t p2, struct zmk_input_processor_state *s) {
    const struct zip_pointer_2s_mixer_config *config = dev->config;
    struct zip_pointer_2s_mixer_data *data = dev->data;
    const uint32_t now = (uint32_t) k_uptime_get();
    const bool frame_end = g_zrc_frame_sync ? event->sync : true;

    if (unlikely(!data->initialized)) {
        if (!data_init(dev)) {
            LOG_ERR("Failed to initialize mixer driver data!");
            return -1;
        }
    }

    zrc_cache_refresh_if_due(now);

    if (p1 & INPUT_MIXER_SENSOR1) {
        on_sensor_event(data, 0, event, frame_end, now);
    } else if (p1 & INPUT_MIXER_SENSOR2) {
        on_sensor_event(data, 1, event, frame_end, now);
    }

    event->value = 0;
    event->sync = false;

#if IS_ENABLED(CONFIG_POINTER_2S_MIXER_ENSURE_SYNC)
    if (unlikely(abs((int32_t) (data->last_sensor1_report - data->last_sensor2_report)) > CONFIG_POINTER_2S_MIXER_SYNC_WINDOW_MS)) {
        memset(&data->frame, 0, sizeof(struct p2sm_dataframe));
        memset(&data->twist_values, 0, sizeof(struct p2sm_dataframe));
        memset(data->rotated_x, 0, sizeof(data->rotated_x));
        memset(data->rotated_y, 0, sizeof(data->rotated_y));
        data->s1_synced = false;
        data->s2_synced = false;
        return 0;
    }
#endif

    if (data->s1_synced && data->s2_synced && now - data->last_rpt_time > config->sync_report_ms) {
        data->s1_synced = false;
        data->s2_synced = false;
        process_and_report(dev);
    }

    const bool global_enabled = g_zrc_twist_global_en;
    if (data->twist_enabled && global_enabled && now - data->last_rpt_time_twist > config->sync_scroll_report_ms) {
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

            if (g_zrc_feedback_en) {
                data->twist_accumulator += abs(twist_int);

                const bool direction = twist_float > 0;
                const uint16_t fb_thres = g_zrc_fb_thres;
                if (config->feedback_gpios.port != NULL &&
                    (data->twist_accumulator >= fb_thres || data->twist_feedback_direction != direction) &&
                    fb_thres > 0) {
                    data->twist_accumulator = 0;

                    if (data->feedback_is_in_cooldown && now < data->feedback_cooldown_until) {
                        LOG_DBG("Twist feedback skipped (in cooldown until %d)", data->feedback_cooldown_until - now);
                        data->twist_feedback_direction = direction;
                        return 0;
                    }

                    if (data->feedback_start_time > 0 && (now - data->feedback_start_time) >= g_zrc_fb_max_cont) {
                        k_work_cancel_delayable(&data->twist_feedback_off_work);
                        k_work_cancel_delayable(&data->twist_feedback_extra_delay_work);

                        gpio_pin_set_dt(&config->feedback_gpios, 0);
                        if (config->feedback_extra_gpios.port != NULL) {
                            gpio_pin_set_dt(&config->feedback_extra_gpios, data->previous_feedback_extra_state);
                        }

                        data->feedback_start_time = 0;
                        data->feedback_is_in_cooldown = true;
                        data->feedback_cooldown_until = now + g_zrc_fb_cooldown;
                        k_work_reschedule(&data->twist_feedback_cooldown_work, K_MSEC(g_zrc_fb_cooldown));

                        LOG_DBG("Twist feedback forced off after max continuous duration, cooldown for %d ms", g_zrc_fb_cooldown);
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
            }
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

    if (!sensor_surface_pos(radius, config->sensor1_pos, surface_p1)) {
        LOG_ERR("Failed to get surface position for sensor 1!");
        return 0;
    }

    if (!sensor_surface_pos(radius, config->sensor2_pos, surface_p2)) {
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
    data->twist_enabled = true;

    data->ema_delta_y = 0.0f;
    data->ema_translation = 0.0f;
    data->ema_initialized = false;

    data->sma_enabled = false;
    data->sma_buffer = NULL;
    data->sma_window_size = CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE;
    data->sma_head_index = 0;
    data->sma_count = 0;

    // going >1 means losing precision
    // acceptable for scroll but not movement
    if (data->move_coef > 1.0f) {
        data->move_coef = 1.0f;
    }

    LOG_DBG("Sensor mixer driver initialized");
    LOG_DBG("  > Ball radius: %d", (int) config->ball_radius);
    LOG_DBG("  > Surface trackpoint 1 ≈ (%d, %d, %d)", (int) surface_p1[0], (int) surface_p1[1], (int) surface_p1[2]);
    LOG_DBG("  > Surface trackpoint 2 ≈ (%d, %d, %d)", (int) surface_p2[0], (int) surface_p2[1], (int) surface_p2[2]);

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

    g_dev = (struct device *) dev;
    data->initialized = true;

    p2sm_sens_driver_init();

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&p2sm_save_work, p2sm_save_work_cb);
#endif

    k_work_init_delayable(&data->twist_filter_cleanup_work, twist_filter_cleanup_work_cb);
    return 1;
}

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
    const uint32_t elapsed = data->feedback_start_time > 0 ? now - data->feedback_start_time : 0;
    const uint32_t remaining_duration = g_zrc_fb_max_cont > elapsed ? g_zrc_fb_max_cont - elapsed : 0;
    const uint32_t feedback_duration = g_zrc_fb_dur < remaining_duration ? g_zrc_fb_dur : remaining_duration;

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
        data->feedback_cooldown_until = now + g_zrc_fb_cooldown;
        k_work_reschedule(&data->twist_feedback_cooldown_work, K_MSEC(g_zrc_fb_cooldown));
        LOG_DBG("Twist feedback after delay immediately off, max duration reached, cooldown for %d ms", g_zrc_fb_cooldown);
    }
}

static void twist_feedback_cooldown_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zip_pointer_2s_mixer_data *data = CONTAINER_OF(dwork, struct zip_pointer_2s_mixer_data, twist_feedback_cooldown_work);

    data->feedback_is_in_cooldown = false;
    data->feedback_cooldown_until = 0;
    LOG_DBG("Twist feedback cooldown period ended");
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static void p2sm_save_one(const char *suffix, const void *value, const size_t len) {
    char key[36];
    snprintf(key, sizeof(key), "%s/%s", P2SM_SETTINGS_PREFIX, suffix);
    const int err = settings_save_one(key, value, len);
    if (err < 0) {
        LOG_ERR("Failed to save settings %d", err);
    }
}

static void p2sm_save_work_cb(struct k_work *work) {
    const struct zip_pointer_2s_mixer_data *data = g_dev->data;
    const float values[2] = { data->move_coef, data->twist_coef };

    p2sm_save_one("global", values, sizeof(values));
    p2sm_save_one("twist_reversed", &data->twist_reversed, sizeof(data->twist_reversed));
    p2sm_save_one("sma_en", &data->sma_enabled, sizeof(data->sma_enabled));
    p2sm_save_one("sma_win", &data->sma_window_size, sizeof(data->sma_window_size));
}

static void p2sm_save_config() {
    k_work_reschedule(&p2sm_save_work, K_MSEC(CONFIG_POINTER_2S_MIXER_SETTINGS_SAVE_DELAY));
}
#endif

static __attribute__((noinline)) struct zip_pointer_2s_mixer_data *p2sm_data(void) {
    if (g_dev == NULL) {
        LOG_ERR("Device not initialized!");
        return NULL;
    }
    return g_dev->data;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define P2SM_PERSIST() p2sm_save_config()
#else
#define P2SM_PERSIST() ((void)0)
#endif

float p2sm_get_move_coef() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->move_coef : 0;
}

float p2sm_get_twist_coef() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->twist_coef : 0;
}

void p2sm_set_move_coef(const float coef) {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->move_coef = coef;
    P2SM_PERSIST();
}

void p2sm_set_twist_coef(const float coef) {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->twist_coef = coef;
    P2SM_PERSIST();
}

bool p2sm_twist_enabled() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->twist_enabled : false;
}

bool p2sm_twist_is_reversed() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->twist_reversed : false;
}

void p2sm_toggle_twist_reverse() {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->twist_reversed = !data->twist_reversed;
    P2SM_PERSIST();
}

static void p2sm_toggle_twist_set_reversed(const bool reversed) {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->twist_reversed = reversed;
}

void p2sm_toggle_twist() {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->twist_enabled = !data->twist_enabled;
}

bool p2sm_sma_enabled() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->sma_enabled : false;
}

void p2sm_set_sma_enabled(const bool enabled) {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->sma_enabled = enabled;
    P2SM_PERSIST();
}

uint8_t p2sm_get_sma_window() {
    const struct zip_pointer_2s_mixer_data *data = p2sm_data();
    return data ? data->sma_window_size : 0;
}

void p2sm_set_sma_window(const uint8_t window_size) {
    struct zip_pointer_2s_mixer_data *data = p2sm_data();
    if (!data) return;
    data->sma_window_size = window_size > CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX ? CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX : window_size;
    data->sma_head_index = 0;
    data->sma_count = 0;
    P2SM_PERSIST();
}

#if IS_ENABLED(CONFIG_SETTINGS)
// ReSharper disable once CppParameterMayBeConst
static int p2sm_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (settings_name_steq(name, "twist_reversed", NULL)) {
        bool reverse = false;
        const int rd = read_cb(cb_arg, &reverse, sizeof(reverse));
        if (rd == sizeof(bool)) {
            p2sm_toggle_twist_set_reversed(reverse);
        } else {
            LOG_ERR("Failed to load twist reversed");
        }

        return 0;
    }

    if (settings_name_steq(name, "sma_en", NULL)) {
        bool sma_en = false;
        const int rd = read_cb(cb_arg, &sma_en, sizeof(sma_en));
        if (rd == sizeof(bool)) {
            if (g_dev != NULL) {
                struct zip_pointer_2s_mixer_data *data = g_dev->data;
                data->sma_enabled = sma_en;
            }
        } else {
            LOG_ERR("Failed to load sma_en");
        }

        return 0;
    }

    if (settings_name_steq(name, "sma_win", NULL)) {
        uint8_t sma_win = CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE;
        const int rd = read_cb(cb_arg, &sma_win, sizeof(sma_win));
        if (rd == sizeof(uint8_t)) {
            if (g_dev != NULL) {
                struct zip_pointer_2s_mixer_data *data = g_dev->data;
                data->sma_window_size = sma_win > CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX ? CONFIG_POINTER_2S_MIXER_SMA_WINDOW_SIZE_MAX : sma_win;
            }
        } else {
            LOG_ERR("Failed to load sma_win");
        }

        return 0;
    }

    if (!settings_name_steq(name, "global", NULL)) {
        return 0;
    }

    const int err = read_cb(cb_arg, g_from_settings, sizeof(g_from_settings));
    if (err < 0) {
        LOG_ERR("Failed to load settings (err = %d)", err);
    } else {
        if (g_dev == NULL) {
            LOG_ERR("Device not initialized!");
            return -EBUSY;
        }

        struct zip_pointer_2s_mixer_data *data = g_dev->data;
        data->move_coef = g_from_settings[0];
        data->twist_coef = g_from_settings[1];
    }
    
    return err;
}

SETTINGS_STATIC_HANDLER_DEFINE(p2sm_settings, P2SM_SETTINGS_PREFIX, NULL, p2sm_settings_load_cb, NULL, NULL);
#endif

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
static const struct zrc_param_def {
    const char *key;
    int32_t default_val, min_val, max_val;
} zrc_param_defs[] = {
    { "p2sm/scroll_dis_ptr",   IS_ENABLED(CONFIG_POINTER_2S_MIXER_SCROLL_DISABLES_POINTER), 0, 1 },
    { "p2sm/ptr_after_scroll", CONFIG_POINTER_2S_MIXER_POINTER_AFTER_SCROLL_ACTIVATION, 0, 5000 },
    { "p2sm/twist_global_en",  IS_ENABLED(CONFIG_POINTER_2S_MIXER_TWIST_EN), 0, 1 },
    { "p2sm/ema_alpha",        CONFIG_POINTER_2S_MIXER_EMA_ALPHA, 1, 50 },
    { "p2sm/twist_dy_mag_mul", CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_MUL, 1, 100 },
    { "p2sm/twist_dy_mag_div", CONFIG_POINTER_2S_MIXER_DELTA_Y_OVER_TRANS_MAG_DIV, 1, 100 },
    { "p2sm/twist_hyst_en",    IS_ENABLED(CONFIG_POINTER_2S_MIXER_TWIST_HYST_EN), 0, 1 },
    { "p2sm/twist_hyst_thres", CONFIG_POINTER_2S_MIXER_TWIST_HYST_THRES, 1, 100 },
    { "p2sm/twist_hyst_mul",   CONFIG_POINTER_2S_MIXER_TWIST_HYST_MUL, 1, 100 },
    { "p2sm/twist_hyst_div",   CONFIG_POINTER_2S_MIXER_TWIST_HYST_DIV, 1, 100 },
    { "p2sm/twist_thres",      CONFIG_POINTER_2S_MIXER_TWIST_THRES, 1, 100 },
    { "p2sm/twist_ttl",        CONFIG_POINTER_2S_MIXER_TWIST_FILTER_TTL, 0, 5000 },
    { "p2sm/twist_deb",        CONFIG_POINTER_2S_MIXER_TWIST_FILTER_DEBOUNCE, 0, 5000 },
    { "p2sm/steady_thres",     CONFIG_POINTER_2S_MIXER_STEADY_THRES, 0, 255 },
    { "p2sm/steady_cd",        CONFIG_POINTER_2S_MIXER_STEADY_COOLDOWN, 0, 5000 },
    { "p2sm/feedback_en",      IS_ENABLED(CONFIG_POINTER_2S_MIXER_FEEDBACK_EN), 0, 1 },
    { "p2sm/fb_max_cont",      CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_CONTINUOUS, 0, 5000 },
    { "p2sm/fb_cooldown",      CONFIG_POINTER_2S_MIXER_FEEDBACK_COOLDOWN, 0, 5000 },
    { "p2sm/fb_thres",         CONFIG_POINTER_2S_MIXER_TWIST_FEEDBACK_THRESHOLD, 0, 5000 },
    { "p2sm/fb_dur",           CONFIG_POINTER_2S_MIXER_TWIST_FEEDBACK_DURATION, 0, 5000 },
    { "p2sm/frame_sync",       IS_ENABLED(CONFIG_POINTER_2S_MIXER_FRAME_SYNC), 0, 1 },
};

static int p2sm_register_runtime_params(void) {
    for (size_t i = 0; i < ARRAY_SIZE(zrc_param_defs); i++) {
        const struct zrc_param_def *d = &zrc_param_defs[i];
        zrc_register(d->key, d->default_val, d->min_val, d->max_val);
    }
    return 0;
}
SYS_INIT(p2sm_register_runtime_params, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif /* CONFIG_ZMK_RUNTIME_CONFIG */

static struct zip_pointer_2s_mixer_data data = {};
static struct zip_pointer_2s_mixer_config config = {
    .sync_report_ms = DT_INST_PROP(0, sync_report_ms),
    .sync_scroll_report_ms = DT_INST_PROP(0, sync_scroll_report_ms),
    .twist_interference_thres = DT_INST_PROP(0, twist_interference_thres),
    .twist_interference_window = DT_INST_PROP_OR(0, twist_interference_window, 0),
    .sensor1_pos = DT_INST_PROP(0, sensor1_pos),
    .sensor2_pos = DT_INST_PROP(0, sensor2_pos),
    .ball_radius = DT_INST_PROP(0, ball_radius),
    .feedback_gpios = GPIO_DT_SPEC_INST_GET_OR(0, feedback_gpios, { .port = NULL }),
    .feedback_extra_gpios = GPIO_DT_SPEC_INST_GET_OR(0, feedback_extra_gpios, { .port = NULL }),
    .twist_feedback_delay = DT_INST_PROP_OR(0, twist_feedback_delay, 0),
};
DEVICE_DT_INST_DEFINE(0, &sy_init, NULL, &data, &config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &sy_driver_api);
