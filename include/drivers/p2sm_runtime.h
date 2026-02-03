#pragma once

void p2sm_sens_driver_init();

float p2sm_get_move_coef();
float p2sm_get_twist_coef();
void p2sm_set_move_coef(float coef);
void p2sm_set_twist_coef(float coef);

bool p2sm_twist_enabled();
bool p2sm_twist_is_reversed();
void p2sm_toggle_twist();
void p2sm_toggle_twist_reverse();

struct p2sm_sens_behavior_config {
    uint16_t step;
    uint16_t min_step, max_step;
    uint8_t max_multiplier;
    bool wrap, feedback_on_limit;
    uint16_t feedback_duration;
    uint8_t feedback_wrap_pattern_len;
    int feedback_wrap_pattern[CONFIG_POINTER_2S_MIXER_FEEDBACK_MAX_ARR_VALUES];
    char* display_name;
    bool scroll;
};

uint8_t p2sm_sens_num_behaviors();
struct p2sm_sens_behavior_config p2sm_sens_behavior_get_config(uint8_t id);
int p2sm_sens_behavior_set_config(uint8_t id, struct p2sm_sens_behavior_config config);
void p2sm_sens_behaviors_save_all();
void p2sm_sens_load_and_apply_behaviors_config();
