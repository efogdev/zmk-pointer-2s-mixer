#pragma once

void p2sm_sens_driver_init();
void p2sm_accel_driver_init();
float p2sm_get_move_coef();
float p2sm_get_twist_coef();
void p2sm_set_move_coef(float coef);
void p2sm_set_twist_coef(float coef);

bool p2sm_get_twist_accel_enabled();
void p2sm_set_twist_accel_enabled(bool enabled);
float p2sm_get_twist_accel_value();
void p2sm_set_twist_accel_value(float value);
