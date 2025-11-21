#pragma once
#include <zephyr/dt-bindings/dt-util.h>

#define P2SM_SETTINGS_PREFIX "p2sm_sens"
#define P2SM_ACCEL_SETTINGS_PREFIX "p2sm_accel"

#define INPUT_MIXER_SENSOR1 BIT(0)
#define INPUT_MIXER_SENSOR2 BIT(1)

#define P2SM_INC BIT(0)
#define P2SM_DEC BIT(1)

#define P2SM_ACCEL_DIS 0
#define P2SM_ACCEL_EN 1
#define P2SM_ACCEL_TOGGLE 2
