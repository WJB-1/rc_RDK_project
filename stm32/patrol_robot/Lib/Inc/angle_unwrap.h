//
// Created by lml on 2026/3/10.
//

#ifndef PATROL_ROBOT_ANGLE_UNWRAP_H
#define PATROL_ROBOT_ANGLE_UNWRAP_H

#include "main.h"

extern float unwrapped_yaw;
extern float unwrapped_pitch;
extern float unwrapped_roll;

float angle_unwrap(float current_angle, float prev_angle, float *unwrapped_angle);
void euler_angle_unwrap(float yaw, float pitch, float roll);

#endif //PATROL_ROBOT_ANGLE_UNWRAP_H