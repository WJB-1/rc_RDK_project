/*
 * speed_PID.h
 *
 *  Created on: May 10, 2025
 *      Author: lml
 */

#ifndef INC_SPEED_PID_H_
#define INC_SPEED_PID_H_

#include "main.h"
#include "math.h"

float speed_pid_L(const float *speed,float target);
float speed_pid_R(const float *speed,float target);
int16_t predict_feedback_L(const float *target_speed);
int16_t predict_feedback_R(const float *target_speed);
void PID_change(float p,float i,float d);
void PID_out_clear();

#endif /* INC_SPEED_PID_H_ */
