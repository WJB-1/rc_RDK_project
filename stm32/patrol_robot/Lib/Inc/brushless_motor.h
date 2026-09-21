//
// Created by lml on 2026/1/16.
//

#ifndef RM_C_TEST_BRUSHLESS_MOTOR_H
#define RM_C_TEST_BRUSHLESS_MOTOR_H

#include "main.h"
#include "string.h"
#include "math.h"
#include "simple_CAN.h"
#include "can.h"

#define PI 3.14159265

//电机结构体
typedef struct
{
    int32_t original_angle_data;         // 原始角度数据
    int16_t original_gyro_data;          // 原始角速度数据
    float original_gyro,gyro;			 // °/s
    float angle, offsetAngle;            // °
    float voltage, maxVoltage;           // V
    float torque, torqueRatio;           // Nm, voltage = torque / torqueRatio
    float speed;		                 // cm/s
    float distance;                      //整车行走距离（单位：cm）
    float (*calcRevVolt)(float speed_value);      // 指向反电动势计算函数
} Motor;

//小车整体
extern float car_speed;
extern float car_distance;
//电机结构体
extern Motor leftWheel;
extern Motor rightWheel;

void Motor_data_calculation();
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float (*calcRevVolt)(float speed));
float Motor_CalcRevVolt(const float gyro_value);
void Motor_InitAll();
void Motor_SetTorque(Motor *motor,const float torque);
void Motor_UpdateVoltage(Motor *motor,const float *sourceVoltage);
void Motor_SendTask(Motor *motor_L,Motor *motor_R,const float *sourceVoltage,int16_t *L_voltage,int16_t *R_voltage);

#endif //RM_C_TEST_BRUSHLESS_MOTOR_H