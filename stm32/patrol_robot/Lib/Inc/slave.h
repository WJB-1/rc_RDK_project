//
// Created by lml on 2026/3/26.
//

#ifndef PATROL_ROBOT_SLAVE_H
#define PATROL_ROBOT_SLAVE_H

#include "main.h"
#include "math.h"
#include "tim.h"

typedef struct {
    int work_times;             // 根据设置的总运行时间计算需要运行多少次
    int work_ccr;               // 每次运行需要增加多少占空比
    int work;                   // 当前已经运行了多少次
    uint32_t crr_value;         // 当前的crr数值
    float PWM;                  // 目标crr数值
    int status;                 // 舵机运行状态（1：正在运行   0：空闲）
} slave;

extern slave slave_0;
extern slave slave_1;

void slave_Init();
void reset_slave(slave *Slave);
void cloud_control_one(int ID,float angle,float use_time);

#endif //PATROL_ROBOT_SLAVE_H