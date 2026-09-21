//
// Created by lml on 2026/3/10.
//

#include "angle_unwrap.h"
#include <math.h>

// 全局变量用于存储解缠绕后的角度
float unwrapped_yaw = 0.0f;
float unwrapped_pitch = 0.0f;
float unwrapped_roll = 0.0f;
float prev_yaw = 0.0f;
float prev_pitch = 0.0f;
float prev_roll = 0.0f;

// 解缠绕函数
float angle_unwrap(float current_angle, float prev_angle, float *unwrapped_angle) {
    float delta = current_angle - prev_angle;

    // 检测跳变（超过180度阈值）
    if (delta > 180.0f) {
        delta -= 360.0f;  // 从+180跳变到-180的情况
    } else if (delta < -180.0f) {
        delta += 360.0f;  // 从-180跳变到+180的情况
    }

    // 更新解缠绕后的角度
    *unwrapped_angle += delta;

    return *unwrapped_angle;
}

// 三轴欧拉角解缠绕
void euler_angle_unwrap(float yaw, float pitch, float roll) {
    // 对航向角（yaw）解缠绕
    unwrapped_yaw = angle_unwrap(yaw, prev_yaw, &unwrapped_yaw);
    prev_yaw = yaw;

    // 对俯仰角（pitch）解缠绕
    unwrapped_pitch = angle_unwrap(pitch, prev_pitch, &unwrapped_pitch);
    prev_pitch = pitch;

    // 对横滚角（roll）解缠绕
    unwrapped_roll = angle_unwrap(roll, prev_roll, &unwrapped_roll);
    prev_roll = roll;
}
