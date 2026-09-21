/*
 * speed_PID.c
 *
 *  Created on: May 10, 2025
 *      Author: lml
 */

#include "speed_PID.h"

/* ================= PID 参数 ================= */

float Kp_L = 88.0f;
float Ki_L = 28.0f;
float Kd_L = 0.8f;

float Kp_R = 88.0f;
float Ki_R = 28.0f;
float Kd_R = 0.8f;

/* ================= 控制周期 ================= */

static const float dt = 0.02f;

/* ================= 左轮状态 ================= */

static float error_L = 0.0f;
static float last_error_L = 0.0f;
static float integral_L = 0.0f;

/* ================= 右轮状态 ================= */

static float error_R = 0.0f;
static float last_error_R = 0.0f;
static float integral_R = 0.0f;
/* =========================================================
 * 左轮速度 PID
 * ========================================================= */
float speed_pid_L(const float *speed, float target)
{
    float derivative;
    float pid_output;
    float output;
    float feedforward;

    /* ===== 目标太小时直接停车 ===== */
    if (fabsf(target) < 1.0f)
    {
        integral_L = 0.0f;
        last_error_L = 0.0f;
        return 0.0f;
    }

    /* ===== 计算误差 ===== */
    error_L = target - *speed;

    /* ===== 误差死区 ===== */
    if (fabsf(error_L) < 0.2f)
    {
        error_L = 0.0f;
    }

    /* ===== 积分 ===== */
    integral_L += error_L * dt;

    /* ===== 抗积分饱和 ===== */
    if (integral_L > 1000.0f)
    {
        integral_L = 1000.0f;
    }

    if (integral_L < -1000.0f)
    {
        integral_L = -1000.0f;
    }

    /* ===== 微分 ===== */
    derivative = (error_L - last_error_L) / dt;

    last_error_L = error_L;

    /* ===== PID ===== */
    pid_output =
            Kp_L * error_L +
            Ki_L * integral_L +
            Kd_L * derivative;

    /* ===== 前馈 ===== */
    feedforward = (float)predict_feedback_L(&target);

    /* ===== 总输出 ===== */
    output = pid_output + feedforward;

    /* ===== 输出限幅 ===== */
    if (output > 10000.0f)
    {
        output = 10000.0f;
    }

    if (output < -10000.0f)
    {
        output = -10000.0f;
    }

    return output * 0.00001f;
}

/* =========================================================
 * 右轮速度 PID
 * ========================================================= */
float speed_pid_R(const float *speed, float target)
{
    float derivative;
    float pid_output;
    float output;
    float feedforward;

    /* ===== 目标太小时直接停车 ===== */
    if (fabsf(target) < 1.0f)
    {
        integral_R = 0.0f;
        last_error_R = 0.0f;
        return 0.0f;
    }

    /* ===== 计算误差 ===== */
    error_R = target - *speed;

    /* ===== 误差死区 ===== */
    if (fabsf(error_R) < 0.2f)
    {
        error_R = 0.0f;
    }

    /* ===== 积分 ===== */
    integral_R += error_R * dt;

    /* ===== 抗积分饱和 ===== */
    if (integral_R > 1000.0f)
    {
        integral_R = 1000.0f;
    }

    if (integral_R < -1000.0f)
    {
        integral_R = -1000.0f;
    }

    /* ===== 微分 ===== */
    derivative = (error_R - last_error_R) / dt;

    last_error_R = error_R;

    /* ===== PID ===== */
    pid_output =
            Kp_R * error_R +
            Ki_R * integral_R +
            Kd_R * derivative;

    /* ===== 前馈 ===== */
    feedforward = (float)predict_feedback_R(&target);

    /* ===== 总输出 ===== */
    output = pid_output + feedforward;

    /* ===== 输出限幅 ===== */
    if (output > 10000.0f)
    {
        output = 10000.0f;
    }

    if (output < -10000.0f)
    {
        output = -10000.0f;
    }

    return output * 0.00001f;
}

// float Kp_L = 75.0f, Ki_L = 0.0f, Kd_L = 30.0f;
// float jf_out_L = 0.0f;
// float Kp_R = 75.0f, Ki_R = 0.0f, Kd_R = 30.0f;
// float jf_out_R = 0.0f;

/*
 * 原来的误差历史和积分量都放在函数内部，外部调用 PID_out_clear()
 * 其实并不能真正清零。
 * 这里把状态提升到文件作用域，停车、转向、重新起步时就能完整复位。
 */
// static float error_L = 0.0f;
// static float last_error_L = 0.0f;
// static float last_last_error_L = 0.0f;
// static int16_t vol_out_L = 0;
// static int16_t jf_sum_L = 0;
//
// static float error_R = 0.0f;
// static float last_error_R = 0.0f;
// static float last_last_error_R = 0.0f;
// static int16_t vol_out_R = 0;
// static int16_t jf_sum_R = 0;

/**
 * @brief 左轮速度控制
 *
 * @param speed 真实速度
 * @param target 目标速度
 * @return 输出扭矩
 */
// float speed_pid_L(const float *speed, float target)
// {
//     static int16_t feedforward = 0;
//
//     if (fabsf(target) <= 2.0f)
//     {
//         jf_out_L = 0.0f;
//         vol_out_L = predict_feedback_L(&target);
//         return 0.0f;
//     }
//
//     if ((*speed == 0.0f) && (fabsf(target) > 2.0f))
//     {
//         error_L = 0.0f;
//         last_error_L = 0.0f;
//         last_last_error_L = 0.0f;
//         return 0.01f;
//     }
//
//     error_L = target - *speed;
//     jf_out_L = Kp_L * (error_L - last_error_L) +
//                Ki_L * error_L +
//                Kd_L * (error_L - 2.0f * last_error_L + last_last_error_L);
//
//     last_last_error_L = last_error_L;
//     last_error_L = error_L;
//
//     feedforward = predict_feedback_L(&target);
//     jf_sum_L = (int16_t)(jf_sum_L + (int16_t)jf_out_L);
//     vol_out_L = (int16_t)(jf_sum_L + feedforward);
//
//     if (vol_out_L > 10000)
//     {
//         vol_out_L = 10000;
//     }
//     else if (vol_out_L < -10000)
//     {
//         vol_out_L = -10000;
//     }
//
//     return vol_out_L * 0.00001f;
// }
//
// /**
//  * @brief 右轮速度控制
//  *
//  * @param speed 真实速度
//  * @param target 目标速度
//  * @return 输出扭矩
//  */
// float speed_pid_R(const float *speed, float target)
// {
//     static int16_t feedforward = 0;
//
//     if (fabsf(target) <= 2.0f)
//     {
//         jf_out_R = 0.0f;
//         vol_out_R = predict_feedback_R(&target);
//         return 0.0f;
//     }
//
//     if ((*speed == 0.0f) && (fabsf(target) > 2.0f))
//     {
//         error_R = 0.0f;
//         last_error_R = 0.0f;
//         last_last_error_R = 0.0f;
//         return 0.01f;
//     }
//
//     error_R = target - *speed;
//     jf_out_R = Kp_R * (error_R - last_error_R) +
//                Ki_R * error_R +
//                Kd_R * (error_R - 2.0f * last_error_R + last_last_error_R);
//
//     last_last_error_R = last_error_R;
//     last_error_R = error_R;
//
//     feedforward = predict_feedback_R(&target);
//     jf_sum_R = (int16_t)(jf_sum_R + (int16_t)jf_out_R);
//     vol_out_R = (int16_t)(jf_sum_R + feedforward);
//
//     if (vol_out_R > 10000)
//     {
//         vol_out_R = 10000;
//     }
//     else if (vol_out_R < -10000)
//     {
//         vol_out_R = -10000;
//     }
//
//     return vol_out_R * 0.00001f;
// }

/**
 * @brief 左轮前馈控制
 *
 * @param target_speed 目标速度
 * @return feedforward 输出前馈控制量
 */
int16_t predict_feedback_L(const float *target_speed)
{
    static float one_coefficient = 7.5952f;
    static float two_coefficient = -0.1929f;
    static float three_coefficient = 0.0033f;
    static float constant = 800.0f;
    static float feedforward = 0.0f;

    if (*target_speed == 0.0f)
    {
        return 0;
    }

    feedforward = three_coefficient * (*target_speed) * (*target_speed) * (*target_speed) +
                  two_coefficient * (*target_speed) * (*target_speed) +
                  one_coefficient * (*target_speed) +
                  constant;

    if (*target_speed > 0.0f)
    {
        return (int16_t)feedforward;
    }

    return (int16_t)-feedforward;
}

/**
 * @brief 右轮前馈控制
 *
 * @param target_speed 目标速度
 * @return feedforward 输出前馈控制量
 */
int16_t predict_feedback_R(const float *target_speed)
{
    static float one_coefficient = 7.5952f;
    static float two_coefficient = -0.1929f;
    static float three_coefficient = 0.0033f;
    static float constant = 800.0f;
    static float feedforward = 0.0f;

    if (*target_speed == 0.0f)
    {
        return 0;
    }

    feedforward = three_coefficient * (*target_speed) * (*target_speed) * (*target_speed) +
                  two_coefficient * (*target_speed) * (*target_speed) +
                  one_coefficient * (*target_speed) +
                  constant;

    if (*target_speed > 0.0f)
    {
        return (int16_t)feedforward;
    }

    return (int16_t)-feedforward;
}

void PID_change(float p, float i, float d)
{
    Kp_L = p;
    Ki_L = i;
    Kd_L = d;
    Kp_R = p;
    Ki_R = i;
    Kd_R = d;
}
void PID_out_clear(void)
{
    error_L = 0.0f;
    last_error_L = 0.0f;
    integral_L = 0.0f;

    error_R = 0.0f;
    last_error_R = 0.0f;
    integral_R = 0.0f;
}
// void PID_out_clear(void)
// {
//     jf_out_L = 0.0f;
//     jf_out_R = 0.0f;
//
//     error_L = 0.0f;
//     last_error_L = 0.0f;
//     last_last_error_L = 0.0f;
//     vol_out_L = 0;
//     jf_sum_L = 0;
//
//     error_R = 0.0f;
//     last_error_R = 0.0f;
//     last_last_error_R = 0.0f;
//     vol_out_R = 0;
//     jf_sum_R = 0;
// }
