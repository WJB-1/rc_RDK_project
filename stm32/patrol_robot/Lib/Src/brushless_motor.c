//
// Created by lml on 2026/1/16.
//

#include "brushless_motor.h"

//左电机：leftWheel，右电机：rightWheel
Motor leftWheel;
Motor rightWheel;
//整车
float car_speed = 0;                      //整车速度（单位：cm/s）
float car_distance = 0;                   //整车行走距离（单位：cm）

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &rx_header, rx_data);
    switch (rx_header.StdId) {
    case 0x101:     //左侧电机
        memcpy(&leftWheel.original_angle_data, &rx_data[0], sizeof(leftWheel.original_angle_data));
        memcpy(&leftWheel.original_gyro_data, &rx_data[4], sizeof(leftWheel.original_gyro_data));
        break;
    case 0x102:     //右侧电机
        memcpy(&rightWheel.original_angle_data, &rx_data[0], sizeof(rightWheel.original_angle_data));
        memcpy(&rightWheel.original_gyro_data, &rx_data[4], sizeof(rightWheel.original_gyro_data));
        break;
    }
}

void Motor_data_calculation()
{
    //时间间隔（固定为20ms = 0.02s）
    static const float dt = 0.02f;
    static const float filtered_gyro_alpha = 0.4f;    //轻微滤波增益
    //左电机数据
    leftWheel.angle = (double)leftWheel.original_angle_data / 1000.0f * 180.0f / PI;
    leftWheel.original_gyro = (double)leftWheel.original_gyro_data / 95.0f * 180.0f / PI;
    leftWheel.gyro = filtered_gyro_alpha * leftWheel.original_gyro + (1.0f - filtered_gyro_alpha) * leftWheel.gyro;     //轻微滤波
    leftWheel.speed = leftWheel.gyro / 180.0f * PI * 2.4f;
    if (fabsf(leftWheel.speed) < 0.1f)
    {
        leftWheel.speed = 0;
    }
    leftWheel.distance = leftWheel.speed * dt + leftWheel.distance;
    //右电机数据
    rightWheel.angle = (double)rightWheel.original_angle_data / 1000.0f * 180.0f / PI;
    rightWheel.original_gyro = (double)rightWheel.original_gyro_data / 95.0f * 180.0f / PI;
    rightWheel.gyro = filtered_gyro_alpha * rightWheel.original_gyro + (1.0f - filtered_gyro_alpha) * rightWheel.gyro;     //轻微滤波
    rightWheel.speed = rightWheel.gyro / 180.0f * PI * 2.4f;
    if (fabsf(rightWheel.speed) < 0.1f)
    {
        rightWheel.speed = 0;
    }
    rightWheel.distance = rightWheel.speed * dt + rightWheel.distance;
    //整车数据
    car_speed = (leftWheel.speed + rightWheel.speed) / 2.0f;
    car_distance = (leftWheel.distance + rightWheel.distance) / 2.0f;
}

//初始化一个电机对象
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float (*calcRevVolt)(float speed))
{
    motor->speed = motor->angle = motor->voltage = motor->original_gyro = motor->original_gyro = motor->distance = 0;
    motor->original_gyro_data = 0;
    motor->original_angle_data = 0;
    motor->offsetAngle = offsetAngle;
    motor->maxVoltage = maxVoltage;
    motor->torqueRatio = torqueRatio;
    motor->calcRevVolt = calcRevVolt;
}

//电机反电动势计算函数(输入速度，输出反电动势)
float Motor_CalcRevVolt(const float gyro_value)
{
    // 反电动势 = K_e * ω，其中K_e是电机反电动势常数（V/(rad/s)）
    static const float motor_Kv = 416.0f;
    // 转换为反电动势常数，K_e = 60 / (2π * Kv)  (V/(rad/s))
    const float K_e = 60.0f / (2.0f * PI * motor_Kv);
    // 角速度单位转换(rad/s)
    float omega = gyro_value * PI / 180.0f;  // rad/s
    // 反电动势 = K_e * ω
    float backEMF = K_e * omega;
    // 限制最小反电动势（避免低速时过度补偿）
    if (fabsf(backEMF) < 0.01f) {
        backEMF = 0;
    }

    return backEMF;
}

//初始化所有电机对象
//各个参数需要通过实际测量或拟合得到
void Motor_InitAll()
{
    Motor_Init(&leftWheel, 0, 6.90f, 0.01092f, Motor_CalcRevVolt);
    Motor_Init(&rightWheel, 0, 6.90f, 0.01092f, Motor_CalcRevVolt);
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, const float torque)
{
    motor->torque = torque;
}

//由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
//补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
void Motor_UpdateVoltage(Motor *motor, const float *sourceVoltage)
{
    static const float kStaticFrictionVoltage = 0.10f;
    static const float kTorqueDeadband = 0.001f;

    float motorOutRatio = (12.0f - *sourceVoltage) / 10.0f + 0.7f;
    float voltage = motor->torque / motor->torqueRatio * motorOutRatio;

    /* 反电动势补偿 */
    voltage += motor->calcRevVolt(motor->original_gyro);

    /* 静摩擦补偿：扭矩为0时绝对不能添加电压 */
    if (motor->torque > kTorqueDeadband)
    {
        voltage += kStaticFrictionVoltage;
    }
    else if (motor->torque < -kTorqueDeadband)
    {
        voltage -= kStaticFrictionVoltage;
    }

    if (voltage > motor->maxVoltage)
    {
        voltage = motor->maxVoltage;
    }
    else if (voltage < -motor->maxVoltage)
    {
        voltage = -motor->maxVoltage;
    }

    motor->voltage = voltage;
}
//电机指令发送任务
void Motor_SendTask(Motor *motor_L,Motor *motor_R,const float *sourceVoltage,int16_t *L_voltage,int16_t *R_voltage)
{
    static uint8_t Can_Send_Data[8] = {0,0,0,0,0,0,0,0};
    static int16_t motor_voltage_L = 0;
    static int16_t motor_voltage_R = 0;
    Motor_UpdateVoltage(motor_L,sourceVoltage); //计算补偿后的电机电压
    Motor_UpdateVoltage(motor_R,sourceVoltage);

    motor_voltage_L = (int16_t)(motor_L->voltage * 1000.0f);
    motor_voltage_R = (int16_t)(motor_R->voltage * 1000.0f);
    *L_voltage = motor_voltage_L;
    *R_voltage = motor_voltage_R;

    memcpy(&Can_Send_Data[0], &motor_voltage_L, sizeof(motor_voltage_L));  // 将左电机数据放入1和2号位（左电机：0x101）
    memcpy(&Can_Send_Data[2], &motor_voltage_R, sizeof(motor_voltage_R));  // 将右电机数据放入3和4号位（右电机：0x102）
    CAN_Send_Data(&hcan, 0x100, Can_Send_Data, 4);       //发送指令给电机
}
