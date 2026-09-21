//
// Created by lml on 2026/3/19.
//

#include "mix_algorithm.h"

//上位机调整参数，提取浮点数
float safe_extract_float(const char *receiveData) {
    // 1. 查找冒号位置
    const char *colon = strchr(receiveData, ':');
    if (colon == NULL) {
        return -1.0f; // 或根据需求返回错误码
    }

    // 2. 跳过冒号，指向数字部分
    const char *number_start = colon + 1;
    char *endptr;
    errno = 0; // 清零错误码

    // 3. 使用 strtof 转换
    float value = strtof(number_start, &endptr);

    return value;
}

//陀螺仪数据读取
void Read_IMU_Data()
{
    if (Gyroscope_Process())
    {
        euler_angle_unwrap(gyro_data.gyroscope.angle[2], gyro_data.gyroscope.angle[1], gyro_data.gyroscope.angle[0]);
        imuData.roll  = gyro_data.gyroscope.angle[0];
        imuData.pitch = gyro_data.gyroscope.angle[1];
        imuData.yaw = unwrapped_yaw;
        imuData.gyro[2] = gyro_data.gyroscope.gyro[0];
        imuData.gyro[1] = gyro_data.gyroscope.gyro[1];
        imuData.gyro[0] = gyro_data.gyroscope.gyro[2];
        imuData.accel[2] = gyro_data.gyroscope.accele[0];
        imuData.accel[1] = gyro_data.gyroscope.accele[1];
        imuData.accel[0] = gyro_data.gyroscope.accele[2];
        imuData.temperature = gyro_data.temp;
        imuData.magnet[2] = gyro_data.gyroscope.magnet[0];
        imuData.magnet[1] = gyro_data.gyroscope.magnet[1];
        imuData.magnet[0] = gyro_data.gyroscope.magnet[2];
    }
}
