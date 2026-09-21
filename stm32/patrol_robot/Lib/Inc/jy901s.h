#ifndef JY901S_H
#define JY901S_H
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
/************************ 预处理命令芯片版本选择 ************************/
#define JY901S_H_Vision 1
#if   (JY901S_H_Vision==1)
#include "stm32f1xx_hal.h"
#elif (JY901S_H_Vision==4)
#include "stm32f4xx_hal.h"
#elif (JY901S_H_Vision==7)
#include "stm32h7xx_hal.h"
#endif
/************************ ottohesl文件包含管理 ************************/
#define OTTOHELS   0
#if OTTOHELS
#include "ottohesl.h"
#endif

/************************ 宏定义 ************************/
#define Frame_Head         0x55        // 数据帧头
#define Frame_Accele       0x51        // 加速度数据帧类型
#define Frame_Gyro         0x52        // 角速度数据帧类型
#define Frame_Angle        0x53        // 角度数据帧类型
#define Frame_Magnet       0x54        // 磁场数据帧类型
#define Frame_Quater       0x59        // 四元数数据帧类型
#define Frame_Length       11          // 单帧数据长度（字节）
#define RX_SIZE            256         // DMA接收缓冲区大小

/************************ 枚举定义 ************************/
// JY901S帧解析状态机
typedef enum Jy901s_FrameState {
    SEEK_FRAME_HEAD = 0,  // 寻找帧头
    SEEK_FRAME_TYPE = 1,  // 寻找帧类型
    SEEK_FRAME_DATA = 2,  // 接收帧数据
} Frame_State;

/************************ 结构体定义 ************************/
// 陀螺仪核心数据结构体
typedef struct Jy901s_Gyroscope {
    float accele[3];      // 加速度(x/y/z)，单位m/s²
    float gyro[3];        // 角速度(x/y/z)，单位°/s
    float angle[3];       // 角度(横滚/俯仰/偏航)，单位°
    float magnet[3];      // 磁场(x/y/z)，单位uT
    float quaternion[4];  // 四元数(w/x/y/z)
} JG;

// JY901S完整数据结构体
typedef struct Jy901s_Data {
    JG  gyroscope;        // 陀螺仪参数
    float temp;           // 温度，单位℃
} jy901;

/************************ 函数声明 ************************/
/* 主要函数 */
void Gyroscope_Init(UART_HandleTypeDef *h_senor,UART_HandleTypeDef *h_debug);   // 启动DMA接收陀螺仪数据
bool Gyroscope_Process();                                 // 解析接收的陀螺仪数据
/* 数据修改 */
void Gyroscope_Alter_Bit(UART_HandleTypeDef *huart);      // 修改JY901S波特率
void Gyroscope_Accele_Calibra(UART_HandleTypeDef *huart); // 加速度计校准
void Gyroscope_Rrate(UART_HandleTypeDef *huart);          // 配置数据输出速率
void Gyroscope_Gyro_Calibra(UART_HandleTypeDef *huart);   // 陀螺仪校准
/* 串口发送 */
void Gyroscope_Data_Send(UART_HandleTypeDef *huart);      // 发送解析后的陀螺仪数据
/************************ 结构声明 ************************/
extern jy901 gyro_data;
#endif //JY901S_H
