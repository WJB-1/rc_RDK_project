//
// Created by lml on 2025/12/22.
//

#ifndef RM_C_TEST_SIMPLE_CAN_H
#define RM_C_TEST_SIMPLE_CAN_H

#include "main.h"
#include "string.h"
#include "math.h"

//CAN数据包
extern CAN_RxHeaderTypeDef rx_header;
extern uint8_t rx_data[8];

#define PI 3.14159265

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, const uint8_t *Data, uint16_t Length);
void LED_Control_0(uint8_t data);

#endif //RM_C_TEST_SIMPLE_CAN_H