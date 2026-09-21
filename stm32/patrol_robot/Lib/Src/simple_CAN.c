//
// Created by lml on 2025/12/22.
//

#include "simple_CAN.h"

CAN_RxHeaderTypeDef rx_header;    //接收数据结构体
uint8_t rx_data[8];               //can接收数据数组

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // 看第0位ID, 判断是数据帧还是遥控帧
  // 遥控帧暂不处理
  if (Object_Para & 0x01)
  {
    return;
  }

  // 看第1位ID, 判断是标准帧还是扩展帧
  // 扩展帧暂不处理
  if ((Object_Para & 0x02) >> 1)
  {
    return;
  }

  // 标准帧

  // ID配置, 标准帧的ID是11bit, 按规定放在高16bit中的[15:5]位
  // 掩码后ID的高16bit
  can_filter_init_structure.FilterIdHigh = (ID & 0x7FF) << 5;
  // 掩码后ID的低16bit
  can_filter_init_structure.FilterIdLow = 0x0000;
  // 掩码后屏蔽位的高16bit
  can_filter_init_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
  // 掩码后屏蔽位的低16bit
  can_filter_init_structure.FilterMaskIdLow = 0x0000;

  // 滤波器配置
  // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
  can_filter_init_structure.FilterBank = (Object_Para >> 3) & 0x1F;
  // 滤波器模式, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤波
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
  // 使能滤波器
  can_filter_init_structure.FilterActivation = ENABLE;

  // 从机模式配置
  // 从机模式选择开始单元, 一般均分14个单元给CAN1和CAN2
  can_filter_init_structure.SlaveStartFilterBank = 14;

  // 滤波器绑定FIFOx, 只能绑定一个
  can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, const uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t used_mailbox;

  // 检测关键传参
  assert_param(hcan != NULL);

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = 0;
  tx_header.RTR = 0;
  tx_header.DLC = Length;

  return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}
