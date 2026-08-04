#ifndef PATROL_ROBOT_MOVE_H
#define PATROL_ROBOT_MOVE_H

#include "main.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * move 模块说明
 *
 * 这个模块专门负责“上位机发命令 -> 小车解析并执行”。
 *
 * 统一串口帧格式：
 * [A5][5A][CMD/TYPE][LEN][PAYLOAD...][CHECKSUM]
 *
 * 字段含义：
 * A5 5A
 *   固定帧头，用来告诉下位机“这里是一帧新数据的开始”
 *
 * CMD/TYPE
 *   如果是上位机发给下位机，就是命令字 CMD
 *   如果是下位机回给上位机，就是反馈类型 TYPE
 *
 * LEN
 *   后面有效负载 PAYLOAD 的长度，单位是字节
 *
 * PAYLOAD
 *   真正的参数内容，例如速度、角度、距离、方向等
 *
 * CHECKSUM
 *   前面所有字节累加和的低 8 位
 */

#define MOVE_FRAME_HEAD_0               0xA5U
#define MOVE_FRAME_HEAD_1               0x5AU
#define MOVE_MAX_PAYLOAD_LEN            24U
#define MOVE_RX_CACHE_LEN               64U
#define MOVE_TX_QUEUE_LEN               8U
#define MOVE_TX_FRAME_LEN               (MOVE_MAX_PAYLOAD_LEN + 5U)

/*
 * 上位机命令字定义
 *
 * CMD_MOVE_VECTOR
 *   线速度 + 角速度控制
 *   PAYLOAD = [vx_L][vx_H][wz_L][wz_H]
 *   vx: 线速度，单位 mm/s，int16，小端
 *   wz: 角速度，单位 mrad/s，int16，小端
 *
 * CMD_ACTION
 *   离散动作命令，例如急停、清零里程
 *   PAYLOAD = [action]
 *
 * CMD_TURN_IMU
 *   原地按角度转向
 *   PAYLOAD = [angle_L][angle_H]
 *   angle: 转向角度，单位度，int16，小端
 *   正数通常表示左转，负数通常表示右转
 *
 * CMD_SERVO
 *   舵机控制命令
 *
 * CMD_INTERSECTION_TURN
 *   路口边走边转命令
 *   PAYLOAD = [distance_L][distance_H][direction]
 *   或      [distance_L][distance_H][direction][speed_L][speed_H]
 *   distance: 摄像头测得离路口距离，单位 mm，int16，小端
 *   direction: 1=左转，2=右转
 *   speed: 可选，边走边转时继续前进的速度，单位 mm/s，int16，小端
 */
typedef enum
{
    CMD_MOVE_VECTOR        = 0x01U,
    CMD_ACTION             = 0x02U,
    CMD_TURN_IMU           = 0x03U,
    CMD_SERVO              = 0x04U,
    CMD_INTERSECTION_TURN  = 0x05U,
    CMD_VISION_ERROR       = 0x06
} MoveCommandId;

/* CMD_ACTION 的动作参数 */
typedef enum
{
    MOVE_ACTION_STOP       = 0x01U, /* 急停 */
    MOVE_ACTION_RESET_ODOM = 0x02U, /* 里程清零 */
    MOVE_ACTION_START_UID  = 0x03U  /* 打开 UID 读取，当前预留 */
} MoveActionId;

/* 路口边走边转命令中的方向字段 */
typedef enum
{
    MOVE_TURN_DIR_LEFT  = 0x01U, /* 左转 */
    MOVE_TURN_DIR_RIGHT = 0x02U  /* 右转 */
} MoveTurnDirection;

/*
 * 下位机回传给上位机的反馈类型
 *
 * FB_ODOMETRY
 *   PAYLOAD = 4 字节里程，单位 mm，uint32，小端
 *
 * FB_STATUS
 *   PAYLOAD = 1 字节状态值
 *
 * FB_SENSOR
 *   PAYLOAD = 传感器数据，目前用于 UID 回传
 */
typedef enum
{
    FB_ODOMETRY = 0x01U,
    FB_STATUS   = 0x02U,
    FB_SENSOR   = 0x03U
} MoveFeedbackId;

/* 下位机状态值 */
typedef enum
{
    MOVE_STATUS_IDLE      = 0U, /* 空闲 */
    MOVE_STATUS_TURNING   = 1U, /* 正在转向 */
    MOVE_STATUS_TURN_DONE = 2U, /* 转向完成 */
    MOVE_STATUS_OBSTACLE  = 3U  /* 障碍状态，当前预留 */
} MoveStatusId;

extern  float delta_distance;
extern float vision_error ;
extern float vision_correction;
/*
 * Move_Init
 * 作用：
 * 1. 清空 move 模块内部状态
 * 2. 保存上位机串口句柄
 * 3. 里程清零
 * 4. 进入空闲状态
 *
 * 参数：
 * upper_uart
 *   和上位机通信使用的串口句柄，传给 huart1
 */
void Move_Init(UART_HandleTypeDef *upper_uart);

/*
 * Move_InputBytes
 * 作用：
 * 把串口收到的一段原始字节流交给 move 模块，
 * move 模块内部会自己完成拼包、校验和命令分发。
 *
 * 参数：
 * data
 *   串口收到的原始数据首地址
 * size
 *   这次收到的数据长度，单位字节
 */
void Move_InputBytes(const uint8_t *data, uint16_t size);

/*
 * Move_Process20ms
 * 作用：
 * 每 20ms 调用一次，用来运行底盘运动逻辑。
 *
 * 当前会处理三种状态：
 * 1. 普通巡航
 * 2. 原地 IMU 转向
 * 3. 路口边走边转
 *
 * 同时会周期性把里程打包进回传队列。
 */
void Move_Process20ms(void);

/*
 * Move_UartTask
 * 作用：
 * 从发送队列里取出一帧反馈数据，通过上位机串口发出去。
 * 一次只发一帧，避免阻塞主循环太久。
 */
void Move_UartTask(void);

/*
 * Move_EmergencyStop
 * 作用：
 * 1. 取消当前运动命令
 * 2. 清 PID 输出
 * 3. 把左右轮目标速度清零
 * 4. 让主循环停止正常动力输出
 */
void Move_EmergencyStop(void);

/*
 * Move_ResetOdometry
 * 作用：
 * 清零左轮、右轮和整车累计里程。
 */
void Move_ResetOdometry(void);

/*
 * Move_ReportSensorUid
 * 作用：
 * 把外部传感器识别到的 UID 打包成反馈帧回给上位机。
 *
 * 参数：
 * uid
 *   32 位 UID 数值
 */
void Move_ReportSensorUid(const char *uid_str);

/*
 * 调试接口：读取 move 模块当前状态值。
 * 返回值对应 MoveStatusId。
 */
uint8_t Move_GetStatus(void);


#endif /* PATROL_ROBOT_MOVE_H */
