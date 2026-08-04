"""
通讯层模块 (Communication Layer)

负责与下位机(STM32)的数据交互:
- stm32_protocol: STM32 通讯协议 V2.1 (适配下位机 move.c/move.h)
- robot_bridge: 机器人主桥接层 (50Hz闭环、OdomIntegrator、Agent集成)

设计原则:
1. 串口通讯不阻塞主线程
2. 协议封装统一入口
3. 支持指令发送和反馈接收
4. 级联控制: 外环(视觉30Hz) -> 内环(STM32 PID 100Hz)

帧格式 (与下位机一致):
    [A5][5A][CMD][LEN][Payload...][Checksum]
    校验: 累加和低8位
"""

from .stm32_protocol import (
    FRAME_HEAD_0, FRAME_HEAD_1,
    CmdID, ActionCode, TurnDirection,
    FbID, StmStatusCode,
    _build_frame, decode_frame,
    encode_velocity, encode_action, encode_turn,
    encode_servo, encode_intersection_turn,
    encode_lane_offset,
    decode_odom, decode_status, decode_sensor_uid,
    frame_to_hex, decode_velocity_cmd, decode_turn_cmd,
)
from .robot_bridge import OdomIntegrator, RobotBridge

__all__ = [
    # 帧格式
    'FRAME_HEAD_0', 'FRAME_HEAD_1',
    # 协议枚举
    'CmdID', 'ActionCode', 'TurnDirection',
    'FbID', 'StmStatusCode',
    # 帧操作
    '_build_frame', 'decode_frame',
    # 编码器
    'encode_velocity', 'encode_action', 'encode_turn',
    'encode_servo', 'encode_intersection_turn',
    'encode_lane_offset',
    # 解码器
    'decode_odom', 'decode_status', 'decode_sensor_uid',
    # 调试辅助
    'frame_to_hex', 'decode_velocity_cmd', 'decode_turn_cmd',
    # 桥接层
    'OdomIntegrator', 'RobotBridge',
]
