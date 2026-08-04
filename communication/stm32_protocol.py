"""
communication/stm32_protocol.py - STM32 通讯协议 V2.1

适配下位机 move.c/move.h 协议规范。

============================================================
一、帧格式 (下位机定义)
============================================================

  +--------+--------+--------+----------+----------+
  | 帧头0   | 帧头1   | 命令    | 长度      | 数据      | 校验      |
  | 1 Byte  | 1 Byte | 1 Byte | 1 Byte   | N Byte   | 1 Byte   |
  | 0xA5    | 0x5A   | CMD    | LEN      | Payload  | Checksum |
  +--------+--------+--------+----------+----------+

Len = Payload 长度 (仅数据部分，不含 CMD/LEN/Checksum)
总帧长 = 5 + Len

校验: 前面所有字节累加和的低8位 (不含校验字节本身)
      checksum = (HEAD0 + HEAD1 + CMD + LEN + Payload[0] + ... + Payload[N-1]) & 0xFF

============================================================
二、上位机 -> 下位机 指令集
============================================================

CMD_MOVE_VECTOR        0x01  速度指令
  【注意】下位机 move_3.c 只读取前2字节(vx)，忽略wz
  PAYLOAD = [vx_L][vx_H]  (仅2字节！)
  vx: 线速度 mm/s, int16, 小端
  数据长度: 2, 总帧长: 7

CMD_ACTION             0x02  离散动作
  PAYLOAD = [action]
  action: 0x01=急停, 0x02=里程清零, 0x03=开始UID扫描
  数据长度: 1, 总帧长: 6

CMD_TURN_IMU           0x03  原地IMU转向
  PAYLOAD = [angle_L][angle_H]
  angle: 转向角度(度), int16, 小端
         正数=左转, 负数=右转
  数据长度: 2, 总帧长: 7

CMD_SERVO              0x04  舵机控制
  PAYLOAD = [servo_id][angle] 或 [angle]
  数据长度: 1-4, 总帧长: 6-9

CMD_INTERSECTION_TURN  0x05  路口边走边转
  PAYLOAD = [distance_L][distance_H][direction]
  distance: 离路口距离 mm, int16, 小端 (下位机当前固定用80mm, 但协议预留)
  direction: 0x01=左转, 0x02=右转
  数据长度: 3, 总帧长: 8

============================================================
三、下位机 -> 上位机 反馈集
============================================================

FB_ODOMETRY  0x01  里程反馈
  PAYLOAD 版本A (当前下位机): [dist_L][dist_LH][dist_H][dist_HH]
    dist: uint32 mm, 小端
  PAYLOAD 版本B (扩展后): [dist: uint32][yaw: int16]
    yaw: int16, 单位 0.1度, 小端
  PAYLOAD 版本C (完整): [dist: uint32][yaw: int16][speed: int16]

FB_STATUS    0x02  状态上报
  PAYLOAD = [status]
  status: 0=空闲, 1=转向中, 2=转向完成, 3=障碍

FB_SENSOR    0x03  传感器数据 (RFID UID)
  PAYLOAD 格式待确认

============================================================
四、单位统一约定
============================================================

+------------------+----------+------------+--------------+
| 物理量           | 单位      | 数据类型    | 范围          |
+------------------+----------+------------+--------------+
| 线速度 (vx)      | mm/s     | int16      | -500 ~ 500   |
| 角速度 (wz)      | mrad/s   | int16      | -500 ~ 500   |
| 距离 (dist)      | mm       | uint32     | 0 ~ 50000    |
| 角度 (yaw)       | 0.1 deg  | int16      | -1800 ~ 1800 |
| 转向角度 (turn)  | deg      | int16      | -180 ~ 180   |
| 云台角度 (servo) | deg      | uint8      | 0 ~ 180      |
+------------------+----------+------------+--------------+

坐标系约定:
  Y+ = 前进方向(车头)  X+ = 右侧  Z+ = 下方(右手系)
  yaw=0 时车头朝向Y+, yaw>0 顺时针旋转(右转)
  注: 下位机正角度表示左转，与坐标系约定相反，编码时已做转换
"""

import struct
from enum import IntEnum
from typing import Optional, Tuple


# ============================================================
# 帧格式常量
# ============================================================
FRAME_HEAD_0 = 0xA5
FRAME_HEAD_1 = 0x5A


def _checksum(data: bytes) -> int:
    """累加和校验: 前面所有字节累加，取低8位"""
    return sum(data) & 0xFF


def _build_frame(cmd_id: int, payload: bytes) -> bytes:
    """
    构建完整数据帧

    格式: [A5][5A][CMD][LEN][Payload...][Checksum]
    """
    length = len(payload)
    frame = bytes([FRAME_HEAD_0, FRAME_HEAD_1, cmd_id, length]) + payload
    checksum = _checksum(frame)
    return frame + bytes([checksum])


def decode_frame(data: bytes) -> Optional[Tuple[int, bytes, int]]:
    """
    从字节流解析一帧。

    下位机帧格式: [A5][5A][CMD][LEN][Payload...][Checksum]

    Args:
        data: 原始字节流

    Returns:
        (cmd_id, payload_bytes, frame_total_length) 或 None
    """
    if len(data) < 5:
        return None

    # 找帧头 A5 5A
    idx = 0
    found = False
    while idx + 1 < len(data):
        if data[idx] == FRAME_HEAD_0 and data[idx + 1] == FRAME_HEAD_1:
            found = True
            break
        idx += 1

    if not found:
        return None

    # 丢弃帧头前的垃圾数据
    if idx > 0:
        data = data[idx:]

    if len(data) < 5:
        return None

    cmd_id = data[2]
    payload_len = data[3]

    if payload_len > 24:  # 下位机 MOVE_MAX_PAYLOAD_LEN = 24
        # 长度异常，尝试从下一个字节重新找帧头
        return None

    frame_total = 5 + payload_len
    if len(data) < frame_total:
        return None  # 帧不完整，等待更多数据

    payload = data[4:4 + payload_len]
    rx_checksum = data[frame_total - 1]

    calc = _checksum(data[:frame_total - 1])
    if calc != rx_checksum:
        return None  # 校验失败

    return (cmd_id, payload, frame_total)


# ============================================================
# 上位机 -> 下位机 指令 (CMD_ID)
# ============================================================
class CmdID(IntEnum):
    """上位机->下位机指令类型 (与下位机 move.h 保持一致)"""
    MOVE_VECTOR = 0x01       # 速度指令: [vx: int16 mm/s, wz: int16 mrad/s]
    ACTION = 0x02            # 离散动作: [action: uint8]
    TURN_IMU = 0x03          # 原地IMU转向: [angle: int16 deg]
    SERVO = 0x04             # 舵机控制
    INTERSECTION_TURN = 0x05  # 路口边走边转: [distance: int16 mm, direction: uint8]
    LANE_OFFSET = 0x06       # 车道横向偏移: [offset: int16 mm], 负=偏左需右修


class ActionCode(IntEnum):
    """离散动作码 (与下位机 MoveActionId 保持一致)"""
    STOP = 0x01          # 急停
    RESET_ODOM = 0x02    # 里程清零
    START_UID_SCAN = 0x03  # 开始UID扫描


class TurnDirection(IntEnum):
    """路口转向方向"""
    LEFT = 0x01   # 左转
    RIGHT = 0x02  # 右转

# ============================================================
# 下位机 -> 上位机 反馈 (FB_ID)
# ============================================================
class FbID(IntEnum):
    """下位机->上位机反馈类型 (与下位机 MoveFeedbackId 保持一致)"""
    ODOMETRY = 0x01   # 里程反馈
    STATUS = 0x02     # 状态上报
    SENSOR = 0x03     # 传感器数据


class StmStatusCode(IntEnum):
    """下位机状态码 (与下位机 MoveStatusId 保持一致)"""
    IDLE = 0
    TURNING = 1
    TURN_DONE = 2
    OBSTACLE = 3


# ============================================================
# 编码器 (上位机 -> 下位机)
# ============================================================

def encode_velocity(vx_mms: float, wz_mrads: float = 0) -> bytes:
    """
    速度指令 — 主巡航指令 CMD_MOVE_VECTOR (0x01)

    【重要】下位机 move_3.c 只读取前2字节(vx)，不读取wz。
    如果发送4字节会导致下位机帧解析错位！
    因此只发送2字节：[vx_L][vx_H]

    Args:
        vx_mms: 前进速度 mm/s (正=前进, -500~500)
        wz_mrads: 【被忽略】下位机不接收角速度，角速度由下位机视觉纠偏计算
    """
    vx = max(-500, min(500, int(vx_mms)))
    # 只发送vx（2字节），不发送wz！
    return _build_frame(CmdID.MOVE_VECTOR, struct.pack("<h", vx))


def encode_action(code: ActionCode) -> bytes:
    """
    离散动作 CMD_ACTION (0x02)

    Args:
        code: ActionCode.STOP / RESET_ODOM / START_UID_SCAN
    """
    return _build_frame(CmdID.ACTION, bytes([code]))


def encode_turn(angle_deg: float) -> bytes:
    """
    原地IMU转向 CMD_TURN_IMU (0x03)

    Args:
        angle_deg: 目标转向角度 (度)
                   正数=左转, 负数=右转
                   90=左转90度, -90=右转90度, 180/-180=掉头

    注意: 下位机正角度表示左转，与坐标系约定相反。
          这里直接发送角度值，不再乘以10。
    """
    a = max(-180, min(180, int(angle_deg)))
    return _build_frame(CmdID.TURN_IMU, struct.pack("<h", a))


def encode_servo(angle_deg: int, servo_id: int = 0, move_time_ms: int = 300) -> bytes:
    """
    舵机控制 CMD_SERVO (0x04)

    Args:
        angle_deg: 0-180度 (90=水平向前)
        servo_id: 舵机ID (0或1)
        move_time_ms: 动作时间 ms
    """
    a = max(0, min(180, int(angle_deg)))
    if servo_id == 0:
        # 简化格式: [angle]
        return _build_frame(CmdID.SERVO, bytes([a]))
    else:
        # 完整格式: [servo_id][angle][time_L][time_H]
        return _build_frame(CmdID.SERVO, struct.pack("<BBH", servo_id, a, move_time_ms))


def encode_intersection_turn(direction: str, distance_mm: int = 150) -> bytes:
    """
    路口边走边转 CMD_INTERSECTION_TURN (0x05)

    Args:
        direction: "left" 或 "right"
        distance_mm: 离路口距离 mm（保留参数，但下位机 move_3.c 实际只看第1字节方向，
                     距离由下位机固定用80mm阈值判断，不读取此参数）

    下位机行为 (move_3.c:454-486):
        1. 记录当前里程和方向 (只读 payload[0])
        2. 继续前进直到达到触发距离 (80mm)
        3. 开始边走边转90度
        4. 完成后自动恢复直行

    【重要】下位机 Move_HandleIntersectionTurn 只读取 payload[0] 作为方向，
           不读取 distance 字段。因此 payload 只发 1 字节方向。
    """
    if direction == "left":
        dir_code = TurnDirection.LEFT
    elif direction == "right":
        dir_code = TurnDirection.RIGHT
    else:
        raise ValueError(f"方向必须是 'left' 或 'right', 得到: {direction}")
    # 下位机实际只读 payload[0] 作为方向，只发 1 字节
    payload = bytes([dir_code])
    return _build_frame(CmdID.INTERSECTION_TURN, payload)


def encode_lane_offset(offset_mm: float) -> bytes:
    """
    车道横向偏移 CMD_LANE_OFFSET (0x06)

    Payload: [offset: int16 mm]
      正值 = 车偏右, 需左修
      负值 = 车偏左, 需右修
    限幅 ±200mm 防止异常值。

    下位机 move_3.c 接收 CMD_VISION_ERROR (0x06)，
    用 kp=5.0 计算 vision_correction，通过左右轮差速实现纠偏。
    """
    off = max(-200, min(200, int(offset_mm)))
    return _build_frame(CmdID.LANE_OFFSET, struct.pack("<h", off))


# ============================================================
# 解码器 (下位机 -> 上位机)
# ============================================================

def decode_odom(payload: bytes) -> Optional[Tuple[int, float, int]]:
    """
    解析里程反馈 FB_ODOMETRY (0x01)

    兼容三种下位机版本:
        - 版本A (4字节): [dist: uint32] -> 返回 (dist, 0.0, 0)
        - 版本B (6字节): [dist: uint32][yaw: int16] -> 返回 (dist, yaw_deg, 0)
        - 版本C (8字节): [dist: uint32][yaw: int16][speed: int16] -> 返回 (dist, yaw_deg, speed)

    Returns:
        (dist_mm, yaw_deg, speed_mms) 或 None
        dist:  累计行驶距离 mm
        yaw:   当前航向角 deg (下位机扩展后)
        speed: 当前速度 mm/s (下位机扩展后)
    """
    if len(payload) >= 8:
        dist, yaw_01deg, speed = struct.unpack("<Ihh", payload[:8])
        return (dist, yaw_01deg / 10.0, speed)
    elif len(payload) >= 6:
        dist = struct.unpack("<I", payload[:4])[0]
        yaw_01deg = struct.unpack("<h", payload[4:6])[0]
        return (dist, yaw_01deg / 10.0, 0)
    elif len(payload) >= 4:
        dist = struct.unpack("<I", payload[:4])[0]
        return (dist, 0.0, 0)
    return None


def decode_status(payload: bytes) -> Optional[int]:
    """解析状态码 FB_STATUS (0x02)"""
    if len(payload) < 1:
        return None
    return payload[0]


def decode_sensor_uid(payload: bytes) -> Optional[str]:
    """
    解析传感器数据 FB_SENSOR (0x03)

    当前下位机实现待确认，这里先按字符串格式解析。
    如果下位机回传的是数字UID，需要调整。
    """
    if len(payload) < 1:
        return None
    uid_len = payload[0]
    if len(payload) < 1 + uid_len:
        return None
    try:
        return payload[1:1 + uid_len].decode("ascii").strip()
    except (UnicodeDecodeError, AttributeError):
        return None


# ============================================================
# 调试辅助
# ============================================================

def frame_to_hex(frame: bytes) -> str:
    """将帧转为十六进制字符串，便于调试"""
    return " ".join(f"{b:02X}" for b in frame)


def decode_velocity_cmd(payload: bytes) -> Optional[Tuple[int, Optional[int]]]:
    """解析速度指令 (调试用)

    下位机 move_3.c 只读取 vx（前2字节），忽略 wz。
    兼容旧版4字节格式 [vx][wz] 和 新版2字节格式 [vx]。
    """
    if len(payload) < 2:
        return None
    vx = struct.unpack("<h", payload[:2])[0]
    wz = None
    if len(payload) >= 4:
        wz = struct.unpack("<h", payload[2:4])[0]
    return (vx, wz)


def decode_turn_cmd(payload: bytes) -> Optional[int]:
    """解析转向指令 (调试用)"""
    if len(payload) < 2:
        return None
    return struct.unpack("<h", payload[:2])[0]
