"""
contracts/events.py — 导航层事件与位姿/里程计/道路状况数据结构
"""
from dataclasses import dataclass

from .states import CulvertType


@dataclass
class Pose:
    """小车位姿（全局坐标系）"""
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0        # 0°=Y+方向, 90°=X+方向


@dataclass
class OdomUpdate:
    """里程计增量（车体坐标系）"""
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    dyaw_deg: float = 0.0
    timestamp: float = 0.0


@dataclass
class RoadCondition:
    """
    当前车道状态 — perception 每帧上报给 navigation

    注意：数据由 perception 守护进程产生，
    navigation 被动接收但不作每帧响应。
    lane_offset 的 PID 闭环在 perception 层独立运行。
    """
    offset_mm: float = 0.0           # 横向偏差 (mm)，负=偏左
    quality_score: float = 1.0       # 帧质量 0~1
    lane_angle_rad: float = 0.0      # 车道方向角 (rad)
    is_intersection: bool = False    # IPM 路口检测结果
    distance_to_crossroad_mm: float = -1.0  # 到路口距离
    duty_cycle: float = 0.0          # 路口检测占空比
    timestamp: float = 0.0


@dataclass
class CrossroadEvent:
    """
    路口检测事件 — perception → navigation 中断上报
    """
    distance_mm: float                # 到路口横向线距离
    duty_cycle: float                 # 检测占空比
    confidence: float = 1.0           # 置信度
    timestamp: float = 0.0


@dataclass
class CulvertEvent:
    """
    涵洞检测事件 — navigation 内部产生
    （navigation 调用 vision 工具后自行构造此事件）
    """
    culvert_type: CulvertType
    local_x_mm: float                 # 车体坐标系下 X
    local_y_mm: float                 # 车体坐标系下 Y
    confidence: float = 1.0
    timestamp: float = 0.0


@dataclass
class ObstacleEvent:
    """
    障碍物检测事件 — navigation 内部产生
    """
    distance_mm: float                # 车体到障碍物距离
    confidence: float = 1.0
    timestamp: float = 0.0


@dataclass
class RfidEvent:
    """
    RFID 打卡事件 — STM32 → navigation
    """
    uid: str
    node_name: str = ""
    timestamp: float = 0.0
