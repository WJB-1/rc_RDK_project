"""
navigation/contracts.py — 导航层数据结构与接口契约

本模块定义导航领域的所有数据类型和抽象接口，
不包含任何具体实现逻辑。
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple, Protocol, Dict, Any
from datetime import datetime


# ================================================================
# 枚举
# ================================================================

class AgentState(Enum):
    """Agent 状态枚举 — 与 state_machine.py 同步"""
    IDLE = auto()
    GLOBAL_PLANNING = auto()
    EDGE_EXECUTING = auto()
    APPROACHING = auto()
    TURNING = auto()
    NODE_ARRIVAL = auto()
    CULVERT_RECON = auto()
    OBSTACLE_STOP = auto()
    BACKTRACK = auto()
    FINISHED = auto()


class EdgeTaskStatus(Enum):
    """边级任务状态"""
    PENDING = "pending"
    EXECUTING = "executing"
    DONE = "done"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


class TurnAction(Enum):
    """转向动作枚举"""
    STRAIGHT = "straight"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    UTURN = "uturn"
    STOP = "stop"


class CulvertType(Enum):
    """涵洞检测类型"""
    FRONT = "front"      # 正前方涵洞入口
    SIDE = "side"        # 侧面涵洞（路口/拐角处）


# ================================================================
# 数据结构
# ================================================================

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
class EdgeTask:
    """边级任务 — 全局路径规划的最小执行单元"""
    edge_id: int
    from_node: str
    to_node: str
    expected_yaw: float              # 期望航向 (deg)
    distance_mm: float               # 边长度
    is_tunnel: bool = False
    speed_limit_ms: float = 0.3      # 限速 m/s
    status: EdgeTaskStatus = EdgeTaskStatus.PENDING


@dataclass
class MapEdgeDynamic:
    """
    地图边的运行时动态属性

    与 map_topology.MapEdge（静态属性）互补。
    导航层读写，vision 层只读。
    """
    edge_id: int
    is_blocked: bool = False          # 是否被障碍物封锁
    has_culvert: bool = False         # 是否发现涵洞
    visit_count: int = 0              # 经过次数（用于路径加权）
    last_update: float = 0.0          # 最后更新时间戳


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


@dataclass
class TurnCommand:
    """
    转向指令 — navigation → 下位机
    """
    action: TurnAction
    target_node: str
    expected_yaw: float
    speed_ms: float = 0.3


@dataclass
class NavigationState:
    """
    navigation 全量状态 — 供 debug / web 面板查询

    这是一个快照，不参与内部逻辑。
    """
    agent_state: AgentState = AgentState.IDLE
    pose: Pose = field(default_factory=Pose)
    current_node: str = ""
    target_node: str = ""
    visited_nodes: List[str] = field(default_factory=list)
    edge_sequence: List[str] = field(default_factory=list)  # 节点名序列
    current_edge_id: int = -1
    blocked_edges: List[int] = field(default_factory=list)
    found_culverts: List[int] = field(default_factory=list)


@dataclass
class CulvertReconResult:
    """涵洞侦查结果"""
    culvert_type: CulvertType
    face_detected: bool = False       # 是否检测到人脸
    face_id: str = ""                 # 人脸识别结果
    ocr_text: str = ""               # OCR 识别文本
    timestamp: float = 0.0


# ================================================================
# 接口契约 (Protocol)
# ================================================================

class VisionTools(Protocol):
    """
    vision/ 工具库契约 — navigation 通过此接口调用视觉能力。

    所有方法都是无状态的纯函数式调用。
    navigation 传入数据和上下文，vision 返回结构化结果。
    vision 不持有状态机引用，不主动回调。
    """

    def detect_crossroad(self, frame) -> Optional[CrossroadEvent]:
        """
        YOLO 路口检测。
        :param frame: BGR 图像 (H, W, 3)
        :return: None 或 CrossroadEvent
        """
        ...

    def detect_culvert(self, frame,
                       is_tunnel: bool = False) -> Optional[CulvertEvent]:
        """
        涵洞检测。
        :param frame: BGR 图像
        :param is_tunnel: 当前边是否为隧道（由 navigation 传入，用于区分隧道 vs 涵洞）
        :return: None 或 CulvertEvent
        """
        ...

    def detect_obstacle(self, frame) -> Optional[ObstacleEvent]:
        """
        前向障碍物检测。
        :param frame: BGR 图像
        :return: None 或 ObstacleEvent
        """
        ...


class NavigationAgent(Protocol):
    """
    navigation/ 大脑契约 — 外部模块（main.py, perception）通过此接口与导航层交互。

    这是整个系统唯一的决策入口。
    """

    # ---- 生命周期 ----
    def start(self) -> None: ...
    def tick(self) -> Optional[TurnCommand]: ...

    # ---- 事件输入 (passive) ----
    def on_odom_update(self, odom: OdomUpdate) -> None: ...
    def on_road_condition(self, rc: RoadCondition) -> None: ...
    def on_crossroad_detected(self, event: CrossroadEvent) -> None: ...
    def on_rfid_scanned(self, event: RfidEvent) -> None: ...

    # ---- 状态查询 (for debug/web) ----
    def get_state(self) -> NavigationState: ...

    # ---- 感知工具注入 ----
    def set_vision_tools(self, tools: VisionTools) -> None: ...
