"""
contracts/commands.py — 导航层命令/结果/边任务数据结构 + 接口协议
"""
from dataclasses import dataclass, field
from typing import List, Optional, Protocol

from .states import AgentState, EdgeTaskStatus, TurnAction, CulvertType
from .events import Pose, OdomUpdate, RoadCondition, CrossroadEvent, CulvertEvent, ObstacleEvent, RfidEvent

NavigationPose = Pose


@dataclass(frozen=True)
class Goal:
    goal_id: str
    goal_type: str
    node_name: str = ""
    edge_id: int | None = None
    reward: float = 0.0


@dataclass(frozen=True)
class RuntimeMapView:
    visited_nodes: frozenset[str] = frozenset()
    blocked_edges: frozenset[int] = frozenset()
    discovered_culverts: frozenset[int] = frozenset()
    recon_culverts: frozenset[int] = frozenset()
    culvert_targets: frozenset[int] = frozenset()
    culvert_endpoints: tuple[tuple[int, str | None], ...] = ()
    version: int = 0


RuntimeMapSnapshot = RuntimeMapView


@dataclass(frozen=True)
class MapUpdateIntent:
    kind: str
    edge_id: int | None = None
    node_name: str = ""
    value: object = None
    timestamp: float = 0.0


@dataclass(frozen=True)
class GoalContext:
    goal_type: str
    task_id: str = ""
    safe_anchor: str = ""


@dataclass(frozen=True)
class RouteQuery:
    pose: NavigationPose
    candidates: tuple[Goal, ...]
    map_view: RuntimeMapView
    policy: object
    start_node: str = "START"


@dataclass(frozen=True)
class CandidateRoute:
    goal: Goal
    reachable: bool
    steps: tuple["DirectedStep", ...] = ()
    route_cost: float = float("inf")
    distance_mm: float = 0.0
    turn_cost: float = 0.0
    failure_reason: str = ""


@dataclass(frozen=True)
class NavigationPlan:
    goal: Goal
    route: CandidateRoute
    goal_context: GoalContext = field(default_factory=lambda: GoalContext("explore"))
    map_version: int = 0


@dataclass(frozen=True)
class ActionCommand:
    action_id: str
    kind: str
    parameters: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True)
class ActionPlan:
    plan_id: str
    actions: tuple[ActionCommand, ...]
    goal_context: GoalContext
    map_version: int = 0


@dataclass(frozen=True)
class ActionFeedback:
    action_id: str
    status: str
    distance_mm: float = 0.0
    timestamp: float = 0.0
    reason: str = ""


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
class DirectedStep:
    """
    带朝向的有向行驶步骤（Task 12 / 线图产物）。

    一条「从 from_node 沿 heading 方向驶向 to_node」的有向边，
    heading 来自规划（edge 物理方向），而非事后由坐标反推。
    """
    from_node: str
    to_node: str
    heading: float                    # 该有向边的行驶朝向 (世界 yaw_deg)
    distance_mm: float = 0.0
    is_tunnel: bool = False


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
    discovered_culverts: List[int] = field(default_factory=list)  # 已发现（视觉/侧视）
    recon_culverts: List[int] = field(default_factory=list)        # 已侦查（真正驶过）


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
