"""定义视觉层向导航感知适配器交付的相对观察数据契约。"""

# 导入不可变数据类装饰器，保证同一帧观察在适配期间保持稳定。
from dataclasses import dataclass
# 导入枚举基类，限制感知翻译结果只能处于已确认或无法确认状态。
from enum import Enum
# 导入 Python 3.8 兼容的可选值和元组类型注解。
from typing import Optional, Tuple

# 导入绝对地图更新类型，保证适配器输出不会重新发明地图数据结构。


class EdgePassability(Enum):
    """一次观察对道路通行性的结论。"""

    UNKNOWN = "unknown"
    CLEAR = "clear"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class CoverageInterval:
    """涵洞可见区间，使用道路起点到终点的归一化比例表示。"""

    start_ratio: float
    end_ratio: float

    def __post_init__(self) -> None:
        if not 0.0 <= self.start_ratio <= self.end_ratio <= 1.0:
            raise ValueError("涵洞覆盖区间必须位于 [0.0, 1.0] 且起点不晚于终点")


@dataclass(frozen=True)
class EdgeObservation:
    """感知适配器完成绝对映射后的单边独立观察事实。"""

    edge_id: str
    passability: EdgePassability
    culvert_found: bool = False
    no_culvert_coverage: Tuple[CoverageInterval, ...] = ()
    observation_scope: str = "observation_zone"
    obstacle_distance_mm: Optional[float] = None
    culvert_distance_mm: Optional[float] = None

    def __post_init__(self) -> None:
        if not self.edge_id:
            raise ValueError("edge_id 不能为空")
        object.__setattr__(self, "no_culvert_coverage", tuple(self.no_culvert_coverage))
        if any(not isinstance(item, CoverageInterval) for item in self.no_culvert_coverage):
            raise TypeError("no_culvert_coverage 必须全部是 CoverageInterval")
        if self.observation_scope not in ("junction_full", "observation_zone"):
            raise ValueError("observation_scope 必须是 junction_full 或 observation_zone")
        if self.obstacle_distance_mm is not None and self.obstacle_distance_mm < 0:
            raise ValueError("obstacle_distance_mm 不能为负数")
        if self.culvert_distance_mm is not None and self.culvert_distance_mm < 0:
            raise ValueError("culvert_distance_mm 不能为负数")


class PerceptionOutcome(Enum):
    """感知适配器对一帧相对事实能否安全翻译的结论。"""

    # 适配器已经完成绝对映射，可以由协调器消费其更新和校正。
    CONFIRMED = "confirmed"
    # 适配器无法安全确定绝对含义，只能携带原因并由协调器继续流程。
    INCONCLUSIVE = "inconclusive"


@dataclass(frozen=True)
class RoadFeatures:
    """单帧中独立于目标检测的道路分支事实，供感知适配器解释相对位置。"""

    # 车头正前方是否检测到可继续通行的支路。
    has_forward_branch: bool
    # 车头左侧是否检测到支路。
    has_left_branch: bool
    # 车头右侧是否检测到支路。
    has_right_branch: bool
    # 上述道路结构判断的整体置信度，具体阈值由感知层约定。
    confidence: float


@dataclass(frozen=True)
class TargetDetection:
    """单帧中的一个有效目标，仍是相对车体事实，不含地图边标识。"""

    # 目标语义类型，例如 `CULVERT` 或 `OBSTACLE`。
    kind: str
    # 目标相对车体或画面的区域，例如 `LEFT`、`RIGHT` 或 `FORWARD`。
    relative_region: str
    # 当前检测的模型置信度。
    confidence: float
    # 感知层是否已通过自身质量门槛确认该检测可用。
    valid: bool


@dataclass(frozen=True)
class PerceptionFrame:
    """一次观察动作返回的完整相对视觉事实。

    谁调用：真实或虚拟感知系统通过执行器上报。
    谁响应：`NavigationRuntime` 将帧交给 `PerceptionAdapter`。
    输入输出：输入为帧时间、道路特征和目标列表；输出由适配器转换为地图更新或位置校正。
    状态影响：本对象不直接修改运行时地图。
    """

    # 帧唯一标识，用于关联观察完成中断和实际感知帧。
    frame_id: str
    # 图像或观察产生的运行环境时间。
    timestamp: float
    # 与目标列表分离传输的道路结构事实。
    road_features: RoadFeatures
    # 本帧全部有效目标，空元组表示未发现目标。
    targets: Tuple[TargetDetection, ...]
    # 调试时可选关联的图像或日志引用，不参与导航决策。
    debug_attachment: Optional[str] = None


@dataclass(frozen=True)
class PositionCorrection:
    """视觉或 IPM 对当前巡航边剩余距离的绝对位置校正。"""

    # 本次位置校正的稳定标识，供调试和重复帧排查使用。
    correction_id: str
    # 产生校正的观察帧标识，便于追溯校正来源。
    source_observation_id: str
    # 校正所针对的有向巡航边标识。
    traversal_id: str
    # 从当前位置到目标路口中心的视觉估计剩余距离，单位毫米。
    remaining_to_node_mm: float
    # 校正产生的运行环境时间。
    timestamp: float
    # 视觉校正置信度，范围为 0 到 1。
    confidence: float

    def __post_init__(self) -> None:
        """拒绝无法用于位置投影的负距离和非法置信度。"""

        # 剩余距离不能为负，否则会把机器人投影到目标路口之外。
        if self.remaining_to_node_mm < 0:
            raise ValueError("remaining_to_node_mm 不能为负数")
        # 置信度采用统一闭区间，避免下游误解异常数值。
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence 必须位于 0 到 1 之间")


@dataclass(frozen=True)
class PerceptionTranslation:
    """感知适配器交给协调器的一帧翻译结果。"""

    # 必须对应输入感知帧的唯一标识。
    frame_id: str
    # 本帧翻译是否已经足够可靠。
    outcome: PerceptionOutcome
    # 已完成绝对映射的道路观察事实，由 EventProjector 转成地图更新。
    edge_observations: Tuple[EdgeObservation, ...] = ()
    # 可选的视觉位置校正，由协调器交给位置投影器。
    position_correction: Optional[PositionCorrection] = None
    # 无法翻译时的诊断原因；已确认结果必须为空。
    reason: Optional[str] = None

    def __post_init__(self) -> None:
        """严格校验结果状态与载荷组合，避免下游猜测字段含义。"""

        # 空帧标识无法关联观察完成中断，必须在契约边界拒绝。
        if not self.frame_id:
            raise ValueError("frame_id 不能为空")
        # 即使调用方传入列表，也立即冻结成元组，避免结果在异步处理中被修改。
        observations = tuple(self.edge_observations)
        object.__setattr__(self, "edge_observations", observations)
        if any(not isinstance(observation, EdgeObservation) for observation in observations):
            raise TypeError("edge_observations 必须全部是 EdgeObservation")
        # 已确认结果表示翻译完成，不应同时携带失败原因。
        if self.outcome is PerceptionOutcome.CONFIRMED:
            if self.reason is not None:
                raise ValueError("CONFIRMED 结果的 reason 必须为空")
            return
        # 无法确认时必须给出可读原因，供协调器记录诊断。
        if self.outcome is PerceptionOutcome.INCONCLUSIVE:
            if self.reason is None or not self.reason.strip():
                raise ValueError("INCONCLUSIVE 结果必须提供 reason")
            # 不确定时不能伪造任何绝对地图事实。
            if observations:
                raise ValueError("INCONCLUSIVE 结果不能携带 edge_observations")
            # 不确定时也不能把不可靠位置送入状态投影器。
            if self.position_correction is not None:
                raise ValueError("INCONCLUSIVE 结果不能携带 position_correction")
            return
        # 枚举扩展但未同步字段规则时立即失败，防止默认放行新状态。
        raise ValueError("不支持的感知翻译结果")
