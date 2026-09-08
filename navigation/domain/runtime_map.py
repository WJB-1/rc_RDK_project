"""定义动态地图事实、来源权限、幂等写入和不可变读取快照。"""

# 导入不可变数据类装饰器，保证更新包和读取快照不能被调用方原地修改。
from dataclasses import dataclass
# 导入枚举基类，限制地图事实类型和写入权限来源。
from enum import Enum
# 导入 Python 3.8 兼容的集合与可选值类型注解。
from typing import FrozenSet, Optional, Set, Tuple

from navigation.contracts.perception import CoverageInterval


class MapObservationScope(Enum):
    """描述观察是否覆盖整条物理边的证据范围。"""

    # 路口中心的观察可以完整看清下一条边，允许确认整条边无障碍。
    JUNCTION_FULL = "junction_full"
    # 观察区只能看到边的一部分，不足以证明整条边无障碍。
    OBSERVATION_ZONE = "observation_zone"


class EdgeKnowledgeStatus(Enum):
    """运行时地图对单条物理边的障碍状态。"""

    # 尚未获得足以判断整条边的证据。
    UNKNOWN = "unknown"
    # 已由路口完整观察确认当前没有障碍。
    CLEAR = "clear"
    # 已检测到障碍物，确定不可通行。
    BLOCKED = "blocked"


class CulvertKnowledgeStatus(Enum):
    """运行时地图对单条边涵洞存在性的证据状态。"""

    UNKNOWN = "unknown"
    DISCOVERED = "discovered"
    CONFIRMED_ABSENT = "confirmed_absent"


class MapUpdateAuthority(Enum):
    """绝对地图事实的业务授权来源，防止外部模块越权宣告任务完成。"""

    # 感知适配器只能提交视觉直接可证实的道路阻塞、恢复和涵洞发现事实。
    PERCEPTION_ADAPTER = "perception_adapter"
    # 协调器只能在校验任务或执行中断后提交完成、打卡和阻塞派生事实。
    COORDINATOR = "coordinator"


class AbsoluteMapUpdateKind(Enum):
    """写入 `RuntimeMap` 的绝对动态事实类型。"""

    # 将一条物理道路标记为当前不可通行。
    BLOCK_EDGE = "block_edge"
    # 由一次可靠观察确认道路当前没有障碍。
    CONFIRM_EDGE_CLEAR = "confirm_edge_clear"
    # 确认某条道路上存在待侦查涵洞。
    DISCOVER_CULVERT = "discover_culvert"
    CONFIRM_NO_CULVERT = "confirm_no_culvert"
    # 确认某个已发现涵洞已经由匹配任务成功中断确认侦查完成。
    RECON_CULVERT = "recon_culvert"
    # 确认机器人已由匹配打卡任务成功中断确认访问节点。
    VISIT_NODE = "visit_node"


@dataclass(frozen=True)
class AbsoluteMapUpdate:
    """已转换为绝对标识的动态地图事实，`RuntimeMap` 是唯一写入者。

    谁调用：感知适配器提交视觉事实，协调器提交已校验的任务或执行结果。
    谁响应：`RuntimeMap.apply()` 先验证 `authority`，再按事实类型更新快照。
    输入输出：输入为类型、授权来源和受影响边或节点；输出为地图是否发生有效变化。
    状态影响：本对象自身不改状态，只有被地图接受后才可能推进地图版本。
    """

    # 要写入的动态事实类型。
    kind: AbsoluteMapUpdateKind
    # 声明该事实由哪类业务边界产生，地图据此拒绝越权组合。
    authority: MapUpdateAuthority
    # 受影响物理边，节点访问事实时为空。
    edge_id: Optional[str] = None
    # 受影响路口节点，边事实时为空。
    node_id: Optional[str] = None
    # 该事实被确认的运行环境时间。
    timestamp: float = 0.0
    # 产生道路观察事实的范围；非观察类更新保持为空。
    observation_scope: Optional[MapObservationScope] = None
    coverage_intervals: Tuple[CoverageInterval, ...] = ()
    # 视觉估计的涵洞距离，未来可用于亚边级定位；非涵洞更新保持为空。
    culvert_distance_mm: Optional[float] = None

    def __post_init__(self) -> None:
        """校验可选距离，避免把非法视觉测距写入地图事实。"""

        if self.culvert_distance_mm is not None and self.culvert_distance_mm < 0:
            raise ValueError("culvert_distance_mm 不能为负数")


@dataclass(frozen=True)
class RuntimeMapSnapshot:
    """运行时地图的不可变读取视图，供规划、面板和测试安全消费。"""

    # 每次有效动态事实变化后递增的地图版本。
    version: int
    # 当前已知不可通行的物理边集合。
    blocked_edge_ids: FrozenSet[str]
    # 已发现但未必已经侦查的涵洞边集合。
    discovered_culvert_edge_ids: FrozenSet[str]
    # 已由协调器确认完成侦查的涵洞边集合。
    recon_culvert_edge_ids: FrozenSet[str]
    # 已由协调器确认到达或打卡的节点集合。
    visited_node_ids: FrozenSet[str]
    # 已由可靠观察确认当前可通行的物理边集合。
    confirmed_clear_edge_ids: FrozenSet[str] = frozenset()
    confirmed_no_culvert_edge_ids: FrozenSet[str] = frozenset()
    culvert_coverage_by_edge: Tuple[Tuple[str, Tuple[CoverageInterval, ...]], ...] = ()
    culvert_distance_by_edge: Tuple[Tuple[str, float], ...] = ()

    def edge_status(self, edge_id: str) -> EdgeKnowledgeStatus:
        """返回单条物理边当前的确定/不确定状态。"""

        if edge_id in self.blocked_edge_ids:
            return EdgeKnowledgeStatus.BLOCKED
        if edge_id in self.confirmed_clear_edge_ids:
            return EdgeKnowledgeStatus.CLEAR
        return EdgeKnowledgeStatus.UNKNOWN

    def culvert_status(self, edge_id: str) -> CulvertKnowledgeStatus:
        """返回单条边当前涵洞证据状态。"""
        if edge_id in self.discovered_culvert_edge_ids:
            return CulvertKnowledgeStatus.DISCOVERED
        if edge_id in self.confirmed_no_culvert_edge_ids:
            return CulvertKnowledgeStatus.CONFIRMED_ABSENT
        return CulvertKnowledgeStatus.UNKNOWN

    def culvert_coverage(self, edge_id: str) -> Tuple[CoverageInterval, ...]:
        """返回指定边已合并的涵洞可见区间。"""
        for known_edge_id, intervals in self.culvert_coverage_by_edge:
            if known_edge_id == edge_id:
                return intervals
        return ()

    def culvert_distance_mm(self, edge_id: str) -> Optional[float]:
        """返回指定边最近一次视觉估计的涵洞距离。"""

        for known_edge_id, distance_mm in self.culvert_distance_by_edge:
            if known_edge_id == edge_id:
                return distance_mm
        return None


class RuntimeMap:
    """动态地图事实与版本的唯一所有者。

    谁调用：`NavigationRuntime` 收到感知适配结果或协调器派生事实后调用。
    谁响应：本类以幂等方式写入允许的事实，并向规划器和面板提供快照。
    输入输出：`apply()` 接收 `AbsoluteMapUpdate` 并返回是否变化；`snapshot()` 返回不可变视图。
    状态影响：只有有效新事实会推进版本；越权或缺少前置事实的更新会被拒绝。
    """

    # 感知适配器只能写入视觉可直接证实的道路和涵洞发现事实。
    _PERCEPTION_ALLOWED_KINDS = frozenset(
        (
            AbsoluteMapUpdateKind.BLOCK_EDGE,
            AbsoluteMapUpdateKind.CONFIRM_EDGE_CLEAR,
            AbsoluteMapUpdateKind.DISCOVER_CULVERT,
            AbsoluteMapUpdateKind.CONFIRM_NO_CULVERT,
        )
    )
    # 协调器只能在校验完成中断后写入任务完成、打卡或运动阻塞派生事实。
    _COORDINATOR_ALLOWED_KINDS = frozenset(
        (
            AbsoluteMapUpdateKind.BLOCK_EDGE,
            AbsoluteMapUpdateKind.RECON_CULVERT,
            AbsoluteMapUpdateKind.VISIT_NODE,
        )
    )

    def __init__(self) -> None:
        """创建没有动态事实、地图版本为零的运行时地图。

        谁调用：应用入口、仿真装配或测试。
        谁响应：本类初始化内部集合，后续只通过 `apply()` 改变它们。
        输入输出：不接收参数，不返回值。
        状态影响：创建独立的新地图，不影响其他地图实例。
        """

        # 保存每次有效事实变化后的单调递增版本号。
        self._version = 0
        # 保存当前不可通行的物理边标识。
        self._blocked_edge_ids: Set[str] = set()
        # 保存已经由可靠观察确认没有障碍的物理边标识。
        self._confirmed_clear_edge_ids: Set[str] = set()
        # 保存已经由视觉确认存在涵洞的物理边标识。
        self._discovered_culvert_edge_ids: Set[str] = set()
        # 保存已经由协调器确认侦查完成的涵洞边标识。
        self._recon_culvert_edge_ids: Set[str] = set()
        # 保存已经由协调器确认到达或打卡的节点标识。
        self._visited_node_ids: Set[str] = set()
        self._confirmed_no_culvert_edge_ids: Set[str] = set()
        self._culvert_coverage_by_edge = {}
        self._culvert_distance_by_edge = {}

    def apply(self, update: AbsoluteMapUpdate) -> bool:
        """验证来源与前置条件后，幂等写入一条绝对动态地图事实。

        谁调用：`NavigationRuntime` 的地图更新分发逻辑。
        谁响应：本类更新内部集合并返回是否产生新事实。
        输入输出：输入为带授权来源的 `AbsoluteMapUpdate`；输出为是否推进地图版本。
        状态影响：新事实使版本加一；重复事实不变；越权或无前置条件事实抛出 `ValueError`。
        """

        # 先检查权限，确保感知适配器不会触发任务完成类事实的幂等检验。
        self._validate_authority(update)
        # 根据事实种类选择受影响集合，并计算本次是否引入新状态。
        if update.kind is AbsoluteMapUpdateKind.BLOCK_EDGE:
            # 阻塞事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 只有此前未阻塞的道路会改变地图。
            changed = edge_id not in self._blocked_edge_ids
            # 写入阻塞事实；重复添加仍保持集合不变。
            self._blocked_edge_ids.add(edge_id)
            # 新阻塞事实会使此前的安全确认失效，避免快照同时表达矛盾状态。
            self._confirmed_clear_edge_ids.discard(edge_id)
        elif update.kind is AbsoluteMapUpdateKind.CONFIRM_EDGE_CLEAR:
            # 安全事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 只有路口完整观察能够证明整条边无障碍，观察区证据必须拒绝。
            if update.observation_scope is MapObservationScope.OBSERVATION_ZONE:
                raise ValueError("观察区不能确认整条道路安全")
            # 已阻塞的道路不能被普通确认安全事实覆盖，必须先有明确解除阻塞事件。
            if edge_id in self._blocked_edge_ids:
                raise ValueError("已阻塞道路不能直接确认安全")
            # 只有此前没有确认安全的道路会改变地图。
            changed = edge_id not in self._confirmed_clear_edge_ids
            self._confirmed_clear_edge_ids.add(edge_id)
        elif update.kind is AbsoluteMapUpdateKind.DISCOVER_CULVERT:
            # 涵洞发现事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 只有此前未知的涵洞会改变地图。
            previous_distance = self._culvert_distance_by_edge.get(edge_id)
            changed = edge_id not in self._discovered_culvert_edge_ids
            # 写入发现事实，供后续任务选择器读取。
            self._discovered_culvert_edge_ids.add(edge_id)
            if update.culvert_distance_mm is not None and update.culvert_distance_mm != previous_distance:
                self._culvert_distance_by_edge[edge_id] = update.culvert_distance_mm
                changed = True
        elif update.kind is AbsoluteMapUpdateKind.CONFIRM_NO_CULVERT:
            edge_id = self._require_edge_id(update)
            if edge_id in self._discovered_culvert_edge_ids:
                changed = False
            else:
                intervals = self._merge_coverage(edge_id, update.coverage_intervals)
                changed = intervals != self._culvert_coverage_by_edge.get(edge_id, ())
                self._culvert_coverage_by_edge[edge_id] = intervals
                if self._covers_entire_edge(intervals) and edge_id not in self._confirmed_no_culvert_edge_ids:
                    self._confirmed_no_culvert_edge_ids.add(edge_id)
                    changed = True
        elif update.kind is AbsoluteMapUpdateKind.RECON_CULVERT:
            # 涵洞完成事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 未发现涵洞不能被直接标记完成，防止跳过任务发现生命周期。
            if edge_id not in self._discovered_culvert_edge_ids:
                raise ValueError("不能完成尚未发现的涵洞")
            # 只有此前未完成侦查的涵洞会改变地图。
            changed = edge_id not in self._recon_culvert_edge_ids
            # 写入已完成侦查事实，保留发现事实作为历史信息。
            self._recon_culvert_edge_ids.add(edge_id)
        elif update.kind is AbsoluteMapUpdateKind.VISIT_NODE:
            # 节点访问事实必须引用一个路口或出发区节点。
            node_id = self._require_node_id(update)
            # 只有此前未访问的节点会改变地图。
            changed = node_id not in self._visited_node_ids
            # 写入节点访问事实，供打卡任务和面板读取。
            self._visited_node_ids.add(node_id)
        else:
            # 枚举扩展但未同步实现时立即失败，避免静默丢失动态事实。
            raise ValueError("不支持的地图更新类型")

        # 只有公开快照发生真实变化时才推进地图版本。
        if changed:
            self._version += 1
        # 向调用方返回是否产生新事实，供运行时决定是否延迟重规划。
        return changed

    def snapshot(self) -> RuntimeMapSnapshot:
        """返回当前动态地图的不可变读取快照。

        谁调用：规划器、编排器、面板和测试。
        谁响应：本类复制内部集合并封装为 `RuntimeMapSnapshot`。
        输入输出：不接收参数，返回当前版本和全部动态事实的不可变集合。
        状态影响：只读取内部状态，不改变地图版本或事实。
        """

        # 用新的不可变集合封装内部状态，调用方无法修改地图真实集合。
        return RuntimeMapSnapshot(
            version=self._version,
            blocked_edge_ids=frozenset(self._blocked_edge_ids),
            confirmed_clear_edge_ids=frozenset(self._confirmed_clear_edge_ids),
            discovered_culvert_edge_ids=frozenset(self._discovered_culvert_edge_ids),
            recon_culvert_edge_ids=frozenset(self._recon_culvert_edge_ids),
            visited_node_ids=frozenset(self._visited_node_ids),
            confirmed_no_culvert_edge_ids=frozenset(self._confirmed_no_culvert_edge_ids),
            culvert_coverage_by_edge=tuple(sorted((edge_id, intervals) for edge_id, intervals in self._culvert_coverage_by_edge.items())),
            culvert_distance_by_edge=tuple(sorted(self._culvert_distance_by_edge.items())),
        )

    def _merge_coverage(self, edge_id: str, additions: Tuple[CoverageInterval, ...]) -> Tuple[CoverageInterval, ...]:
        """合并同一物理边的重叠或相邻可见区间。"""
        intervals = sorted(self._culvert_coverage_by_edge.get(edge_id, ()) + tuple(additions), key=lambda item: item.start_ratio)
        merged = []
        for interval in intervals:
            if not merged or interval.start_ratio > merged[-1].end_ratio:
                merged.append(interval)
            else:
                merged[-1] = CoverageInterval(merged[-1].start_ratio, max(merged[-1].end_ratio, interval.end_ratio))
        return tuple(merged)

    @staticmethod
    def _covers_entire_edge(intervals: Tuple[CoverageInterval, ...]) -> bool:
        return bool(intervals) and intervals[0].start_ratio <= 0.0 and intervals[-1].end_ratio >= 1.0

    def _validate_authority(self, update: AbsoluteMapUpdate) -> None:
        """拒绝不具备业务授权来源的地图事实。"""

        # 感知适配器的许可范围只包含视觉能直接证实的事实。
        if update.authority is MapUpdateAuthority.PERCEPTION_ADAPTER:
            # 越权的感知事实必须在进入幂等逻辑前被拒绝。
            if update.kind not in self._PERCEPTION_ALLOWED_KINDS:
                raise ValueError("感知适配器无权写入该地图事实")
            # 已确认的感知授权无需继续检查。
            return
        # 协调器的许可范围只包含经执行身份校验后可派生的事实。
        if update.authority is MapUpdateAuthority.COORDINATOR:
            # 越权的协调器事实同样不能写入地图。
            if update.kind not in self._COORDINATOR_ALLOWED_KINDS:
                raise ValueError("协调器无权写入该地图事实")
            # 已确认的协调器授权无需继续检查。
            return
        # 枚举扩展但未声明规则时立即拒绝，避免默认放行新来源。
        raise ValueError("不支持的地图更新授权来源")

    @staticmethod
    def _require_edge_id(update: AbsoluteMapUpdate) -> str:
        """返回边事实的边标识，缺失时拒绝不完整更新。"""

        # 边事实没有边标识无法定位地图对象，必须立即失败。
        if update.edge_id is None:
            raise ValueError("边地图更新必须提供 edge_id")
        # 返回已经验证存在的边标识。
        return update.edge_id

    @staticmethod
    def _require_node_id(update: AbsoluteMapUpdate) -> str:
        """返回节点事实的节点标识，缺失时拒绝不完整更新。"""

        # 节点事实没有节点标识无法定位地图对象，必须立即失败。
        if update.node_id is None:
            raise ValueError("节点地图更新必须提供 node_id")
        # 返回已经验证存在的节点标识。
        return update.node_id
