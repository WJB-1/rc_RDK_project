"""定义动态地图事实、来源权限、幂等写入和不可变读取快照。"""

# 导入不可变数据类装饰器，保证更新包和读取快照不能被调用方原地修改。
from dataclasses import dataclass
# 导入枚举基类，限制地图事实类型和写入权限来源。
from enum import Enum
# 导入 Python 3.8 兼容的集合与可选值类型注解。
from typing import FrozenSet, Optional, Set


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
    # 将此前阻塞的道路恢复为可通行。
    UNBLOCK_EDGE = "unblock_edge"
    # 确认某条道路上存在待侦查涵洞。
    DISCOVER_CULVERT = "discover_culvert"
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
            AbsoluteMapUpdateKind.UNBLOCK_EDGE,
            AbsoluteMapUpdateKind.DISCOVER_CULVERT,
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
        # 保存已经由视觉确认存在涵洞的物理边标识。
        self._discovered_culvert_edge_ids: Set[str] = set()
        # 保存已经由协调器确认侦查完成的涵洞边标识。
        self._recon_culvert_edge_ids: Set[str] = set()
        # 保存已经由协调器确认到达或打卡的节点标识。
        self._visited_node_ids: Set[str] = set()

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
        elif update.kind is AbsoluteMapUpdateKind.UNBLOCK_EDGE:
            # 解除阻塞事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 只有此前已阻塞的道路会改变地图。
            changed = edge_id in self._blocked_edge_ids
            # 移除阻塞事实；不存在时保持集合不变。
            self._blocked_edge_ids.discard(edge_id)
        elif update.kind is AbsoluteMapUpdateKind.DISCOVER_CULVERT:
            # 涵洞发现事实必须引用一条物理边。
            edge_id = self._require_edge_id(update)
            # 只有此前未知的涵洞会改变地图。
            changed = edge_id not in self._discovered_culvert_edge_ids
            # 写入发现事实，供后续任务选择器读取。
            self._discovered_culvert_edge_ids.add(edge_id)
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
            discovered_culvert_edge_ids=frozenset(self._discovered_culvert_edge_ids),
            recon_culvert_edge_ids=frozenset(self._recon_culvert_edge_ids),
            visited_node_ids=frozenset(self._visited_node_ids),
        )

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
