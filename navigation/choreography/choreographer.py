"""将规划层路线展开为不可变经验流程剧本的纯编排器。"""

# 导入 SHA-1 摘要工具，以稳定输入生成可复现的流程剧本标识。
import hashlib
# 导入三角函数工具，按当前车头与目标巡航的几何夹角计算左转、右转或直行。
import math
# 导入 Python 3.8 兼容的联合类型工具，明确启动入口只接受两种规划层计划。
from typing import Optional, Tuple, Union

# 导入只读静态拓扑，编排器只查询巡航边长度和道路性质，不修改赛道。
from navigation.domain import TrackTopology
# 导入规划层语义计划及步骤类型，避免编排器了解搜索算法或任务选择逻辑。
from navigation.planning import RecoveryPlan, RoutePlan
# 导入跨模块流程契约，编排器只创建这些不可变对象而不直接触碰执行器。
from navigation.contracts import (
    Action,
    ChoreographyAdvanceResult,
    ChoreographyAdvanceStatus,
    ChoreographyPlan,
    ChoreographyProgress,
    ChoreographyRejection,
    ChoreographyRejectionCode,
    ChoreographySourceKind,
    ChoreographyStage,
    ChoreographyStageKind,
    ChoreographyStartResult,
    ChoreographyStartStatus,
    NavigationStateQuery,
    TurnDirection,
)

# 导入本包不可变经验距离参数，保持流程展开规则可装配、可测试。
from .profile import MotionProfile
from .departure_factory import DepartureChoreographyFactory
from .route_factory import RouteChoreographyFactory
from .stage_compiler import StageCompiler


class Choreographer:
    """把路线语义预展开为经验流程剧本，但不提交动作或管理异步状态。

    谁调用：后续 `Coordinator` 在接受 `RoutePlan` 或 `RecoveryPlan` 后调用。
    谁响应：本类返回不可变 `ChoreographyPlan` 与第一阶段 `ChoreographyProgress`。
    输入输出：输入规划层路线和已注入的只读依赖；输出不含任何执行器请求的流程剧本。
    状态影响：只读取拓扑、标定与导航状态查询口，不写机器人、地图、任务或执行器。
    """

    def __init__(self, topology: TrackTopology, profile: MotionProfile, state_query: NavigationStateQuery) -> None:
        """保存纯编排所需的静态拓扑、经验标定与只读运行时查询口。

        谁调用：导航系统的装配代码创建编排器时调用一次。
        谁响应：后续 `start()` 与 `compile_next()` 通过这些依赖生成或校验流程。
        输入输出：输入三个只读或不可变依赖；不返回值。
        状态影响：只保存引用，不写入三个依赖拥有的任何状态。
        """

        # 保存只读静态赛道，用于按路口步骤查询巡航边的长度和道路类别。
        self._topology = topology
        # 保存不可变经验标定，避免把距离常量散落到流程分支中。
        self._profile = profile
        # 保存最小只读查询口，为下一任务的逐步动作生成预留状态读取边界。
        self._state_query = state_query
        self.departure_factory = DepartureChoreographyFactory(self)
        self.route_factory = RouteChoreographyFactory(self)
        self.stage_compiler = StageCompiler(self)

    def start(self, route_or_recovery: Union[RoutePlan, RecoveryPlan]) -> ChoreographyStartResult:
        """将已验证的正常路线或倒车恢复语义展开为流程剧本与首个指针。

        谁调用：后续 `Coordinator` 在接受规划层结果后调用一次。
        谁响应：本类根据路线中的步骤、静态拓扑和经验规则创建阶段列表。
        输入输出：输入正常或恢复计划；输出 STARTED 结果中的剧本和第零阶段指针。
        状态影响：不读取感知、不生成动作、不提交执行器，也不修改导航域状态。
        """

        # 正常路线和恢复路线统一交给路线工厂展开，门面不重复实现分支。
        return self.route_factory.start(route_or_recovery)

    def start_departure(self) -> ChoreographyStartResult:
        """创建固定的出发剧本；出发分析器首次运行时调用，不由外层注入。"""

        return self.departure_factory.start_departure()

    def start_departure_left_turn(self) -> ChoreographyStartResult:
        """创建右侧受阻后的左转替代出发剧本。"""

        return self.departure_factory.start_left_turn()

    def replace_current_traversal_with_culvert(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        traversal_id: str,
        task_id: str,
    ) -> ChoreographyStartResult:
        """将当前巡航边尚未执行的普通阶段替换为涵洞探索阶段。

        谁调用：`Coordinator` 已确认涵洞位于活动路线后调用。
        谁响应：本方法保留已完成阶段和后续路线阶段，重建当前边剩余阶段。
        输入输出：输入活动剧本、当前游标、巡航标识和涵洞任务标识；输出新剧本及首个游标。
        状态影响：只创建不可变剧本，不修改旧剧本、地图、任务或机器人状态。
        """

        return self.route_factory.replace_current_traversal_with_culvert(
            plan, progress, traversal_id, task_id
        )

    def start_junction_escape(self, side: TurnDirection) -> ChoreographyStartResult:
        """按协调器指定的左侧或右侧支路创建局部脱困剧本。

        本入口只负责把指定侧支路展开为“前向转弯、转弯后观察”两个阶段；
        是否继续驶入该支路以及正式路线是否合法，仍由 Coordinator 和 RoutePlanner 决定。
        """

        return self.route_factory.start_junction_escape(side)

    def start_retrace_turn(self, source_action_id: str) -> ChoreographyStartResult:
        """为已完成但验证失败的侧支转弯创建单步同轨迹撤回剧本。"""

        return self.route_factory.start_retrace_turn(source_action_id)

    def compile_next(self, plan: ChoreographyPlan, progress: ChoreographyProgress) -> ChoreographyAdvanceResult:
        """委托阶段编译器生成当前游标对应的动作。"""

        return self.stage_compiler.compile_next(plan, progress)

    @staticmethod
    def _is_valid_progress(plan: ChoreographyPlan, progress: ChoreographyProgress) -> bool:
        """判断流程指针是否属于指定剧本且未越过允许的结束位置。"""

        # 剧本标识不一致意味着协调器试图把旧路线指针用于新流程，必须拒绝。
        if progress.choreography_id != plan.choreography_id:
            return False
        # 允许下标等于阶段数表达流程结束，但不允许负数或更大的未知阶段。
        return 0 <= progress.stage_index <= len(plan.stages)

    @staticmethod
    def _split_traversal_id(traversal_id: str) -> Tuple[str, str]:
        """解析固定的有向巡航标识，格式异常时显式失败而不是猜测端点。"""

        # 有向巡航标识必须恰好含有一个起终点分隔符，且两端都不能为空。
        parts = traversal_id.split("->")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ValueError("巡航标识必须为起点->终点格式")
        # 返回按行驶方向排列的静态路口标识。
        return parts[0], parts[1]

    def _turn_difference_deg(self, from_node_id: str, to_node_id: str, heading_deg: float) -> float:
        """计算当前车头旋转到目标巡航方向的归一化有符号角度差。"""

        # 读取相邻路口中心坐标，以此定义目标巡航的世界几何朝向。
        from_node = self._topology.get_node(from_node_id)
        to_node = self._topology.get_node(to_node_id)
        # 使用 x/y 向量得到以东为零、逆时针为正的目标朝向角度。
        target_heading_deg = math.degrees(math.atan2(to_node.y_mm - from_node.y_mm, to_node.x_mm - from_node.x_mm))
        # 归一化到 [-180, 180) 区间，正值为左侧旋转，负值为右侧旋转。
        return (target_heading_deg - heading_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _action_id(plan: ChoreographyPlan, stage_index: int) -> str:
        """由剧本标识和阶段下标生成稳定动作标识，供协调器匹配异步中断。"""

        # 动作对应剧本中的唯一阶段，稳定标识无需读取时间或全局递增计数。
        return "action:{}:{}".format(plan.choreography_id, stage_index)

    @staticmethod
    def _ready(action: Action, plan: ChoreographyPlan, next_stage_index: int) -> ChoreographyAdvanceResult:
        """集中创建严格 READY 结果，保证动作与下一指针始终同时返回。"""

        # 由编排器而非协调器创建下一不可变指针，协调器只能保存它等待动作成功。
        next_progress = ChoreographyProgress(plan.choreography_id, next_stage_index)
        # 返回满足契约字段组合校验的唯一可提交动作结果。
        return ChoreographyAdvanceResult(ChoreographyAdvanceStatus.READY, action, next_progress)

    @staticmethod
    def _rejected(code: ChoreographyRejectionCode, reason: str) -> ChoreographyAdvanceResult:
        """集中创建严格 REJECTED 结果，确保拒绝路径绝不夹带可执行动作。"""

        # 将受控原因和面板可读文本组合为明确拒绝对象。
        rejection = ChoreographyRejection(code, reason)
        # 返回严格拒绝结果，协调器后续负责销毁流程或上抛错误。
        return ChoreographyAdvanceResult(ChoreographyAdvanceStatus.REJECTED, rejection=rejection)

    def _needs_observation_zone(self, road_kind: str, length_mm: float) -> bool:
        """判断一条巡航边是否需要在距终点五百毫米内设置边中观察区。"""

        # 隧道始终不能在边中插入观察区，直接按短边流程驶入下一路口。
        if road_kind == "TUNNEL":
            return False
        # 普通边只有长度超过观察区剩余距离阈值时才有独立观察区空间。
        return length_mm > self._profile.observation_zone_max_remaining_mm

    @staticmethod
    def _stage(
        step_index: int,
        suffix: str,
        kind: ChoreographyStageKind,
        traversal_id: str,
        node_id: Optional[str],
        safe_node_id: Optional[str] = None,
    ) -> ChoreographyStage:
        """按稳定步骤序号创建一条不可变流程阶段，集中避免阶段标识拼写分散。"""

        # 阶段标识只由路线内顺序、语义后缀和巡航标识组成，便于面板和日志稳定关联。
        stage_id = "stage:{}:{}:{}".format(step_index, suffix, traversal_id)
        # 返回不含任何可变执行状态的阶段定义。
        return ChoreographyStage(stage_id, kind, traversal_id, node_id, safe_node_id=safe_node_id)

    @staticmethod
    def _choreography_id(source_plan_id: str, stage_ids: Tuple[str, ...]) -> str:
        """由稳定来源计划和阶段清单生成可复现流程剧本标识。"""

        # 将所有稳定输入串联成唯一可哈希文本，不读取时钟或全局计数器。
        raw_identity = "{}|{}".format(source_plan_id, "|".join(stage_ids))
        # 取 SHA-1 十六进制摘要作为紧凑稳定后缀，足以用于本地流程关联。
        digest = hashlib.sha1(raw_identity.encode("utf-8")).hexdigest()
        # 保留来源计划可读性，同时避免不同阶段列表使用同一标识。
        return "choreography:{}:{}".format(source_plan_id, digest[:12])
