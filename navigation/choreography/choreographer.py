"""将规划层路线展开为不可变经验流程剧本的纯编排器。"""

# 导入 SHA-1 摘要工具，以稳定输入生成可复现的流程剧本标识。
import hashlib
# 导入三角函数工具，按当前车头与目标巡航的几何夹角计算左转、右转或直行。
import math
# 导入 Python 3.8 兼容的联合类型工具，明确启动入口只接受两种规划层计划。
from typing import Optional, Tuple, Union

# 导入只读静态拓扑，编排器只查询巡航边长度和道路性质，不修改赛道。
from navigation.domain import AtNode, OnCruiseEdge, TrackTopology
# 导入规划层语义计划及步骤类型，避免编排器了解搜索算法或任务选择逻辑。
from navigation.planning import RecoveryPlan, RecoveryStepKind, RoutePlan
# 导入跨模块流程契约，编排器只创建这些不可变对象而不直接触碰执行器。
from navigation.contracts import (
    Action,
    AdvanceOnTraversalEffect,
    AlignToTraversalEffect,
    ArriveAtNodeEffect,
    AwaitObservationEffect,
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
    DriveDistanceCommand,
    DrivePurpose,
    CompleteTaskEffect,
    ExecuteTaskCommand,
    NavigationStateQuery,
    ObservationScope,
    ObserveCommand,
    RetraceTurnCommand,
    RetraceTurnEffect,
    ReverseDistanceCommand,
    TurnAtJunctionCommand,
    TurnDirection,
)

# 导入本包不可变经验距离参数，保持流程展开规则可装配、可测试。
from .profile import MotionProfile


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

    def start(self, route_or_recovery: Union[RoutePlan, RecoveryPlan]) -> ChoreographyStartResult:
        """将已验证的正常路线或倒车恢复语义展开为流程剧本与首个指针。

        谁调用：后续 `Coordinator` 在接受规划层结果后调用一次。
        谁响应：本类根据路线中的步骤、静态拓扑和经验规则创建阶段列表。
        输入输出：输入正常或恢复计划；输出 STARTED 结果中的剧本和第零阶段指针。
        状态影响：不读取感知、不生成动作、不提交执行器，也不修改导航域状态。
        """

        # 正常路线按每一个相邻路口巡航步骤展开为观察、可选转弯和行驶阶段。
        if isinstance(route_or_recovery, RoutePlan):
            return self._start_route(route_or_recovery)
        # 恢复路线按受限 BACKTRACK 语义展开为唯一允许的倒车阶段。
        if isinstance(route_or_recovery, RecoveryPlan):
            return self._start_recovery(route_or_recovery)
        # 其他对象不是规划层冻结的输入类型，直接拒绝可避免错误的隐式解释。
        raise TypeError("start 只接受 RoutePlan 或 RecoveryPlan")

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

        # 先验证游标属于当前剧本，避免在替换时误用已经销毁的旧路线。
        if not self._is_valid_progress(plan, progress):
            return ChoreographyStartResult(
                ChoreographyStartStatus.REJECTED,
                rejection=ChoreographyRejection(
                    ChoreographyRejectionCode.INVALID_PROGRESS,
                    "涵洞替换的流程指针不属于当前剧本",
                ),
            )
        # 活动路线必须包含待替换的巡航边，未知边不能被编排器猜测端点。
        if traversal_id not in plan.route_steps:
            return ChoreographyStartResult(
                ChoreographyStartStatus.REJECTED,
                rejection=ChoreographyRejection(
                    ChoreographyRejectionCode.INVALID_PROGRESS,
                    "涵洞替换目标不在活动路线中",
                ),
            )
        # 解析当前巡航两端并查询静态长度，保证新剧本仍遵守原路线几何。
        from_node_id, to_node_id = self._split_traversal_id(traversal_id)
        cruise_edge = self._topology.get_cruise_edge(from_node_id, to_node_id)
        # 游标之前的阶段已经执行或正在等待其终局，必须原样保留。
        preserved_stages = list(plan.stages[: progress.stage_index])
        # 当前阶段及其后续阶段中，属于目标巡航的普通阶段全部删除；后续巡航阶段继续保留。
        preserved_stages.extend(
            stage
            for stage in plan.stages[progress.stage_index :]
            if stage.traversal_id != traversal_id
        )
        # 新阶段插入在已完成前缀之后，游标指向第一条替换动作。
        insertion_index = len(plan.stages[: progress.stage_index])
        replacement_stages = []
        preserved_current_kinds = {
            stage.kind
            for stage in plan.stages[: progress.stage_index]
            if stage.traversal_id == traversal_id
        }
        # 尚未完成进入前观察时保留观察闭环的第一段。
        if ChoreographyStageKind.OBSERVE_PRE_ENTRY not in preserved_current_kinds:
            replacement_stages.append(
                self._stage(0, "culvert_pre", ChoreographyStageKind.OBSERVE_PRE_ENTRY, traversal_id, from_node_id)
            )
        # 尚未完成路口转向时继续生成编排器负责的前向转弯阶段。
        if ChoreographyStageKind.TURN_AT_JUNCTION not in preserved_current_kinds:
            replacement_stages.append(
                self._stage(0, "culvert_turn", ChoreographyStageKind.TURN_AT_JUNCTION, traversal_id, from_node_id)
            )
        # 涵洞不进入边中观察区，但转弯后仍需观察一次目标边。
        if ChoreographyStageKind.OBSERVE_POST_TURN not in preserved_current_kinds:
            replacement_stages.append(
                self._stage(0, "culvert_post", ChoreographyStageKind.OBSERVE_POST_TURN, traversal_id, from_node_id)
            )
        # 涵洞任务的运动收口仍是驶入相邻路口中心。
        if ChoreographyStageKind.DRIVE_TO_NEXT_CENTER not in preserved_current_kinds:
            replacement_stages.append(
                self._stage(0, "culvert_center", ChoreographyStageKind.DRIVE_TO_NEXT_CENTER, traversal_id, to_node_id)
            )
        # 到达涵洞目标路口后执行任务，任务阶段由 Coordinator 后续推进注册表生命周期。
        replacement_stages.append(
            ChoreographyStage(
                "stage:0:culvert_task:{}".format(task_id),
                ChoreographyStageKind.EXECUTE_TASK,
                traversal_id,
                to_node_id,
                task_id,
            )
        )
        # 将替换阶段插回已完成前缀与后续路线之间，保持路线步骤顺序不变。
        new_stages = preserved_stages[:insertion_index] + replacement_stages + preserved_stages[insertion_index:]
        # 以源计划和新阶段清单生成新的稳定剧本身份，旧剧本自然失效。
        choreography_id = self._choreography_id(plan.source_plan_id, tuple(stage.stage_id for stage in new_stages))
        new_plan = ChoreographyPlan(
            choreography_id,
            plan.source_plan_id,
            plan.source_kind,
            plan.source_map_version,
            plan.route_steps,
            tuple(new_stages),
        )
        # 返回替换段第一阶段的游标，协调器随后按普通串行规则提交动作。
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED,
            new_plan,
            ChoreographyProgress(choreography_id, insertion_index),
        )

    def _start_route(self, route: RoutePlan) -> ChoreographyStartResult:
        """把正常路线的每条巡航边预展开为统一的经验流程阶段。"""

        # 创建可追加的阶段列表，完成后会转换为不可变元组写入剧本。
        stages = []
        # 依次处理规划器已经排好顺序的相邻路口步骤。
        for step_index, step in enumerate(route.steps):
            # 查询静态派生巡航边，验证路线步骤与静态拓扑的长度、道路性质一致。
            cruise_edge = self._topology.get_cruise_edge(step.from_junction, step.to_junction)
            # 进入一条边前，车辆位于路口中心，必须先观察该边当前前方路况。
            stages.append(self._stage(step_index, "pre", ChoreographyStageKind.OBSERVE_PRE_ENTRY, step.traversal_id, step.from_junction))
            # 是否真正需要转弯由 compile_next 在当时的真实朝向下决定，剧本只保留该业务机会。
            stages.append(self._stage(step_index, "turn", ChoreographyStageKind.TURN_AT_JUNCTION, step.traversal_id, step.from_junction))
            # 转弯后再观察面向的目标边；直行时该阶段同样会在后续统一生成观察动作。
            stages.append(self._stage(step_index, "post", ChoreographyStageKind.OBSERVE_POST_TURN, step.traversal_id, step.from_junction))
            # 只有普通道路且总长度严格大于 500 毫米时，才在边中额外插入观察区阶段。
            if self._needs_observation_zone(cruise_edge.road_kind, cruise_edge.length_mm):
                # 固定前进到观察区；视觉校正后的剩余距离不能在创建剧本时伪造。
                stages.append(self._stage(step_index, "zone_drive", ChoreographyStageKind.DRIVE_TO_OBSERVATION_ZONE, step.traversal_id, None))
                # 在观察区等待感知系统的单一最终观察结果。
                stages.append(self._stage(step_index, "zone_observe", ChoreographyStageKind.OBSERVE_AT_ZONE, step.traversal_id, None))
            # 每条巡航最后都要按届时位置校正后的剩余距离驶入下一个路口中心。
            stages.append(self._stage(step_index, "center", ChoreographyStageKind.DRIVE_TO_NEXT_CENTER, step.traversal_id, step.to_junction))
        # 用路线身份与阶段身份生成可复现剧本标识，避免模块级自增编号。
        choreography_id = self._choreography_id(route.plan_id, tuple(stage.stage_id for stage in stages))
        # 创建不可变正常流程剧本，只保留规划步骤标识供调试关联。
        plan = ChoreographyPlan(
            choreography_id=choreography_id,
            source_plan_id=route.plan_id,
            source_kind=ChoreographySourceKind.NORMAL,
            source_map_version=route.map_version,
            route_steps=tuple(step.traversal_id for step in route.steps),
            stages=tuple(stages),
        )
        # 返回剧本和由编排器创建的第零阶段指针，协调器不应自行计算下标。
        return ChoreographyStartResult(ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0))

    def _start_recovery(self, recovery: RecoveryPlan) -> ChoreographyStartResult:
        """把受限恢复计划展开为沿已进入边倒车的唯一流程阶段。"""

        # 创建可追加的恢复阶段列表，首版只允许规划层已验证的 BACKTRACK 语义。
        stages = []
        # 依次读取恢复步骤，虽然首版只有一步，仍保持与正常路线一致的通用结构。
        for step_index, step in enumerate(recovery.steps):
            # 其他恢复语义尚未冻结，不能被编排器擅自翻译为普通路线动作。
            if step.kind != RecoveryStepKind.BACKTRACK:
                raise ValueError("当前编排器只支持 BACKTRACK 恢复步骤")
            # 倒车阶段引用原进入巡航，并在成功后抵达规划层验证过的安全路口。
            stages.append(
                self._stage(
                    step_index,
                    "reverse",
                    ChoreographyStageKind.REVERSE_TO_SAFE_JUNCTION,
                    step.traversal_id,
                    step.from_junction,
                    safe_node_id=step.to_junction,
                )
            )
        # 用恢复计划身份与阶段身份生成同样可复现的剧本标识。
        choreography_id = self._choreography_id(recovery.plan_id, tuple(stage.stage_id for stage in stages))
        # 恢复计划尚未携带动态地图版本，首版使用零表示它不依赖普通路线快照版本。
        plan = ChoreographyPlan(
            choreography_id=choreography_id,
            source_plan_id=recovery.plan_id,
            source_kind=ChoreographySourceKind.RECOVERY,
            source_map_version=0,
            route_steps=tuple(step.traversal_id for step in recovery.steps),
            stages=tuple(stages),
        )
        # 返回恢复剧本与第零阶段指针，后续仍由协调器管理其异步生命周期。
        return ChoreographyStartResult(ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0))

    def compile_next(self, plan: ChoreographyPlan, progress: ChoreographyProgress) -> ChoreographyAdvanceResult:
        """按当前流程指针与只读状态生成唯一下一动作和其成功后的下一指针。

        谁调用：后续 `Coordinator` 在上一动作成功且已投影状态后调用。
        谁响应：本类读取剧本、指针和最小状态查询口，返回 READY、FINISHED 或 REJECTED。
        输入输出：输入活动剧本与指针；READY 只输出一条动作和成功后可用的新指针。
        状态影响：不修改状态、不提交动作；协调器是唯一保存与消费动作结果的模块。
        """

        # 指针必须属于当前剧本且处于闭区间 [0, 阶段数]，否则不能继续解释旧流程。
        if not self._is_valid_progress(plan, progress):
            return self._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "流程指针不属于当前剧本或超出阶段范围",
            )
        # 指向阶段数说明上一条动作已经完成最后一个阶段，协调器应重新进入规划门禁。
        if progress.stage_index == len(plan.stages):
            return ChoreographyAdvanceResult(ChoreographyAdvanceStatus.FINISHED)
        # 读取当前待解释阶段；剧本本身不可变，不会在此被消费或改写。
        stage = plan.stages[progress.stage_index]
        # 路口转弯是唯一可能不生成实际动作的阶段，直行时跳过后继续生成下一动作。
        if stage.kind == ChoreographyStageKind.TURN_AT_JUNCTION:
            return self._compile_turn_or_skip(plan, progress, stage)
        # 三类观察阶段都只生成一次规定范围的观察命令，等待由后续协调器管理。
        if stage.kind in (
            ChoreographyStageKind.OBSERVE_PRE_ENTRY,
            ChoreographyStageKind.OBSERVE_POST_TURN,
            ChoreographyStageKind.OBSERVE_AT_ZONE,
        ):
            return self._ready_observation(plan, progress, stage)
        # 固定进入观察区动作只推进边上经验里程，不能伪造已到达下一路口。
        if stage.kind == ChoreographyStageKind.DRIVE_TO_OBSERVATION_ZONE:
            return self._ready_observation_zone_drive(plan, progress, stage)
        # 驶向下一个中心时才读取当前校正后的边上进度来计算剩余距离。
        if stage.kind == ChoreographyStageKind.DRIVE_TO_NEXT_CENTER:
            return self._ready_next_center_drive(plan, progress, stage)
        # 受限恢复只允许沿原进入巡航的同一路径倒车至已验证安全路口。
        if stage.kind == ChoreographyStageKind.REVERSE_TO_SAFE_JUNCTION:
            return self._ready_reverse_to_safe_junction(plan, progress, stage)
        # 路口局部恢复必须沿已完成的真实前向转弯轨迹反向撤回，绝不重新计算左右转。
        if stage.kind == ChoreographyStageKind.RETRACE_TURN:
            return self._ready_retrace_turn(plan, progress, stage)
        # 到达涵洞或打卡目标后生成任务动作，任务结果由协调器确认并推进生命周期。
        if stage.kind == ChoreographyStageKind.EXECUTE_TASK:
            return self._ready_execute_task(plan, progress, stage)
        # 本任务暂未实现任务、倒车和撤回的单步动作；未知阶段不得伪造可执行指令。
        return self._rejected(
            ChoreographyRejectionCode.INVALID_PROGRESS,
            "当前流程阶段尚未实现动作翻译：{}".format(stage.kind.value),
        )

    def _ready_execute_task(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """将任务阶段翻译为单条类型化任务命令。"""

        # 缺少任务标识时无法路由任务执行器，必须显式拒绝。
        if stage.task_id is None:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "任务阶段缺少 task_id")
        # 创建任务动作；完成后由 Coordinator 推进 TaskRegistry，而不是编排器直接改状态。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=ExecuteTaskCommand(stage.task_id),
            expected_effect=CompleteTaskEffect(stage.task_id),
        )
        return self._ready(action, plan, progress.stage_index + 1)

    def _compile_turn_or_skip(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """在真实车头朝向下生成左/右转弯，或在直行时跳过虚拟转弯阶段。"""

        # 转弯阶段必须关联一条有效目标巡航，否则无法依据几何计算车头需要面向的方向。
        if stage.traversal_id is None:
            return self._rejected(ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION, "转弯阶段缺少目标巡航标识")
        # 从稳定的有向巡航标识解析两端路口；剧本创建器保证格式，仍保留防御性校验。
        from_node_id, to_node_id = self._split_traversal_id(stage.traversal_id)
        # 读取当前机器人朝向并以静态中心坐标计算目标巡航的几何方向。
        heading_difference = self._turn_difference_deg(from_node_id, to_node_id, self._state_query.robot_state().heading_deg)
        # 允许非常小的浮点误差；车头已对齐时不应生成名为“直行转弯”的伪动作。
        if abs(heading_difference) < 0.000001:
            return self.compile_next(plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1))
        # 绝对一百八十度意味着原地掉头；无论普通路线或逃生流程都必须由协调器上抛错误。
        if abs(abs(heading_difference) - 180.0) < 0.000001:
            return self._rejected(
                ChoreographyRejectionCode.FORBIDDEN_UTURN,
                "目标巡航要求绝对禁止的一百八十度原地掉头",
            )
        # 正角度表示逆时针向车头左侧旋转九十度，负角度表示向右侧旋转九十度。
        if abs(heading_difference - 90.0) < 0.000001:
            turn_direction = TurnDirection.LEFT
        elif abs(heading_difference + 90.0) < 0.000001:
            turn_direction = TurnDirection.RIGHT
        else:
            # 非直行或九十度的方向尚无冻结标定轨迹，不能让运动层猜测真实车身动作。
            return self._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                "当前朝向与目标巡航无法组成已标定的直行或九十度转弯",
            )
        # 创建唯一前向路口转弯动作，并约定成功后由协调器投影为已对齐目标巡航。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=TurnAtJunctionCommand(turn_direction, stage.traversal_id),
            expected_effect=AlignToTraversalEffect(stage.traversal_id),
        )
        # 返回当前动作成功后才能使用的下一阶段指针，协调器负责原子保存它。
        return self._ready(action, plan, progress.stage_index + 1)

    def _ready_reverse_to_safe_junction(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """把已验证的 BACKTRACK 阶段翻译为沿原进入边反向倒车的唯一动作。"""

        # 恢复阶段必须保留原进入巡航和规划器验证过的安全目的地，缺失时不能猜测倒车方向。
        if stage.traversal_id is None or stage.safe_node_id is None or stage.node_id is None:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "倒车恢复阶段缺少进入巡航、当前位置或安全路口")
        # 原进入巡航定义为安全路口到当前路口，读取它即可得到同一路物理路径总长度。
        source_node_id, current_node_id = self._split_traversal_id(stage.traversal_id)
        # 静态标识与阶段当前位置或安全目的地不一致时，绝不把恢复误解释为替代路线。
        if current_node_id != stage.node_id or source_node_id != stage.safe_node_id:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "倒车恢复阶段与原进入巡航端点不一致")
        # 倒车只能从原进入巡航的终点路口执行；当前位置不匹配时不允许盲目后退。
        location = self._state_query.robot_state().location
        if not isinstance(location, AtNode) or location.node_id != current_node_id:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "倒车恢复必须从当前安全路口中心开始")
        # 查询原巡航总距离，倒车沿同一物理路径回退完整长度，不搜索替代出边。
        cruise_edge = self._topology.get_cruise_edge(source_node_id, current_node_id)
        # 逻辑上倒车抵达安全路口时的进入巡航方向与原进入方向相反，供后续规划禁止立即掉头。
        reverse_entry_traversal_id = "{}->{}".format(current_node_id, source_node_id)
        # 创建唯一倒车动作；其命令仍引用原进入巡航，明确这是该路径的反向执行版本。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=ReverseDistanceCommand(stage.traversal_id, cruise_edge.length_mm, stage.safe_node_id),
            expected_effect=ArriveAtNodeEffect(stage.safe_node_id, reverse_entry_traversal_id),
        )
        # 倒车成功后流程结束，协调器将以最后指针再次调用编排器取得 FINISHED。
        return self._ready(action, plan, progress.stage_index + 1)

    def _ready_retrace_turn(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """将局部恢复阶段翻译为引用原前向动作身份的同轨迹撤回命令。"""

        # 局部恢复必须由协调器在创建剧本时写入已成功前向转弯的动作身份。
        if stage.source_action_id is None:
            return self._rejected(
                ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                "同轨迹撤回阶段缺少已完成前向转弯动作标识",
            )
        # 命令只携带来源动作身份，执行适配器随后从成功历史读取真实轨迹并反向执行。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=RetraceTurnCommand(stage.source_action_id),
            expected_effect=RetraceTurnEffect(stage.source_action_id),
        )
        # 撤回动作成功后才可执行局部恢复剧本的后续阶段或结束该剧本。
        return self._ready(action, plan, progress.stage_index + 1)

    def _ready_observation(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """将一个观察语义阶段翻译为单一观察命令及等待观察投影模板。"""

        # 观察阶段没有巡航标识就不知道面向哪条道路，必须返回错误而不是请求模糊观察。
        if stage.traversal_id is None:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "观察阶段缺少目标巡航标识")
        # 按阶段语义选择感知系统可理解的观察范围，不将原始感知事实传入编排器。
        scope_by_stage = {
            ChoreographyStageKind.OBSERVE_PRE_ENTRY: ObservationScope.PRE_ENTRY,
            ChoreographyStageKind.OBSERVE_POST_TURN: ObservationScope.POST_TURN,
            ChoreographyStageKind.OBSERVE_AT_ZONE: ObservationScope.OBSERVATION_ZONE,
        }
        # 当前调用方已保证阶段属于映射表，集中读取可避免在协调器复制该语义。
        scope = scope_by_stage[stage.kind]
        # 生成唯一观察动作；成功时的帧处理和地图更新由协调器接管。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=ObserveCommand(scope, stage.traversal_id),
            expected_effect=AwaitObservationEffect(),
        )
        # 观察成功后才能进入下一经验阶段。
        return self._ready(action, plan, progress.stage_index + 1)

    def _ready_observation_zone_drive(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """生成固定驶入普通长边观察区的前进动作，不预先猜测视觉校正值。"""

        # 驶入观察区必须知道当前沿哪条巡航边前进，缺失时不能生成底盘命令。
        if stage.traversal_id is None:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "观察区前进阶段缺少巡航标识")
        # 创建固定经验距离的前进命令，后续观察会用视觉/IPM 覆盖此经验误差。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=DriveDistanceCommand(stage.traversal_id, self._profile.initial_observation_advance_mm, DrivePurpose.TO_OBSERVATION_ZONE),
            expected_effect=AdvanceOnTraversalEffect(stage.traversal_id, self._profile.initial_observation_advance_mm),
        )
        # 固定前进成功后才可进行观察区观察。
        return self._ready(action, plan, progress.stage_index + 1)

    def _ready_next_center_drive(
        self,
        plan: ChoreographyPlan,
        progress: ChoreographyProgress,
        stage: ChoreographyStage,
    ) -> ChoreographyAdvanceResult:
        """根据当前边上校正进度生成驶入下一路口中心的剩余前进动作。"""

        # 驶向中心阶段必须关联目标巡航和终点路口，二者缺任一个都会造成错误的状态投影。
        if stage.traversal_id is None or stage.node_id is None:
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "驶向路口中心阶段缺少巡航或目标路口")
        # 即将驶入目标巡航前再次查询动态阻塞事实，旧流程绝不能越过已更新的地图。
        if self._state_query.is_traversal_blocked(stage.traversal_id):
            return self._rejected(ChoreographyRejectionCode.STALE_BLOCKED_TRAVERSAL, "目标巡航边已被运行时地图封锁")
        # 从目标巡航标识解析静态两端，读取完整物理距离。
        from_node_id, to_node_id = self._split_traversal_id(stage.traversal_id)
        cruise_edge = self._topology.get_cruise_edge(from_node_id, to_node_id)
        # 读取只读机器人位置，观察区校正已由协调器在此前成功中断后写入这里。
        location = self._state_query.robot_state().location
        # 位于相同巡航边时，优先按视觉/IPM 已校正的真实进度计算剩余距离。
        if isinstance(location, OnCruiseEdge) and location.traversal_id == stage.traversal_id:
            remaining_distance_mm = cruise_edge.length_mm - location.progress_mm
        # 短边或隧道没有观察区，路口观察后仍从起点中心开始计算整条边。
        elif isinstance(location, AtNode) and location.node_id == from_node_id:
            remaining_distance_mm = cruise_edge.length_mm
        else:
            # 当前位置不能安全解释为该巡航的起点或边上进度，拒绝而不下发可能错误的长距离动作。
            return self._rejected(ChoreographyRejectionCode.INVALID_PROGRESS, "当前位置与驶向路口中心阶段不匹配")
        # 在剩余真实距离上增加经验路口中心补偿，确保到达后可安全执行下一次转弯。
        distance_mm = remaining_distance_mm + self._profile.junction_center_entry_extra_mm
        # 生成唯一驶入中心动作；成功后由协调器以实际中断证据吸附到目标路口。
        action = Action(
            action_id=self._action_id(plan, progress.stage_index),
            command=DriveDistanceCommand(stage.traversal_id, distance_mm, DrivePurpose.TO_NEXT_CENTER),
            expected_effect=ArriveAtNodeEffect(stage.node_id, stage.traversal_id),
        )
        # 驶入中心成功后才可解释后续边的路口观察阶段。
        return self._ready(action, plan, progress.stage_index + 1)

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
