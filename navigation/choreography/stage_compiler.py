"""将剧本单一阶段翻译为类型化动作的编译器。"""

# 导入阶段与结果枚举，编译器据此决定当前游标的动作翻译分支。
from navigation.contracts import (
    Action,
    AdvanceOnTraversalEffect,
    AlignToTraversalEffect,
    ArriveAtNodeEffect,
    AwaitObservationEffect,
    ChoreographyAdvanceResult,
    ChoreographyAdvanceStatus,
    ChoreographyProgress,
    ChoreographyRejectionCode,
    ChoreographyStageKind,
    CompleteTaskEffect,
    DriveDistanceCommand,
    DrivePurpose,
    ExecuteTaskCommand,
    ObserveCommand,
    ObservationScope,
    RetraceTurnCommand,
    RetraceTurnEffect,
    ReverseDistanceCommand,
    TurnAtJunctionCommand,
    TurnDirection,
)
# 导入当前位置类型，用于严格校验驶入路口和倒车动作的起始位置。
from navigation.domain import AtNode, OnCruiseEdge


class StageCompiler:
    """负责把剧本游标编译为一条类型化 Action。"""

    def __init__(self, choreographer):
        """保存只读拓扑、状态查询和动作构造辅助的宿主引用。"""

        # 编译器不保存游标，也不写入导航状态；所有输入均来自本次调用。
        self._choreographer = choreographer

    def compile_next(self, plan, progress):
        """编译游标指向的唯一阶段，返回动作、结束或显式拒绝。"""

        # 指针错配意味着尝试消费已销毁剧本，必须停止而非猜测下一动作。
        if not self._choreographer._is_valid_progress(plan, progress):
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "流程指针不属于当前剧本或超出阶段范围",
            )
        # 游标指向阶段总数表示整个剧本已完成，协调器随后重新进入规划门禁。
        if progress.stage_index == len(plan.stages):
            return ChoreographyAdvanceResult(ChoreographyAdvanceStatus.FINISHED)
        stage = plan.stages[progress.stage_index]
        # 转弯可能因朝向已对齐而跳过，其余阶段一律只生成一条实际动作。
        if stage.kind is ChoreographyStageKind.TURN_AT_JUNCTION:
            return self._compile_turn_or_skip(plan, progress, stage)
        if stage.kind in (
            ChoreographyStageKind.OBSERVE_PRE_ENTRY,
            ChoreographyStageKind.OBSERVE_POST_TURN,
            ChoreographyStageKind.OBSERVE_AT_ZONE,
        ):
            return self._ready_observation(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.DRIVE_TO_OBSERVATION_ZONE:
            return self._ready_observation_zone_drive(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.DRIVE_TO_NEXT_CENTER:
            return self._ready_next_center_drive(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.REVERSE_TO_SAFE_JUNCTION:
            return self._ready_reverse_to_safe_junction(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.RETRACE_TURN:
            return self._ready_retrace_turn(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.EXECUTE_TASK:
            return self._ready_execute_task(plan, progress, stage)
        return self._choreographer._rejected(
            ChoreographyRejectionCode.INVALID_PROGRESS,
            "当前流程阶段尚未实现动作翻译：{}".format(stage.kind.value),
        )

    def _ready_execute_task(self, plan, progress, stage):
        """将任务阶段翻译为一条类型化任务执行动作。"""

        if stage.task_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "任务阶段缺少 task_id"
            )
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            ExecuteTaskCommand(stage.task_id),
            CompleteTaskEffect(stage.task_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _compile_turn_or_skip(self, plan, progress, stage):
        """按实时朝向生成左转或右转；直行时跳过该虚拟阶段。"""

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION, "转弯阶段缺少目标巡航标识"
            )
        from_node_id, to_node_id = self._choreographer._split_traversal_id(stage.traversal_id)
        heading_difference = self._choreographer._turn_difference_deg(
            from_node_id,
            to_node_id,
            self._choreographer._state_query.robot_state().heading_deg,
        )
        if abs(heading_difference) < 0.000001:
            # 已对齐时直接递归编译下一阶段，不生成“直行转弯”伪动作。
            return self.compile_next(
                plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1)
            )
        if abs(abs(heading_difference) - 180.0) < 0.000001:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.FORBIDDEN_UTURN,
                "目标巡航要求绝对禁止的一百八十度原地掉头",
            )
        if abs(heading_difference - 90.0) < 0.000001:
            direction = TurnDirection.LEFT
        elif abs(heading_difference + 90.0) < 0.000001:
            direction = TurnDirection.RIGHT
        else:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                "当前朝向与目标巡航无法组成已标定的直行或九十度转弯",
            )
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            TurnAtJunctionCommand(direction, stage.traversal_id),
            AlignToTraversalEffect(stage.traversal_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_reverse_to_safe_junction(self, plan, progress, stage):
        """将恢复规划的 BACKTRACK 阶段翻译为唯一允许的倒车动作。"""

        if stage.traversal_id is None or stage.safe_node_id is None or stage.node_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "倒车恢复阶段缺少进入巡航、当前位置或安全路口",
            )
        source_node_id, current_node_id = self._choreographer._split_traversal_id(stage.traversal_id)
        if current_node_id != stage.node_id or source_node_id != stage.safe_node_id:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "倒车恢复阶段与原进入巡航端点不一致",
            )
        location = self._choreographer._state_query.robot_state().location
        if not isinstance(location, AtNode) or location.node_id != current_node_id:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "倒车恢复必须从当前安全路口中心开始",
            )
        cruise_edge = self._choreographer._topology.get_cruise_edge(source_node_id, current_node_id)
        reverse_entry_id = "{}->{}".format(current_node_id, source_node_id)
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            ReverseDistanceCommand(stage.traversal_id, cruise_edge.length_mm, stage.safe_node_id),
            ArriveAtNodeEffect(stage.safe_node_id, reverse_entry_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_retrace_turn(self, plan, progress, stage):
        """将撤回阶段翻译为引用原前向动作的同轨迹撤回命令。"""

        if stage.source_action_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                "同轨迹撤回阶段缺少已完成前向转弯动作标识",
            )
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            RetraceTurnCommand(stage.source_action_id),
            RetraceTurnEffect(stage.source_action_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_observation(self, plan, progress, stage):
        """将观察阶段翻译为一次受控观察动作。"""

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "观察阶段缺少目标巡航标识"
            )
        scope_by_stage = {
            ChoreographyStageKind.OBSERVE_PRE_ENTRY: ObservationScope.PRE_ENTRY,
            ChoreographyStageKind.OBSERVE_POST_TURN: ObservationScope.POST_TURN,
            ChoreographyStageKind.OBSERVE_AT_ZONE: ObservationScope.OBSERVATION_ZONE,
        }
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            ObserveCommand(scope_by_stage[stage.kind], stage.traversal_id),
            AwaitObservationEffect(),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_observation_zone_drive(self, plan, progress, stage):
        """生成普通长边驶入观察区的固定距离前进动作。"""

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "观察区前进阶段缺少巡航标识"
            )
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(
                stage.traversal_id,
                self._choreographer._profile.initial_observation_advance_mm,
                DrivePurpose.TO_OBSERVATION_ZONE,
            ),
            AdvanceOnTraversalEffect(
                stage.traversal_id, self._choreographer._profile.initial_observation_advance_mm
            ),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_next_center_drive(self, plan, progress, stage):
        """按当前校正位置生成驶入下一路口中心的剩余前进动作。"""

        if stage.traversal_id is None or stage.node_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "驶向路口中心阶段缺少巡航或目标路口"
            )
        if self._choreographer._state_query.is_traversal_blocked(stage.traversal_id):
            return self._choreographer._rejected(
                ChoreographyRejectionCode.STALE_BLOCKED_TRAVERSAL, "目标巡航边已被运行时地图封锁"
            )
        from_node_id, to_node_id = self._choreographer._split_traversal_id(stage.traversal_id)
        cruise_edge = self._choreographer._topology.get_cruise_edge(from_node_id, to_node_id)
        location = self._choreographer._state_query.robot_state().location
        if isinstance(location, OnCruiseEdge) and location.traversal_id == stage.traversal_id:
            remaining_distance_mm = cruise_edge.length_mm - location.progress_mm
        elif isinstance(location, AtNode) and location.node_id == from_node_id:
            remaining_distance_mm = cruise_edge.length_mm
        else:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "当前位置与驶向路口中心阶段不匹配"
            )
        distance_mm = remaining_distance_mm + self._choreographer._profile.junction_center_entry_extra_mm
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(stage.traversal_id, distance_mm, DrivePurpose.TO_NEXT_CENTER),
            ArriveAtNodeEffect(stage.node_id, stage.traversal_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)
