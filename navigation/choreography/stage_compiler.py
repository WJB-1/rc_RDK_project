"""将剧本单一阶段翻译为类型化动作的编译器。"""
from typing import Optional

# 导入阶段与结果枚举，编译器据此决定当前游标的动作翻译分支。
from navigation.contracts import (
    Action,
    AdvanceOnTraversalEffect,
    AlignToTraversalEffect,
    AdvanceAtNodeEffect,
    SequentialEffect,
    ArriveAtNodeEffect,
    AwaitObservationEffect,
    ChoreographyAdvanceResult,
    ChoreographyAdvanceStatus,
    ChoreographyProgress,
    ChoreographyRejectionCode,
    ChoreographyStageKind,
    CompleteTaskEffect,
    CorrectPoseCommand,
    DriveDistanceCommand,
    DrivePurpose,
    ExecuteTaskCommand,
    ObserveCommand,
    ObservationScope,
    RetraceTurnCommand,
    RetraceTurnEffect,
    ReverseDistanceCommand,
    StopCommand,
    StopEffect,
    TurnAtJunctionCommand,
    TurnDirection,
    TeleportToNodeEffect,
)
# 导入当前位置类型，用于严格校验驶入路口和倒车动作的起始位置。
from navigation.domain import AtNode, OnCruiseEdge

from .profile import *


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
        if stage.kind is ChoreographyStageKind.DRIVE_TO_TURN_WINDOW:
            return self._ready_turn_window_drive(plan, progress, stage)
        if stage.kind in (ChoreographyStageKind.CORRECT_AT_JUNCTION, ChoreographyStageKind.CORRECT_AT_OBSERVATION_ZONE):
            return self._ready_correction(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.DRIVE_TO_NEXT_CENTER:
            return self._ready_next_center_drive(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.REVERSE_TO_SAFE_JUNCTION:
            return self._ready_reverse_to_safe_junction(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.RETRACE_TURN:
            return self._ready_retrace_turn(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.EXECUTE_TASK:
            return self._ready_execute_task(plan, progress, stage)
        if stage.kind is ChoreographyStageKind.STOP_AT_START:
            return self._ready_stop_at_start(plan, progress, stage)
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

    def _ready_stop_at_start(self, plan, progress, stage):
        """确认机器人已经抵达 START 中心后生成最终停车动作。"""

        location = self._choreographer._state_query.robot_state().location
        if not isinstance(location, AtNode) or location.node_id != "START":
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS,
                "最终停车必须在 START 中心执行",
            )
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            StopCommand("已驶入 START 中心，结束返场"),
            StopEffect(),
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
        if stage.retrace_trajectory_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                "同轨迹撤回阶段缺少已标定反向轨迹",
            )

        # ---- 入口边 = 当前 AtNode 的 entry_traversal_id ----
        robot_state = self._choreographer._state_query.robot_state()
        location = robot_state.location
        if not isinstance(location, AtNode) or location.entry_traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                "撤回转弯无法确定入口边",
            )
        entry_traversal_id = location.entry_traversal_id

        # ---- 出口边 = 最近一次成功前向转弯的 target_traversal_id ----
        last_motion = self._choreographer._context.last_motion
        if last_motion is None or last_motion.target_traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                "撤回转弯缺少前向转弯的目标巡航边记录",
            )
        exit_traversal_id = last_motion.target_traversal_id

        # ---- 三子效果：后退到中心 → 转向 → 后退到撤回窗口 ----
        effect = SequentialEffect((
            # 子 1：从落点沿出口边后退到路口中心
            AdvanceAtNodeEffect(exit_traversal_id, -TAIL_ANCHOR_FROM_CENTER_MM),
            # 子 2：朝向改为入口边方向
            AlignToTraversalEffect(entry_traversal_id),
            # 子 3：从路口中心沿入口边后退到撤回窗口
            AdvanceAtNodeEffect(entry_traversal_id, -RETRACE_ANCHOR_FROM_CENTER_MM),
        ))

        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            RetraceTurnCommand(stage.source_action_id, stage.retrace_trajectory_id),
            effect,
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

    def _ready_correction(self, plan, progress, stage):
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            CorrectPoseCommand(),
            AwaitObservationEffect(),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_observation_zone_drive(self, plan, progress, stage):
        """生成普通长边驶入观察区的固定距离前进动作。"""

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "观察区前进阶段缺少巡航标识"
            )
        distance_mm = self._remaining_distance(stage.traversal_id)
        distance_mm -= OBSERVATION_ZONE_FROM_CENTER_MM
        if distance_mm < 0.0:
            distance_mm = 0.0
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(
                stage.traversal_id,
                distance_mm,
                DrivePurpose.TO_OBSERVATION_ZONE,
            ),
            AdvanceOnTraversalEffect(
                stage.traversal_id, distance_mm
            ),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _ready_turn_window_drive(self, plan, progress, stage):
        """生成驶向目标路口端点前转弯窗口的前进动作。

        转弯窗口统一归类为 AtNode：
        - 逻辑位置吸附到目标路口；
        - 物理位置沿当前边推进到转弯窗口；
        - 不再区分下一条是不是任务。
        """

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "提前转弯阶段缺少巡航标识"
            )
        if self._choreographer._state_query.is_traversal_blocked(stage.traversal_id):
            return self._choreographer._rejected(
                ChoreographyRejectionCode.STALE_BLOCKED_TRAVERSAL, "目标巡航边已被运行时地图封锁"
            )

        # 计算前进距离：从当前位置到目标路口中心前的转弯窗口。
        if stage.node_id is None:
            # 出发剧本：从 J_START 到 N1 的固定前进距离。
            distance_mm = DEPARTURE_FORWARD_MM
        else:
            distance_mm = self._remaining_distance(stage.traversal_id)
            distance_mm -= TURN_WINDOW_FROM_CENTER_MM
        if distance_mm < 0.0:
            distance_mm = 0.0

        # 目标节点：优先用 stage.node_id，缺失时从 traversal_id 推导终点。
        if stage.node_id is not None:
            target_node_id = stage.node_id
        else:
            _, target_node_id = self._choreographer._split_traversal_id(stage.traversal_id)

        # 统一 effect：物理位置推进到转弯窗口，逻辑位置吸附为 AtNode。
        effect = SequentialEffect((
            AdvanceAtNodeEffect(stage.traversal_id, distance_mm),
            ArriveAtNodeEffect(target_node_id, stage.traversal_id),
        ))

        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(stage.traversal_id, distance_mm, DrivePurpose.TO_TURN_WINDOW),
            effect,
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)
    
    def _remaining_distance(self, traversal_id):
        """按当前物理位姿推算沿指定巡航边到终点的剩余距离。

        以拓扑节点坐标为参照，把 `world_pose` 投影到 from_node → to_node 的
        有向线段上，得到已走过的等效距离，再从边长中减去。
        投影比例夹紧到 [0, 1]，保证结果始终落在 [0, length_mm]。
        """

        topology = self._choreographer._topology
        from_node_id, to_node_id = self._choreographer._split_traversal_id(traversal_id)
        cruise_edge = topology.get_cruise_edge(from_node_id, to_node_id)
        robot_state = self._choreographer._state_query.robot_state()

        from_node = topology.get_node(from_node_id)
        to_node = topology.get_node(to_node_id)
        dx = to_node.x_mm - from_node.x_mm
        dy = to_node.y_mm - from_node.y_mm
        length_sq = dx * dx + dy * dy
        if length_sq <= 0.0:
            return cruise_edge.length_mm

        # 把 world_pose 投影到从起点中心到终点中心的有向线段上。
        projection = (
            (robot_state.world_pose.x_mm - from_node.x_mm) * dx
            + (robot_state.world_pose.y_mm - from_node.y_mm) * dy
        ) / length_sq

        # 夹紧到 [0, 1]，避免越界造成负剩余或超长剩余。
        clamped = max(0.0, min(1.0, projection))
        projected_mm = clamped * cruise_edge.length_mm

        return max(0.0, cruise_edge.length_mm - projected_mm)

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

        # 已在目标路口：逻辑位置已吸附为 AtNode，无需再走一段，直接跳过。
        if isinstance(location, AtNode) and location.node_id == stage.node_id:
            return self.compile_next(
                plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1)
            )

        if isinstance(location, OnCruiseEdge) and location.traversal_id == stage.traversal_id:
            remaining_distance_mm = cruise_edge.length_mm - location.progress_mm
        elif isinstance(location, AtNode) and location.node_id == from_node_id:
            remaining_distance_mm = cruise_edge.length_mm
        else:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.INVALID_PROGRESS, "当前位置与驶向路口中心阶段不匹配"
            )
        distance_mm = remaining_distance_mm + JUNCTION_CENTER_ENTRY_EXTRA_MM
        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(stage.traversal_id, distance_mm, DrivePurpose.TO_NEXT_CENTER),
            ArriveAtNodeEffect(stage.node_id, stage.traversal_id),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _compile_turn_or_skip(self, plan, progress, stage):
        """按实时朝向生成左转或右转；直行时补齐到路口中心。"""

        if stage.traversal_id is None:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                "转弯阶段缺少目标巡航标识",
            )

        from_node_id, to_node_id = self._choreographer._split_traversal_id(stage.traversal_id)
        robot_state = self._choreographer._state_query.robot_state()
        node_id = stage.node_id or from_node_id
        exit_traversal_id = stage.traversal_id
        location = robot_state.location

        # ---- 启动转弯特殊处理 ----
        if (
            isinstance(location, AtNode)
            and location.node_id == "START"
            and node_id == "J_START"
        ):
            return self._compile_start_turn(plan, progress, stage)

        # ---- 推断入口边 ----
        entry_traversal_id: Optional[str] = None
        if isinstance(location, OnCruiseEdge):
            entry_traversal_id = location.traversal_id
        elif isinstance(location, AtNode) and location.entry_traversal_id is not None:
            entry_traversal_id = location.entry_traversal_id
        elif isinstance(location, AtNode) and location.node_id == node_id:
            entry_traversal_id = None
        else:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                "转弯无法确定入口边",
            )

        # ---- 计算方向 ----
        heading_difference = self._choreographer._turn_difference_deg(
            from_node_id, to_node_id, robot_state.world_pose.yaw_deg,
        )
        if stage.requested_turn_direction is not None:
            direction = stage.requested_turn_direction
        elif abs(heading_difference) < 0.000001:
            # 已对齐 → 直行：先把物理位置补到路口中心，再让后续阶段从中心出发。
            return self._compile_straight_through(
                plan, progress, stage, entry_traversal_id,
            )
        if stage.requested_turn_direction is None and abs(abs(heading_difference) - 180.0) < 0.000001:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.FORBIDDEN_UTURN,
                "目标巡航要求绝对禁止的一百八十度原地掉头",
            )
        if stage.requested_turn_direction is not None:
            pass
        elif abs(heading_difference - 90.0) < 0.000001:
            direction = TurnDirection.LEFT
        elif abs(heading_difference + 90.0) < 0.000001:
            direction = TurnDirection.RIGHT
        else:
            return self._choreographer._rejected(
                ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                "当前朝向与目标巡航无法组成已标定的直行或九十度转弯",
            )

        # ---- 转弯：Teleport + Arrive + Align + Advance ----
        # ArriveAtNodeEffect 第二个参数应当是"进入该节点的入口边"，
        # 否则会污染 AtNode.entry_traversal_id 并让后续直行补偿误判当前路口。
        if entry_traversal_id is not None:
            effect = SequentialEffect((
                TeleportToNodeEffect(node_id),
                ArriveAtNodeEffect(node_id, entry_traversal_id),
                AlignToTraversalEffect(exit_traversal_id),
                AdvanceAtNodeEffect(exit_traversal_id, TAIL_ANCHOR_FROM_CENTER_MM),
            ))
        else:
            # 无已知入口边时保留原有语义，避免丢失到达记录。
            effect = SequentialEffect((
                TeleportToNodeEffect(node_id),
                ArriveAtNodeEffect(node_id, exit_traversal_id),
                AlignToTraversalEffect(exit_traversal_id),
                AdvanceAtNodeEffect(exit_traversal_id, TAIL_ANCHOR_FROM_CENTER_MM),
            ))

        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            TurnAtJunctionCommand(
                direction,
                exit_traversal_id,
                forward_trajectory_id="turn:{}:forward".format(direction.value),
                retrace_trajectory_id="turn:{}:retrace".format(direction.value),
            ),
            effect,
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _compile_start_turn(self, plan, progress, stage):
        """启动右转：一次完成 START → J_START->N1 边上目标点的物理转移。

        注意：这里写入的 ArriveAtNodeEffect 第二个参数是"进入 J_START 的入口边"，
        也就是 START->J_START，而不是本阶段要驶入的出口边 J_START->N1。
        之前误用 exit_traversal_id 会污染 AtNode.entry_traversal_id，
        导致后续直行补偿误判当前路口，直接驶向下个路口中心。
        """

        exit_traversal_id = stage.traversal_id
        from_node_id, to_node_id = self._choreographer._split_traversal_id(exit_traversal_id)
        cruise_edge = self._choreographer._topology.get_cruise_edge(from_node_id, to_node_id)
        land_dist = cruise_edge.length_mm - START_TURN_LAND_FROM_N1_MM
        if land_dist < 0.0:
            land_dist = 0.0

        direction = stage.requested_turn_direction or TurnDirection.RIGHT

        # 进入 J_START 的真正入口边是 START->J_START；若拓扑上不存在，则退回不写入。
        entry_traversal_id: Optional[str] = None
        try:
            self._choreographer._topology.get_cruise_edge("START", "J_START")
            entry_traversal_id = "START->J_START"
        except Exception:
            entry_traversal_id = None

        if entry_traversal_id is not None:
            effect = SequentialEffect((
                TeleportToNodeEffect("J_START"),
                ArriveAtNodeEffect("J_START", entry_traversal_id),
                AlignToTraversalEffect(exit_traversal_id),
                AdvanceAtNodeEffect(exit_traversal_id, land_dist),
            ))
        else:
            effect = SequentialEffect((
                TeleportToNodeEffect("J_START"),
                AlignToTraversalEffect(exit_traversal_id),
                AdvanceAtNodeEffect(exit_traversal_id, land_dist),
            ))

        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            TurnAtJunctionCommand(
                direction,
                exit_traversal_id,
                forward_trajectory_id="turn:{}:forward".format(direction.value),
                retrace_trajectory_id="turn:{}:retrace".format(direction.value),
            ),
            effect,
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)

    def _compile_straight_through(self, plan, progress, stage, entry_traversal_id):
        """直行时把物理位置补到入口边的终点路口中心，逻辑位置吸附为 AtNode。

        - 入口边为空：说明已在路口中心，直接跳过该虚拟阶段。
        - 入口边终点不是当前所在路口：说明入口边被污染（例如写成了出口边），
          直接跳过该虚拟阶段，避免把机器人送向下个路口中心。
        - 剩余距离小于 1mm：容差，直接跳过。
        - 其余情况发一个 DriveDistanceCommand，
          从当前位置沿入口边推进到终点路口中心，同时吸附 AtNode。
        """

        if entry_traversal_id is None:
            return self.compile_next(
                plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1)
            )

        # 直行的终点就是入口边的终点路口，不用 stage.node_id。
        entry_from_id, entry_to_id = self._choreographer._split_traversal_id(entry_traversal_id)

        # 防御性校验：入口边的终点必须是当前所在路口。
        # 若上游误把出口边写入 entry_traversal_id，此处会命中并跳过，
        # 从而避免机器人被直接送到下个路口中心。
        location = self._choreographer._state_query.robot_state().location
        if isinstance(location, AtNode) and location.node_id != entry_to_id:
            return self.compile_next(
                plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1)
            )

        distance_mm = self._remaining_distance(entry_traversal_id)
        if distance_mm < 1.0:
            return self.compile_next(
                plan, ChoreographyProgress(plan.choreography_id, progress.stage_index + 1)
            )

        action = Action(
            self._choreographer._action_id(plan, progress.stage_index),
            DriveDistanceCommand(
                entry_traversal_id,
                distance_mm,
                DrivePurpose.TO_NEXT_CENTER,
            ),
            SequentialEffect((
                AdvanceAtNodeEffect(entry_traversal_id, distance_mm),
                ArriveAtNodeEffect(entry_to_id, entry_traversal_id),
            )),
        )
        return self._choreographer._ready(action, plan, progress.stage_index + 1)
