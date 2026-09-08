"""把已校验的执行终局投影为导航域状态变化。"""

from navigation.contracts import (
    AdvanceOnTraversalEffect,
    ArriveAtNodeEffect,
    Action,
    CompleteTaskEffect,
    ExecutionInterrupt,
    ExecuteTaskCommand,
    ObserveCommand,
    PerceptionOutcome,
    EdgePassability,
    DriveDistanceCommand,
    ReverseDistanceCommand,
    RetraceTurnCommand,
    TurnAtJunctionCommand,
)
from navigation.domain import (
    AbsoluteMapUpdate,
    AbsoluteMapUpdateKind,
    AtNode,
    MapUpdateAuthority,
    MapObservationScope,
    OnCruiseEdge,
    ProgressSource,
    RobotState,
    TaskKind,
)
from .context import LastMotionRecord


class EventProjector:
    """集中处理动作成功后的机器人逻辑位置投影。"""

    def __init__(self, state, task_registry=None, perception_adapter=None, location_projector=None, on_map_update=None, diagnostics=None, context=None) -> None:
        """保存状态、任务、感知和地图影响回调依赖。"""

        self._state = state
        self._task_registry = task_registry
        self._perception_adapter = perception_adapter
        self._location_projector = location_projector
        self._on_map_update = on_map_update
        self._diagnostics = diagnostics if diagnostics is not None else []
        self._context = context

    def project_success(self, action: Action, interrupt: ExecutionInterrupt) -> None:
        """按动作预期效果把成功中断写入机器人状态。"""

        self._record_motion(action)

        effect = action.expected_effect
        if isinstance(effect, ArriveAtNodeEffect):
            if self._state is None:
                self._diagnostics.append("未装配导航状态，无法投影路口到达")
                return
            current = self._state.robot_state()
            self._state.replace_robot_state(
                RobotState(
                    AtNode(effect.node_id, effect.entry_traversal_id),
                    current.heading_deg,
                    current.progress_source,
                    current.pending_replan,
                )
            )

        if not isinstance(effect, AdvanceOnTraversalEffect):
            return
        if self._state is None or interrupt.odometry_delta_mm is None:
            if self._state is None:
                self._diagnostics.append("未装配导航状态，无法投影里程增量")
            return
        current = self._state.robot_state()
        from_node_id, to_node_id = effect.traversal_id.split("->", 1)
        if isinstance(current.location, OnCruiseEdge):
            if current.location.traversal_id != effect.traversal_id:
                return
            base_progress_mm = current.location.progress_mm
        elif isinstance(current.location, AtNode) and current.location.node_id == from_node_id:
            base_progress_mm = 0.0
        else:
            return
        self._state.replace_robot_state(
            RobotState(
                OnCruiseEdge(
                    effect.traversal_id,
                    from_node_id,
                    to_node_id,
                    base_progress_mm + interrupt.odometry_delta_mm,
                ),
                current.heading_deg,
                ProgressSource.ODOMETRY,
                current.pending_replan,
            )
        )

    def _record_motion(self, action: Action) -> None:
        """记录成功运动动作摘要，供撤回转弯等恢复流程使用。"""

        if self._context is None or action is None:
            return
        command = action.command
        is_turn = isinstance(command, TurnAtJunctionCommand)
        if not isinstance(command, (TurnAtJunctionCommand, DriveDistanceCommand, ReverseDistanceCommand, RetraceTurnCommand)):
            return
        trajectory_id = getattr(command, "forward_trajectory_id", None)
        if is_turn and trajectory_id is None:
            self._diagnostics.append("成功转弯缺少 forward_trajectory_id")
            return
        self._context.last_motion = LastMotionRecord(
            action_id=action.action_id,
            command_type=type(command).__name__,
            is_turn=is_turn,
            traversal_id=getattr(command, "traversal_id", None),
            forward_trajectory_id=trajectory_id,
            junction_id=(
                getattr(command, "target_traversal_id", "").split("->", 1)[0]
                if is_turn and getattr(command, "target_traversal_id", None)
                else None
            ),
        )

    def project_task(self, action: Action) -> None:
        """消费任务成功动作并写入任务与对应地图事实。"""

        if action is None:
            return
        effect = action.expected_effect
        if not isinstance(effect, CompleteTaskEffect):
            return
        if self._task_registry is None:
            self._diagnostics.append("未装配任务注册表，无法完成任务 {}".format(effect.task_id))
            return
        transition = self._task_registry.complete(effect.task_id)
        if not transition.accepted:
            self._diagnostics.append("任务完成状态转换被拒绝：{}".format(transition.reason))
            return
        completed_task = transition.after
        if completed_task is None or self._state is None:
            return
        if completed_task.kind is TaskKind.CHECK_IN:
            update = AbsoluteMapUpdate(AbsoluteMapUpdateKind.VISIT_NODE, MapUpdateAuthority.COORDINATOR, node_id=completed_task.target_id)
        elif completed_task.kind is TaskKind.CULVERT_RECON:
            update = AbsoluteMapUpdate(AbsoluteMapUpdateKind.RECON_CULVERT, MapUpdateAuthority.COORDINATOR, edge_id=completed_task.target_id)
        else:
            self._diagnostics.append("任务类型没有完成地图事实：{}".format(completed_task.kind.value))
            return
        try:
            self._state.apply_map_update(update)
        except ValueError as error:
            self._diagnostics.append("任务完成地图事实被拒绝：{}".format(error))

    def project_perception(self, action: Action, interrupt: ExecutionInterrupt) -> None:
        """消费观察终局，翻译并写入绝对地图事实及位置校正。"""

        if action is None:
            return
        if not isinstance(action.command, ObserveCommand):
            if interrupt.perception_frame is not None:
                self._diagnostics.append("非观察动作携带感知帧，已忽略")
            return
        frame = interrupt.perception_frame
        if frame is None:
            self._diagnostics.append("观察完成中断缺少 perception_frame")
            return
        if self._perception_adapter is None:
            self._diagnostics.append("未装配感知适配器，已忽略观察帧 {}".format(frame.frame_id))
            return
        translation = self._perception_adapter.translate(frame)
        if translation.frame_id != frame.frame_id:
            self._diagnostics.append("感知翻译 frame_id 不匹配，已忽略")
            return
        if translation.outcome is PerceptionOutcome.INCONCLUSIVE:
            self._diagnostics.append("感知翻译无法确认：{}".format(translation.reason))
            return
        if self._state is None and translation.edge_observations:
            self._diagnostics.append("未装配 RuntimeMap，无法应用感知地图更新")
        elif self._state is not None:
            for observation in translation.edge_observations:
                updates = []
                if observation.passability is EdgePassability.BLOCKED:
                    updates.append(AbsoluteMapUpdate(AbsoluteMapUpdateKind.BLOCK_EDGE, MapUpdateAuthority.PERCEPTION_ADAPTER, edge_id=observation.edge_id))
                elif observation.passability is EdgePassability.CLEAR:
                    scope = MapObservationScope.JUNCTION_FULL if observation.observation_scope == "junction_full" else MapObservationScope.OBSERVATION_ZONE
                    updates.append(AbsoluteMapUpdate(AbsoluteMapUpdateKind.CONFIRM_EDGE_CLEAR, MapUpdateAuthority.PERCEPTION_ADAPTER, edge_id=observation.edge_id, observation_scope=scope))
                if observation.culvert_found:
                    updates.append(AbsoluteMapUpdate(AbsoluteMapUpdateKind.DISCOVER_CULVERT, MapUpdateAuthority.PERCEPTION_ADAPTER, edge_id=observation.edge_id, culvert_distance_mm=observation.culvert_distance_mm))
                if observation.no_culvert_coverage:
                    updates.append(AbsoluteMapUpdate(AbsoluteMapUpdateKind.CONFIRM_NO_CULVERT, MapUpdateAuthority.PERCEPTION_ADAPTER, edge_id=observation.edge_id, coverage_intervals=observation.no_culvert_coverage))
                for update in updates:
                    changed = self._state.apply_map_update(update)
                    if changed and self._on_map_update is not None:
                        self._on_map_update(update)
        correction = translation.position_correction
        if correction is None:
            return
        if self._state is None or self._location_projector is None:
            self._diagnostics.append("未装配位置投影器，无法应用感知位置校正")
            return
        current_state = self._state.robot_state()
        self._state.replace_robot_state(self._location_projector.correct_from_landmark(current_state, correction))
