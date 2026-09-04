"""把已校验的执行终局投影为导航域状态变化。"""

from navigation.contracts import AdvanceOnTraversalEffect, ArriveAtNodeEffect, Action, ExecutionInterrupt
from navigation.domain import AtNode, OnCruiseEdge, ProgressSource, RobotState


class EventProjector:
    """集中处理动作成功后的机器人逻辑位置投影。"""

    def __init__(self, state) -> None:
        """保存统一导航状态层写入口。"""

        self._state = state

    def project_success(self, action: Action, interrupt: ExecutionInterrupt) -> None:
        """按动作预期效果把成功中断写入机器人状态。"""

        effect = action.expected_effect
        if isinstance(effect, ArriveAtNodeEffect):
            current = self._state.robot_state()
            self._state.replace_robot_state(
                RobotState(
                    AtNode(effect.node_id, effect.entry_traversal_id),
                    current.heading_deg,
                    current.progress_source,
                    current.pending_replan,
                )
            )
            return
        if not isinstance(effect, AdvanceOnTraversalEffect):
            return
        if interrupt.odometry_delta_mm is None:
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

