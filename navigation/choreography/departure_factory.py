"""负责组装出发阶段的固定编排剧本。"""

# 导入编排契约，出发工厂只创建剧本和首个游标，不生成执行请求。
from navigation.contracts import (
    ChoreographyPlan,
    ChoreographyProgress,
    ChoreographySourceKind,
    ChoreographyStageKind,
    ChoreographyStartResult,
    ChoreographyStartStatus,
    TurnDirection,
)


class DepartureChoreographyFactory:
    """为出发分析器提供固定启动与左转替代剧本。"""

    def __init__(self, choreographer):
        """保存共享只读依赖与阶段标识工具的宿主编排器。"""

        # 工厂通过宿主读取静态拓扑和共享的稳定标识规则，不保留可变导航状态。
        self._choreographer = choreographer

    def start_departure(self):
        """固定出发：右转、直行160mm、左转、打卡N1。"""
        right_traversal_id = "J_START->N1"
        left_traversal_id = "N1->T1_R"
        stages = (
            self._choreographer._stage(
                0, "right", ChoreographyStageKind.TURN_AT_JUNCTION,
                right_traversal_id, "J_START", requested_turn_direction=TurnDirection.RIGHT,
            ),
            self._choreographer._stage(
                1, "forward160", ChoreographyStageKind.DRIVE_TO_TURN_WINDOW,
                right_traversal_id, None,
            ),
            self._choreographer._stage(
                2, "left", ChoreographyStageKind.TURN_AT_JUNCTION,
                left_traversal_id, "N1", requested_turn_direction=TurnDirection.LEFT,
            ),
            self._choreographer._stage(
                3, "check_n1", ChoreographyStageKind.EXECUTE_TASK,
                left_traversal_id, "N1", task_id="check-in-N1",
            ),
        )
        choreography_id = self._choreographer._choreography_id(
            "departure:fixed", tuple(stage.stage_id for stage in stages)
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED,
            ChoreographyPlan(
                choreography_id, "departure:fixed", ChoreographySourceKind.DEPARTURE,
                0, (right_traversal_id, left_traversal_id), stages,
            ),
            ChoreographyProgress(choreography_id, 0),
        )

    def start_left_turn(self):
        """生成兼容旧调用方的左转替代剧本。"""
        return self._build("J_START->N1", "departure:left", include_bridge=False)

    def _build(self, first_traversal_id, source_id, include_bridge):
        """按启动桥、固定转向和转后观察顺序组装不可变出发剧本。"""

        # 出发桥固定连接起始区与正式赛道入口，只有首次出发需要驶过它。
        bridge_traversal_id = "START->J_START"
        from_node_id, _ = self._choreographer._split_traversal_id(first_traversal_id)
        stages = []
        if include_bridge:
            stages.append(
                self._choreographer._stage(
                    0,
                    "bridge",
                    ChoreographyStageKind.DRIVE_TO_NEXT_CENTER,
                    bridge_traversal_id,
                    "J_START",
                )
            )
        offset = len(stages)
        stages.extend(
            (
                self._choreographer._stage(
                    offset,
                    "turn",
                    ChoreographyStageKind.TURN_AT_JUNCTION,
                    first_traversal_id,
                    from_node_id,
                ),
                self._choreographer._stage(
                    offset + 1,
                    "observe",
                    ChoreographyStageKind.OBSERVE_POST_TURN,
                    first_traversal_id,
                    from_node_id,
                ),
            )
        )
        frozen_stages = tuple(stages)
        choreography_id = self._choreographer._choreography_id(
            source_id, tuple(stage.stage_id for stage in frozen_stages)
        )
        plan = ChoreographyPlan(
            choreography_id,
            source_id,
            ChoreographySourceKind.DEPARTURE,
            0,
            tuple(([bridge_traversal_id] if include_bridge else []) + [first_traversal_id]),
            frozen_stages,
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED,
            plan,
            ChoreographyProgress(choreography_id, 0),
        )
