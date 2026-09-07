"""负责组装出发阶段的固定编排剧本。"""

# 导入编排契约，出发工厂只创建剧本和首个游标，不生成执行请求。
from navigation.contracts import (
    ChoreographyPlan,
    ChoreographyProgress,
    ChoreographySourceKind,
    ChoreographyStageKind,
    ChoreographyStartResult,
    ChoreographyStartStatus,
)


class DepartureChoreographyFactory:
    """为出发分析器提供固定启动与左转替代剧本。"""

    def __init__(self, choreographer):
        """保存共享只读依赖与阶段标识工具的宿主编排器。"""

        # 工厂通过宿主读取静态拓扑和共享的稳定标识规则，不保留可变导航状态。
        self._choreographer = choreographer

    def start_departure(self):
        """生成 START 到 J_START 后固定右转并观察的剧本。"""
        return self._build("J_START->N12", "departure:right", include_bridge=True)

    def start_left_turn(self):
        """生成撤回右转后左转并观察的剧本。"""
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
