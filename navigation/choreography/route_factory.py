"""路线相关剧本工厂，负责构造所有非出发的不可变剧本。"""

# 导入路口位置类型，局部脱困和撤回转弯必须从路口中心开始。
from navigation.domain import AtNode
# 导入规划层公开计划类型，用于在工厂边界拒绝未知输入。
from navigation.planning import RecoveryPlan, RecoveryStepKind, RoutePlan
# 导入所有剧本构造契约；工厂只创建这些不可变数据。
from navigation.contracts import (
    ChoreographyPlan,
    ChoreographyProgress,
    ChoreographyRejection,
    ChoreographyRejectionCode,
    ChoreographySourceKind,
    ChoreographyStage,
    ChoreographyStageKind,
    ChoreographyStartResult,
    ChoreographyStartStatus,
    TurnDirection,
)


class RouteChoreographyFactory:
    """承载正常路线、恢复路线与局部恢复剧本的统一入口。"""

    def __init__(self, choreographer):
        """保存实际剧本构造器的共享只读依赖。"""

        # 工厂通过宿主查询静态拓扑及实时只读位姿，不保存可变导航状态。
        self._choreographer = choreographer

    def start(self, route_or_recovery):
        """根据规划结果类型选择正常路线或倒车恢复构造流程。"""

        # 仅允许规划层冻结的两类计划进入编排层。
        if isinstance(route_or_recovery, RoutePlan):
            return self._build_route(route_or_recovery)
        if isinstance(route_or_recovery, RecoveryPlan):
            return self._build_recovery(route_or_recovery)
        raise TypeError("start 只接受 RoutePlan 或 RecoveryPlan")

    def replace_current_traversal_with_culvert(self, plan, progress, traversal_id, task_id):
        """将活动路线指定巡航边替换为涵洞探索剧本。"""

        if not self._choreographer._is_valid_progress(plan, progress):
            return self._rejected_start("涵洞替换的流程指针不属于当前剧本")
        if progress.stage_index >= len(plan.stages) or not any(
            stage.traversal_id == traversal_id for stage in plan.stages[progress.stage_index:]
        ):
            return self._rejected_start("涵洞替换目标已完成或当前剧本没有剩余阶段")
        if traversal_id not in plan.route_steps:
            return self._rejected_start("涵洞替换目标不在活动路线中")
        from_node_id, to_node_id = self._choreographer._split_traversal_id(traversal_id)
        # 校验替换边存在于静态拓扑，禁止借替换接口伪造赛道道路。
        self._choreographer._topology.get_cruise_edge(from_node_id, to_node_id)
        completed_stages = list(plan.stages[:progress.stage_index])
        remaining_stages = [
            stage for stage in plan.stages[progress.stage_index:]
            if stage.traversal_id != traversal_id
        ]
        completed_kinds = {
            stage.kind for stage in completed_stages if stage.traversal_id == traversal_id
        }
        replacements = []
        if ChoreographyStageKind.OBSERVE_PRE_ENTRY not in completed_kinds:
            replacements.append(self._choreographer._stage(
                0, "culvert_pre", ChoreographyStageKind.OBSERVE_PRE_ENTRY,
                traversal_id, from_node_id,
            ))
        if ChoreographyStageKind.TURN_AT_JUNCTION not in completed_kinds:
            replacements.append(self._choreographer._stage(
                0, "culvert_turn", ChoreographyStageKind.TURN_AT_JUNCTION,
                traversal_id, from_node_id,
            ))
        if ChoreographyStageKind.OBSERVE_POST_TURN not in completed_kinds:
            replacements.append(self._choreographer._stage(
                0, "culvert_post", ChoreographyStageKind.OBSERVE_POST_TURN,
                traversal_id, from_node_id,
            ))
        if ChoreographyStageKind.DRIVE_TO_NEXT_CENTER not in completed_kinds:
            replacements.append(self._choreographer._stage(
                0, "culvert_center", ChoreographyStageKind.DRIVE_TO_NEXT_CENTER,
                traversal_id, to_node_id,
            ))
        # 涵洞任务在抵达另一端路口后执行，运动收口仍统一为路口中心。
        replacements.append(ChoreographyStage(
            "stage:0:culvert_task:{}".format(task_id),
            ChoreographyStageKind.EXECUTE_TASK,
            traversal_id,
            to_node_id,
            task_id,
        ))
        insertion_index = len(completed_stages)
        new_stages = tuple(completed_stages + replacements + remaining_stages)
        choreography_id = self._choreographer._choreography_id(
            plan.source_plan_id, tuple(stage.stage_id for stage in new_stages)
        )
        replacement_plan = ChoreographyPlan(
            choreography_id,
            plan.source_plan_id,
            plan.source_kind,
            plan.source_map_version,
            plan.route_steps,
            new_stages,
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED,
            replacement_plan,
            ChoreographyProgress(choreography_id, insertion_index),
        )

    def start_junction_escape(self, side):
        """生成指定侧支路的局部脱困剧本。"""

        robot_state = self._choreographer._state_query.robot_state()
        if not isinstance(robot_state.location, AtNode):
            return self._rejected_start("局部脱困必须从路口中心开始")
        traversal_id = None
        for cruise_edge in self._choreographer._topology.outgoing_cruise_edges(
            robot_state.location.node_id
        ):
            difference = self._choreographer._turn_difference_deg(
                cruise_edge.from_junction,
                cruise_edge.to_junction,
                robot_state.heading_deg,
            )
            if side is TurnDirection.LEFT and abs(difference - 90.0) < 0.000001:
                traversal_id = cruise_edge.traversal_id
                break
            if side is TurnDirection.RIGHT and abs(difference + 90.0) < 0.000001:
                traversal_id = cruise_edge.traversal_id
                break
        if traversal_id is None:
            return ChoreographyStartResult(
                ChoreographyStartStatus.REJECTED,
                rejection=ChoreographyRejection(
                    ChoreographyRejectionCode.MISSING_TURN_CONFIGURATION,
                    "指定侧支路不存在已标定的九十度前向转弯",
                ),
            )
        stages = (
            self._choreographer._stage(
                0, "escape_turn", ChoreographyStageKind.TURN_AT_JUNCTION,
                traversal_id, robot_state.location.node_id,
            ),
            self._choreographer._stage(
                0, "escape_observe", ChoreographyStageKind.OBSERVE_POST_TURN,
                traversal_id, robot_state.location.node_id,
            ),
        )
        source_id = "junction-escape:{}".format(robot_state.location.node_id)
        choreography_id = self._choreographer._choreography_id(
            source_id, tuple(stage.stage_id for stage in stages)
        )
        plan = ChoreographyPlan(
            choreography_id, source_id, ChoreographySourceKind.JUNCTION_RECOVERY,
            0, (traversal_id,), stages,
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0)
        )

    def start_retrace_turn(self, source_action_id):
        """生成引用原前向动作的同轨迹撤回剧本。"""

        if not source_action_id:
            return ChoreographyStartResult(
                ChoreographyStartStatus.REJECTED,
                rejection=ChoreographyRejection(
                    ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                    "撤回转弯缺少原前向动作标识",
                ),
            )
        robot_state = self._choreographer._state_query.robot_state()
        if not isinstance(robot_state.location, AtNode):
            return self._rejected_start("撤回转弯必须从路口中心开始")
        base_stage = self._choreographer._stage(
            0, "retrace", ChoreographyStageKind.RETRACE_TURN,
            None, robot_state.location.node_id,
        )
        stage = ChoreographyStage(
            base_stage.stage_id, base_stage.kind, base_stage.traversal_id,
            base_stage.node_id, source_action_id=source_action_id,
        )
        source_id = "junction-retrace:{}".format(robot_state.location.node_id)
        choreography_id = self._choreographer._choreography_id(
            source_id, (stage.stage_id, source_action_id)
        )
        plan = ChoreographyPlan(
            choreography_id, source_id, ChoreographySourceKind.JUNCTION_RECOVERY,
            0, (), (stage,),
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0)
        )

    def _build_route(self, route):
        """把正常路线的每条巡航边展开为观察、转弯和巡航阶段。"""

        stages = []
        for step_index, step in enumerate(route.steps):
            cruise_edge = self._choreographer._topology.get_cruise_edge(
                step.from_junction, step.to_junction
            )
            stages.extend((
                self._choreographer._stage(
                    step_index, "pre", ChoreographyStageKind.OBSERVE_PRE_ENTRY,
                    step.traversal_id, step.from_junction,
                ),
                self._choreographer._stage(
                    step_index, "turn", ChoreographyStageKind.TURN_AT_JUNCTION,
                    step.traversal_id, step.from_junction,
                ),
                self._choreographer._stage(
                    step_index, "post", ChoreographyStageKind.OBSERVE_POST_TURN,
                    step.traversal_id, step.from_junction,
                ),
            ))
            if self._choreographer._needs_observation_zone(
                cruise_edge.road_kind, cruise_edge.length_mm
            ):
                stages.extend((
                    self._choreographer._stage(
                        step_index, "zone_drive", ChoreographyStageKind.DRIVE_TO_OBSERVATION_ZONE,
                        step.traversal_id, None,
                    ),
                    self._choreographer._stage(
                        step_index, "zone_observe", ChoreographyStageKind.OBSERVE_AT_ZONE,
                        step.traversal_id, None,
                    ),
                ))
            stages.append(self._choreographer._stage(
                step_index, "center", ChoreographyStageKind.DRIVE_TO_NEXT_CENTER,
                step.traversal_id, step.to_junction,
            ))
        frozen_stages = tuple(stages)
        choreography_id = self._choreographer._choreography_id(
            route.plan_id, tuple(stage.stage_id for stage in frozen_stages)
        )
        plan = ChoreographyPlan(
            choreography_id, route.plan_id, ChoreographySourceKind.NORMAL,
            route.map_version, tuple(step.traversal_id for step in route.steps), frozen_stages,
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0)
        )

    def _build_recovery(self, recovery):
        """将恢复规划器选定的 BACKTRACK 步骤展开为倒车阶段。"""

        stages = []
        for step_index, step in enumerate(recovery.steps):
            if step.kind is not RecoveryStepKind.BACKTRACK:
                raise ValueError("当前编排器只支持 BACKTRACK 恢复步骤")
            stages.append(self._choreographer._stage(
                step_index, "reverse", ChoreographyStageKind.REVERSE_TO_SAFE_JUNCTION,
                step.traversal_id, step.from_junction, safe_node_id=step.to_junction,
            ))
        frozen_stages = tuple(stages)
        choreography_id = self._choreographer._choreography_id(
            recovery.plan_id, tuple(stage.stage_id for stage in frozen_stages)
        )
        plan = ChoreographyPlan(
            choreography_id, recovery.plan_id, ChoreographySourceKind.RECOVERY,
            0, tuple(step.traversal_id for step in recovery.steps), frozen_stages,
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED, plan, ChoreographyProgress(choreography_id, 0)
        )

    @staticmethod
    def _rejected_start(reason):
        """集中构造构建前置条件不成立时的剧本拒绝结果。"""

        return ChoreographyStartResult(
            ChoreographyStartStatus.REJECTED,
            rejection=ChoreographyRejection(ChoreographyRejectionCode.INVALID_PROGRESS, reason),
        )
