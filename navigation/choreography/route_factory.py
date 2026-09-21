"""路线相关剧本工厂，负责构造所有非出发的不可变剧本。"""

# 导入路口位置类型，局部脱困和撤回转弯必须从路口中心开始。
from navigation.domain import AbsoluteMapUpdateKind, AtNode, TaskKind
# 导入规划层公开计划类型，用于在工厂边界拒绝未知输入。
from navigation.planning import RecoveryPlan, RecoveryStepKind, RoutePlan
# 导入所有剧本构造契约；工厂只创建这些不可变数据。
from navigation.contracts import (
    ChoreographyPlan,
    ChoreographyMapUpdateImpact,
    ChoreographyMapUpdateResult,
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

    def start_final_return(self):
        """定位 J_START 到 START 的有向启动桥，并生成最终驶入中心剧本。"""

        robot_state = self._choreographer._state_query.robot_state()
        if not isinstance(robot_state.location, AtNode) or robot_state.location.node_id != "J_START":
            return self._rejected_start("最终返场必须从 J_START 路口中心开始")
        candidates = tuple(
            edge for edge in self._choreographer._topology.outgoing_cruise_edges("J_START")
            if edge.to_junction == "START"
        )
        if len(candidates) != 1:
            return self._rejected_start("J_START 到 START 的有向返场边不唯一或不存在")
        edge = candidates[0]
        turn_stage = self._choreographer._stage(
            0, "final_turn", ChoreographyStageKind.TURN_AT_JUNCTION,
            edge.traversal_id, "J_START",
        )
        stage = self._choreographer._stage(
            1, "final_correct", ChoreographyStageKind.CORRECT_AT_JUNCTION,
            edge.traversal_id, "J_START",
        )
        drive_stage = self._choreographer._stage(
            2, "final_return", ChoreographyStageKind.DRIVE_TO_NEXT_CENTER,
            edge.traversal_id, "START",
        )
        stop_stage = self._choreographer._stage(
            3, "final_stop", ChoreographyStageKind.STOP_AT_START,
            edge.traversal_id, "START",
        )
        source_id = "final-return:J_START->START"
        choreography_id = self._choreographer._choreography_id(
            source_id, (turn_stage.stage_id, stage.stage_id, stop_stage.stage_id)
        )
        plan = ChoreographyPlan(
            choreography_id, source_id, ChoreographySourceKind.FINAL_RETURN,
            0, (edge.traversal_id,), (turn_stage, stage, drive_stage, stop_stage),
        )
        return ChoreographyStartResult(
            ChoreographyStartStatus.STARTED,
            plan,
            ChoreographyProgress(choreography_id, 0),
        )

    def handle_map_update(self, plan, progress, update):
        """检查新增地图事实是否命中尚未执行的路线，并在涵洞命中时内部替换剧本。"""

        if not self._choreographer._is_valid_progress(plan, progress) or update.edge_id is None:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.UNAFFECTED)
        traversal_id = self._find_remaining_traversal_for_edge(plan, progress, update.edge_id)
        if traversal_id is None:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.UNAFFECTED)
        if update.kind is AbsoluteMapUpdateKind.BLOCK_EDGE:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.ROUTE_BLOCKED)
        if update.kind is not AbsoluteMapUpdateKind.DISCOVER_CULVERT:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.UNAFFECTED)
        task_id = self._find_pending_culvert_task_id(update.edge_id)
        if task_id is None:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.UNAFFECTED)
        replacement = self.replace_current_traversal_with_culvert(plan, progress, traversal_id, task_id)
        if replacement.status is not ChoreographyStartStatus.STARTED:
            return ChoreographyMapUpdateResult(ChoreographyMapUpdateImpact.UNAFFECTED)
        return ChoreographyMapUpdateResult(
            ChoreographyMapUpdateImpact.CULVERT_REPLACED,
            replacement.plan,
            replacement.progress,
        )

    def _find_remaining_traversal_for_edge(self, plan, progress, edge_id):
        """在未完成阶段中查找包含指定物理边的第一条有向巡航边。"""

        checked = set()
        for stage in plan.stages[progress.stage_index:]:
            traversal_id = stage.traversal_id
            if traversal_id is None or traversal_id in checked:
                continue
            checked.add(traversal_id)
            from_node_id, to_node_id = self._choreographer._split_traversal_id(traversal_id)
            cruise_edge = self._choreographer._topology.get_cruise_edge(from_node_id, to_node_id)
            if edge_id in cruise_edge.physical_edge_ids:
                return traversal_id
        return None

    def _find_pending_culvert_task_id(self, edge_id):
        """从只读任务注册表找到该物理边唯一尚未执行的涵洞任务。"""

        task_registry = self._choreographer._task_registry
        if task_registry is None:
            return None
        for task in task_registry.pending_tasks():
            if task.kind is TaskKind.CULVERT_RECON and task.target_id == edge_id:
                return task.task_id
        return None

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
        # 只替换当前游标之后的阶段，保留当前边已经成功完成的观察或转弯阶段。
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
        if not any(
            kind in completed_kinds
            for kind in (
                ChoreographyStageKind.DRIVE_TO_NEXT_CENTER,
                ChoreographyStageKind.DRIVE_TO_TURN_WINDOW,
            )
        ):
            replacements.append(self._choreographer._stage(
                0, "culvert_correct", ChoreographyStageKind.CORRECT_AT_JUNCTION,
                traversal_id, from_node_id,
            ))
            replacements.append(self._choreographer._stage(
                0, "culvert_turn_window", ChoreographyStageKind.DRIVE_TO_TURN_WINDOW,
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
        # 新指针指向替换段首个阶段，且此前已完成阶段的数量保持不变。
        insertion_index = progress.stage_index
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
                robot_state.world_pose.yaw_deg,
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

    def start_retrace_turn(self, source_action_id, retrace_trajectory_id=None):
        """生成引用原前向动作的同轨迹撤回剧本。"""

        if not source_action_id or not retrace_trajectory_id:
            return ChoreographyStartResult(
                ChoreographyStartStatus.REJECTED,
                rejection=ChoreographyRejection(
                    ChoreographyRejectionCode.MISSING_RETRACE_SOURCE,
                    "撤回转弯缺少原动作或反向轨迹标识",
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
            base_stage.node_id,
            source_action_id=source_action_id,
            retrace_trajectory_id=retrace_trajectory_id,
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
            # 第一步：为第一条边插入转向阶段，保证机器人先对准进入边的方向。
            # 若机器人已对准（heading_difference 为 0），StageCompiler 会自动跳过该虚拟阶段。
            if step_index == 0:
                stages.append(self._choreographer._stage(
                    step_index, "align_first", ChoreographyStageKind.TURN_AT_JUNCTION,
                    step.traversal_id, step.from_junction,
                ))
            stages.append(self._choreographer._stage(
                step_index, "pre", ChoreographyStageKind.OBSERVE_PRE_ENTRY,
                step.traversal_id, step.from_junction,
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
                        step_index, "zone_correct", ChoreographyStageKind.CORRECT_AT_OBSERVATION_ZONE,
                        step.traversal_id, None,
                    ),
                    self._choreographer._stage(
                        step_index, "zone_observe", ChoreographyStageKind.OBSERVE_AT_ZONE,
                        step.traversal_id, None,
                    ),
                ))
            stages.append(self._choreographer._stage(
                step_index, "turn_window", ChoreographyStageKind.DRIVE_TO_TURN_WINDOW,
                step.traversal_id, step.to_junction,
            ))
            if step_index + 1 < len(route.steps):
                next_step = route.steps[step_index + 1]
                stages.extend((
                    self._choreographer._stage(
                        step_index, "turn", ChoreographyStageKind.TURN_AT_JUNCTION,
                        next_step.traversal_id, step.to_junction,
                    ),
                    self._choreographer._stage(
                        step_index, "post", ChoreographyStageKind.OBSERVE_POST_TURN,
                        next_step.traversal_id, step.to_junction,
                    ),
                    self._choreographer._stage(
                        step_index, "junction_correct", ChoreographyStageKind.CORRECT_AT_JUNCTION,
                        next_step.traversal_id, step.to_junction,
                    ),
                ))
        selected_goal = getattr(route, "selected_goal", None)
        task_id = getattr(selected_goal, "task_id", None)
        if route.steps and task_id is not None:
            # 有任务：交给任务系统在目标路口完成，剧本以 EXECUTE_TASK 收尾。
            final_step = route.steps[-1]
            stages.append(ChoreographyStage(
                "stage:task:{}".format(task_id),
                ChoreographyStageKind.EXECUTE_TASK,
                final_step.traversal_id,
                final_step.to_junction,
                task_id,
            ))
        # 无任务目标（如探索目标）无需追加动作：
        # 最后一步 DRIVE_TO_TURN_WINDOW 的 effect 已经把逻辑位置吸附为 AtNode。
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