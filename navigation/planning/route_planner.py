"""实现只按合法物理距离搜索的有向正常路径规划器。"""

# 导入最小堆，Dijkstra 以累计物理距离优先扩展候选状态。
# 导入反正切函数，将起点车头与首段道路比较为是否原地掉头。
import math
# 导入 Python 3.8 兼容的字典、可选值和元组类型注解。
from typing import Dict, Optional, Tuple

# 导入领域目标、巡航边和拓扑，规划器只读取这些静态与不可变事实。
from navigation.domain import AtNode, CruiseEdge, EdgeKnowledgeStatus, Goal, GoalKind, RuntimeMapSnapshot, TrackTopology
# 导入本层查询、步骤、计划和结果数据包，避免返回裸列表或异常表达不可达。
from .models import (
    EscapeAssessment,
    EscapeDirectionAssessment,
    JunctionPassability,
    PlanningPhase,
    PlanningStateQuery,
    RoutePlan,
    RoutePlanOutcome,
    RoutePlanResult,
    RouteQuery,
    RouteStep,
)
from .shortest_path import GlobalShortestPath
from .target_selection import TargetSelector
from .reachability import ReachabilityAnalyzer
from .escape_assessor import EscapeAssessor


class RoutePlanner:
    """在静态拓扑和动态地图快照上搜索不立即掉头的最短正常路线。

    谁调用：后续 `Coordinator` 仅在机器人位于安全路口且规划门禁满足时调用。
    谁响应：本类返回 `RoutePlanResult`，供协调器交给后续编排器或处理不可达。
    输入输出：输入 `RouteQuery` 与一个或多个 `Goal`；输出路口级路线或显式 `NO_ROUTE`。
    状态影响：不修改拓扑、地图、任务、机器人状态或执行器。
    """

    def __init__(self, topology: TrackTopology, state_query: Optional[PlanningStateQuery] = None) -> None:
        """保存只读端口拓扑，后续每次规划只读取其有向巡航查询。"""

        # 保存拓扑依赖；动态阻塞事实始终来自每次查询传入的地图快照。
        self._topology = topology
        # 可选只读状态口供局部脱困查询使用，普通路径搜索仍以 RouteQuery 快照为准。
        self._state_query = state_query
        self._target_selector = TargetSelector(topology)
        self._reachability_analyzer = ReachabilityAnalyzer(topology)
        self._escape_assessor = EscapeAssessor(topology, self._reachability_analyzer)

    def bind_read_state(self, state_query) -> None:
        """由组合层注入规划只读端口，不改变任务、地图或机器人状态的所有权。"""

        self._state_query = state_query

    def assess_escape(self) -> EscapeAssessment:
        """报告当前路口前后左右的局部通路事实，不选择方向或完整路线。"""

        # 局部查询必须依赖装配层提供的共享只读状态，避免调用者重复传入位姿和地图。
        if self._state_query is None:
            raise RuntimeError("assess_escape 需要装配 PlanningStateQuery")
        # 只有路口中心才存在可决策的前后左右方向。
        robot_state = self._state_query.robot_state()
        if not isinstance(robot_state.location, AtNode):
            raise ValueError("只有 AtNode 才能进行局部脱困查询")
        # 读取同一时刻的动态地图快照，保证四个方向使用一致的阻塞事实。
        map_snapshot = self._state_query.runtime_map_snapshot()
        if all(hasattr(self._state_query, name) for name in ("pending_tasks", "mission_phase", "mission_finished")):
            selection = self._target_selector.select(
                self._state_query.mission_phase(),
                tuple(self._state_query.pending_tasks()),
                bool(self._state_query.mission_finished()),
                map_snapshot,
            )
            return self._escape_assessor.assess(
                robot_state,
                map_snapshot,
                selection.goals,
                self._passability,
                self._relative_direction,
            )
        results = {
            "forward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "left": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "right": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "backward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
        }
        # 将每条静态出边按相对朝向归类，并把物理边阻塞映射为局部状态。
        for cruise_edge in self._topology.outgoing_cruise_edges(robot_state.location.node_id):
            direction = self._relative_direction(robot_state.location.node_id, cruise_edge, robot_state.world_pose.yaw_deg)
            if direction is None:
                continue
            passability = self._passability(cruise_edge, map_snapshot)
            worth_trying = self._worth_trying(direction, cruise_edge, passability)
            candidate = EscapeDirectionAssessment(passability, worth_trying, cruise_edge.traversal_id)
            results[direction] = self._merge_assessment(results[direction], candidate)
        return EscapeAssessment(**results)

    def _passability(self, cruise_edge: CruiseEdge, map_snapshot) -> JunctionPassability:
        """将一条完整巡航边的物理事实合成为四状态之一。"""

        if any(self._edge_status(map_snapshot, edge_id) is EdgeKnowledgeStatus.BLOCKED for edge_id in cruise_edge.physical_edge_ids):
            return JunctionPassability.BLOCKED
        if all(self._edge_status(map_snapshot, edge_id) is EdgeKnowledgeStatus.CLEAR for edge_id in cruise_edge.physical_edge_ids):
            return JunctionPassability.CLEAR
        return JunctionPassability.UNOBSERVED

    def _worth_trying(self, direction: str, cruise_edge: CruiseEdge, status: JunctionPassability) -> bool:
        """返回方向是否值得尝试；完整目标可达性可由状态查询口提供。"""

        if status in (JunctionPassability.ABSENT, JunctionPassability.BLOCKED):
            return False
        return True

    def plan(
        self,
        query: Optional[RouteQuery] = None,
        candidates: Optional[Tuple[Goal, ...]] = None,
    ) -> RoutePlanResult:
        """在候选目标中选择合法 Dijkstra 距离最小且标识稳定的路线。

        谁调用：后续 `Coordinator` 需要从待办任务候选中选择下一个目标时调用。
        谁响应：本类逐个规划候选并返回最短成功路线或统一不可达结果。
        输入输出：输入查询和目标元组；输出一个 `RoutePlanResult`。
        状态影响：只读搜索，不会开始任务或写入地图。
        """

        if query is None or candidates is None:
            return self._plan_from_read_state()

        return self._plan_candidates_globally(query, candidates)

    def _plan_candidates_globally(self, query: RouteQuery, candidates: Tuple[Goal, ...]) -> RoutePlanResult:
        """一次全局 Dijkstra 覆盖全部候选目标，再按距离和标识选择最优路线。"""

        map_snapshot = self._current_map_snapshot(query)
        start_state = (query.start_node_id, query.entry_traversal_id)
        distances, predecessors = GlobalShortestPath(self._topology, map_snapshot).search(start_state, query.heading_deg)
        best_result = None
        best_plan = None
        for candidate in candidates:
            for state, distance_mm in distances.items():
                node_id, entry_traversal_id = state
                if node_id != candidate.arrival_node_id:
                    continue
                if candidate.required_final_traversal_id is not None and entry_traversal_id != candidate.required_final_traversal_id:
                    continue
                if self._entered_via_approach_edge(entry_traversal_id, candidate):
                    continue
                result = self._build_result(query, candidate, state, predecessors, distance_mm, map_snapshot)
                result = self._enforce_entry_constraint(query, result)
                if result.outcome is not RoutePlanOutcome.PLANNED:
                    continue
                if result.plan is None or not result.plan.steps:
                    continue
                plan = result.plan
                if best_result is None or best_plan is None:
                    best_result = result
                    best_plan = plan
                    continue
                if (
                    plan.total_distance_mm,
                    plan.selected_goal.goal_id,
                ) < (
                    best_plan.total_distance_mm,
                    best_plan.selected_goal.goal_id,
                ):
                    best_result = result
                    best_plan = plan
        if best_result is not None:
            return best_result
        return RoutePlanResult(RoutePlanOutcome.NO_ROUTE, None, "没有合法路线")

    def _plan_from_read_state(self) -> RoutePlanResult:
        """从只读状态端口读取任务态现场；任务完成时拒绝越权规划返回路线。"""

        if self._state_query is None:
            return RoutePlanResult(RoutePlanOutcome.INVALID_STATE, None, "未装配规划只读状态端口")
        mission_phase = getattr(self._state_query, "mission_phase", None)
        mission_finished = getattr(self._state_query, "mission_finished", None)
        pending_tasks = getattr(self._state_query, "pending_tasks", None)
        if mission_phase is None or mission_finished is None or pending_tasks is None:
            return RoutePlanResult(RoutePlanOutcome.INVALID_STATE, None, "规划状态端口缺少任务或阶段只读查询")
        phase = mission_phase()
        selection = self._target_selector.select(
            phase,
            tuple(pending_tasks()),
            bool(mission_finished()),
            self._state_query.runtime_map_snapshot(),
        )
        if selection.outcome is not None:
            return RoutePlanResult(selection.outcome, None, selection.reason)
        state = self._state_query.robot_state()
        if not isinstance(state.location, AtNode):
            return RoutePlanResult(RoutePlanOutcome.INVALID_STATE, None, "正常规划要求机器人位于安全路口")

        # 已经位于某个待办任务的到达节点：交给上层完成任务，而不是生成 0 步空计划。
        for goal in selection.goals:
            if goal.kind is not GoalKind.TASK_ARRIVAL:
                continue
            if goal.task_id is None:
                continue
            if goal.arrival_node_id != state.location.node_id:
                continue
            if goal.required_final_traversal_id is not None and state.location.entry_traversal_id != goal.required_final_traversal_id:
                continue
            if self._entered_via_approach_edge(state.location.entry_traversal_id, goal):
                continue
            return RoutePlanResult(
                RoutePlanOutcome.TASKS_COMPLETED,
                None,
                "已位于任务目标节点 {}".format(goal.arrival_node_id),
            )

        query = RouteQuery(state.location.node_id, state.location.entry_traversal_id, state.world_pose.yaw_deg)
        report = self._reachability_analyzer.analyze(
            query,
            selection.goals,
            self._state_query.runtime_map_snapshot(),
        )
        if report.is_trapped:
            return RoutePlanResult(RoutePlanOutcome.TRAPPED, None, "按当前阶段目标集合分析，机器人处于受困状态")
        return self.plan(query, selection.goals)
    def plan_to(self, query: RouteQuery, target: Goal) -> RoutePlanResult:
        """只计算到指定目标的最短正常路线，不在内部重新选择任务。

        谁调用：后续 `Coordinator` 已明确返航或指定任务目标时调用，`plan()` 也会复用。
        谁响应：本类执行有向 Dijkstra，返回该目标的路线或不可达结果。
        输入输出：输入查询和单个目标；输出该目标的 `RoutePlanResult`。
        状态影响：不修改任何输入对象和运行时状态。
        """

        # 先验证起点与目标节点存在，未知静态标识不能被搜索器猜测处理。
        self._topology.get_node(query.start_node_id)
        self._topology.get_node(target.arrival_node_id)
        # 首边约束的节点和朝向必须与本次查询一致，否则独立返回约束拒绝。
        if query.entry_constraint is not None:
            if query.entry_constraint.applicable_node_id != query.start_node_id:
                return RoutePlanResult(RoutePlanOutcome.CONSTRAINT_UNSATISFIED, None, "首边约束节点与查询起点不一致")
            if abs(query.entry_constraint.required_heading_deg - query.heading_deg) > 0.000001:
                return RoutePlanResult(RoutePlanOutcome.CONSTRAINT_UNSATISFIED, None, "首边约束朝向与查询朝向不一致")
        result = self._plan_candidates_globally(query, (target,))
        if result.outcome is not RoutePlanOutcome.NO_ROUTE or query.entry_constraint is None:
            return result
        unconstrained_query = RouteQuery(
            query.start_node_id,
            query.entry_traversal_id,
            query.heading_deg,
        )
        unconstrained_result = self._plan_candidates_globally(unconstrained_query, (target,))
        if unconstrained_result.outcome is RoutePlanOutcome.PLANNED:
            return RoutePlanResult(RoutePlanOutcome.CONSTRAINT_UNSATISFIED, None, "路线首边不满足指定首边约束")
        return result

    @staticmethod
    def _enforce_entry_constraint(query: RouteQuery, result: RoutePlanResult) -> RoutePlanResult:
        """在路线生成后确认其第一条巡航满足首边约束。"""

        if query.entry_constraint is None or result.outcome is not RoutePlanOutcome.PLANNED:
            return result
        if result.plan is None or not result.plan.steps:
            return RoutePlanResult(RoutePlanOutcome.CONSTRAINT_UNSATISFIED, None, "首边约束要求路线必须包含指定首边")
        if result.plan.steps[0].traversal_id != query.entry_constraint.required_first_traversal_id:
            return RoutePlanResult(RoutePlanOutcome.CONSTRAINT_UNSATISFIED, None, "路线首边不满足指定首边约束")
        return result

    @staticmethod
    def _edge_status(map_snapshot, edge_id):
        """统一通过快照边状态接口读取阻塞、畅通或未知。"""

        status_method = getattr(map_snapshot, "edge_status", None)
        if status_method is not None:
            return status_method(edge_id)
        return EdgeKnowledgeStatus.BLOCKED if edge_id in map_snapshot.blocked_edge_ids else EdgeKnowledgeStatus.UNKNOWN

    def _relative_direction(self, node_id: str, cruise_edge: CruiseEdge, heading_deg: float) -> Optional[str]:
        """将一条出边按当前车头归类为前、左、右或后。"""

        start_node = self._topology.get_node(node_id)
        end_node = self._topology.get_node(cruise_edge.to_junction)
        road_heading = math.degrees(math.atan2(end_node.y_mm - start_node.y_mm, end_node.x_mm - start_node.x_mm))
        difference = (road_heading - heading_deg + 180.0) % 360.0 - 180.0
        if abs(difference) < 0.000001:
            return "forward"
        if abs(difference - 90.0) < 0.000001:
            return "left"
        if abs(difference + 90.0) < 0.000001:
            return "right"
        if abs(abs(difference) - 180.0) < 0.000001:
            return "backward"
        return None

    @staticmethod
    def _merge_assessment(current: EscapeDirectionAssessment, observed: EscapeDirectionAssessment) -> EscapeDirectionAssessment:
        """合并同一相对方向的多条出边，保留更安全且值得尝试的事实。"""

        if current.status is JunctionPassability.ABSENT:
            return observed
        if current.status is JunctionPassability.BLOCKED or observed.status is JunctionPassability.BLOCKED:
            return EscapeDirectionAssessment(
                JunctionPassability.BLOCKED,
                False,
                current.traversal_id or observed.traversal_id,
            )
        if observed.status is JunctionPassability.CLEAR:
            return observed
        if current.status is JunctionPassability.CLEAR:
            return current
        return EscapeDirectionAssessment(
            current.status,
            current.worth_trying or observed.worth_trying,
            current.traversal_id or observed.traversal_id,
        )

    @staticmethod
    def _is_reverse(entry_traversal_id: Optional[str], next_traversal_id: str) -> bool:
        """判断下一巡航是否恰好为当前进入巡航的反向道路。"""

        # 初始位置没有进入道路，因此不适用“立即反向”规则。
        if entry_traversal_id is None:
            return False
        # 有向巡航标识固定为“起点->终点”，可直接构造其反向标识。
        from_node_id, to_node_id = entry_traversal_id.split("->", 1)
        # 只有恰好回到刚离开的路口才是普通路线禁止的 180° 掉头。
        return next_traversal_id == "{}->{}".format(to_node_id, from_node_id)

    def _current_map_snapshot(self, query: RouteQuery) -> RuntimeMapSnapshot:
        """返回规划时刻地图；地图只能从装配的共享只读状态口读取。"""

        if self._state_query is None:
            raise RuntimeError("RoutePlanner 必须装配 PlanningStateQuery")
        return self._state_query.runtime_map_snapshot()

    def _is_initial_uturn(self, query: RouteQuery, cruise_edge: CruiseEdge) -> bool:
        """在没有进入边的初始路口，判断当前车头到首段道路是否为原地掉头。"""

        # 读取起终点中心坐标，巡航首段方向由两中心的世界坐标确定。
        start_node = self._topology.get_node(cruise_edge.from_junction)
        end_node = self._topology.get_node(cruise_edge.to_junction)
        # 用世界坐标系的 x/y 向量计算首段道路朝向，单位为度。
        road_heading_deg = math.degrees(math.atan2(end_node.y_mm - start_node.y_mm, end_node.x_mm - start_node.x_mm))
        # 将角度差归一到零至 180 度，避免 0/360 度边界造成错误判断。
        difference_deg = abs((road_heading_deg - query.heading_deg + 180.0) % 360.0 - 180.0)
        # 仅绝对 180 度被首版规则禁止；直行和一次 90 度转向仍合法。
        return abs(difference_deg - 180.0) < 0.000001

    def _entered_via_approach_edge(self, entry_traversal_id: Optional[str], target: Goal) -> bool:
        """判断到达涵洞任务端点时是否沿同一物理道路驶入，从而必然需要掉头。"""

        # 打卡和返航目标没有涵洞道路限制，起点本身也没有最后驶入道路可比较。
        if target.approach_edge_id is None or entry_traversal_id is None:
            return False
        # 从有向巡航标识恢复其两端路口，再读取组成该巡航的全部物理边。
        from_junction, to_junction = entry_traversal_id.split("->", 1)
        entry_edge = self._topology.get_cruise_edge(from_junction, to_junction)
        # 只要最后巡航包含涵洞道路，无论其方向如何，面对该道路都会要求 180° 掉头。
        return target.approach_edge_id in entry_edge.physical_edge_ids

    @staticmethod
    def _build_result(
        query: RouteQuery,
        target: Goal,
        state: Tuple[str, Optional[str]],
        predecessors: Dict[Tuple[str, Optional[str]], Tuple[Tuple[str, Optional[str]], CruiseEdge]],
        distance_mm: float,
        map_snapshot: RuntimeMapSnapshot,
    ) -> RoutePlanResult:
        """从 Dijkstra 前驱链重建顺序正确的路口步骤并包装成功结果。"""

        # 逆向累积终点到起点的巡航边，稍后反转为机器人实际行驶顺序。
        reversed_steps = []
        # 从目标状态持续回溯，直到到达没有前驱的起始状态。
        while state in predecessors:
            # 读取前驱状态和本次到达当前状态所使用的巡航边。
            previous_state, cruise_edge = predecessors[state]
            # 将静态巡航解释转换为只含路口与标识的规划步骤。
            reversed_steps.append(RouteStep(cruise_edge.from_junction, cruise_edge.to_junction, cruise_edge.traversal_id))
            # 继续向起点回溯。
            state = previous_state
        # 反转回溯结果，得到起点到目标的实际执行顺序。
        steps = tuple(reversed(reversed_steps))
        # 用地图版本、起点和目标标识生成确定性计划标识，便于日志与回归比较。
        plan_id = "route:{}:{}:{}".format(map_snapshot.version, query.start_node_id, target.goal_id)
        # 创建不含动作的正常路线对象。
        plan = RoutePlan(plan_id, map_snapshot.version, query.start_node_id, target, steps, distance_mm)
        # 返回明确成功结果，调用方无需通过步骤是否为空猜测含义。
        return RoutePlanResult(RoutePlanOutcome.PLANNED, plan)
