"""实现只按合法物理距离搜索的有向正常路径规划器。"""

# 导入最小堆，Dijkstra 以累计物理距离优先扩展候选状态。
import heapq
# 导入反正切函数，将起点车头与首段道路比较为是否原地掉头。
import math
# 导入 Python 3.8 兼容的字典、可选值和元组类型注解。
from typing import Dict, Optional, Tuple

# 导入领域目标、巡航边和拓扑，规划器只读取这些静态与不可变事实。
from navigation.domain import AtNode, CruiseEdge, Goal, TrackTopology
# 导入本层查询、步骤、计划和结果数据包，避免返回裸列表或异常表达不可达。
from .models import (
    EscapeAssessment,
    EscapeDirectionAssessment,
    JunctionPassability,
    PlanningStateQuery,
    RoutePlan,
    RoutePlanOutcome,
    RoutePlanResult,
    RouteQuery,
    RouteStep,
)


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
        results = {
            "forward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "left": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "right": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "backward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
        }
        # 将每条静态出边按相对朝向归类，并把物理边阻塞映射为局部状态。
        for cruise_edge in self._topology.outgoing_cruise_edges(robot_state.location.node_id):
            direction = self._relative_direction(robot_state.location.node_id, cruise_edge, robot_state.heading_deg)
            if direction is None:
                continue
            passability = self._passability(cruise_edge, map_snapshot)
            worth_trying = self._worth_trying(direction, cruise_edge, passability)
            candidate = EscapeDirectionAssessment(passability, worth_trying, cruise_edge.traversal_id)
            results[direction] = self._merge_assessment(results[direction], candidate)
        return EscapeAssessment(**results)

    def _passability(self, cruise_edge: CruiseEdge, map_snapshot) -> JunctionPassability:
        """将一条完整巡航边的物理事实合成为四状态之一。"""

        if self._is_blocked_snapshot(cruise_edge, map_snapshot.blocked_edge_ids):
            return JunctionPassability.BLOCKED
        if all(edge_id in map_snapshot.confirmed_clear_edge_ids for edge_id in cruise_edge.physical_edge_ids):
            return JunctionPassability.CLEAR
        return JunctionPassability.UNOBSERVED

    def _worth_trying(self, direction: str, cruise_edge: CruiseEdge, status: JunctionPassability) -> bool:
        """返回方向是否值得尝试；完整目标可达性可由状态查询口提供。"""

        if status in (JunctionPassability.ABSENT, JunctionPassability.BLOCKED):
            return False
        reachability = getattr(self._state_query, "can_escape_via", None)
        if reachability is not None:
            return bool(reachability(direction, cruise_edge.traversal_id))
        return True

    def plan(self, query: RouteQuery, candidates: Tuple[Goal, ...]) -> RoutePlanResult:
        """在候选目标中选择合法 Dijkstra 距离最小且标识稳定的路线。

        谁调用：后续 `Coordinator` 需要从待办任务候选中选择下一个目标时调用。
        谁响应：本类逐个规划候选并返回最短成功路线或统一不可达结果。
        输入输出：输入查询和目标元组；输出一个 `RoutePlanResult`。
        状态影响：只读搜索，不会开始任务或写入地图。
        """

        # 保存当前最优成功结果，初始时尚未找到任何可达候选。
        best_result: Optional[RoutePlanResult] = None
        # 逐个搜索候选，避免目标派生器承担路径距离计算职责。
        for candidate in candidates:
            # 复用指定目标搜索，保证两个公开入口遵循同一掉头和阻塞规则。
            result = self.plan_to(query, candidate)
            # 不可达候选不能参与最短路线比较，继续检查其余候选。
            if result.outcome is not RoutePlanOutcome.PLANNED:
                continue
            # 第一个成功候选暂时成为当前最优路线。
            if best_result is None:
                best_result = result
                continue
            # 先按总物理距离比较，再按目标标识稳定打破完全相同距离的平局。
            current_key = (result.plan.total_distance_mm, result.plan.selected_goal.goal_id)
            best_key = (best_result.plan.total_distance_mm, best_result.plan.selected_goal.goal_id)
            if current_key < best_key:
                best_result = result
        # 至少一个候选成功时返回最优结果，而不是重新包装或修改路线对象。
        if best_result is not None:
            return best_result
        # 候选为空或全部不可达时明确返回失败，协调器据此进入等待或异常策略。
        return RoutePlanResult(RoutePlanOutcome.NO_ROUTE, None, "没有合法路线")

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
        # 状态由当前路口和进入边共同组成，才能在同一节点保留不同的掉头约束。
        start_state = (query.start_node_id, query.entry_traversal_id)
        # 记录每个状态已知的最短物理距离。
        distances: Dict[Tuple[str, Optional[str]], float] = {start_state: 0.0}
        # 记录每个状态的前驱状态和到达该状态所选巡航边，用于最终重建 RouteStep。
        predecessors: Dict[Tuple[str, Optional[str]], Tuple[Tuple[str, Optional[str]], CruiseEdge]] = {}
        # 堆中的序号稳定处理相同距离，避免直接比较可选值造成不确定性。
        queue = [(0.0, 0, query.start_node_id, query.entry_traversal_id)]
        # 保存递增序号，保证每次压入队列有确定的次序。
        sequence = 1
        # 当起点就是目标时，仍需返回一条零长度的正常路线供协调器处理已到达语义。
        if query.start_node_id == target.arrival_node_id:
            result = self._build_result(query, target, start_state, predecessors, 0.0)
            return self._enforce_entry_constraint(query, result)
        # 持续扩展累计距离最小的尚未过期状态。
        while queue:
            # 取出当前累计距离最小的有向状态。
            distance_mm, _, node_id, entry_traversal_id = heapq.heappop(queue)
            # 队列中较旧的同状态条目已不可能产生更优路线，直接跳过。
            if distance_mm != distances[(node_id, entry_traversal_id)]:
                continue
            # 到达目标路口且末段没有沿涵洞道路驶入时，当前距离才是合法最短距离。
            if node_id == target.arrival_node_id and not self._entered_via_approach_edge(entry_traversal_id, target):
                result = self._build_result(query, target, (node_id, entry_traversal_id), predecessors, distance_mm)
                return self._enforce_entry_constraint(query, result)
            # 读取当前路口的静态有向出边，拓扑已保证返回顺序稳定。
            for cruise_edge in self._topology.outgoing_cruise_edges(node_id):
                # 任何组成物理边被动态地图阻塞时，本次巡航整体不可通行。
                if self._is_blocked(cruise_edge, query):
                    continue
                # 从刚驶入的边立即反向离开属于被禁止的普通掉头。
                if self._is_reverse(entry_traversal_id, cruise_edge.traversal_id):
                    continue
                # 初始位置没有进入边时，用实际车头与首段方向阻止原地 180° 掉头。
                if entry_traversal_id is None and self._is_initial_uturn(query, cruise_edge):
                    continue
                # 新状态记录本次巡航作为到达下一路口的进入边。
                next_state = (cruise_edge.to_junction, cruise_edge.traversal_id)
                # 距离只累加道路物理长度，不引入未标定的转向成本。
                next_distance_mm = distance_mm + cruise_edge.length_mm
                # 已知同状态的路线更短或等长时保留旧路线，稳定性由出边顺序与目标排序保证。
                if next_state in distances and distances[next_state] <= next_distance_mm:
                    continue
                # 写入新的最短距离与重建所需前驱信息。
                distances[next_state] = next_distance_mm
                predecessors[next_state] = ((node_id, entry_traversal_id), cruise_edge)
                # 将新状态压入最小堆，供后续继续扩展。
                heapq.heappush(queue, (next_distance_mm, sequence, next_state[0], next_state[1]))
                # 递增稳定序号，避免相同距离时依赖 Python 对其他字段的比较。
                sequence += 1
        # 队列耗尽仍未到达目标，说明阻塞或掉头约束下不存在合法路线。
        return RoutePlanResult(RoutePlanOutcome.NO_ROUTE, None, "没有合法路线")

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
    def _is_blocked_snapshot(cruise_edge: CruiseEdge, blocked_edge_ids) -> bool:
        """根据同一份地图快照判断巡航边是否被阻塞。"""

        return any(edge_id in blocked_edge_ids for edge_id in cruise_edge.physical_edge_ids)

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

    @staticmethod
    def _is_blocked(cruise_edge: CruiseEdge, query: RouteQuery) -> bool:
        """判断巡航组成物理边是否与规划快照中的阻塞事实相交。"""

        # 任一物理段被阻塞都使中心到中心巡航无法安全通过。
        return any(edge_id in query.map_snapshot.blocked_edge_ids for edge_id in cruise_edge.physical_edge_ids)

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
        plan_id = "route:{}:{}:{}".format(query.map_snapshot.version, query.start_node_id, target.goal_id)
        # 创建不含动作的正常路线对象。
        plan = RoutePlan(plan_id, query.map_snapshot.version, query.start_node_id, target, steps, distance_mm)
        # 返回明确成功结果，调用方无需通过步骤是否为空猜测含义。
        return RoutePlanResult(RoutePlanOutcome.PLANNED, plan)
