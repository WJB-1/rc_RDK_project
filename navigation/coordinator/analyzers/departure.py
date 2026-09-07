"""出发主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus
from navigation.domain import AbsoluteMapUpdateKind
from ..states import CoordinatorState
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class DepartureAnalyzer(BaseAnalyzer):
    """负责出发剧本的编排、执行反馈分析和任务态转移。"""

    def __init__(self, coordinator):
        super().__init__(coordinator)
        self._phase = "initial"
        self._right_blocked = False
        self._right_traversal_id = "J_START->N12"

    def run(self) -> AnalyzerDecision:
        """有出发剧本时提交下一动作，否则等待外部注入剧本。"""

        coordinator = self._coordinator
        if coordinator._choreographer is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "未装配出发编排器")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待出发动作终局")
        if coordinator.active_choreography is None:
            if self._phase in ("retrace", "right_blocked"):
                self._phase = "retrace"
                escape = coordinator._analyzers.get(CoordinatorState.ESCAPE)
                if escape is None or not escape.start_retrace_turn(self._turn_source_action_id):
                    coordinator.enter_exception("出发右转阻塞后无法生成撤回剧本")
                    return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "撤回剧本生成失败")
            elif self._phase == "left":
                factory = getattr(coordinator._choreographer, "departure_factory", None)
                result = (factory.start_left_turn() if factory is not None
                          else coordinator._choreographer.start_departure_left_turn())
                if not coordinator._load_choreography_result(result):
                    coordinator.enter_exception("出发左转剧本生成失败")
                    return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "左转剧本生成失败")
            else:
                factory = getattr(coordinator._choreographer, "departure_factory", None)
                result = (factory.start_departure() if factory is not None
                          else coordinator._choreographer.start_departure())
                if not coordinator._load_choreography_result(result):
                    coordinator.enter_exception("出发剧本生成失败")
                    return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "出发剧本生成失败")
        action = None
        progress_index = coordinator._context.active_progress.stage_index
        if progress_index < len(coordinator._context.active_plan.stages):
            action = coordinator._context.active_plan.stages[progress_index]
            if action.kind.name == "OBSERVE_POST_TURN" and action.traversal_id == self._right_traversal_id:
                self._phase = "right_observe"
        ack = coordinator._dispatch_current_action()
        if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
            if self._phase == "retrace":
                self._phase = "left"
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "右转已撤回，准备左转观察")
            if self._phase == "left":
                coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "左侧出发观察完成，进入任务处理")
            if self._phase == "right_observe" and self._right_blocked:
                self._phase = "retrace"
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "右侧阻塞，准备撤回转弯")
            coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "出发剧本完成，进入任务处理")
        if ack is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "出发剧本暂时没有可派发动作")
        return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发出发动作")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费出发动作中断后继续出发分析，不直接构造动作。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的出发中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进出发流程")
        return self.run()

    def handle_map_update(self, update):
        """记录出发右侧观察发现的阻塞，地图写入仍由统一投影层完成。"""
        if update.kind is not AbsoluteMapUpdateKind.BLOCK_EDGE or update.edge_id is None:
            return None
        topology = self._coordinator._topology
        if topology is None:
            return None
        try:
            edge = topology.get_cruise_edge("J_START", "N12")
        except KeyError:
            return None
        if update.edge_id in edge.physical_edge_ids and self._phase == "right_observe":
            self._right_blocked = True
            self._phase = "right_blocked"
            self._coordinator._context.active_plan = None
            self._coordinator._context.active_progress = None
            return True
        return None

    @property
    def _turn_source_action_id(self):
        """返回最近一次成功前向转弯的动作标识，供 RETRACE_TURN 引用。"""
        motion = self._coordinator._context.last_motion
        return motion.action_id if motion is not None and motion.is_turn else None
