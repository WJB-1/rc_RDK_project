"""出发主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus, ChoreographyStageKind
from navigation.domain import AbsoluteMapUpdateKind
from ..states import CoordinatorState
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class DepartureAnalyzer(BaseAnalyzer):
    """负责出发剧本的编排、执行反馈分析和任务态转移。

    出发剧本固定包含一段启动桥 START->J_START，然后从 J_START 转向 N1，
    最后左转进入 N1->T1_R 并在 N1 打卡。启动桥由 DepartureChoreographyFactory
    作为第一段 DRIVE_TO_NEXT_CENTER 阶段下发。

    本分析器只负责按当前剧本游标生成并提交动作、消费中断，并把剧本完成后
    的主状态切换到 TASK_PROCESSING。右转观察和阻塞撤回是可选分支，仅在剧本
    包含 OBSERVE_POST_TURN 阶段时才会被激活。
    """

    def __init__(self, coordinator):
        super().__init__(coordinator)
        self._phase = "initial"
        self._right_blocked = False
        self._right_traversal_id = "J_START->N1"

    def run(self) -> AnalyzerDecision:
        """有出发剧本时提交下一动作，否则等待外部注入剧本。"""

        coordinator = self._coordinator
        if coordinator._choreographer is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "未装配出发编排器")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待出发动作终局")

        # ---- 1. 没有活动剧本时按当前阶段生成一个 ----
        if coordinator.active_choreography is None:
            self._load_initial_choreography(coordinator)
            if coordinator.active_choreography is None:
                return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "出发剧本生成失败")

        # ---- 2. 进入右侧观察阶段时记录相位，供 handle_map_update 分流 ----
        progress_index = coordinator._context.active_progress.stage_index
        stages = coordinator._context.active_plan.stages
        if progress_index < len(stages):
            stage = stages[progress_index]
            if (
                stage.kind is ChoreographyStageKind.OBSERVE_POST_TURN
                and stage.traversal_id == self._right_traversal_id
            ):
                self._phase = "right_observe"

        # ---- 3. 提交下一动作 ----
        ack = coordinator._dispatch_current_action()

        # ---- 4. 剧本已结束，按相位决定后续状态 ----
        if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
            return self._on_choreography_finished(coordinator)

        if ack is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "出发剧本暂时没有可派发动作")
        return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发出发动作")

    def _load_initial_choreography(self, coordinator) -> None:
        """按当前阶段选择出发、左转替代或撤回剧本，并保存到活动流程。"""

        # 右转阻塞后需要先生成同轨迹撤回，再走左转替代剧本。
        if self._phase in ("retrace", "right_blocked"):
            self._phase = "retrace"
            escape = coordinator._analyzers.get(CoordinatorState.ESCAPE)
            if escape is None or not escape.start_retrace_turn(self._turn_source_action_id):
                coordinator.enter_exception("出发右转阻塞后无法生成撤回剧本")
            return

        # 左转替代：不经过固定出发剧本，直接转向 N1 并观察。
        if self._phase == "left":
            factory = getattr(coordinator._choreographer, "departure_factory", None)
            result = (
                factory.start_left_turn()
                if factory is not None
                else coordinator._choreographer.start_departure_left_turn()
            )
            if not coordinator._load_choreography_result(result):
                coordinator.enter_exception("出发左转剧本生成失败")
            return

        # 常规出发：bridge + 右转 + forward160 + 左转 + 打卡 N1。
        factory = getattr(coordinator._choreographer, "departure_factory", None)
        result = (
            factory.start_departure()
            if factory is not None
            else coordinator._choreographer.start_departure()
        )
        if not coordinator._load_choreography_result(result):
            coordinator.enter_exception("出发剧本生成失败")

    def _on_choreography_finished(self, coordinator) -> AnalyzerDecision:
        """出发剧本结束后的分支：撤回后转左转、观察完成后转任务态。"""

        if self._phase == "retrace":
            self._phase = "left"
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "右转已撤回，准备左转观察")

        if self._phase == "left":
            coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "左侧出发观察完成，进入任务处理")

        if self._phase == "right_observe" and self._right_blocked:
            self._phase = "retrace"
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "右侧阻塞，准备撤回转弯")

        # 固定出发剧本（bridge + 右转 + 左转 + 打卡）走这里，直接转任务态。
        coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
        return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "出发剧本完成，进入任务处理")

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
            edge = topology.get_cruise_edge("J_START", "N1")
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