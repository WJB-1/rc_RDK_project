"""实现严格串行的导航动作协调器。"""

# 导入时间函数，为异步请求生成可追踪的创建时间。
import time
# 导入 Python 3.8 兼容的可选类型。
from typing import Callable, List, Optional

# 导入执行层类型化请求和命令。
from navigation.contracts import (
    Action,
    ChoreographyAdvanceStatus,
    ChoreographyStartStatus,
    ChoreographyPlan,
    ChoreographyProgress,
    CompleteTaskEffect,
    DispatchAck,
    ExecutionInterrupt,
    ExecutionOutcome,
    ExecutionRequest,
    IAsyncExecutor,
)
from navigation.domain import (
    RobotState,
)
from navigation.domain import TrackTopology
from navigation.planning import RecoveryPlan
from .execution_bridge import ExecutionBridge
from .event_projection import EventProjector
from .departure_flow import DepartureFlow
from .escape_flow import EscapeFlow
from .return_flow import ReturnFlow
from .task_flow import TaskFlow
from .context import CoordinatorContext, LastMotionRecord
from .state_adapter import LegacyNavigationStateAdapter
from .states import CoordinatorState, DepartureSubstate, EscapeSubstate, ReturnSubstate, TaskSubstate
from .analyzers import (
    AnalyzerDecisionKind,
    DepartureAnalyzer,
    EscapeAnalyzer,
    ReturningAnalyzer,
    TaskProcessingAnalyzer,
)


class Coordinator:
    """拥有活动剧本、唯一在途动作并负责异步终局身份校验的中转者。"""

    def __init__(
        self,
        executor: IAsyncExecutor,
        choreographer,
        clock: Callable[[], float] = time.time,
        state_store=None,
        perception_adapter=None,
        runtime_map=None,
        navigation_state=None,
        location_projector=None,
        topology: Optional[TrackTopology] = None,
        task_registry=None,
        route_planner=None,
        recovery_planner=None,
    ) -> None:
        """保存已装配依赖，并把自身注册为执行器唯一终局消费者。"""

        # 执行器只负责受理命令和回传终局，协调器不判断仿真或真实环境。
        self._executor = executor
        # 编排器只负责把剧本指针解释为一条 Action。
        self._choreographer = choreographer
        # 注入时钟以便测试请求身份和时间字段，不读取系统时间以外的业务状态。
        self._clock = clock
        # 状态端口由 Runtime 装配，Coordinator 是唯一可以提交新 RobotState 的业务方。
        # 新版只接收统一状态层；旧参数仅通过适配器兼容迁移中的测试和装配代码。
        self._navigation_state = navigation_state
        if self._navigation_state is None and (state_store is not None or runtime_map is not None):
            self._navigation_state = LegacyNavigationStateAdapter(state_store, runtime_map)
        # 感知适配器只负责翻译单帧，地图和位置仍由 Coordinator 统一消费。
        self._perception_adapter = perception_adapter
        # 运行时地图由 Coordinator 写入，适配器本身不持有地图写权限。
        # 位置投影器只接收已确认的视觉校正，不直接参与感知翻译。
        self._location_projector = location_projector
        # 静态拓扑用于把地图更新的物理边映射到活动路线的巡航边。
        self._topology = topology
        # 任务注册表由 Coordinator 在提交和确认任务动作时推进生命周期。
        self._task_registry = task_registry
        # 正常规划器负责自行选目标和生成路线，Coordinator 只消费其结果。
        self._route_planner = route_planner
        # 恢复规划器负责自行选择倒车目标，Coordinator 不传入安全路口参数。
        self._recovery_planner = recovery_planner
        # 四个内部流程对象按外层状态承载阶段业务，公共执行入口仍由本类统一维护。
        self._departure_flow = DepartureFlow(self)
        self._task_flow = TaskFlow(self)
        self._escape_flow = EscapeFlow(self)
        self._return_flow = ReturnFlow(self)
        # 保存外层状态与内部阶段，供事件入口和调试面板读取。
        self._context = CoordinatorContext()
        # 分析器是各主状态的业务决策者；Coordinator 只负责选择它并统一执行动作。
        self._analyzers = {
            CoordinatorState.DEPARTURE: DepartureAnalyzer(self),
            CoordinatorState.TASK_PROCESSING: TaskProcessingAnalyzer(self),
            CoordinatorState.ESCAPE: EscapeAnalyzer(self),
            CoordinatorState.RETURNING: ReturningAnalyzer(self),
        }
        # 防止执行器回调在泵循环内部重入，保证一次只运行一个状态机泵。
        self._pump_running = False
        self._pump_requested = False
        # 只有通过 start() 启动后的 Coordinator 才自动消费中断并继续泵循环；
        # 直接调用旧派发接口的测试和调试代码保持手动推进兼容。
        self._auto_drive = False
        # 记录最近成功转弯，供局部恢复引用而不要求执行器缓存历史。
        # 记录诊断信息但不把迟到事件重新解释为业务动作。
        self.diagnostics: List[str] = []
        # 投影器集中处理执行反馈写入，地图影响仍回调 Coordinator 做路线生命周期分流。
        self._event_projector = (
            EventProjector(
                self._navigation_state,
                task_registry=self._task_registry,
                perception_adapter=self._perception_adapter,
                location_projector=self._location_projector,
                on_map_update=self._task_flow.handle_map_update,
                diagnostics=self.diagnostics,
                context=self._context,
            )
            
        )
        # Coordinator 直接接收执行器终局，NavigationRuntime 不参与中断转发。
        if self._executor is not None:
            self._executor.on_interrupt(self.handle_execution_interrupt)

    @property
    def current_action(self) -> Optional[Action]:
        """返回当前唯一在途动作，供运行时和调试面板只读查看。"""

        return self._context.current_action

    @property
    def current_request(self) -> Optional[ExecutionRequest]:
        """返回当前唯一在途执行请求，未受理或已终局时为空。"""

        return self._context.current_request

    @property
    def active_choreography(self) -> Optional[ChoreographyPlan]:
        """返回当前活动剧本，剧本销毁或完成后返回空值。"""

        return self._context.active_plan

    @property
    def is_waiting_interrupt(self) -> bool:
        """返回是否已经提交请求并等待同一身份的最终中断。"""

        return self._context.current_request is not None

    @property
    def last_completed_turn(self) -> Optional[LastMotionRecord]:
        """返回最近一次成功前向转弯记录。"""

        return self._context.last_motion if self._context.last_motion and self._context.last_motion.is_turn else None

    @property
    def state(self) -> CoordinatorState:
        """返回协调器当前外层有限状态。"""

        return self._context.state

    @property
    def substate(self):
        """返回当前外层状态对应的内部阶段。"""

        if self._context.state is CoordinatorState.DEPARTURE:
            return self._context.departure_substate
        if self._context.state is CoordinatorState.TASK_PROCESSING:
            return self._context.task_substate
        if self._context.state is CoordinatorState.ESCAPE:
            return self._context.escape_substate
        if self._context.state is CoordinatorState.RETURNING:
            return self._context.return_substate
        return None

    def enter_exception(self, reason: str) -> None:
        """记录异常原因并进入异常终局，不再生成普通动作。"""

        self.diagnostics.append(reason)
        self._context.transition_main(CoordinatorState.EXCEPTION)

    def start(
        self,
        departure_plan: Optional[ChoreographyPlan] = None,
        departure_progress: Optional[ChoreographyProgress] = None,
    ) -> None:
        """启动导航主状态机，并运行到下一个异步等待点。

        外层组合根可以提供已经生成的出发剧本；Coordinator 不直接构造动作，
        而是交给当前主状态分析器调用编排器和统一执行入口。方法不会忙等设备。
        """

        # 首次启动允许注入出发剧本，后续重复启动不得覆盖活动流程。
        if self._context.active_plan is None and departure_plan is not None:
            self._context.active_plan = departure_plan
            self._context.active_progress = departure_progress
        self._auto_drive = True
        self._pump()

    def _select_analyzer(self):
        """按当前主状态选择唯一业务分析器。"""

        return self._analyzers.get(self._context.state)

    def _pump(self) -> None:
        """运行状态机直到异步等待、无法推进或进入终局。"""

        # 回调可能在泵运行期间到达，只登记一次待处理请求，避免递归调用。
        if self._pump_running:
            self._pump_requested = True
            return
        self._pump_running = True
        try:
            while True:
                self._pump_requested = False
                if self._context.state is CoordinatorState.EXCEPTION:
                    return
                analyzer = self._select_analyzer()
                if analyzer is None:
                    return
                previous_state = self._context.state
                decision = analyzer.run()
                # 分析器可能在一次判断中完成主状态转移；此时立即重新选择新的分析器。
                if self._context.state is not previous_state:
                    continue
                if decision.kind in (
                    AnalyzerDecisionKind.DISPATCHED,
                    AnalyzerDecisionKind.WAITING,
                    AnalyzerDecisionKind.TERMINAL,
                ):
                    return
                if not self._pump_requested:
                    continue
        finally:
            self._pump_running = False

    def dispatch_next(
        self,
        plan: Optional[ChoreographyPlan] = None,
        progress: Optional[ChoreographyProgress] = None,
    ) -> Optional[DispatchAck]:
        """解释并提交下一动作；已有在途请求时保持串行并拒绝重复提交。

        首次调用可提供剧本和指针；后续调用直接使用协调器保存的活动剧本与
        `resume_progress`，避免外部重复传递或篡改流程游标。
        """

        # 异步动作未结束前不能再次推进剧本，避免下位机同时执行两项导航动作。
        if self._context.current_request is not None:
            self.diagnostics.append("已有在途请求，忽略重复 dispatch_next")
            return None
        # 允许首次调用装载剧本；已有活动剧本时忽略外部重复参数，保证游标所有权在协调器。
        if self._context.active_plan is None:
            if plan is None or progress is None:
                self.diagnostics.append("首次 dispatch_next 必须提供剧本和指针")
                return None
            self._context.active_plan = plan
            self._context.active_progress = progress
        elif plan is not None or progress is not None:
            self.diagnostics.append("已有活动剧本，忽略外部传入的剧本和指针")
        if self._context.active_progress is None:
            self.diagnostics.append("活动剧本缺少 resume_progress")
            return None
        # 编排器负责检查指针和运行时安全，协调器只消费其显式结果。
        result = self._choreographer.compile_next(self._context.active_plan, self._context.active_progress)
        if result.status is ChoreographyAdvanceStatus.FINISHED:
            self._context.active_plan = None
            self._context.active_progress = None
            self.diagnostics.append("活动编排流程已结束")
            return None
        if result.status is not ChoreographyAdvanceStatus.READY:
            self.diagnostics.append("编排器未返回 READY：{}".format(result.status.value))
            return None
        # 在调用执行器前原子保存剧本、动作和下一指针，保证受理期间状态完整。
        action = result.action
        self._context.active_progress = result.next_progress
        self._context.current_action = action
        if not self._auto_drive and self._context.state is CoordinatorState.DEPARTURE:
            self._context.transition(CoordinatorState.DEPARTURE, DepartureSubstate.EXECUTE)
        elif not self._auto_drive and self._context.state is CoordinatorState.TASK_PROCESSING:
            self._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.EXECUTING)
        request = self._translate_action(action)
        self._context.current_request = request
        # 任务动作只有在注册表成功占用后才能交给执行器，避免重复执行同一任务。
        if isinstance(action.expected_effect, CompleteTaskEffect) and self._task_registry is not None:
            transition = self._task_registry.begin(action.expected_effect.task_id)
            if not transition.accepted:
                self.diagnostics.append("任务无法进入执行态：{}".format(transition.reason))
                self._clear_in_flight()
                return DispatchAck(request.request_id, False, transition.reason or "任务无法开始")
        # 同步拒绝表示动作从未开始，必须撤销刚才预保存的在途记录。
        ack = self._executor.submit(request)
        if not ack.accepted:
            self._clear_in_flight()
        return ack

    def replan(self) -> bool:
        """在安全路口触发一次无参规划，并按结果装载正常或恢复剧本。

        正常规划和倒车恢复都由各自规划器自行读取共享地图、位姿和任务上下文；
        Coordinator 只负责选择恢复分支、保存编排结果并继续统一的异步执行入口。
        """

        # 在途动作尚未终局时不能销毁剧本或启动另一条规划流程。
        if self._context.current_request is not None:
            self.diagnostics.append("已有在途请求，不能开始重新规划")
            return False
        if self._context.state is CoordinatorState.ESCAPE:
            return self._escape_flow.assess_and_replan()
        if self._context.state is CoordinatorState.RETURNING:
            return self._return_flow.plan()
        return self._task_flow.plan()

    def _load_planning_result(self, planning_plan) -> bool:
        """把规划器返回的路线或恢复路线交给编排器并保存新游标。"""

        start_method = getattr(self._choreographer, "start", None)
        if start_method is None:
            self.diagnostics.append("编排器未提供 start 入口")
            return False
        start_result = start_method(planning_plan)
        if (
            start_result.status is not ChoreographyStartStatus.STARTED
            or start_result.plan is None
            or start_result.progress is None
        ):
            self.diagnostics.append("编排器拒绝规划结果")
            return False
        self._context.active_plan = start_result.plan
        self._context.active_progress = start_result.progress
        if self._auto_drive:
            self._context.transition_main(self._context.state)
        elif self._context.state is CoordinatorState.TASK_PROCESSING:
            self._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.CHOREOGRAPHING)
        elif self._context.state is CoordinatorState.ESCAPE:
            # RecoveryPlan 进入倒车编排；普通路线则表示脱困已成功，回到任务流程。
            if isinstance(planning_plan, RecoveryPlan):
                self._context.transition(CoordinatorState.ESCAPE, EscapeSubstate.BACKTRACK_CHOREOGRAPHING)
            else:
                self._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.CHOREOGRAPHING)
        elif self._context.state is CoordinatorState.RETURNING:
            self._context.transition(CoordinatorState.RETURNING, ReturnSubstate.CHOREOGRAPH_TO_START)
        self._clear_pending_replan()
        return True

    def _clear_pending_replan(self) -> None:
        """新剧本成功装载后清除已经兑现的待重规划标记。"""

        if self._navigation_state is None:
            return
        current_state = self._navigation_state.robot_state()
        if not current_state.pending_replan:
            return
        self._navigation_state.replace_robot_state(
            RobotState(
                current_state.location,
                current_state.heading_deg,
                current_state.progress_source,
                False,
            )
        )

    def handle_execution_interrupt(self, interrupt: ExecutionInterrupt) -> bool:
        """把执行器中断直接交给当前分析器，并自动继续状态机泵。"""

        analyzer = self._select_analyzer()
        if analyzer is None:
            self.diagnostics.append("当前主状态没有对应分析器")
            return False
        decision = analyzer.analyze_interrupt(interrupt)
        # 分析器已经完成中断公共投影和业务判断；泵负责提交下一动作或重新规划。
        if self._auto_drive and (decision.kind is not AnalyzerDecisionKind.WAITING or self._context.current_request is None):
            self._pump()
        return decision.kind is not AnalyzerDecisionKind.WAITING or self._context.current_request is None

    def _handle_execution_interrupt_core(self, interrupt: ExecutionInterrupt) -> bool:
        """消费第一条完整匹配的终局；迟到、重复或未知身份只记录诊断。"""

        request = self._context.current_request
        if request is None:
            self.diagnostics.append("无在途请求，忽略终局 {}".format(interrupt.request_id))
            return False
        # 四个稳定身份必须全部匹配，防止旧计划终局推进新计划。
        if (
            interrupt.request_id != request.request_id
            or interrupt.execution_id != request.execution_id
            or interrupt.action_id != request.action_id
            or interrupt.source is not request.target
        ):
            self.diagnostics.append("终局身份不匹配，忽略 {}".format(interrupt.request_id))
            return False
        # 同一请求的任何最终结果都只能被消费一次。
        # 执行器的 BLOCKED 只表示本次动作未能完成，不是视觉确认的地图边阻塞。
        if interrupt.outcome is ExecutionOutcome.BLOCKED:
            self._handle_execution_failure(interrupt)
        # 成功终局先按动作效果投影状态，再清理在途身份，避免丢失动作语义。
        elif interrupt.outcome is ExecutionOutcome.COMPLETED:
            self._event_projector.project_success(self._context.current_action, interrupt)
            self._event_projector.project_perception(self._context.current_action, interrupt)
            self._event_projector.project_task(self._context.current_action)
        self._clear_in_flight()
        # 观察中断已经完成清理后，才允许装载撤回剧本，保持单一在途请求。
        if self._context.pending_retrace is not None:
            source_action_id = self._context.pending_retrace.source_action_id
            self._context.pending_retrace = None
            self._escape_flow.start_retrace_turn(source_action_id)
        elif (
            interrupt.outcome is ExecutionOutcome.COMPLETED
            and self._context.state is CoordinatorState.TASK_PROCESSING
            and not self._auto_drive
        ):
            self._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.ANALYZING)
        if interrupt.outcome is not ExecutionOutcome.COMPLETED:
            self.diagnostics.append("动作 {} 以 {} 结束".format(interrupt.action_id, interrupt.outcome.value))
        return True

    def _handle_execution_failure(self, interrupt: ExecutionInterrupt) -> None:
        """处理执行端中止，不把执行失败误判为地图阻塞或脱困场景。

        `BLOCKED` 是执行器对当前动作无法完成的终局反馈。真实道路是否阻塞只能由
        观察结果中的 `BLOCK_EDGE` 地图更新确认，因此这里必须进入异常终局，不能调用
        `EscapeFlow`、写入 `RuntimeMap` 或生成撤回/倒车动作。
        """

        # 异常终局后销毁活动剧本，禁止错误动作继续被重新派发。
        self._context.active_plan = None
        self._context.active_progress = None
        # 清除可能残留的局部撤回意图，避免异常状态继续装载恢复剧本。
        self._context.pending_retrace = None
        # 交由统一错误入口记录原因并切换到不可继续派发的异常态。
        reason = "动作 {} 被执行器中止：{}".format(interrupt.action_id, interrupt.error_code or "BLOCKED")
        self.enter_exception(reason)

    def _mark_pending_replan(self) -> None:
        """在不改变当前位置的前提下设置待安全路口兑现的重规划标记。"""

        if self._navigation_state is None:
            return
        current_state = self._navigation_state.robot_state()
        if current_state is None:
            return
        if current_state.pending_replan:
            return
        self._navigation_state.replace_robot_state(
            RobotState(
                current_state.location,
                current_state.heading_deg,
                current_state.progress_source,
                True,
            )
        )

    def _translate_action(self, action: Action) -> ExecutionRequest:
        """委托 ExecutionBridge 翻译编排动作，保留旧私有入口兼容测试。"""

        choreography_id = self._context.active_plan.choreography_id if self._context.active_plan is not None else "execution:unknown"
        return ExecutionBridge.translate(action, choreography_id, self._clock())

    def _clear_in_flight(self) -> None:
        """清理当前动作和请求，但保留活动剧本供上层决定是否继续。"""

        self._context.current_action = None
        self._context.current_request = None
