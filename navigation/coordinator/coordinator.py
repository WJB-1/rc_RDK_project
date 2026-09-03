"""实现严格串行的导航动作协调器。"""

# 导入时间函数，为异步请求生成可追踪的创建时间。
import time
# 导入 Python 3.8 兼容的可选类型。
from typing import Callable, List, Optional

# 导入执行层类型化请求和命令。
from navigation.contracts import (
    Action,
    AdvanceOnTraversalEffect,
    ArriveAtNodeEffect,
    ChoreographyAdvanceStatus,
    ChoreographyPlan,
    ChoreographyProgress,
    DispatchAck,
    DriveDistanceCommand,
    DriveExecutionCommand,
    ExecuteTaskCommand,
    ExecuteTaskExecutionCommand,
    ExecutionInterrupt,
    ExecutionOutcome,
    ExecutionRequest,
    ExecutionTarget,
    IAsyncExecutor,
    ObserveCommand,
    ObserveExecutionCommand,
    PerceptionOutcome,
    RetraceTurnCommand,
    RetraceTurnExecutionCommand,
    ReverseDistanceCommand,
    ReverseExecutionCommand,
    StopCommand,
    StopExecutionCommand,
    TurnAtJunctionCommand,
    TurnExecutionCommand,
)
from navigation.domain import AtNode, OnCruiseEdge, ProgressSource, RobotState


class LastCompletedTurn:
    """记录最近一次成功前向转弯，供同轨迹撤回使用。"""

    def __init__(self, source_action_id: str, forward_trajectory_id: str) -> None:
        """保存动作身份和已标定的前向轨迹身份。"""

        self.source_action_id = source_action_id
        self.forward_trajectory_id = forward_trajectory_id


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
        location_projector=None,
    ) -> None:
        """保存已装配依赖；中断回调由 NavigationRuntime 负责注册。"""

        # 执行器只负责受理命令和回传终局，协调器不判断仿真或真实环境。
        self._executor = executor
        # 编排器只负责把剧本指针解释为一条 Action。
        self._choreographer = choreographer
        # 注入时钟以便测试请求身份和时间字段，不读取系统时间以外的业务状态。
        self._clock = clock
        # 状态端口由 Runtime 装配，Coordinator 是唯一可以提交新 RobotState 的业务方。
        self._state_store = state_store
        # 感知适配器只负责翻译单帧，地图和位置仍由 Coordinator 统一消费。
        self._perception_adapter = perception_adapter
        # 运行时地图由 Coordinator 写入，适配器本身不持有地图写权限。
        self._runtime_map = runtime_map
        # 位置投影器只接收已确认的视觉校正，不直接参与感知翻译。
        self._location_projector = location_projector
        # 保存当前活动剧本和动作成功后才能兑现的下一指针。
        self._plan: Optional[ChoreographyPlan] = None
        self._progress: Optional[ChoreographyProgress] = None
        # 保存唯一在途动作及其执行请求，直到终局到达或同步拒绝。
        self._current_action: Optional[Action] = None
        self._current_request: Optional[ExecutionRequest] = None
        # 记录最近成功转弯，供局部恢复引用而不要求执行器缓存历史。
        self._last_completed_turn: Optional[LastCompletedTurn] = None
        # 记录诊断信息但不把迟到事件重新解释为业务动作。
        self.diagnostics: List[str] = []

    @property
    def current_action(self) -> Optional[Action]:
        """返回当前唯一在途动作，供运行时和调试面板只读查看。"""

        return self._current_action

    @property
    def current_request(self) -> Optional[ExecutionRequest]:
        """返回当前唯一在途执行请求，未受理或已终局时为空。"""

        return self._current_request

    @property
    def is_waiting_interrupt(self) -> bool:
        """返回是否已经提交请求并等待同一身份的最终中断。"""

        return self._current_request is not None

    @property
    def last_completed_turn(self) -> Optional[LastCompletedTurn]:
        """返回最近一次成功前向转弯记录。"""

        return self._last_completed_turn

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
        if self._current_request is not None:
            self.diagnostics.append("已有在途请求，忽略重复 dispatch_next")
            return None
        # 允许首次调用装载剧本；已有活动剧本时忽略外部重复参数，保证游标所有权在协调器。
        if self._plan is None:
            if plan is None or progress is None:
                self.diagnostics.append("首次 dispatch_next 必须提供剧本和指针")
                return None
            self._plan = plan
            self._progress = progress
        elif plan is not None or progress is not None:
            self.diagnostics.append("已有活动剧本，忽略外部传入的剧本和指针")
        if self._progress is None:
            self.diagnostics.append("活动剧本缺少 resume_progress")
            return None
        # 编排器负责检查指针和运行时安全，协调器只消费其显式结果。
        result = self._choreographer.compile_next(self._plan, self._progress)
        if result.status is ChoreographyAdvanceStatus.FINISHED:
            self._plan = None
            self._progress = None
            self.diagnostics.append("活动编排流程已结束")
            return None
        if result.status is not ChoreographyAdvanceStatus.READY:
            self.diagnostics.append("编排器未返回 READY：{}".format(result.status.value))
            return None
        # 在调用执行器前原子保存剧本、动作和下一指针，保证受理期间状态完整。
        action = result.action
        self._progress = result.next_progress
        self._current_action = action
        request = self._translate_action(action)
        self._current_request = request
        # 同步拒绝表示动作从未开始，必须撤销刚才预保存的在途记录。
        ack = self._executor.submit(request)
        if not ack.accepted:
            self._clear_in_flight()
        return ack

    def handle_execution_interrupt(self, interrupt: ExecutionInterrupt) -> bool:
        """消费第一条完整匹配的终局；迟到、重复或未知身份只记录诊断。"""

        request = self._current_request
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
        # 成功终局先按动作效果投影状态，再清理在途身份，避免丢失动作语义。
        if interrupt.outcome is ExecutionOutcome.COMPLETED:
            self._project_success(self._current_action, interrupt)
            self._consume_perception(self._current_action, interrupt)
        self._clear_in_flight()
        if interrupt.outcome is not ExecutionOutcome.COMPLETED:
            self.diagnostics.append("动作 {} 以 {} 结束".format(interrupt.action_id, interrupt.outcome.value))
        return True

    def _consume_perception(self, action: Optional[Action], interrupt: ExecutionInterrupt) -> None:
        """消费观察完成中断中的唯一感知帧，并应用已确认的翻译结果。"""

        # 非观察动作不应携带感知帧，避免把错误载荷写入地图。
        if action is None or not isinstance(action.command, ObserveCommand):
            if interrupt.perception_frame is not None:
                self.diagnostics.append("非观察动作携带感知帧，已忽略")
            return
        # 观察成功必须有最终感知帧；缺失时记录协议诊断但不猜测事实。
        frame = interrupt.perception_frame
        if frame is None:
            self.diagnostics.append("观察完成中断缺少 perception_frame")
            return
        # 未装配适配器时不能越过感知边界自行解释相对事实。
        if self._perception_adapter is None:
            self.diagnostics.append("未装配感知适配器，已忽略观察帧 {}".format(frame.frame_id))
            return
        # 适配器只接收一帧，内部多帧融合对 Coordinator 透明。
        translation = self._perception_adapter.translate(frame)
        # 翻译结果必须回显同一帧身份，防止迟到帧污染当前状态。
        if translation.frame_id != frame.frame_id:
            self.diagnostics.append("感知翻译 frame_id 不匹配，已忽略")
            return
        # 无法确认时只记录原因并继续，不写地图、不校正位置、不重试观察。
        if translation.outcome is PerceptionOutcome.INCONCLUSIVE:
            self.diagnostics.append("感知翻译无法确认：{}".format(translation.reason))
            return
        # 已确认地图事实只能由 Coordinator 逐条提交 RuntimeMap。
        if self._runtime_map is None and translation.map_updates:
            self.diagnostics.append("未装配 RuntimeMap，无法应用感知地图更新")
        elif self._runtime_map is not None:
            for update in translation.map_updates:
                self._runtime_map.apply(update)
        # 已确认位置校正交给位置投影器，Coordinator 不重复实现几何换算。
        correction = translation.position_correction
        if correction is not None:
            if self._state_store is None or self._location_projector is None:
                self.diagnostics.append("未装配位置投影器，无法应用感知位置校正")
                return
            current_state = self._state_store.robot_state()
            next_state = self._location_projector.correct_from_landmark(current_state, correction)
            self._state_store.replace_robot_state(next_state)

    def _project_success(self, action: Optional[Action], interrupt: ExecutionInterrupt) -> None:
        """将当前阶段已确认的前进里程投影为边上逻辑位置。"""

        # 本阶段没有状态端口时仍允许纯串行内核运行，后续 Runtime 装配时再启用投影。
        if action is None:
            return
        self._record_completed_turn(action)
        if self._state_store is None:
            return
        effect = action.expected_effect
        if isinstance(effect, ArriveAtNodeEffect):
            # 执行层已完成驶入中心动作，协调器将逻辑位置吸附到目标路口。
            current_state = self._state_store.robot_state()
            next_state = RobotState(
                AtNode(effect.node_id, effect.entry_traversal_id),
                current_state.heading_deg,
                current_state.progress_source,
                current_state.pending_replan,
            )
            self._state_store.replace_robot_state(next_state)
            return
        if not isinstance(effect, AdvanceOnTraversalEffect):
            return
        # 执行层只反馈本次增量；缺失增量时不能伪造边上累计进度。
        if interrupt.odometry_delta_mm is None:
            self.diagnostics.append("前进成功中断缺少 odometry_delta_mm")
            return
        current_state = self._state_store.robot_state()
        from_node_id, to_node_id = effect.traversal_id.split("->", 1)
        if isinstance(current_state.location, OnCruiseEdge):
            if current_state.location.traversal_id != effect.traversal_id:
                self.diagnostics.append("前进效果与当前巡航边不一致")
                return
            base_progress_mm = current_state.location.progress_mm
        elif isinstance(current_state.location, AtNode) and current_state.location.node_id == from_node_id:
            base_progress_mm = 0.0
        else:
            self.diagnostics.append("前进效果与当前逻辑位置不匹配")
            return
        # 用新的不可变状态替换旧状态，保留朝向和待重规划标记。
        next_state = RobotState(
            OnCruiseEdge(
                effect.traversal_id,
                from_node_id,
                to_node_id,
                base_progress_mm + interrupt.odometry_delta_mm,
            ),
            current_state.heading_deg,
            ProgressSource.ODOMETRY,
            current_state.pending_replan,
        )
        self._state_store.replace_robot_state(next_state)

    def _record_completed_turn(self, action: Action) -> None:
        """从成功动作记录前向转弯，不读取执行器角度或历史。"""

        if not isinstance(action.command, TurnAtJunctionCommand):
            return
        trajectory_id = getattr(action.command, "forward_trajectory_id", None)
        if trajectory_id is None:
            self.diagnostics.append("成功转弯缺少 forward_trajectory_id")
            return
        self._last_completed_turn = LastCompletedTurn(action.action_id, trajectory_id)

    def _translate_action(self, action: Action) -> ExecutionRequest:
        """把编排动作翻译为执行器可直接路由的类型化命令。"""

        command = action.command
        if isinstance(command, ObserveCommand):
            target = ExecutionTarget.PERCEPTION_SYSTEM
            execution_command = ObserveExecutionCommand()
        elif isinstance(command, TurnAtJunctionCommand):
            # 巡航边标识不是底盘转弯轨迹；缺少标定轨迹时必须拒绝下发而不是猜测。
            trajectory_id = getattr(command, "forward_trajectory_id", None)
            if trajectory_id is None:
                raise ValueError("转弯动作缺少已标定的 forward_trajectory_id")
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = TurnExecutionCommand(trajectory_id)
        elif isinstance(command, DriveDistanceCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = DriveExecutionCommand(command.distance_mm)
        elif isinstance(command, ReverseDistanceCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = ReverseExecutionCommand(command.distance_mm)
        elif isinstance(command, RetraceTurnCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = RetraceTurnExecutionCommand(command.source_action_id)
        elif isinstance(command, ExecuteTaskCommand):
            target = ExecutionTarget.TASK_SYSTEM
            execution_command = ExecuteTaskExecutionCommand(command.task_id)
        elif isinstance(command, StopCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = StopExecutionCommand(command.reason)
        else:
            raise TypeError("未知 ActionCommand：{}".format(type(command).__name__))
        # 请求身份由协调器生成，执行器只消费本次动作必需的类型化命令。
        return ExecutionRequest(
            request_id="request:{}".format(action.action_id),
            execution_id=self._plan.choreography_id if self._plan is not None else "execution:unknown",
            action_id=action.action_id,
            target=target,
            command=execution_command,
            timestamp=self._clock(),
        )

    def _clear_in_flight(self) -> None:
        """清理当前动作和请求，但保留活动剧本供上层决定是否继续。"""

        self._current_action = None
        self._current_request = None
