"""把编排器 Action 转换为执行器可直接消费的类型化请求。"""

from navigation.contracts import (
    Action,
    DispatchAck,
    DriveDistanceCommand,
    DriveExecutionCommand,
    CorrectExecutionCommand,
    CorrectPoseCommand,
    ExecuteTaskCommand,
    ExecuteTaskExecutionCommand,
    ExecutionRequest,
    ExecutionTarget,
    IAsyncExecutor,
    ObserveCommand,
    ObserveExecutionCommand,
    RetraceTurnCommand,
    RetraceTurnExecutionCommand,
    ReverseDistanceCommand,
    ReverseExecutionCommand,
    StopCommand,
    StopExecutionCommand,
    TurnAtJunctionCommand,
    TurnExecutionCommand,
)


class ExecutionBridge:
    """集中封装 Action 到 ExecutionRequest 的唯一翻译规则。"""

    @staticmethod
    def translate(action: Action, choreography_id: str, timestamp: float) -> ExecutionRequest:
        """将一条编排动作转换为带身份的异步执行请求。"""

        command = action.command
        if isinstance(command, ObserveCommand):
            target = ExecutionTarget.PERCEPTION_SYSTEM
            execution_command = ObserveExecutionCommand(command.scope.value, command.traversal_id)
        elif isinstance(command, CorrectPoseCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = CorrectExecutionCommand()
        elif isinstance(command, TurnAtJunctionCommand):
            if command.forward_trajectory_id is None:
                raise ValueError("转弯动作缺少已标定的 forward_trajectory_id")
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = TurnExecutionCommand(command.forward_trajectory_id, command.target_traversal_id)
        elif isinstance(command, DriveDistanceCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = DriveExecutionCommand(command.distance_mm)
        elif isinstance(command, ReverseDistanceCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = ReverseExecutionCommand(command.distance_mm)
        elif isinstance(command, RetraceTurnCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = RetraceTurnExecutionCommand(command.retrace_trajectory_id)
        elif isinstance(command, ExecuteTaskCommand):
            target = ExecutionTarget.TASK_SYSTEM
            execution_command = ExecuteTaskExecutionCommand(command.task_id)
        elif isinstance(command, StopCommand):
            target = ExecutionTarget.MOTION_CONTROLLER
            execution_command = StopExecutionCommand(command.reason)
        else:
            raise TypeError("未知 ActionCommand：{}".format(type(command).__name__))
        return ExecutionRequest(
            request_id="request:{}".format(action.action_id),
            execution_id=choreography_id,
            action_id=action.action_id,
            target=target,
            command=execution_command,
            timestamp=timestamp,
        )

    @classmethod
    def submit(cls, action: Action, choreography_id: str, executor: IAsyncExecutor, timestamp: float) -> DispatchAck:
        """翻译并向执行器提交一条动作，返回同步受理结果。"""

        return executor.submit(cls.translate(action, choreography_id, timestamp))
