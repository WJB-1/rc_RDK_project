"""实现三个与真实目标边界一致的异步仿真端口。"""

from collections import deque

from navigation.contracts import (
    DispatchAck, ExecutionOutcome, ExecutionTarget, ExecutionRequest,
    ExecutionTargetPort, TargetCompletion,
    DriveExecutionCommand, ReverseExecutionCommand, TurnExecutionCommand,
    RetraceTurnExecutionCommand, StopExecutionCommand,
    ObserveExecutionCommand, ExecuteTaskExecutionCommand,
)
from .world import SimWorld


class _SimPort:
    """保存待完成请求并在显式 complete_next 时推进一个终局。"""

    target = None
    command_types = ()

    def __init__(self, world: SimWorld) -> None:
        self.world = world
        self._pending = deque()

    def submit(self, request: ExecutionRequest, completion_sink) -> DispatchAck:
        """受理匹配目标的请求，不同步执行动作。"""

        if request.target is not self.target:
            return DispatchAck(request.request_id, False, "目标不匹配", rejection_code="TARGET_MISMATCH")
        if not isinstance(request.command, self.command_types):
            return DispatchAck(request.request_id, False, "命令不匹配", rejection_code="COMMAND_MISMATCH")
        self._pending.append((request, completion_sink))
        return DispatchAck(request.request_id, True, "")

    def cancel(self, request_id: str, reason: str) -> DispatchAck:
        """取消待执行请求并发布唯一取消终局。"""

        for index, (request, sink) in enumerate(self._pending):
            if request.request_id == request_id:
                del self._pending[index]
                _publish(sink, TargetCompletion(ExecutionOutcome.CANCELLED, self.world.snapshot().now, error_code=reason))
                return DispatchAck(request_id, True, reason)
        return DispatchAck(request_id, False, "请求不在端口队列", rejection_code="NOT_ACTIVE")

    def complete_next(self):
        """推进队首请求并通过注入出口发布终局，空队列返回 None。"""

        if not self._pending:
            return None
        request, sink = self._pending.popleft()
        completion = self._complete(request.command)
        _publish(sink, completion)
        return completion

    def has_pending(self):
        """返回端口是否有待推进请求。"""

        return bool(self._pending)

    def _complete(self, command):
        raise NotImplementedError


class SimMotionPort(_SimPort):
    """消费运动命令并把世界运动结果转换为 TargetCompletion。"""

    target = ExecutionTarget.MOTION_CONTROLLER
    command_types = (DriveExecutionCommand, ReverseExecutionCommand, TurnExecutionCommand,
                     RetraceTurnExecutionCommand, StopExecutionCommand)

    def _complete(self, command):
        result = self.world.execute_motion(command)
        return TargetCompletion(result.outcome, result.timestamp, result.odometry_delta_mm,
                                result.actual_heading_deg, error_code=result.error_code)


class SimPerceptionPort(_SimPort):
    """消费观察命令并返回一帧相对感知事实。"""

    target = ExecutionTarget.PERCEPTION_SYSTEM
    command_types = (ObserveExecutionCommand,)

    def _complete(self, command):
        frame = self.world.observe()
        return TargetCompletion(ExecutionOutcome.COMPLETED, self.world.snapshot().now,
                                perception_frame=frame)


class SimTaskPort(_SimPort):
    """消费任务命令并返回虚拟任务系统终局。"""

    target = ExecutionTarget.TASK_SYSTEM
    command_types = (ExecuteTaskExecutionCommand,)

    def _complete(self, command):
        result = self.world.execute_task(command)
        return TargetCompletion(result.outcome, result.timestamp, task_result=result.task_result,
                                error_code=result.error_code)


def _publish(sink, completion):
    """兼容正式 publish 接口和测试使用的可调用 sink。"""

    if hasattr(sink, "publish"):
        sink.publish(completion)
    else:
        sink(completion)
