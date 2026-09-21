"""实现按目标路由请求并收敛异步终局的执行器。"""

# 导入字典和可选类型，保存执行器当前已受理的最小路由记录。
from typing import Dict, Iterable, List, Optional, Tuple

from navigation.contracts import (
    DispatchAck,
    DriveExecutionCommand,
    ExecuteTaskExecutionCommand,
    ExecutionCommand,
    ExecutionEnvironment,
    ExecutionInterrupt,
    ExecutionOutcome,
    ExecutionRequest,
    ExecutionTarget,
    ExecutionTargetPort,
    ObserveExecutionCommand,
    RetraceTurnExecutionCommand,
    CorrectExecutionCommand,
    ReverseExecutionCommand,
    StopExecutionCommand,
    TargetCompletion,
    TargetCompletionSink,
    TurnExecutionCommand,
)


class BoundCompletionSink:
    """绑定单条请求身份并只允许发布一次终局的执行器内部闸门。

    谁创建：`RoutedExecutor.submit()` 为每次受理尝试创建。
    谁调用：目标端口在动作完成、失败、超时或取消时调用 `publish()`。
    状态影响：只有执行器开放闸门后的第一次发布会生成 `ExecutionInterrupt`。
    """

    def __init__(self, request: ExecutionRequest, callback, diagnostics: List[str]) -> None:
        """冻结请求身份、上行回调和诊断列表。"""

        # 保存不可变请求引用，防止目标端口自行伪造导航身份。
        self._request = request
        # 保存执行器唯一的导航终局出口。
        self._callback = callback
        # 复用执行器诊断列表记录过早、重复或非法终局。
        self._diagnostics = diagnostics
        # 目标端口返回 accepted=True 前，闸门保持关闭。
        self._opened = False
        # 同一请求只允许消费一次终局。
        self._published = False

    def open(self) -> None:
        """在目标端口受理请求后开放终局发布。"""

        self._opened = True

    def close(self) -> None:
        """关闭终局发布闸门。"""

        self._opened = False

    def publish(self, completion: TargetCompletion) -> None:
        """校验并发布一次无身份终局，重复调用只记录诊断。"""

        # 端口尚未受理或执行器已关闭该路由时，不能产生导航终局。
        if not self._opened:
            self._diagnostics.append("目标端口在终局闸门开放前发布结果")
            return
        # 已消费的终局不能再次推进 Coordinator。
        if self._published:
            self._diagnostics.append("目标端口重复发布终局：{}".format(self._request.request_id))
            return
        # 只接受正式的 TargetCompletion，拒绝任意对象越过契约边界。
        if not isinstance(completion, TargetCompletion):
            self._diagnostics.append("目标端口发布了无效终局对象")
            return
        # 先锁定消费状态，再调用外部回调，避免回调重入产生第二次消费。
        self._published = True
        self._opened = False
        # 将目标端口事实与冻结的请求身份组合成导航唯一终局。
        self._callback(
            ExecutionInterrupt(
                request_id=self._request.request_id,
                execution_id=self._request.execution_id,
                action_id=self._request.action_id,
                source=self._request.target,
                outcome=completion.outcome,
                timestamp=completion.timestamp,
                odometry_delta_mm=completion.odometry_delta_mm,
                actual_heading_deg=completion.actual_heading_deg,
                task_result=completion.task_result,
                perception_frame=completion.perception_frame,
                error_code=completion.error_code,
                details=completion.details,
            )
        )


class RoutedExecutor:
    """按目标和命令类型路由请求的统一异步执行器。

    谁调用：`Coordinator` 调用 `submit()`、`cancel()` 和 `on_interrupt()`。
    谁响应：注入的 `ExecutionTargetPort` 负责真实或仿真的具体 I/O。
    状态影响：执行器只保存请求路由和一次性终局闸门，不修改地图、位姿或任务。
    """

    def __init__(
        self,
        environment: ExecutionEnvironment,
        ports: Iterable[ExecutionTargetPort],
    ) -> None:
        """创建固定环境的路由器并校验每个目标只有一个端口。"""

        # 环境在实例创建时固定，运行中不允许在仿真和真实后端间切换。
        if not isinstance(environment, ExecutionEnvironment):
            raise TypeError("environment 必须是 ExecutionEnvironment")
        self._environment = environment
        # 将端口复制成字典，避免装配方在运行中替换路由。
        self._ports: Dict[ExecutionTarget, ExecutionTargetPort] = {}
        for port in ports:
            target = getattr(port, "target", None)
            if not isinstance(target, ExecutionTarget):
                raise TypeError("目标端口必须提供 ExecutionTarget target")
            if target in self._ports:
                raise ValueError("同一 ExecutionTarget 不能装配多个端口")
            self._ports[target] = port
        # 保存当前请求到绑定闸门的路由记录。
        self._routes: Dict[str, BoundCompletionSink] = {}
        # 保存唯一导航终局消费者。
        self._listener = None
        # 执行器只记录诊断，不把错误转换成导航业务事实。
        self._diagnostics: List[str] = []

    def submit(self, request: ExecutionRequest) -> DispatchAck:
        """选择目标端口并提交一条异步请求。"""

        # 只接受正式请求对象，避免字符串协议重新进入执行层。
        if not isinstance(request, ExecutionRequest):
            raise TypeError("request 必须是 ExecutionRequest")
        # 同一请求身份不能并行占用两个目标端口。
        if request.request_id in self._routes:
            return DispatchAck(request.request_id, False, "请求已经在途", rejection_code="DUPLICATE_REQUEST")
        # 目标不存在时同步拒绝，且不产生任何终局。
        port = self._ports.get(request.target)
        if port is None:
            return DispatchAck(request.request_id, False, "未装配目标端口", rejection_code="TARGET_UNAVAILABLE")
        # 目标和命令类型不匹配时在路由边界拒绝。
        if not self._command_matches_target(request.command, request.target):
            return DispatchAck(
                request.request_id,
                False,
                "目标与命令类型不匹配",
                rejection_code="TARGET_COMMAND_MISMATCH",
            )
        # 先登记关闭的身份闸门，阻止目标端口同步抢跑终局。
        sink = BoundCompletionSink(request, self._publish_interrupt, self._diagnostics)
        self._routes[request.request_id] = sink
        try:
            # 目标端口只得到请求和无身份终局出口。
            ack = port.submit(request, sink)
        except Exception as error:
            # 端口启动异常视为同步拒绝，删除临时路由且不产生中断。
            sink.close()
            self._routes.pop(request.request_id, None)
            return DispatchAck(request.request_id, False, str(error), rejection_code="TARGET_SUBMIT_ERROR")
        # 端口必须返回统一受理确认。
        if not isinstance(ack, DispatchAck):
            sink.close()
            self._routes.pop(request.request_id, None)
            return DispatchAck(request.request_id, False, "目标端口返回了无效受理结果", rejection_code="INVALID_ACK")
        if not ack.accepted:
            # 未受理请求绝不允许产生导航终局。
            sink.close()
            self._routes.pop(request.request_id, None)
            return ack
        # 受理后才开放终局闸门；目标端口此后通过 sink 异步回调。
        sink.open()
        return ack

    def cancel(self, request_id: str, reason: str) -> DispatchAck:
        """向当前请求所属端口转发取消，不提前吞掉原请求终局。"""

        # 未知请求不调用任何端口，也不创建虚假终局。
        sink = self._routes.get(request_id)
        if sink is None:
            return DispatchAck(request_id, False, "请求不在途", rejection_code="NOT_ACTIVE")
        request = sink._request
        port = self._ports[request.target]
        try:
            return port.cancel(request_id, reason)
        except Exception as error:
            return DispatchAck(request_id, False, str(error), rejection_code="TARGET_CANCEL_ERROR")

    def on_interrupt(self, callback) -> None:
        """注册唯一导航终局回调，重复注册直接拒绝。"""

        if self._listener is not None:
            raise ValueError("执行器只能注册一个终局回调")
        self._listener = callback

    def environment(self) -> ExecutionEnvironment:
        """返回创建时固定的执行环境。"""

        return self._environment

    @property
    def diagnostics(self) -> Tuple[str, ...]:
        """返回执行器诊断信息的只读副本。"""

        return tuple(self._diagnostics)

    def _publish_interrupt(self, interrupt: ExecutionInterrupt) -> None:
        """消费绑定闸门生成的唯一终局并交给 Coordinator。"""

        # 回调尚未注册时不丢失安全诊断，但也不伪造其他业务出口。
        if self._listener is None:
            self._diagnostics.append("执行器尚未注册终局消费者")
        else:
            self._listener(interrupt)
        # 回调完成后删除路由，使迟到结果无法再次进入导航。
        self._routes.pop(interrupt.request_id, None)

    @staticmethod
    def _command_matches_target(command: ExecutionCommand, target: ExecutionTarget) -> bool:
        """验证命令变体只能提交给其所属执行目标。"""

        target_commands = {
            ExecutionTarget.MOTION_CONTROLLER: (
                DriveExecutionCommand,
                ReverseExecutionCommand,
                TurnExecutionCommand,
                RetraceTurnExecutionCommand,
                CorrectExecutionCommand,
                StopExecutionCommand,
            ),
            ExecutionTarget.PERCEPTION_SYSTEM: (ObserveExecutionCommand,),
            ExecutionTarget.TASK_SYSTEM: (ExecuteTaskExecutionCommand,),
        }
        return isinstance(command, target_commands.get(target, ()))


class RealExecutor(RoutedExecutor):
    """真实测试环境执行器，具体通信由注入的目标端口负责。"""

    def __init__(self, ports: Iterable[ExecutionTargetPort]) -> None:
        """创建固定为 REAL_TEST 的路由执行器。"""

        super().__init__(ExecutionEnvironment.REAL_TEST, ports)
