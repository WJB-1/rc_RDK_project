"""Motion 2.1 真实底盘端口：动作下发、视觉数据转发和终局映射。"""

import time

from navigation.contracts import (
    CorrectExecutionCommand,
    DispatchAck,
    DriveExecutionCommand,
    ExecutionOutcome,
    ExecutionTarget,
    RetraceTurnExecutionCommand,
    ReverseExecutionCommand,
    StopExecutionCommand,
    TargetCompletion,
    TurnExecutionCommand,
)
from .chassis_driver import ChassisDriver
from .protocol import Action, FrameDecoder, FrameType, TurnDirection


class MotionPort:
    target = ExecutionTarget.MOTION_CONTROLLER

    def __init__(self, transport, clock=None, chassis_driver=None, vision_port=None):
        self._transport = transport
        self._chassis = chassis_driver or ChassisDriver(transport)
        self._vision = vision_port
        self._clock = clock or time.monotonic
        self._decoder = FrameDecoder()
        self._active = None
        self._sink = None
        self._ready = False
        self._cancel_requested = False
        self._vision_stopped = True
        self._transport.subscribe(self._on_bytes)

    def start(self):
        self._transport.start()
        self._chassis.send_hello()

    def stop(self):
        if self._active is not None:
            self._stop_vision(self._active.request_id)
        self._transport.stop()
        self._ready = False

    def submit(self, request, completion_sink):
        if request.target is not self.target:
            return DispatchAck(request.request_id, False, "target mismatch", rejection_code="TARGET_MISMATCH")
        if not self._ready:
            return DispatchAck(request.request_id, False, "motion link not ready", rejection_code="MOTION_NOT_READY")
        if self._active is not None:
            return DispatchAck(request.request_id, False, "motion busy", rejection_code="MOTION_BUSY")
        self._active = request
        self._sink = completion_sink
        self._cancel_requested = False
        try:
            self._send_command(request.command)
        except Exception:
            self._active = None
            self._sink = None
            raise
        return DispatchAck(request.request_id, True, acceptance_evidence="FRAME_SUBMITTED")

    def cancel(self, request_id, reason):
        if self._active is None or self._active.request_id != request_id:
            return DispatchAck(request_id, False, "request not active", rejection_code="NOT_ACTIVE")
        self._chassis.send_stop()
        self._cancel_requested = True
        self._stop_vision(request_id)
        return DispatchAck(request_id, True, reason)

    def send_vision_correction(self, request_id, offset_mm, target_yaw_deg):
        if self._active is None or not self._requires_vision(self._active.command) or self._active.request_id != request_id:
            raise RuntimeError("no active vision-enabled request")
        self._chassis.send_vision_correction(offset_mm, target_yaw_deg)

    def send_vision_processing(self, request_id):
        if self._active is None or not self._requires_vision(self._active.command) or self._active.request_id != request_id:
            raise RuntimeError("no active vision-enabled request")
        self._chassis.send_vision_processing()

    def _send_command(self, command):
        if isinstance(command, CorrectExecutionCommand):
            self._chassis.send_action(Action.CORRECT)
            return
        if isinstance(command, DriveExecutionCommand):
            self._chassis.send_straight(command.distance_mm, TurnDirection.FORWARD)
            return
        if isinstance(command, ReverseExecutionCommand):
            self._chassis.send_straight(command.distance_mm, TurnDirection.BACKWARD)
            return
        if isinstance(command, TurnExecutionCommand):
            self._chassis.send_turn(self._turn_action(command.forward_trajectory_id), TurnDirection.FORWARD)
            return
        if isinstance(command, RetraceTurnExecutionCommand):
            self._chassis.send_turn(self._turn_action(command.retrace_trajectory_id), TurnDirection.BACKWARD)
            return
        if isinstance(command, StopExecutionCommand):
            self._chassis.send_stop()
            return
        raise TypeError("unsupported motion command: {}".format(type(command).__name__))

    @staticmethod
    def _turn_action(trajectory_id):
        name = str(trajectory_id).lower()
        if "left" in name:
            return Action.TURN_LEFT
        if "right" in name:
            return Action.TURN_RIGHT
        raise ValueError("trajectory id must identify left or right turn")

    def _on_bytes(self, data):
        for frame_type, payload in self._decoder.feed(data):
            if frame_type is FrameType.HELLO_ACK:
                self._ready = True
            elif frame_type is FrameType.ACTION_ACK and self._active is not None:
                self._on_action_ack(payload)
            elif frame_type is FrameType.ACTION_DONE and self._active is not None:
                if self._matches_active_action(payload):
                    self._complete(payload)
            elif frame_type is FrameType.REJECT and self._active is not None:
                if self._matches_active_action(payload):
                    self._publish(ExecutionOutcome.FAILED, error_code="REJECTED")

    def _on_action_ack(self, payload):
        if (not payload or payload[0] != self._command_action(self._active.command)
                or not self._requires_vision(self._active.command)
                or self._vision is None or not self._vision_stopped):
            return
        try:
            self._vision.start_correction(self._active.request_id)
            self._vision_stopped = False
        except Exception:
            self._publish(ExecutionOutcome.FAILED, error_code="VISION_START_FAILED")

    def _matches_active_action(self, payload):
        if not payload:
            return False
        expected = self._command_action(self._active.command)
        return payload[0] == expected or (self._cancel_requested and payload[0] == Action.STOP)

    @staticmethod
    def _requires_vision(command):
        return isinstance(command, (CorrectExecutionCommand, DriveExecutionCommand, ReverseExecutionCommand))

    @staticmethod
    def _command_action(command):
        if isinstance(command, CorrectExecutionCommand):
            return Action.CORRECT
        if isinstance(command, (DriveExecutionCommand, ReverseExecutionCommand)):
            return Action.STRAIGHT
        if isinstance(command, TurnExecutionCommand):
            return MotionPort._turn_action(command.forward_trajectory_id)
        if isinstance(command, RetraceTurnExecutionCommand):
            return MotionPort._turn_action(command.retrace_trajectory_id)
        if isinstance(command, StopExecutionCommand):
            return Action.STOP
        raise TypeError("unsupported motion command: {}".format(type(command).__name__))

    def _complete(self, payload):
        result = payload[1] if len(payload) > 1 else 2
        outcome = {0: ExecutionOutcome.COMPLETED, 1: ExecutionOutcome.CANCELLED, 2: ExecutionOutcome.ACTUATOR_FAILURE, 3: ExecutionOutcome.TIMEOUT}.get(result, ExecutionOutcome.FAILED)
        self._publish(
            outcome,
            error_code="STM32_RESULT_{}".format(result),
            odometry_delta_mm=self._fallback_odometry_delta_mm(outcome),
        )

    def _fallback_odometry_delta_mm(self, outcome):
        if outcome is not ExecutionOutcome.COMPLETED or self._active is None:
            return None
        command = self._active.command
        if isinstance(command, (DriveExecutionCommand, ReverseExecutionCommand)):
            return command.distance_mm
        return None

    def _publish(self, outcome, error_code=None, odometry_delta_mm=None):
        sink, self._sink = self._sink, None
        if self._active is not None:
            self._stop_vision(self._active.request_id)
        self._active = None
        self._cancel_requested = False
        if sink is not None:
            completion = TargetCompletion(
                outcome,
                self._clock(),
                odometry_delta_mm=odometry_delta_mm,
                error_code=error_code,
            )
            sink.publish(completion) if hasattr(sink, "publish") else sink(completion)

    def _stop_vision(self, request_id):
        if self._vision is not None and not self._vision_stopped:
            self._vision.stop_correction(request_id)
            self._vision_stopped = True
