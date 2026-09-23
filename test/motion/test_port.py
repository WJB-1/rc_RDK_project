import unittest

from motion.port import MotionPort
from navigation.contracts import (
    CorrectExecutionCommand,
    DriveExecutionCommand,
    ExecutionOutcome,
    ExecutionRequest,
    ExecutionTarget,
    ReverseExecutionCommand,
)


class FakeTransport:
    def __init__(self):
        self.frames = []
        self.listener = None

    def start(self):
        pass

    def stop(self):
        pass

    def send(self, frame):
        self.frames.append(frame)

    def subscribe(self, listener):
        self.listener = listener


class FakeVision:
    def __init__(self):
        self.events = []

    def start_correction(self, session_id):
        self.events.append(("start", session_id))

    def stop_correction(self, session_id):
        self.events.append(("stop", session_id))


class MotionPortTest(unittest.TestCase):
    @staticmethod
    def _complete_handshake(transport):
        from motion.protocol import FrameType, build_frame
        transport.listener(build_frame(FrameType.HELLO_ACK))

    def test_correct_forwards_action_and_vision_without_local_completion(self):
        transport = FakeTransport()
        port = MotionPort(transport)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            CorrectExecutionCommand(), 0.0,
        )

        ack = port.submit(request, completions.append)
        self.assertTrue(ack.accepted)
        self.assertEqual(transport.frames[0], bytes.fromhex("c33c011001010792"))
        port.send_vision_correction("request-1", 25, 25.0)
        self.assertEqual(transport.frames[1], bytes.fromhex("c33c0111041900fa00578b"))
        self.assertEqual(completions, [])

    def test_stm32_action_done_completes_correct(self):
        transport = FakeTransport()
        vision = FakeVision()
        port = MotionPort(transport, vision_port=vision)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            CorrectExecutionCommand(), 0.0,
        )
        port.submit(request, completions.append)
        from motion.protocol import Action, FrameType, build_frame
        transport.listener(build_frame(FrameType.ACTION_ACK, bytes((Action.CORRECT,))))
        transport.listener(build_frame(FrameType.ACTION_DONE, bytes((Action.CORRECT, 0))))
        self.assertEqual(completions[0].outcome, ExecutionOutcome.COMPLETED)
        self.assertEqual(vision.events, [("start", "request-1"), ("stop", "request-1")])

    def test_late_done_for_another_action_is_ignored(self):
        transport = FakeTransport()
        port = MotionPort(transport)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            CorrectExecutionCommand(), 0.0,
        )
        port.submit(request, completions.append)
        from motion.protocol import Action, FrameType, build_frame
        transport.listener(build_frame(FrameType.ACTION_DONE, bytes((Action.STRAIGHT, 0))))
        self.assertEqual(completions, [])

    def test_cancel_accepts_stm32_stop_completion(self):
        transport = FakeTransport()
        port = MotionPort(transport)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            CorrectExecutionCommand(), 0.0,
        )
        port.submit(request, completions.append)
        port.cancel("request-1", "operator stop")
        from motion.protocol import Action, FrameType, build_frame
        transport.listener(build_frame(FrameType.ACTION_DONE, bytes((Action.STOP, 1))))
        self.assertEqual(completions[0].outcome, ExecutionOutcome.CANCELLED)

    def test_straight_completion_uses_command_distance_without_odometry(self):
        transport = FakeTransport()
        port = MotionPort(transport)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            DriveExecutionCommand(320.0), 0.0,
        )

        port.submit(request, completions.append)
        from motion.protocol import Action, FrameType, build_frame
        transport.listener(build_frame(FrameType.ACTION_DONE, bytes((Action.STRAIGHT, 0))))

        self.assertEqual(completions[0].outcome, ExecutionOutcome.COMPLETED)
        self.assertEqual(completions[0].odometry_delta_mm, 320.0)

    def test_reverse_completion_uses_command_distance_without_odometry(self):
        transport = FakeTransport()
        port = MotionPort(transport)
        self._complete_handshake(transport)
        completions = []
        request = ExecutionRequest(
            "request-1", "execution-1", "action-1", ExecutionTarget.MOTION_CONTROLLER,
            ReverseExecutionCommand(120.0), 0.0,
        )

        port.submit(request, completions.append)
        from motion.protocol import Action, FrameType, build_frame
        transport.listener(build_frame(FrameType.ACTION_DONE, bytes((Action.STRAIGHT, 0))))

        self.assertEqual(completions[0].odometry_delta_mm, 120.0)
