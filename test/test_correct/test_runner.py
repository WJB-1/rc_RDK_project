import unittest


class FakeSerial:
    is_open = True
    in_waiting = 0

    def __init__(self, *args, **kwargs):
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        return None

    def close(self):
        self.is_open = False


class RunnerTests(unittest.TestCase):
    def test_debug_preview_updates_are_limited_to_five_fps(self):
        from runner import DebugRunner

        runner = DebugRunner("unused", 115200, 5002, vision_only=True)
        runner._last_preview_update_at = 10.0

        self.assertFalse(runner._preview_update_due(now=10.1))
        self.assertTrue(runner._preview_update_due(now=10.2))
    def test_lane_detection_mode_command_forwards_selected_mode_to_tracker(self):
        from runner import DebugRunner

        class FakeTracker:
            def __init__(self):
                self.selected_mode = None

            def set_semantic_lane_mode(self, mode):
                self.selected_mode = mode
                return mode

        tracker = FakeTracker()
        runner = DebugRunner("loopback", 115200, 5002)
        runner._vision_tracker = tracker

        runner.handle_command("set_lane_detection_mode", {"mode": "semantic"})

        self.assertEqual(tracker.selected_mode, "semantic")

    def test_lane_angle_is_sent_as_deci_degree_yaw(self):
        from runner import DebugRunner

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._send(__import__("protocol").vision_yaw(25.0), "VISION_YAW")

        self.assertEqual(serial_device.writes[-1], bytes.fromhex("c33c011102fa002444"))

    def test_start_starts_rx_before_sending_handshake(self):
        from runner import DebugRunner

        events = []
        runner = DebugRunner("loopback", 115200, 5002)
        runner._prepare_vision = lambda: events.append("vision")
        runner.connect = lambda: events.append("connect")
        runner._start_rx_loop = lambda: events.append("rx")
        runner._create_app = lambda: type("App", (), {"run": lambda self, **kwargs: events.append("web")})()

        runner.start()

        self.assertEqual(events, ["vision", "rx", "connect", "web"])

    def test_vision_only_start_skips_serial_and_starts_visual_worker(self):
        from runner import DebugRunner, LinkState

        events = []
        runner = DebugRunner("unused", 115200, 5002, vision_only=True)
        runner._prepare_vision = lambda: events.append("prepare")
        runner._start_rx_loop = lambda: events.append("rx")
        runner.connect = lambda: events.append("connect")
        runner._start_vision_loop = lambda: events.append("vision")
        runner._create_app = lambda: type("App", (), {"run": lambda self, **kwargs: events.append("web")})()

        runner.start()

        self.assertEqual(events, ["prepare", "vision", "web"])
        self.assertEqual(runner.snapshot()["state"], LinkState.IDLE.value)

    def test_vision_only_loop_is_not_gated_by_motion_action(self):
        from runner import DebugRunner, LinkState

        runner = DebugRunner("unused", 115200, 5002, vision_only=True)
        runner._state = LinkState.IDLE

        self.assertTrue(runner._should_process_vision())

    def test_connect_command_resends_hello_on_an_open_serial(self):
        """Detects the reconnect button becoming a no-op after the port is open."""
        from runner import DebugRunner, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        first_hello = serial_device.writes[-1]
        runner._state = LinkState.FAULT

        runner.handle_command("connect", {})

        self.assertEqual(serial_device.writes[-1], first_hello)
        self.assertEqual(runner.snapshot()["state"], LinkState.NEGOTIATING.value)

    def test_recording_is_disabled_by_default(self):
        """Detects image or metadata writing being enabled during link testing."""
        from runner import DebugRunner

        recorder_creations = []
        runner = DebugRunner(
            "loopback",
            115200,
            5002,
            recorder_factory=lambda *args: recorder_creations.append(args),
        )

        self.assertIsNone(runner._create_recorder(lambda *_: True))
        self.assertEqual(recorder_creations, [])

    def test_straight_command_sends_protocol_frame_after_handshake(self):
        """Detects a Web straight command mapped to the wrong UART frame."""
        from runner import DebugRunner, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._state = LinkState.IDLE
        runner.handle_command("straight", {"distance_mm": 500})

        self.assertEqual(serial_device.writes[-1], bytes.fromhex("c33c0110040400f401f8df"))

    def test_backward_straight_command_sets_backward_direction_field(self):
        from runner import DebugRunner, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._state = LinkState.IDLE
        runner.handle_command("straight", {"distance_mm": 500, "direction": "backward"})

        self.assertEqual(serial_device.writes[-1], bytes.fromhex("c33c0110040401f401c8e8"))

    def test_straight_command_starts_visual_worker_before_ack_without_visual_tx(self):
        from runner import DebugRunner, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._state = LinkState.IDLE
        started = []
        runner._start_vision_loop = lambda: started.append(True)

        runner.handle_command("straight", {"distance_mm": 500})

        self.assertEqual(started, [True])
        self.assertEqual(len(serial_device.writes), 2)
        self.assertEqual(serial_device.writes[-1], bytes.fromhex("c33c0110040400f401f8df"))

    def test_straight_ack_opens_visual_transmit_gate(self):
        from runner import DebugRunner, FrameType, Action, LinkState

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: FakeSerial())
        runner.connect()
        runner._state = LinkState.WAITING
        runner._pending_action = Action.STRAIGHT
        runner._vision_action = Action.STRAIGHT
        runner._handle_frame(FrameType.ACTION_ACK, bytes((Action.STRAIGHT,)))

        self.assertTrue(runner._vision_tx_enabled.is_set())

    def test_motion_done_closes_visual_transmit_gate(self):
        from runner import DebugRunner, FrameType, Action, LinkState

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: FakeSerial())
        runner._state = LinkState.CORRECTING
        runner._vision_action = Action.STRAIGHT
        runner._vision_tx_enabled.set()
        runner._handle_frame(FrameType.ACTION_DONE, bytes((Action.STRAIGHT, 0)))

        self.assertFalse(runner._vision_tx_enabled.is_set())

    def test_motion_command_is_rejected_before_hello_ack(self):
        """Detects movement frames sent before the lower controller handshake."""
        from runner import DebugRunner

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()

        with self.assertRaises(RuntimeError):
            runner.handle_command("correct", {})

    def test_correct_ack_sends_initial_vision_keepalive_before_camera_startup(self):
        from runner import DebugRunner, FrameType, Action, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._state = LinkState.IDLE
        runner._handle_frame(FrameType.ACTION_ACK, bytes((Action.CORRECT,)))

        self.assertEqual(serial_device.writes[-1], bytes.fromhex("c3 3c 01 11 04 00 80 00 80 5f 35"))

    def test_correct_heartbeat_keeps_lower_controller_alive_during_inference(self):
        from runner import DebugRunner, LinkState

        serial_device = FakeSerial()
        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: serial_device)
        runner.connect()
        runner._state = LinkState.CORRECTING
        runner._start_vision_heartbeat()
        import time
        time.sleep(0.12)
        runner._vision_heartbeat_stop.set()
        runner._vision_heartbeat_thread.join(timeout=1.0)

        self.assertGreaterEqual(len(serial_device.writes), 2)

    def test_action_done_vision_timeout_is_visible_in_status(self):
        from runner import DebugRunner, FrameType, Action, LinkState

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: FakeSerial())
        runner._state = LinkState.CORRECTING
        runner._handle_frame(FrameType.ACTION_DONE, bytes((Action.CORRECT, 3)))

        self.assertIn("VISION_TIMEOUT", runner.snapshot()["error"])


if __name__ == "__main__":
    unittest.main()
