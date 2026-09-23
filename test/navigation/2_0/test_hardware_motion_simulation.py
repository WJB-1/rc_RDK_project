import unittest

from navigation.contracts import ExecutionEnvironment
from navigation.simulation.composition import build_hardware_motion_simulation_runner


class FakeTransport:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.listener = None
        self.started = False

    def subscribe(self, listener):
        self.listener = listener

    def start(self):
        self.started = True
        from motion.protocol import FrameType, build_frame
        self.listener(build_frame(FrameType.HELLO_ACK))

    def stop(self):
        self.started = False

    def send(self, frame):
        pass


class HardwareMotionSimulationTest(unittest.TestCase):
    def test_composition_uses_real_motion_and_simulated_observation_tasks(self):
        transport = FakeTransport("COM7", 115200)
        runner = build_hardware_motion_simulation_runner(
            "COM7",
            transport_factory=lambda port, baudrate: transport,
        )

        self.assertIs(runner.executor.environment(), ExecutionEnvironment.REAL_TEST)
        self.assertIs(runner.motion_port._transport, transport)
        self.assertEqual(runner.snapshot().navigation.task_progress["culvert_target"], 0)

        runner.start()

        self.assertTrue(transport.started)
        runner.stop()


if __name__ == "__main__":
    unittest.main()
