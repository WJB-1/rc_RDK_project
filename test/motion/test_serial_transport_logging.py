import unittest

from motion.protocol import Action, FrameType, TurnDirection, build_frame, encode_straight
from motion.serial_transport import SerialTransport


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.in_waiting = 0
        self.written = []

    def write(self, data):
        self.written.append(data)

    def flush(self):
        pass

    def read(self, size):
        return b""

    def close(self):
        self.is_open = False


class SerialTransportLoggingTest(unittest.TestCase):
    def test_records_decoded_motion_context_instead_of_only_raw_hex(self):
        transport = SerialTransport("COM7", serial_factory=FakeSerial)
        transport.set_context_provider(lambda: {"request_id": "request-7", "action_type": "Traverse"})
        transport.start()

        transport.send(encode_straight(300, TurnDirection.FORWARD))
        transport.feed(build_frame(FrameType.ACTION_DONE, bytes((Action.STRAIGHT, 0))))
        events = transport.events()
        transport.stop()

        self.assertEqual(events[0]["category"], "motion")
        self.assertEqual(events[0]["fields"]["distance_mm"], 300)
        self.assertIn("300 mm", events[0]["summary"])
        self.assertEqual(events[0]["step"]["request_id"], "request-7")
        self.assertIn("动作结束", events[1]["summary"])


if __name__ == "__main__":
    unittest.main()
