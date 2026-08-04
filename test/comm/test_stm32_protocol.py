"""
单元测试：STM32 通讯协议 V2.1

验证帧格式、编解码与下位机 move.c 协议一致。
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
import struct

from communication.stm32_protocol import (
    FRAME_HEAD_0, FRAME_HEAD_1,
    CmdID, ActionCode, TurnDirection, FbID, StmStatusCode,
    _build_frame, decode_frame,
    encode_velocity, encode_action, encode_turn,
    encode_servo, encode_intersection_turn,
    decode_odom, decode_status, decode_sensor_uid,
    frame_to_hex, decode_velocity_cmd, decode_turn_cmd,
)


class TestFrameFormat(unittest.TestCase):
    """测试帧格式是否符合下位机规范"""

    def test_frame_header(self):
        """帧头必须是 A5 5A"""
        frame = _build_frame(CmdID.ACTION, bytes([ActionCode.STOP]))
        self.assertEqual(frame[0], FRAME_HEAD_0)
        self.assertEqual(frame[1], FRAME_HEAD_1)

    def test_frame_structure(self):
        """帧结构: [A5][5A][CMD][LEN][Payload][Checksum]"""
        payload = bytes([0x01])
        frame = _build_frame(CmdID.ACTION, payload)
        # 总长度 = 5 + len(payload) = 6
        self.assertEqual(len(frame), 6)
        self.assertEqual(frame[2], CmdID.ACTION)  # CMD
        self.assertEqual(frame[3], 1)              # LEN = payload长度
        self.assertEqual(frame[4], 0x01)           # Payload
        # 校验: A5 + 5A + 02 + 01 + 01 = 0xA5 + 0x5A + 0x02 + 0x01 + 0x01 = 0x103
        # 低8位 = 0x03
        self.assertEqual(frame[5], 0x03)

    def test_checksum_calculation(self):
        """校验和 = 前面所有字节累加和低8位"""
        # 手动计算: A5 + 5A + 01 + 04 + vx_L + vx_H + wz_L + wz_H
        vx, wz = 300, 50
        payload = struct.pack("<hh", vx, wz)
        frame = _build_frame(CmdID.MOVE_VECTOR, payload)
        expected_sum = sum(frame[:-1]) & 0xFF
        self.assertEqual(frame[-1], expected_sum)

    def test_decode_valid_frame(self):
        """解析有效帧"""
        payload = bytes([ActionCode.STOP])
        frame = _build_frame(CmdID.ACTION, payload)
        result = decode_frame(frame)
        self.assertIsNotNone(result)
        cmd_id, decoded_payload, frame_len = result
        self.assertEqual(cmd_id, CmdID.ACTION)
        self.assertEqual(decoded_payload, payload)
        self.assertEqual(frame_len, len(frame))

    def test_decode_with_garbage_prefix(self):
        """解析带垃圾前缀的帧"""
        payload = bytes([ActionCode.RESET_ODOM])
        frame = _build_frame(CmdID.ACTION, payload)
        garbage = bytes([0x00, 0x11, 0x22]) + frame
        result = decode_frame(garbage)
        self.assertIsNotNone(result)
        cmd_id, decoded_payload, frame_len = result
        self.assertEqual(cmd_id, CmdID.ACTION)
        self.assertEqual(decoded_payload, payload)
        # frame_len 应该只包含有效帧长度，不包含垃圾
        self.assertEqual(frame_len, len(frame))

    def test_decode_incomplete_frame(self):
        """解析不完整帧返回 None"""
        incomplete = bytes([FRAME_HEAD_0, FRAME_HEAD_1, CmdID.ACTION, 1])
        result = decode_frame(incomplete)
        self.assertIsNone(result)

    def test_decode_invalid_checksum(self):
        """校验失败返回 None"""
        frame = _build_frame(CmdID.ACTION, bytes([ActionCode.STOP]))
        # 篡改最后一个字节
        corrupted = frame[:-1] + bytes([(frame[-1] + 1) & 0xFF])
        result = decode_frame(corrupted)
        self.assertIsNone(result)


class TestEncoders(unittest.TestCase):
    """测试编码器"""

    def test_encode_velocity(self):
        """速度指令编码 — 下位机只接收 vx (2字节)"""
        frame = encode_velocity(300, 50)
        # 帧头
        self.assertEqual(frame[0], FRAME_HEAD_0)
        self.assertEqual(frame[1], FRAME_HEAD_1)
        # CMD
        self.assertEqual(frame[2], CmdID.MOVE_VECTOR)
        # LEN = 2 (只发送 vx，wz 被下位机忽略)
        self.assertEqual(frame[3], 2)
        # Payload: vx=300
        vx = struct.unpack("<h", frame[4:6])[0]
        self.assertEqual(vx, 300)

    def test_encode_velocity_clamping(self):
        """速度限幅 — 下位机只接收 vx (2字节)"""
        frame = encode_velocity(1000, -1000)
        vx = struct.unpack("<h", frame[4:6])[0]
        self.assertEqual(vx, 500)   # 限幅到500

    def test_encode_action_stop(self):
        """急停指令"""
        frame = encode_action(ActionCode.STOP)
        self.assertEqual(frame[2], CmdID.ACTION)
        self.assertEqual(frame[3], 1)
        self.assertEqual(frame[4], ActionCode.STOP)

    def test_encode_action_reset_odom(self):
        """里程清零指令 (0x02)"""
        frame = encode_action(ActionCode.RESET_ODOM)
        self.assertEqual(frame[4], 0x02)

    def test_encode_turn_left(self):
        """左转90度 (正数=左转)"""
        frame = encode_turn(90)
        self.assertEqual(frame[2], CmdID.TURN_IMU)
        self.assertEqual(frame[3], 2)
        angle = struct.unpack("<h", frame[4:6])[0]
        self.assertEqual(angle, 90)

    def test_encode_turn_right(self):
        """右转90度 (负数=右转)"""
        frame = encode_turn(-90)
        angle = struct.unpack("<h", frame[4:6])[0]
        self.assertEqual(angle, -90)

    def test_encode_turn_clamping(self):
        """转向角度限幅"""
        frame = encode_turn(200)
        angle = struct.unpack("<h", frame[4:6])[0]
        self.assertEqual(angle, 180)

    def test_encode_turn_unit_is_degree(self):
        """确认角度单位是度，不是0.1度"""
        frame = encode_turn(90)
        angle = struct.unpack("<h", frame[4:6])[0]
        # 如果是0.1度，这里应该是900
        self.assertEqual(angle, 90)

    def test_encode_servo(self):
        """舵机控制"""
        frame = encode_servo(90)
        self.assertEqual(frame[2], CmdID.SERVO)
        self.assertEqual(frame[4], 90)

    def test_encode_intersection_turn_left(self):
        """路口左转 — payload: [distance_L][distance_H][direction] 共3字节"""
        frame = encode_intersection_turn("left")
        self.assertEqual(frame[2], CmdID.INTERSECTION_TURN)
        # LEN = 3 (distance int16 + direction uint8)
        self.assertEqual(frame[3], 3)
        # direction 在 payload 的第3字节 (frame[6])
        self.assertEqual(frame[6], TurnDirection.LEFT)

    def test_encode_intersection_turn_right(self):
        """路口右转 — direction 在 payload 第3字节 (frame[6])"""
        frame = encode_intersection_turn("right")
        self.assertEqual(frame[6], TurnDirection.RIGHT)

    def test_encode_intersection_turn_invalid(self):
        """无效方向抛出异常"""
        with self.assertRaises(ValueError):
            encode_intersection_turn("up")


class TestDecoders(unittest.TestCase):
    """测试解码器"""

    def test_decode_odom_v4(self):
        """解析4字节里程 (下位机当前版本)"""
        payload = struct.pack("<I", 12345)
        result = decode_odom(payload)
        self.assertIsNotNone(result)
        dist, yaw, speed = result
        self.assertEqual(dist, 12345)
        self.assertEqual(yaw, 0.0)   # 无yaw
        self.assertEqual(speed, 0)    # 无speed

    def test_decode_odom_v6(self):
        """解析6字节里程+yaw (下位机扩展后)"""
        payload = struct.pack("<Ih", 12345, 900)  # yaw = 90.0度
        result = decode_odom(payload)
        self.assertIsNotNone(result)
        dist, yaw, speed = result
        self.assertEqual(dist, 12345)
        self.assertAlmostEqual(yaw, 90.0)
        self.assertEqual(speed, 0)

    def test_decode_odom_v8(self):
        """解析8字节完整数据"""
        payload = struct.pack("<Ihh", 12345, -900, 300)  # yaw=-90度, speed=300
        result = decode_odom(payload)
        self.assertIsNotNone(result)
        dist, yaw, speed = result
        self.assertEqual(dist, 12345)
        self.assertAlmostEqual(yaw, -90.0)
        self.assertEqual(speed, 300)

    def test_decode_odom_too_short(self):
        """payload太短返回None"""
        result = decode_odom(bytes([0x01, 0x02]))
        self.assertIsNone(result)

    def test_decode_status_idle(self):
        """解析空闲状态"""
        result = decode_status(bytes([StmStatusCode.IDLE]))
        self.assertEqual(result, StmStatusCode.IDLE)

    def test_decode_status_turning(self):
        """解析转向中状态"""
        result = decode_status(bytes([StmStatusCode.TURNING]))
        self.assertEqual(result, StmStatusCode.TURNING)

    def test_decode_sensor_uid(self):
        """解析RFID UID"""
        uid_str = "N5"
        payload = bytes([len(uid_str)]) + uid_str.encode("ascii")
        result = decode_sensor_uid(payload)
        self.assertEqual(result, "N5")

    def test_decode_sensor_uid_empty(self):
        """空payload返回None"""
        result = decode_sensor_uid(b"")
        self.assertIsNone(result)


class TestEnumConsistency(unittest.TestCase):
    """测试枚举值与下位机一致"""

    def test_cmd_id_values(self):
        """命令ID值"""
        self.assertEqual(CmdID.MOVE_VECTOR, 0x01)
        self.assertEqual(CmdID.ACTION, 0x02)
        self.assertEqual(CmdID.TURN_IMU, 0x03)
        self.assertEqual(CmdID.SERVO, 0x04)
        self.assertEqual(CmdID.INTERSECTION_TURN, 0x05)

    def test_action_code_values(self):
        """动作码值"""
        self.assertEqual(ActionCode.STOP, 0x01)
        self.assertEqual(ActionCode.RESET_ODOM, 0x02)
        self.assertEqual(ActionCode.START_UID_SCAN, 0x03)

    def test_fb_id_values(self):
        """反馈ID值"""
        self.assertEqual(FbID.ODOMETRY, 0x01)
        self.assertEqual(FbID.STATUS, 0x02)
        self.assertEqual(FbID.SENSOR, 0x03)

    def test_status_code_values(self):
        """状态码值"""
        self.assertEqual(StmStatusCode.IDLE, 0)
        self.assertEqual(StmStatusCode.TURNING, 1)
        self.assertEqual(StmStatusCode.TURN_DONE, 2)
        self.assertEqual(StmStatusCode.OBSTACLE, 3)
        # 确认没有ERROR=4
        self.assertFalse(hasattr(StmStatusCode, 'ERROR'))


class TestDebugHelpers(unittest.TestCase):
    """测试调试辅助函数"""

    def test_frame_to_hex(self):
        """十六进制格式化"""
        frame = bytes([0xA5, 0x5A, 0x01, 0x04, 0x2C, 0x01, 0x32, 0x00, 0xB3])
        hex_str = frame_to_hex(frame)
        self.assertEqual(hex_str, "A5 5A 01 04 2C 01 32 00 B3")

    def test_decode_velocity_cmd(self):
        """解析速度指令payload"""
        payload = struct.pack("<hh", 300, 50)
        vx, wz = decode_velocity_cmd(payload)
        self.assertEqual(vx, 300)
        self.assertEqual(wz, 50)

    def test_decode_turn_cmd(self):
        """解析转向指令payload"""
        payload = struct.pack("<h", 90)
        angle = decode_turn_cmd(payload)
        self.assertEqual(angle, 90)


class TestRoundTrip(unittest.TestCase):
    """测试编解码往返一致性"""

    def test_velocity_roundtrip(self):
        """速度指令往返 — 下位机只接收 vx，wz 不发送"""
        original = encode_velocity(300, 50)
        result = decode_frame(original)
        self.assertIsNotNone(result)
        cmd_id, payload, _ = result
        self.assertEqual(cmd_id, CmdID.MOVE_VECTOR)
        vx, wz = decode_velocity_cmd(payload)
        self.assertEqual(vx, 300)
        self.assertIsNone(wz)  # wz 不发送，解码返回 None

    def test_turn_roundtrip(self):
        """转向指令往返"""
        original = encode_turn(-90)
        result = decode_frame(original)
        self.assertIsNotNone(result)
        cmd_id, payload, _ = result
        self.assertEqual(cmd_id, CmdID.TURN_IMU)
        angle = decode_turn_cmd(payload)
        self.assertEqual(angle, -90)

    def test_action_roundtrip(self):
        """动作指令往返"""
        original = encode_action(ActionCode.RESET_ODOM)
        result = decode_frame(original)
        self.assertIsNotNone(result)
        cmd_id, payload, _ = result
        self.assertEqual(cmd_id, CmdID.ACTION)
        self.assertEqual(payload[0], ActionCode.RESET_ODOM)


if __name__ == "__main__":
    unittest.main()
