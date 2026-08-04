#!/usr/bin/env python3
"""
通信调试服务器 — C/S 可视化界面

功能:
- 打开串口 (/dev/ttyUSB0 @ 115200)
- 实时显示上位机发送的所有帧
- 实时显示下位机回复的所有帧
- 支持手动发送指令 (前进/后退/左转/右转/停止/里程清零/左转90/右转90)
- 解析帧格式 (帧头/CMD/LEN/Payload/Checksum)
- 校验和验证

使用:
    python comm_debug_server.py
    然后浏览器打开 http://<RDK_IP>:5001
"""

import sys
import time
import json
import base64
import struct
import threading
from datetime import datetime
from pathlib import Path
from collections import deque

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from communication.stm32_protocol import (
    FRAME_HEAD_0, FRAME_HEAD_1,
    CmdID, ActionCode, FbID, StmStatusCode,
    encode_velocity, encode_action, encode_turn,
    encode_intersection_turn,
    decode_frame, decode_odom, decode_status,
    frame_to_hex,
)

try:
    import serial
except ImportError:
    serial = None

try:
    from flask import Flask, render_template_string, jsonify, request
except ImportError:
    print("错误: 需要安装 flask: pip3 install flask")
    sys.exit(1)

# ============================================================
# 全局状态
# ============================================================
app = Flask(__name__)

# 串口对象
_ser = None
_serial_lock = threading.Lock()

# 通信日志 (最多保留 500 条)
_comm_log = deque(maxlen=500)
_log_lock = threading.Lock()

# 统计
_stats = {
    "tx_count": 0,
    "rx_count": 0,
    "tx_bytes": 0,
    "rx_bytes": 0,
    "start_time": None,
}

# 下位机最新状态
_latest_status = {
    "dist_mm": 0,
    "yaw_deg": 0.0,
    "speed_mms": 0,
    "stm_status": "UNKNOWN",
    "last_update": "--",
}


# ============================================================
# 日志记录
# ============================================================
def log_tx(data: bytes, desc: str = ""):
    """记录上位机发送的数据"""
    with _log_lock:
        _comm_log.append({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "TX",
            "dir_label": "上位机 → 下位机",
            "hex": data.hex(),
            "parsed": parse_frame(data),
            "desc": desc,
        })
        _stats["tx_count"] += 1
        _stats["tx_bytes"] += len(data)


def log_rx(data: bytes):
    """记录下位机接收的数据"""
    with _log_lock:
        _comm_log.append({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "RX",
            "dir_label": "下位机 → 上位机",
            "hex": data.hex(),
            "parsed": parse_frame(data),
            "desc": "",
        })
        _stats["rx_count"] += 1
        _stats["rx_bytes"] += len(data)


# ============================================================
# 帧解析
# ============================================================
def parse_frame(data: bytes) -> dict:
    """解析一帧数据，返回结构化信息"""
    result = {
        "valid": False,
        "frame_head": "",
        "cmd": "",
        "cmd_hex": "",
        "len": 0,
        "payload_hex": "",
        "checksum": "",
        "checksum_ok": False,
        "details": [],
    }

    if len(data) < 5:
        result["details"].append(f"帧太短 ({len(data)} 字节)")
        return result

    # 帧头
    if data[0] == FRAME_HEAD_0 and data[1] == FRAME_HEAD_1:
        result["frame_head"] = "A5 5A ✓"
    else:
        result["frame_head"] = f"{data[0]:02X} {data[1]:02X} ✗ (期望 A5 5A)"

    cmd_id = data[2]
    payload_len = data[3]
    result["cmd_hex"] = f"0x{cmd_id:02X}"
    result["len"] = payload_len

    # 判断方向 (注意: CMD 和 FB 有重叠的 ID，如 0x01)
    # 这里仅根据 ID 显示可能的类型，实际方向由通信方向决定
    cmd_names = []
    if cmd_id in [c.value for c in CmdID]:
        cmd_names.append(f"CMD.{CmdID(cmd_id).name}")
    if cmd_id in [f.value for f in FbID]:
        cmd_names.append(f"FB.{FbID(cmd_id).name}")
    if cmd_names:
        result["cmd"] = " / ".join(cmd_names)
    else:
        result["cmd"] = f"未知 (0x{cmd_id:02X})"

    # Payload
    if len(data) >= 5 + payload_len:
        payload = data[4:4 + payload_len]
        result["payload_hex"] = payload.hex()
    else:
        result["payload_hex"] = "不完整"
        return result

    # 校验和 (checksum 在 data[4+payload_len]，总帧长 = 5 + payload_len)
    # data[0..4+payload_len-1] 是帧头+CMD+LEN+Payload，data[4+payload_len] 是 checksum
    checksum_idx = 4 + payload_len
    if len(data) > checksum_idx:
        rx_checksum = data[checksum_idx]
        calc_checksum = sum(data[:checksum_idx]) & 0xFF
        result["checksum"] = f"0x{rx_checksum:02X}"
        result["checksum_ok"] = (rx_checksum == calc_checksum)
    else:
        result["checksum"] = "缺失"

    result["valid"] = result["checksum_ok"] and (data[0] == FRAME_HEAD_0)

    # 详细解析 payload
    if result["valid"]:
        result["details"] = parse_payload(cmd_id, payload)

    return result


def parse_payload(cmd_id: int, payload: bytes) -> list:
    """根据命令类型解析 payload"""
    details = []

    if cmd_id == CmdID.MOVE_VECTOR:
        # 下位机 move_3.c 只读取 vx（前2字节），忽略 wz
        if len(payload) >= 2:
            vx = struct.unpack("<h", payload[:2])[0]
            details.append(f"vx (线速度) = {vx} mm/s")
            if len(payload) >= 4:
                wz = struct.unpack("<h", payload[2:4])[0]
                details.append(f"wz (角速度) = {wz} mrad/s 【下位机忽略】")
            else:
                details.append("wz: 未发送（下位机只读vx）")

    elif cmd_id == CmdID.ACTION:
        if len(payload) >= 1:
            action = payload[0]
            if action == ActionCode.STOP:
                details.append("动作: 急停")
            elif action == ActionCode.RESET_ODOM:
                details.append("动作: 里程清零")
            elif action == ActionCode.START_UID_SCAN:
                details.append("动作: 开始UID扫描")
            else:
                details.append(f"动作: 未知 (0x{action:02X})")

    elif cmd_id == CmdID.TURN_IMU:
        if len(payload) >= 2:
            angle = struct.unpack("<h", payload[:2])[0]
            details.append(f"转向角度 = {angle}°")
            details.append("正数=左转, 负数=右转")

    elif cmd_id == CmdID.INTERSECTION_TURN:
        if len(payload) >= 3:
            dist = struct.unpack("<h", payload[:2])[0]
            direction = payload[2]
            details.append(f"距离路口 = {dist} mm")
            details.append(f"方向 = {'左转' if direction == 1 else '右转' if direction == 2 else '未知'}")
        elif len(payload) >= 1:
            direction = payload[0]
            details.append(f"方向 = {'左转' if direction == 1 else '右转' if direction == 2 else '未知'}")

    elif cmd_id == FbID.ODOMETRY:
        parsed = decode_odom(payload)
        if parsed:
            dist_mm, yaw_deg, speed_mms = parsed
            details.append(f"累计距离 = {dist_mm} mm")
            details.append(f"航向角 = {yaw_deg:.1f}°")
            details.append(f"当前速度 = {speed_mms} mm/s")
            # 更新全局状态
            _latest_status["dist_mm"] = dist_mm
            _latest_status["yaw_deg"] = yaw_deg
            _latest_status["speed_mms"] = speed_mms
            _latest_status["last_update"] = datetime.now().strftime("%H:%M:%S")

    elif cmd_id == FbID.STATUS:
        if len(payload) >= 1:
            status = payload[0]
            if status in StmStatusCode._value2member_map_:
                status_name = StmStatusCode(status).name
                details.append(f"状态 = {status_name}")
                _latest_status["stm_status"] = status_name
            else:
                details.append(f"状态 = 未知 ({status})")

    elif cmd_id == FbID.SENSOR:
        details.append(f"传感器数据: {payload.hex()}")

    return details


# ============================================================
# 串口通信线程
# ============================================================
def serial_rx_loop():
    """串口接收线程"""
    rx_buffer = bytearray()

    while True:
        try:
            with _serial_lock:
                if _ser is None or not _ser.is_open:
                    time.sleep(0.1)
                    continue
                if _ser.in_waiting > 0:
                    data = _ser.read(_ser.in_waiting)
                    rx_buffer.extend(data)

            # 解析帧
            while len(rx_buffer) >= 5:
                result = decode_frame(bytes(rx_buffer))
                if result is None:
                    # 找下一个帧头
                    idx = -1
                    for i in range(1, len(rx_buffer) - 1):
                        if rx_buffer[i] == FRAME_HEAD_0 and rx_buffer[i + 1] == FRAME_HEAD_1:
                            idx = i
                            break
                    if idx > 0:
                        rx_buffer = rx_buffer[idx:]
                    else:
                        break
                else:
                    cmd_id, payload, frame_total = result
                    frame_data = bytes(rx_buffer[:frame_total])
                    log_rx(frame_data)
                    rx_buffer = rx_buffer[frame_total:]

            time.sleep(0.001)
        except Exception as e:
            print(f"串口接收异常: {e}")
            time.sleep(0.1)


# ============================================================
# Flask 路由 — HTML 从 web/templates/comm_debug.html 读取
# ============================================================
_TEMPLATE_PATH = Path(__file__).parent.parent.parent.parent / "web" / "templates" / "comm_debug.html"

def _load_html():
    if _TEMPLATE_PATH.exists():
        return _TEMPLATE_PATH.read_text(encoding='utf-8')
    return "<h1>comm_debug.html not found</h1>"

HTML_TEMPLATE = _load_html()
# (旧内嵌 HTML 已提取到 web/templates/comm_debug.html)

@app.route("/")
def index():
    """主页面"""
    with _log_lock:
        logs = list(_comm_log)

    uptime = "--"
    if _stats["start_time"]:
        delta = int(time.time() - _stats["start_time"])
        uptime = f"{delta // 60}m {delta % 60}s"

    return render_template_string(
        HTML_TEMPLATE,
        connected=(_ser is not None and _ser.is_open),
        port=(_ser.port if _ser else "/dev/ttyUSB0"),
        baudrate=(_ser.baudrate if _ser else 115200),
        tx_count=_stats["tx_count"],
        rx_count=_stats["rx_count"],
        tx_bytes=_stats["tx_bytes"],
        rx_bytes=_stats["rx_bytes"],
        uptime=uptime,
        last_status=_latest_status,
        logs=logs,
    )


@app.route("/api/log")
def api_log():
    """获取最新日志"""
    with _log_lock:
        logs = list(_comm_log)

    return jsonify({
        "logs": logs,
        "stats": dict(_stats),
        "last_status": dict(_latest_status),
    })


@app.route("/api/clear", methods=["POST"])
def api_clear():
    """清空日志"""
    with _log_lock:
        _comm_log.clear()
    return jsonify({"ok": True})


@app.route("/api/send", methods=["POST"])
def api_send():
    """发送指令到下位机"""
    global _ser

    data = request.get_json()
    cmd = data.get("cmd", "")
    vx = data.get("vx", 300)
    wz = data.get("wz", 100)

    if _ser is None or not _ser.is_open:
        return jsonify({"ok": False, "error": "串口未连接"})

    # 构建指令帧
    frame = None
    desc = ""

    if cmd == "forward":
        frame = encode_velocity(vx, 0)
        desc = f"前进 vx={vx}"
    elif cmd == "backward":
        frame = encode_velocity(-vx, 0)
        desc = f"后退 vx={-vx}"
    elif cmd == "left":
        frame = encode_velocity(0, wz)
        desc = f"左转 wz={wz}"
    elif cmd == "right":
        frame = encode_velocity(0, -wz)
        desc = f"右转 wz={-wz}"
    elif cmd == "stop":
        frame = encode_action(ActionCode.STOP)
        desc = "急停"
    elif cmd == "reset_odom":
        frame = encode_action(ActionCode.RESET_ODOM)
        desc = "里程清零"
    elif cmd == "turn_left_90":
        frame = encode_turn(90)
        desc = "左转90°"
    elif cmd == "turn_right_90":
        frame = encode_turn(-90)
        desc = "右转90°"
    else:
        return jsonify({"ok": False, "error": f"未知指令: {cmd}"})

    # 发送
    try:
        with _serial_lock:
            _ser.write(frame)
            _ser.flush()
        log_tx(frame, desc)
        return jsonify({"ok": True, "hex": frame.hex()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


# ============================================================
# 主函数
# ============================================================
def main():
    global _ser

    port = "/dev/ttyUSB0"
    baudrate = 115200

    print("=" * 60)
    print("STM32 通信调试服务器")
    print("=" * 60)

    # 打开串口
    if serial is None:
        print("错误: 未安装 pyserial")
        print("请运行: pip3 install pyserial")
        sys.exit(1)

    try:
        _ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0.5)
        print(f"串口已打开: {port} @ {baudrate}bps")
    except Exception as e:
        print(f"串口打开失败: {e}")
        print(f"请检查 {port} 是否存在")
        sys.exit(1)

    _stats["start_time"] = time.time()

    # 启动接收线程
    rx_thread = threading.Thread(target=serial_rx_loop, daemon=True)
    rx_thread.start()
    print("串口接收线程已启动")

    # 启动 Flask
    print("\n请用浏览器打开: http://<RDK_IP>:5001")
    print("按 Ctrl+C 停止\n")

    try:
        app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n正在停止...")
    finally:
        if _ser and _ser.is_open:
            _ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()
