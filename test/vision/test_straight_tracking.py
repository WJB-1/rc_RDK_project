#!/usr/bin/env python3
"""
直道循迹测试脚本 — 摄像头 + 视觉偏移 → STM32 + Web 可视化

用法:
  python robocup_rescue_brain/test/vision/test_straight_tracking.py
  python robocup_rescue_brain/test/vision/test_straight_tracking.py --port COM3 --speed 250 --no-serial

功能:
  1. 打开摄像头, 运行 BiSeNet + IPM 流水线
  2. 打开串口连接 STM32
  3. 以固定速度直行, 每帧发送 CMD_LANE_OFFSET (0x06)
  4. 启动 Web 调试服务器 (端口 5001), 浏览器实时查看分割图+IPM鸟瞰图+偏移量
  5. 实时打印串口回传的里程计和状态数据

安全:
  - Ctrl+C → STOP + 关串口 + 释放摄像头
  - offset 限幅 ±200mm
  - --no-serial 模式不连下位机, 纯视觉调试
"""

import sys
import os
import time
import math
import signal
import argparse
import threading
from pathlib import Path
from collections import deque

PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
if PROJECT_ROOT.name == "robocup_rescue_brain":
    sys.path.insert(0, str(PROJECT_ROOT.parent))

import cv2
import numpy as np
import yaml

from communication.stm32_protocol import (
    FRAME_HEAD_0, FRAME_HEAD_1,
    CmdID, ActionCode, FbID, StmStatusCode,
    encode_velocity, encode_action, encode_lane_offset,
    decode_frame, decode_odom, decode_status,
)
from hardware.camera import CameraManager
from perception.lane_tracker import LaneTracker
from navigation.map_topology import get_topology
from web import WebPushServer


# ================================================================
# 串口
# ================================================================
def open_serial(port="/dev/ttyUSB0", baudrate=115200):
    try:
        import serial
        ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0.5)
        print(f"串口已连接: {port} @ {baudrate}bps")
        return ser
    except Exception as e:
        print(f"串口打开失败: {e}")
        return None


def read_feedback(ser):
    frames = []
    try:
        if ser and ser.is_open and ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buf = bytearray(data)
            while len(buf) >= 5:
                result = decode_frame(bytes(buf))
                if result is None:
                    idx = -1
                    for i in range(1, len(buf) - 1):
                        if buf[i] == FRAME_HEAD_0 and buf[i + 1] == FRAME_HEAD_1:
                            idx = i
                            break
                    if idx > 0:
                        buf = buf[idx:]
                    else:
                        break
                else:
                    cmd_id, payload, frame_total = result
                    frames.append((cmd_id, payload))
                    buf = buf[frame_total:]
    except Exception:
        pass
    return frames


# ================================================================
# 主循环
# ================================================================
def main():
    parser = argparse.ArgumentParser(description="直道循迹测试 — 摄像头视觉 + STM32")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口设备")
    parser.add_argument("--speed", type=int, default=300, help="前进速度 mm/s")
    parser.add_argument("--no-serial", action="store_true", help="不连接串口, 纯视觉调试")
    parser.add_argument("--web-port", type=int, default=5001, help="Web 调试端口")
    parser.add_argument("--config", default="config/settings.yaml", help="配置文件路径")
    args = parser.parse_args()

    print("=" * 60)
    print("直道循迹测试 — 摄像头视觉 + STM32 偏移发送")
    print(f"  速度: {args.speed} mm/s")
    print(f"  串口: {'禁用' if args.no_serial else args.port}")
    print(f"  Web:  http://localhost:{args.web_port}")
    print("=" * 60)

    # --- 加载配置 ---
    config_path = PROJECT_ROOT / args.config
    settings = {}
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            settings = yaml.safe_load(f) or {}
    else:
        settings = {
            "cameras": {"front": {"device_id": 0, "width": 1920, "height": 1080, "fps": 30}},
            "segmentation": {}, "math_ipm": {},
        }

    # --- 初始化摄像头 ---
    print("\n[1/3] 初始化摄像头...")
    cam = CameraManager(settings)
    if not cam.initialize():
        print("摄像头初始化失败!")
        return

    # --- 初始化 LaneTracker ---
    print("[2/3] 初始化视觉追踪器...")
    tracker = LaneTracker(settings)

    # --- Web 调试服务器 ---
    print("[3/3] 启动 Web 调试服务器...")
    web = WebPushServer(host="0.0.0.0", port=args.web_port)
    topo = get_topology()
    web.set_map_topology(
        nodes={name: n.to_dict() for name, n in topo.nodes.items()},
        edges=[e.to_dict() for e in topo.edges],
    )
    web.start()

    # --- 串口 ---
    ser = None
    if not args.no_serial:
        ser = open_serial(args.port)
        if ser is None:
            print("串口失败, 仅视觉模式")
            args.no_serial = True

    # --- 状态 ---
    running = True
    odom_stats = {"dist": 0, "yaw": 0.0, "speed": 0, "status": "UNKNOWN"}
    tx_count = 0
    rx_count = 0
    frame_count = 0

    def signal_handler(sig, frame):
        nonlocal running
        print("\n收到中断信号，停止...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # --- 发送前进指令 ---
    if not args.no_serial and ser:
        ser.write(encode_velocity(args.speed, 0))
        ser.flush()
        print(f"已发送: 直行 vx={args.speed} mm/s")

    # --- 主循环 ---
    print("\n开始循环 (Ctrl+C 停止)...")
    print("浏览器打开 Web 面板查看分割图和 IPM 鸟瞰图\n")
    print(f"{'帧':>6s}  {'offset':>8s}  {'质量':>6s}  {'距离':>8s}  {'航向':>8s}")
    print("-" * 55)

    t_start = time.time()
    last_print = t_start

    while running:
        loop_start = time.time()

        # 1. 读下位机反馈
        if not args.no_serial and ser:
            for cmd_id, payload in read_feedback(ser):
                rx_count += 1
                if cmd_id == FbID.ODOMETRY:
                    parsed = decode_odom(payload)
                    if parsed:
                        odom_stats["dist"] = parsed[0]
                        odom_stats["yaw"] = parsed[1] or 0.0
                        odom_stats["speed"] = parsed[2] or 0
                elif cmd_id == FbID.STATUS:
                    parsed = decode_status(payload)
                    if parsed:
                        odom_stats["status"] = str(parsed)

        # 2. 摄像头帧 → 视觉处理
        frames = cam.get_frames()
        front = frames.get("front")

        offset_mm = 0.0
        is_intersection = False
        quality_score = 1.0
        debug_frame = None

        if front is not None:
            offset_mm, is_intersection, debug_frame = tracker.process(front)
            lane_state = tracker.last_lane_state
            if lane_state:
                quality_score = lane_state.get("quality_score", 1.0)

            # 3. 发送偏移量到 STM32
            if not args.no_serial and ser:
                ser.write(encode_lane_offset(offset_mm))
                ser.flush()
            tx_count += 1

        # 4. 推送 Web 面板
        web.update(seg_frame=debug_frame, offset_mm=offset_mm,
                   is_intersection=is_intersection, quality_score=quality_score)

        frame_count += 1

        # 5. 每秒打印
        current_time = time.time()
        if current_time - last_print >= 1.0:
            print(
                f"{frame_count:5d}  {offset_mm:+7.1f}mm  "
                f"{quality_score:.2f}  "
                f"{odom_stats['dist']:7d}mm  {odom_stats['yaw']:+7.1f}°"
            )
            frame_count = 0
            last_print = current_time

        # 6. 控频 (~20Hz 视觉)
        elapsed_loop = time.time() - loop_start
        sleep_time = max(0, 0.05 - elapsed_loop)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- 清理 ---
    print("\n正在停止...")

    if not args.no_serial and ser:
        ser.write(encode_action(ActionCode.STOP))
        ser.flush()
        print("已发送: STOP")

    cam.release()
    if not args.no_serial and ser:
        ser.close()

    elapsed = time.time() - t_start
    print(f"\n--- 测试结束 ---")
    print(f"  运行: {elapsed:.1f}s | 发送: {tx_count} 帧 ({(tx_count/max(elapsed,0.01)):.1f} Hz) | 接收: {rx_count} 帧")


if __name__ == "__main__":
    main()
