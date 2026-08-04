#!/usr/bin/env python3
"""
数据集采集脚本 — 保存摄像头原始 RGB 视频
用法:
  python robocup_rescue_brain/test/capture_video.py
  python robocup_rescue_brain/test/capture_video.py --output ./my_data --fps 15 --duration 60
  python robocup_rescue_brain/test/capture_video.py --width 1280 --height 720 --fps 30
"""

import sys
import os
import time
import signal
import argparse
from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
if PROJECT_ROOT.name == "robocup_rescue_brain":
    sys.path.insert(0, str(PROJECT_ROOT.parent))

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description="数据集采集 — 保存摄像头原始视频")
    parser.add_argument("--output", default="./captures", help="输出目录")
    parser.add_argument("--device", type=int, default=0, help="摄像头设备ID")
    parser.add_argument("--width", type=int, default=1920, help="采集分辨率宽")
    parser.add_argument("--height", type=int, default=1080, help="采集分辨率高")
    parser.add_argument("--fps", type=int, default=15, help="采集帧率")
    parser.add_argument("--duration", type=int, default=0, help="采集时长(秒), 0=手动停止")
    parser.add_argument("--fourcc", default="mp4v", help="编码格式 mp4v/h264/avc1")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- 打开摄像头 ---
    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    if not cap.isOpened():
        print(f"无法打开摄像头 device_id={args.device}")
        return

    # --- 创建视频写入器 ---
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = output_dir / f"capture_{ts}_{actual_w}x{actual_h}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
    writer = cv2.VideoWriter(str(out_path), fourcc, args.fps, (actual_w, actual_h))

    print("=" * 60)
    print("数据集采集")
    print(f"  分辨率: {actual_w}x{actual_h}")
    print(f"  帧率:   {args.fps} fps (摄像头: {actual_fps:.0f})")
    print(f"  编码:   {args.fourcc}")
    print(f"  时长:   {'手动停止' if args.duration <= 0 else f'{args.duration}s'}")
    print(f"  输出:   {out_path}")
    print("=" * 60)
    print("\n按 Ctrl+C 停止采集\n")

    # --- 信号处理 ---
    running = True
    def on_sig(sig, frame):
        nonlocal running
        print("\n停止信号收到，正在保存...")
        running = False
    signal.signal(signal.SIGINT, on_sig)

    # --- 主循环 ---
    frame_count = 0
    t0 = time.time()
    last_report = t0

    while running:
        ret, frame = cap.read()
        if not ret:
            print("读取帧失败")
            break

        writer.write(frame)
        frame_count += 1

        # 每秒打印进度
        now = time.time()
        if now - last_report >= 1.0:
            elapsed = now - t0
            fps_actual = frame_count / elapsed if elapsed > 0 else 0
            file_size = out_path.stat().st_size / 1024 / 1024 if out_path.exists() else 0
            print(f"  已采集: {frame_count:6d} 帧 | {elapsed:6.1f}s | "
                  f"实际FPS: {fps_actual:5.1f} | 文件: {file_size:.1f} MB")
            last_report = now

        if args.duration > 0 and (now - t0) >= args.duration:
            print(f"\n采集时长 {args.duration}s 已达，停止。")
            break

    # --- 清理 ---
    writer.release()
    cap.release()

    elapsed = time.time() - t0
    fps_actual = frame_count / elapsed if elapsed > 0 else 0
    file_size = out_path.stat().st_size / 1024 / 1024 if out_path.exists() else 0

    print(f"\n--- 采集完成 ---")
    print(f"  总帧数: {frame_count}")
    print(f"  时长:   {elapsed:.1f}s")
    print(f"  平均FPS: {fps_actual:.1f}")
    print(f"  文件大小: {file_size:.1f} MB")
    print(f"  路径: {out_path}")


if __name__ == "__main__":
    main()
