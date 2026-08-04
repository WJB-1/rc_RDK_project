"""
硬件驱动层 (Hardware Layer)

负责所有硬件外设控制:
- camera: 摄像头调度管理 (CameraManager)
- tts_syn6288: SYN6288语音合成模块 (预留)

设计原则:
1. 非阻塞式调用
2. 串口指令简单封装
"""

from .camera import CameraManager, CameraConfig

__all__ = ['CameraManager', 'CameraConfig']