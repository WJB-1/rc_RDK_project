"""
simulation/virtual_robot_bridge/ — 虚拟下位机

底盘运动的仿真替身。签名对齐 communication/robot_bridge.py，
实际由控制层调用（债务 P-23，当前暂由 sim_engine 代调）。
"""
from .virtual_robot_bridge import VirtualRobotBridge

__all__ = ["VirtualRobotBridge"]
