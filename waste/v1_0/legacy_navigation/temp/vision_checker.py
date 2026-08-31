"""
navigation/vision_checker.py — 向后兼容 re-export 层

视觉模拟已迁移到 `navigation/perception/simulation/vision_checker.py`。
本文件保留以维持 `from navigation.vision_checker import VisionChecker` 旧路径不变。
"""
from .perception.simulation.vision_checker import VisionChecker

__all__ = ["VisionChecker"]
