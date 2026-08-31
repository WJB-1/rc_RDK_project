"""
navigation/scene_generator.py — 向后兼容 re-export 层

场景生成已迁移到 `navigation/perception/simulation/scene_generator.py`。
本文件保留以维持 `from navigation.scene_generator import generate_scene` 旧路径不变。
"""
from .perception.simulation.scene_generator import generate_scene

__all__ = ["generate_scene"]
