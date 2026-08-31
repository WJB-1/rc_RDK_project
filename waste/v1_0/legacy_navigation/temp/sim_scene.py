"""
navigation/sim_scene.py — 向后兼容 re-export 层

仿真场景已迁移到 `navigation/perception/simulation/scene.py`。
本文件保留以维持 `from navigation.sim_scene import SimScene` 旧路径不变。
"""
from .perception.simulation.scene import SimScene, SceneGenerationError

__all__ = ["SimScene", "SceneGenerationError"]
