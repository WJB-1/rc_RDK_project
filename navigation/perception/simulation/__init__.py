"""
perception/simulation/ — 仿真替身（真车感知的平行实现）

  - scene.py            : SimScene 地面真值（原 sim_scene）
  - vision_checker.py   : VisionChecker 假视觉（原 vision_checker）
  - scene_generator.py  : generate_scene 随机场景（原 scene_generator）
"""
from .scene import SimScene, SceneGenerationError
from .vision_checker import VisionChecker
from .scene_generator import generate_scene

__all__ = ["SimScene", "SceneGenerationError", "VisionChecker", "generate_scene"]
