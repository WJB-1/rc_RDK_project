"""
navigation/sim_scene.py — 仿真场景数据

定义仿真场景的数据结构：地面真值（障碍物/涵洞位置）+ 发现状态。
"""

from dataclasses import dataclass, field
from typing import Set, Dict


class SceneGenerationError(Exception):
    """场景生成失败（超过最大尝试次数仍无法生成可达场景）"""
    pass


@dataclass
class SimScene:
    """
    仿真场景 — 包含地面真值与运行时发现状态

    地面真值（生成时确定，不可变）:
        seed, obstacle_edge_ids, culvert_edge_ids,
        obstacle_offsets, culvert_offsets

    运行时状态（仿真过程中更新）:
        discovered_obstacles, discovered_culverts（已发现）
        recon_culverts（已探索/侦查，仅涵洞有）
    """
    seed: int
    attempt: int
    obstacle_edge_ids: Set[int] = field(default_factory=set)
    culvert_edge_ids: Set[int] = field(default_factory=set)
    obstacle_offsets: Dict[int, float] = field(default_factory=dict)
    culvert_offsets: Dict[int, float] = field(default_factory=dict)
    discovered_obstacles: Set[int] = field(default_factory=set)
    discovered_culverts: Set[int] = field(default_factory=set)
    recon_culverts: Set[int] = field(default_factory=set)  # 已探索（发现后经过时）

    def to_dict(self) -> dict:
        """导出为可 JSON 序列化的字典"""
        return {
            "seed": self.seed,
            "attempt": self.attempt,
            "obstacle_edge_ids": sorted(self.obstacle_edge_ids),
            "culvert_edge_ids": sorted(self.culvert_edge_ids),
            "discovered_obstacles": sorted(self.discovered_obstacles),
            "discovered_culverts": sorted(self.discovered_culverts),
            "recon_culverts": sorted(self.recon_culverts),
        }
