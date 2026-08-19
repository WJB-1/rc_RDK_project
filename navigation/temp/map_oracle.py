"""
navigation/map_oracle.py — 向后兼容 re-export 层

地图真理查询已迁移到 `navigation/planning/map_oracle.py`。
本文件保留以维持 `from navigation.map_oracle import MapOracle` 旧路径不变。
"""
from .planning.map_oracle import MapOracle

__all__ = ["MapOracle"]
