"""
navigation/line_graph.py — 向后兼容 re-export 层

线图已迁移到 `navigation/domain/line_graph.py`。
本文件保留以维持 `from navigation.line_graph import X` 旧路径不变。
"""
from .domain.line_graph import (
    edge_heading, turn_angle_degs, classify_turn, turn_cost,
    iter_out_edges, has_safe_exit, LineGraph,
)

__all__ = [
    "edge_heading", "turn_angle_degs", "classify_turn", "turn_cost",
    "iter_out_edges", "has_safe_exit", "LineGraph",
]
