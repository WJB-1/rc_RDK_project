"""
navigation/edge_executor.py — 向后兼容 re-export 层

边执行监控已迁移到 `navigation/control/edge_executor.py`。
本文件保留以维持 `from navigation.edge_executor import EdgeExecutor` 旧路径不变。
"""
from .control.edge_executor import EdgeExecutor, EdgeProgress, EdgeInterrupts

__all__ = ["EdgeExecutor", "EdgeProgress", "EdgeInterrupts"]
