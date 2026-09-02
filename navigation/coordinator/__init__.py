"""Navigation 2.0 协调器公共入口。"""

# 导出 Coordinator，调用方不依赖内部实现文件路径。
from .coordinator import Coordinator

__all__ = ("Coordinator",)
