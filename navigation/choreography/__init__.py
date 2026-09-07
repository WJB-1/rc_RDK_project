"""Navigation 2.0 的纯编排公共入口。

谁调用：后续 `Coordinator` 在取得路线或恢复计划后创建并推进流程。
谁响应：本包只提供 `Choreographer` 和它读取的 `MotionProfile`。
输入输出：输入规划层语义计划与只读状态查询；输出不可变流程剧本和类型化动作。
状态影响：不直接提交执行器、不写入机器人状态、动态地图或任务状态。
"""

# 重新导出纯编排器，调用方无需依赖内部实现文件路径。
from .choreographer import Choreographer
# 重新导出不可变运动标定参数，调用方可在运行时装配阶段统一创建。
from .profile import MotionProfile

# 限制外部仅依赖已冻结的编排入口，避免私有实现细节泄漏。
__all__ = ("Choreographer", "MotionProfile")
