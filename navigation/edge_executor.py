"""
navigation/edge_executor.py — 边执行监控器

职责:
1. 单条边的进度跟踪（里程计累积）
2. 超时检测
3. 中断信号收集（供 navigation 决策）
"""

import time
from dataclasses import dataclass, field
from typing import Optional

from .contracts import EdgeTask, EdgeTaskStatus, CrossroadEvent
from .contracts import CulvertEvent, ObstacleEvent
try:
    from .. import config as _cfg
except ImportError:
    import config as _cfg


@dataclass
class EdgeProgress:
    """边执行进度快照"""
    distance_mm: float = 0.0              # 已行走距离
    progress_ratio: float = 0.0           # 0.0 ~ 1.0
    time_elapsed_s: float = 0.0           # 已用时间
    is_stalled: bool = False              # 是否停滞（里程不再增长）
    stall_duration_s: float = 0.0         # 停滞时长
    timeout: bool = False                 # 是否超时


@dataclass
class EdgeInterrupts:
    """边执行期间收集的中断信号"""
    crossroad: Optional[CrossroadEvent] = None
    culvert: Optional[CulvertEvent] = None
    obstacle: Optional[ObstacleEvent] = None

    def any(self) -> bool:
        return self.crossroad is not None or \
               self.culvert is not None or \
               self.obstacle is not None


class EdgeExecutor:
    """
    边执行监控器 — 无状态容器，由 navigation 每 tick 调用更新。

    使用方式:
        exec = EdgeExecutor()
        exec.start(task, current_odom_mm)
        progress, interrupts = exec.update(current_odom_mm, now)

        if progress.progress_ratio > 0.7:
            # 接近路口，允许视觉检测
    """

    # 超时阈值（从 config 加载）
    @property
    def STALL_TIMEOUT_S(self):
        return _cfg.get("state_machine.edge_stall_timeout_s", 2.0)

    @property
    def EXTRA_DISTANCE_RATIO(self):
        return _cfg.get("state_machine.edge_extra_distance_ratio", 1.5)

    @property
    def STALL_ODOM_DELTA_MM(self):
        return _cfg.get("state_machine.edge_stall_odom_delta_mm", 1.0)

    def __init__(self):
        self._task: Optional[EdgeTask] = None
        self._start_odom: float = 0.0
        self._start_time: float = 0.0
        self._last_odom: float = 0.0
        self._last_odom_time: float = 0.0
        self._interrupts = EdgeInterrupts()

    # ================================================================
    # 公开接口
    # ================================================================

    def start(self, task: EdgeTask, current_odom_mm: float):
        """开始一条新边的执行"""
        self._task = task
        self._start_odom = current_odom_mm
        self._last_odom = current_odom_mm
        self._start_time = time.time()
        self._last_odom_time = self._start_time
        self._interrupts = EdgeInterrupts()
        task.status = EdgeTaskStatus.EXECUTING

    def update(self, current_odom_mm: float,
               now: float = None) -> tuple:
        """
        更新进度 → (EdgeProgress, EdgeInterrupts)
        """
        now = now or time.time()

        dist_traveled = current_odom_mm - self._start_odom
        ratio = dist_traveled / max(1e-6, self._task.distance_mm)
        elapsed = now - self._start_time

        # 停滞检测
        odom_delta = abs(current_odom_mm - self._last_odom)
        stalled = False
        stall_dur = 0.0
        if odom_delta < self.STALL_ODOM_DELTA_MM:
            stall_dur = now - self._last_odom_time
            stalled = stall_dur > self.STALL_TIMEOUT_S
        else:
            self._last_odom = current_odom_mm
            self._last_odom_time = now

        # 超时判定
        timeout = (dist_traveled > self._task.distance_mm *
                   self.EXTRA_DISTANCE_RATIO)

        progress = EdgeProgress(
            distance_mm=dist_traveled,
            progress_ratio=min(ratio, 1.0),
            time_elapsed_s=elapsed,
            is_stalled=stalled,
            stall_duration_s=stall_dur,
            timeout=timeout,
        )

        return progress, self._interrupts

    def signal_crossroad(self, event: CrossroadEvent):
        """vision/perception 上报路口检测"""
        self._interrupts.crossroad = event

    def signal_culvert(self, event: CulvertEvent):
        """vision 上报涵洞检测"""
        self._interrupts.culvert = event

    def signal_obstacle(self, event: ObstacleEvent):
        """vision 上报障碍物检测"""
        self._interrupts.obstacle = event

    def clear_interrupts(self):
        """清除中断信号（处理后调用）"""
        self._interrupts = EdgeInterrupts()

    def finish(self, status: EdgeTaskStatus = EdgeTaskStatus.DONE):
        """标记边执行完成"""
        if self._task:
            self._task.status = status

    # ================================================================
    # 查询
    # ================================================================

    @property
    def current_task(self) -> Optional[EdgeTask]:
        return self._task
