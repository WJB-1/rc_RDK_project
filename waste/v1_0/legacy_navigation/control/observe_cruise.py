"""
control/observe_cruise.py — 观察点三段巡航（侧枝骨架，暂不接入主干）

这是一个【侧枝脚本】，承载「边级三段巡航」的逻辑骨架。当前主干
（cruise_state_machine.py）仍是单段巡航，本文件不改变主干任何行为，
只独立搭起三段巡航的状态机骨架，供后续子模块（感知适配层 D-10、
现场 Dijkstra D-03）就绪后整合接入。

三段巡航（普通边）：
  ① 循迹 TRACKING：从出发线循迹到 500mm 观察点
  ② 观察 OBSERVE：停车，主动调感知适配层分析路况 → 更新地图 → 决策
  ③ 驶入 COMMIT：按决策（左/右/直行）交接执行，进入下一路口

分层归属：
  - 本骨架（状态切换）属于控制层 control/
  - 段②的「分析路况」调用感知层 perception/adapter.py（子模块 D-10）
  - 段②的「更新地图」写 domain/runtime_map.py
  - 段③的「决策」调用 planning/ 的 observe_decide（子模块 D-03）
"""
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Any


class ThreeSegmentPhase(Enum):
    """三段巡航的段位（侧枝独立枚举，不并入主干 AgentState）"""
    TRACKING = auto()   # 段①循迹：出发线 → 观察点
    OBSERVE = auto()    # 段②观察：停车分析路况 + 更新地图 + 决策
    COMMIT = auto()     # 段③驶入：按决策交接执行


@dataclass
class ObserveContext:
    """观察点决策上下文（今为占位，供 observe_decide 消费）"""
    current_pose: Any = None              # (x_mm, y_mm, yaw_deg)
    perception_snapshot: Any = None       # 感知适配层产出（D-10 就绪后填）
    map_view: Any = None                  # RuntimeMap 只读视图
    current_edge: Any = None              # 当前 EdgeTask（含 to_node/expected_yaw）


@dataclass
class ObserveDecision:
    """观察点决策结果"""
    action: str                          # "straight" / "left" / "right" / "stop"
    target_node: str = ""
    expected_yaw: float = 0.0
    replan_required: bool = False        # D-03 现场算若发现图权变化置 True


def observe_decide(ctx: ObserveContext) -> ObserveDecision:
    """
    观察点决策接口（D-01 骨架，D-03 只换此函数体）。

    现状占位：直接返回 STOP。待 D-03（现场 Dijkstra）就绪后，
    改为「读 ctx.map_view + ctx.perception_snapshot」算下一路口的左/右/直。
    """
    # TODO(D-03)：现场 Dijkstra 落地前，决策留空，返回 STOP 占位。
    return ObserveDecision(action="stop")


class ThreeSegmentCruise:
    """
    三段巡航骨架（侧枝）。

    不持有真实状态机引用，只定义三段之间的切换逻辑与各段要调用的
    子模块接口点。待子模块就绪后，由主干状态机实例化并驱动。
    """

    def __init__(self):
        self.phase = ThreeSegmentPhase.TRACKING
        self._observe_deadline = 0.0
        self._pending_turn: Optional[ObserveDecision] = None
        # 子模块接口点（占位，后续注入）
        self.perception_adapter = None   # perception/adapter.py 的 PerceptionAdapter
        self.runtime_map = None           # domain/runtime_map.py 的 RuntimeMap

    # ----------------------------------------------------------------
    # 段② OBSERVE：观察分析（核心三步：感知 → 更新地图 → 决策）
    # ----------------------------------------------------------------
    def observe(self, now: float, observe_window_s: float = 0.3) -> Optional[ObserveDecision]:
        """
        观察段：停车 0.3s 稳定窗后，主动调感知适配层分析路况，更新地图，做决策。

        返回 ObserveDecision（决策定型后交给 COMMIT 段交接执行）。
        返回 None 表示还在观察稳定窗内（继续停车）。
        """
        # 稳定窗未到 → 继续停车等待
        if now < self._observe_deadline:
            return None

        # 三步之一：感知 —— 调感知适配层（D-10 就绪后返回道路分析快照）
        snapshot = None
        if self.perception_adapter is not None:
            snapshot = self.perception_adapter.observe()   # 接口点，D-10 填
        # TODO(D-10)：snapshot 产出后，转成 MapUpdateIntent 更新 RuntimeMap

        # 三步之二：更新地图 —— 把感知结果写进 RuntimeMap（接口点留白）
        # TODO(D-10)：例如 runtime_map.mark_culvert_discovered / block_edge 等

        # 三步之三：决策 —— 调 observe_decide 算下一步
        ctx = ObserveContext(
            current_pose=None,               # 由主干注入真实位姿
            perception_snapshot=snapshot,
            map_view=self.runtime_map,
            current_edge=None,               # 由主干注入当前边
        )
        decision = observe_decide(ctx)

        if decision.replan_required:
            return decision   # 交由主干触发 GLOBAL_PLANNING
        self._pending_turn = decision
        self.phase = ThreeSegmentPhase.COMMIT
        return decision

    # ----------------------------------------------------------------
    # 段③ COMMIT：决策定型 → 交接执行
    # ----------------------------------------------------------------
    def commit(self) -> ObserveDecision:
        """驶入段：把决策交接给执行层（转弯走 TURNING，直行走继续巡航）。"""
        d = self._pending_turn
        self._pending_turn = None
        if d is None:
            return ObserveDecision(action="stop")
        # 直行 vs 转弯的分支，由主干在接入时决定去向（见 brief §7 待商榷）
        return d

    # ----------------------------------------------------------------
    # 段① TRACKING：循迹（由主干 EDGE_EXECUTING 承载，本骨架只声明接口）
    # ----------------------------------------------------------------
    def reset(self):
        """重置三段巡航到段①。"""
        self.phase = ThreeSegmentPhase.TRACKING
        self._pending_turn = None
