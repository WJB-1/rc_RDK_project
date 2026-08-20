"""
perception/adapter.py — L1 感知适配层（PerceptionAdapter）

导航内唯一的「适配层」。职责：
  - 输入：外层纯视觉建模给出的 VisualObservation（纯相对事实）
  - 翻译：相对 → 绝对（哪条支路 → 地图 edge_id），加消歧
  - 产出：MapUpdateIntent，交由 RuntimeMap 落库

分界（2026-08-20 拍板）：
  - 外层（perception/ + vision/）做纯视觉建模：形态学推测哪条支路 + 差错控制。
    产出 VisualObservation（相对事实），不碰地图、不碰 edge_id。
  - 本层（navigation 内）做相对→绝对换算 + 消歧 + 批内去重，为地图更新提供数据。
  - 类型 VisualObservation/RelativeLane 定义在 vision/contracts.py（外层视觉侧），
    本层 import 它，依赖方向朝内，外层不反向依赖 navigation。

未落地（后置）：
  - 相对→绝对换算算法（D-01.2）：像素/相对支路 → 地图 edge_id 的空间翻译。
    依赖拓扑 + 绝对位姿，本次只留 translate 接口签名。
"""
from dataclasses import dataclass, field
from typing import Any, List, Optional

from vision.contracts import VisualObservation, RelativeLane


@dataclass
class MapUpdateIntent:
    """
    适配层产出的地图更新意图，交由 RuntimeMap 落库。

    update 取值与 RuntimeMap 写入口对应：
      "mark_culvert_discovered" / "block_edge" / "set_culvert_endpoint" / ...
    """
    edge_id: int
    update: str
    value: Any = None


class PerceptionAdapter:
    """
    感知适配器 —— 外层视觉事实 → 地图绝对更新。

    批接口 adapt() 是纯函数：逐条 translate + 批内去重，产出 MapUpdateIntent
    列表，不直接提交；提交动作由 perception 主流程调用 RuntimeMap 写入口完成。
    """

    def translate(self, obs: VisualObservation) -> Optional[MapUpdateIntent]:
        """
        单条翻译：VisualObservation（相对）→ MapUpdateIntent（绝对）。

        TODO(D-01.2)：相对→绝对换算算法未实现。需基于：
          - obs.relative_lane → 当前路口拓扑对应哪条边
          - 当前绝对位姿 + 朝向 → 哪条直道
          - 消歧（wall→涵洞/隧道，查当前边 is_tunnel）
        本次留白。
        """
        raise NotImplementedError(
            "PerceptionAdapter.translate 相对→绝对换算算法未实现（D-01.2）"
        )

    def adapt(self, observations: List[VisualObservation]) -> List[MapUpdateIntent]:
        """
        批封装：逐条 translate + 批内去重（同 update 类型 + 同 edge_id 只留一次）。

        纯函数，不提交；提交由 perception 主流程做，且由 RuntimeMap 写入口做
        跨批幂等兜底（双向冲突保护）。
        """
        intents: List[MapUpdateIntent] = []
        seen = set()
        for obs in observations:
            intent = self.translate(obs)
            if intent is None:
                continue
            key = (intent.update, intent.edge_id)
            if key in seen:
                continue          # 批内重复，静默丢弃
            seen.add(key)
            intents.append(intent)
        return intents
