"""
perception/adapter.py — L1 感知适配层（PerceptionAdapter）

留白（D-10）：真车「墙→涵洞/隧道」的跨层翻译尚未接入。
当前这一翻译逻辑仍糊在 main.py 的 _perception_loop 里（is_wall_detection → wall_type）。

设计接口见 docs/architecture/边级巡航控制层详细设计.md L1 层：
空间翻译(像素→边+t)、语义翻译(墙→涵洞/隧道，需查地图)、融合去重。
只读地图做消歧，产出 MapUpdateIntent / 导航事件；不写地图、不做决策。
"""


class MapUpdateIntent:
    """地图更新意图（值对象）—— 由适配层产出，交由 RuntimeMapWriter 落库。"""
    def __init__(self, edge_id: int, field: str, value: bool):
        self.edge_id = edge_id
        self.field = field            # e.g. "has_culvert" / "is_blocked"
        self.value = value


class PerceptionAdapter:
    """感知适配器 —— 留白接口，真车接入后实现。"""

    def translate(self, raw_detection, runtime_map) -> object:
        """原始检测 × 只读地图 → 导航事件 / MapUpdateIntent。留白。"""
        raise NotImplementedError(
            "PerceptionAdapter.translate 未实现（D-10）；真车接入时落地"
        )
