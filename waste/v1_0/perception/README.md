# Perception Adapter

感知层不拥有地图，也不做导航决策。

- `source.py`：原始检测源协议；
- `adapter.py`：把 `VisualObservation` 翻译为 `MapUpdateIntent`。

接口流：`VisualObservation → PerceptionAdapter → MapUpdateIntent → NavigationRuntime.dispatch()`。
适配器不得直接修改 `RuntimeMap`、`MissionState` 或动作队列。

## Interfaces

```python
DetectionSource.detect(frame: object) -> list[RawDetection]

PerceptionAdapter.translate(obs: VisualObservation, current_edge_id: int,
                            entry_heading: float) -> MapUpdateIntent | None
PerceptionAdapter.adapt(observations: list[VisualObservation], current_edge_id: int,
                        entry_heading: float) -> list[MapUpdateIntent]
```

`translate()` returns `None` for an unresolved or ignored observation. `adapt()`
deduplicates intents within one batch; its caller submits each result through
`NavigationRuntime.dispatch()`.

## 接口如何使用

视觉侧提供 `VisualObservation`；调用适配器时还必须传当前边和朝向，才能把“左前方障碍”定位成某条 `edge_id`。`translate()` 返回一个 `MapUpdateIntent`，无法定位、无效或应忽略的观察则返回 `None`。

批量观察调用 `adapt()`，得到去重后的意图列表。调用方逐个将这些意图传给 `NavigationRuntime.dispatch()`；适配器绝不调用 `RuntimeMap.apply()`，也不返回路径、目标或动作命令。
