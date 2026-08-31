# Domain

领域事实与规则，不包含流程调度。

- `topology.py`：赛道静态拓扑；
- `runtime_map.py`：动态地图唯一写入者，使用 `apply(MapUpdateIntent)`，对外给出 `snapshot()`；
- `mission_state.py`：任务的 `PENDING / EXECUTING / SUCCEEDED / COOLDOWN` 生命周期；
- `cost_policy.py`：路径成本规则；
- `line_graph.py`：方向、转向成本和禁止普通 UTURN 的规则。

本层不得依赖控制层、仿真器、UART 或视觉实现。

## RuntimeMap API

```python
runtime_map.apply(intent: MapUpdateIntent) -> bool
runtime_map.snapshot() -> RuntimeMapSnapshot
```

`apply()` returns `True` only when it accepts a new fact and increments the
snapshot `version`. Supported `kind` values: `block_edge`, `discover_culvert`,
`recon_culvert`, `visit_node`, `culvert_endpoint`, `set_culvert_targets`.

`RuntimeMapSnapshot` exposes immutable `frozenset` values: `visited_nodes`,
`blocked_edges`, `discovered_culverts`, `recon_culverts`, `culvert_targets`, plus
`culvert_endpoints` and `version`.

## MissionState API

```python
begin(task_id: str) -> bool
succeed(task_id: str) -> None
fail(task_id: str, now: float, cooldown_s: float) -> None
is_candidate(task_id: str, now: float = 0.0) -> bool
is_completed(task_id: str) -> bool
status(task_id: str) -> MissionStatus
```

## 接口如何使用

运行时把地图意图传给 `RuntimeMap.apply(intent)`；返回 `True` 表示这是一个新事实，地图版本号已经增加，返回 `False` 表示重复、冲突或无效。控制层和规划器只调用 `snapshot()` 读取不可修改的地图视图，不能直接改集合。

任务执行结果处理器调用 `MissionState.begin()`、`succeed()` 或 `fail()`。`is_candidate()` 告诉控制层此任务当前是否可再次选择；`is_completed()` 只有在明确成功回执后才返回 `True`，车辆经过目标不构成成功。
