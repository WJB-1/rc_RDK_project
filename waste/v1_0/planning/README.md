# Runtime Planning

实时规划只回答“从当前 pose 到指定候选目标如何合法到达”。

- `directed_dijkstra.py`：有向最短路，处理转向成本、障碍边和普通 UTURN 禁止；
- `route_planner.py`：对每个 `Goal` 返回一个 `CandidateRoute`；
- `recovery_planner.py`：生成 `reverse / reverse_turn / forward_resume` 恢复动作。

本层不选择目标、不执行动作、不计算任务收益。**严禁导入 `navigation.offline`：TSP 不是实时规划。**

## Interfaces

```python
DirectedDijkstra(topology: RaceTrackTopology | None = None)
DirectedDijkstra.shortest_path(start: str, goal: str,
                               blocked_edges: frozenset[int] = frozenset(),
                               entry_heading: float | None = None) -> tuple[float, list[str] | None]

RoutePlanner(dijkstra: DirectedDijkstra)
RoutePlanner.evaluate(query: RouteQuery) -> Sequence[CandidateRoute]

RecoveryPlanner.plan(blocked_edge: int, current_pose: object,
                     reverse_distance_mm: float = 300.0) -> RecoveryPlan
```

`shortest_path()` returns `(math.inf, None)` if unreachable; otherwise it returns
the positive route cost and a node sequence including `start` and `goal`.

## 接口如何使用

调用方是控制层。它一次传入 `RouteQuery`，其中必须有当前位置、当前地图快照和全部候选 `Goal`。`RoutePlanner.evaluate()` 返回同样数量的 `CandidateRoute`，每个都说明可达性、路径步骤、成本或失败原因；规划器从不决定哪一个目标最好。

`DirectedDijkstra.shortest_path()` 仅接收起点、终点、封锁边和进入朝向，返回 `(成本, 节点序列)`；无路时返回 `(math.inf, None)`。它不理解任务奖励、冷却或 RFID。TSP 与它没有运行时接口关系。
