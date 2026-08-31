# Offline Tools

本目录只用于离线分析、报告和基准比较，运行时不得导入。

- `tsp_baseline.py`：一次性 TSP 访问顺序基准，输入 `DirectedDijkstra`，输出成本和节点顺序。

禁止 `NavigationRuntime`、`ControlLayer`、`RoutePlanner`、`RuntimeSimulator` 导入本目录。

## Interface

```python
solve_visit_order(dijkstra: DirectedDijkstra, start_node: str,
                  targets: tuple[str, ...], blocked_edges: frozenset[int] = frozenset())
    -> tuple[float, tuple[str, ...]]
```

The return value is `(total_cost, visit_order)`. It is deliberately not an
`ActionPlan` and has no event, task, or actuator side effect.

## 接口如何使用

离线报告或人工对照脚本调用 `solve_visit_order()`，传入实时 Dijkstra、起点和待访问节点。它只返回 `(总成本, 访问顺序)`，不创建动作、不更新地图、不接收回执。运行时任何一层都禁止导入本目录。
