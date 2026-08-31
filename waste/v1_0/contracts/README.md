# Contracts

跨层数据和端口的唯一来源。

- `events.py`：`Pose`、`OdomUpdate`、传感器事件；
- `commands.py`：`Goal`、`RouteQuery`、`CandidateRoute`、`NavigationPlan`、`ActionPlan`、`ActionFeedback`、`MapUpdateIntent`；
- `ports.py`：`RoutePlannerPort`、`ChoreographerPort`、`IActuator`。

本层只能依赖标准库；不得导入 `control`、`planning`、`simulation` 或真实 I/O。
外部模块用这些不可变记录交接，不共享可变状态。

## Core records

```python
Goal(goal_id: str, goal_type: str, node_name: str = "",
     edge_id: int | None = None, reward: float = 0.0)

MapUpdateIntent(kind: str, edge_id: int | None = None,
                node_name: str = "", value: object = None, timestamp: float = 0.0)

RouteQuery(pose: NavigationPose, candidates: tuple[Goal, ...],
           map_view: RuntimeMapView, policy: CostPolicy, start_node: str = "START")

CandidateRoute(goal: Goal, reachable: bool,
               steps: tuple[DirectedStep, ...] = (), route_cost: float = inf,
               distance_mm: float = 0.0, turn_cost: float = 0.0,
               failure_reason: str = "")

ActionCommand(action_id: str, kind: str,
              parameters: tuple[tuple[str, object], ...] = ())
ActionFeedback(action_id: str, status: str, distance_mm: float = 0.0,
               timestamp: float = 0.0, reason: str = "")
```

`ActionPlan(plan_id, actions, goal_context, map_version)` contains immutable
`ActionCommand` records. Valid action kinds are `turn`, `drive`,
`drive_to_observation`, `observe`, `task`, `reverse`, `reverse_turn`,
`forward_resume`, and `stop`.

## Ports

```python
RoutePlannerPort.evaluate(query: RouteQuery) -> Sequence[CandidateRoute]
ChoreographerPort.compile(plan: NavigationPlan) -> ActionPlan
ChoreographerPort.step(feedback: ActionFeedback) -> ActionPlan | None
ChoreographerPort.cancel(reason: str = "") -> None
IActuator.send(command: ActionCommand) -> ActionFeedback | None
IActuator.cancel(reason: str = "") -> None
```

## 接口如何使用

本层是所有层共享的数据语言，不处理逻辑。感知或仿真报告地图变化时传 `MapUpdateIntent`；控制层请求规划时传 `RouteQuery`；规划器必须返回每个目标各自的 `CandidateRoute`，而不是一个模糊的“最佳路线”。

控制层选定路线后传 `NavigationPlan` 给编排层；编排层返回 `ActionPlan`；执行器接收其中一条 `ActionCommand`，再返回同一 `action_id` 的 `ActionFeedback`。这些记录都是数据包，不携带可变对象引用，因此接收方没有权限借它改其他层状态。
