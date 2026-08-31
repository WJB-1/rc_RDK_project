# Control

控制层负责“选什么、如何变成动作”，不负责搜索图或访问硬件。

- `control_layer.py`：生成候选、触发路径评估、处理中断；
- `goal_selector.py`：从 `CandidateRoute[]` 按效用选择目标；
- `choreographer.py`：将 `NavigationPlan` 编译为 `ActionPlan`，按 `ActionFeedback` 推进；
- `edge_action_packages.py`：长边、短边、隧道的动作包；
- `task_execution.py` / `recovery_execution.py`：任务和恢复结果边界。

只允许依赖 `contracts`、`domain` 和 `planning`。不得导入旧状态机、真实 UART 或仿真世界。

## ControlLayer API

```python
set_goal_supplier(supplier: Callable[[], Iterable[Goal]]) -> None
generate_candidates(checkpoint_goals: Iterable[Goal] = (),
                    culvert_goals: Iterable[Goal] = (), now: float = 0.0) -> tuple[Goal, ...]
start() -> None
handle_event(event: object) -> None
tick(feedback: ActionFeedback | None = None) -> NavigationPlan | None
on_plan_finished(result: object = None) -> None
on_plan_interrupted(reason: str = "") -> None
```

`tick()` calls the planner with all supplied goals, selects one route, and asks
the choreographer to compile it. Its observable state is `ControlSnapshot`:
`phase`, `selected_goal`, and `map_version`.

## Selection and choreography

```python
GoalSelector.select(routes: Iterable[CandidateRoute], policy: CostPolicy) -> CandidateRoute | None
Choreographer.compile(plan: NavigationPlan) -> ActionPlan
Choreographer.step(feedback: ActionFeedback) -> ActionPlan | None
Choreographer.cancel(reason: str = "") -> None
Choreographer.current_action -> ActionCommand | None
```

## 接口如何使用

`NavigationRuntime` 调用 `ControlLayer.tick()`。控制层从目标供应函数取得全部 `Goal`，读取地图快照，调用规划器评估全部目标，再交给 `GoalSelector`。成功时返回一个 `NavigationPlan`，没有可达候选时返回 `None`；控制层不返回 UART 指令。

运行时把 `NavigationPlan` 交给 `Choreographer.compile()`，得到一份 `ActionPlan`。执行一条动作后，运行时将 `ActionFeedback` 传给 `step()`；编排器只推进当前动作，或在失败时取消计划，绝不重新选择目标。
