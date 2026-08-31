# Navigation

导航运行时的唯一主链：

```text
event → NavigationRuntime → ControlLayer → RoutePlanner
      → GoalSelector → Choreographer → ActionCommand → ActionFeedback
```

入口是 `NavigationRuntime.dispatch(event)`；执行方通过 `next_action()` 取得
`ActionCommand`。本包不应导入 `waste/` 中的旧状态机或旧全局规划器。

目录：`contracts` 定义边界；`domain` 保存事实；`planning` 计算路径；`control`
选择与编排行为；`perception` 翻译观察；`simulation` 驱动确定性仿真；`offline`
仅存放不参与运行时的分析工具。

## Public runtime API

```python
runtime = NavigationRuntime(topology: RaceTrackTopology | None = None,
                            clock: Callable[[], float] | None = None)
runtime.set_goal_supplier(supplier: Callable[[], Iterable[Goal]]) -> None
runtime.set_actuator(actuator: IActuator) -> None
runtime.start() -> None
runtime.dispatch(event: OdomUpdate | MapUpdateIntent | ActionFeedback | object) -> ActionPlan | None
runtime.plan() -> ActionPlan | None
runtime.next_action() -> ActionCommand | None
runtime.snapshot() -> RuntimeSnapshot
```

`RuntimeSnapshot` contains `pose: NavigationPose`, `map_version: int`,
`active_action_id: str`, and `started: bool`. `dispatch()` is the only event
entrypoint; callers never mutate `runtime_map`, `mission_state`, or the action queue.

## 接口如何使用

`main.py`、真实 I/O 或仿真器是本层调用方。它们只向 `dispatch(event)` 报告发生过的事实：里程、地图更新或动作回执。运行时按事件类型更新 pose、提交地图事实或推进动作；调用方不接触内部地图和任务状态。

需要下一步动作时，调用方先调用 `plan()`。它返回当前 `ActionPlan`，没有可执行计划则返回 `None`。再调用 `next_action()` 取得一条 `ActionCommand` 发给底盘；底盘完成后，必须用相同 `action_id` 构造 `ActionFeedback` 回传给 `dispatch()`，否则动作不会推进。
