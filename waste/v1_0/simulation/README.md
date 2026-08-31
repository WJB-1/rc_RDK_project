# Simulation

新仿真围绕 `NavigationRuntime` 的公开事件与反馈接口工作。

- `runtime_simulator.py`：`SimClock`、`SimWorld`、`RuntimeSimulator`；
- `virtual_actuator.py`：实现 `IActuator` 的无状态仿真底盘。

闭环：`SimWorld → MapUpdateIntent → NavigationRuntime → ActionCommand → RuntimeSimulator → ActionFeedback`。
仿真不得直接调用 `RuntimeMap.mark_*()` 或 `MissionState.succeed()`。

## Interfaces

```python
clock = SimClock(now: float = 0.0)
clock() -> float
clock.advance(seconds: float) -> None

world = SimWorld(pending_map_intents: list[MapUpdateIntent] = [])
world.observe() -> tuple[MapUpdateIntent, ...]

simulator = RuntimeSimulator(runtime: NavigationRuntime,
                             world: SimWorld | None = None,
                             clock: SimClock | None = None)
simulator.step() -> bool
simulator.publish_world_events() -> int
simulator.run_actions(limit: int = 100) -> int

VirtualActuator.send(command: ActionCommand) -> ActionFeedback
VirtualActuator.cancel(reason: str = "") -> None
```

`step()` returns `False` when no plan/action remains; otherwise it emits odometry
and exactly one completion feedback for the action it executed.

## 接口如何使用

测试代码构造 `RuntimeSimulator(runtime, world, clock)`。`SimWorld` 保存待被观察到的地图意图；`publish_world_events()` 逐条把它们送进 `runtime.dispatch()`，而不是直接改地图。

`step()` 从运行时取当前动作；若是行驶则发送 `OdomUpdate`，随后发送相同 `action_id` 的 `ActionFeedback`。返回 `True` 表示完成了一条仿真动作，返回 `False` 表示没有动作可执行。`run_actions(limit)` 返回实际执行次数，供测试断言。
