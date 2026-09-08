"""把仿真和导航公开快照转换为稳定的 Web JSON 视图。"""

from dataclasses import fields, is_dataclass
from enum import Enum
from typing import Any


def _json_value(value: Any) -> Any:
    """递归转换领域值，禁止把内部对象直接交给 Flask。"""
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {field.name: _json_value(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (set, frozenset, tuple, list)):
        items = [_json_value(item) for item in value]
        return sorted(items, key=str) if isinstance(value, (set, frozenset)) else items
    if hasattr(value, "__dict__") and not isinstance(value, (str, bytes)):
        return {key: _json_value(item) for key, item in vars(value).items() if not key.startswith("_")}
    return value


class WebStateBridge:
    """只读取 Runner 快照并建立前端专用视图模型。"""

    def __init__(self, runner: Any) -> None:
        """保存仿真控制面引用；不保存可变快照。"""
        if runner is None or not callable(getattr(runner, "snapshot", None)):
            raise TypeError("runner 必须提供 snapshot()")
        self._runner = runner

    def snapshot_payload(self) -> dict:
        """返回可直接编码为 JSON 的组合快照。"""
        simulation = self._runner.snapshot()
        simulation_data = _json_value(simulation)
        if not isinstance(simulation_data, dict):
            simulation_data = {"value": simulation_data}
        navigation_snapshot = getattr(simulation, "navigation", None)
        if navigation_snapshot is None:
            runtime = getattr(self._runner, "navigation_runtime", None) or getattr(self._runner, "runtime", None)
            navigation_snapshot = runtime.snapshot() if runtime is not None and callable(getattr(runtime, "snapshot", None)) else None
        navigation_data = _json_value(navigation_snapshot) if navigation_snapshot is not None else {}
        if not isinstance(navigation_data, dict):
            navigation_data = {"value": navigation_data}
        world = simulation_data.pop("world", None)
        if world is None:
            world = {key: simulation_data.pop(key) for key in ("pose", "world_pose", "truth_blocked_edge_ids", "truth_culvert_edge_ids") if key in simulation_data}
        if not isinstance(world, dict):
            world = {"value": world}
        truth = world.pop("truth", None)
        if truth is None:
            truth = {}
            for key in ("truth_blocked_edge_ids", "truth_culvert_edge_ids"):
                if key in world:
                    truth[key.replace("truth_", "")] = world.pop(key)
        world["truth"] = truth if isinstance(truth, dict) else {"value": truth}
        if "runtime_map" not in navigation_data:
            store = getattr(self._runner, "state_store", None)
            navigation_data["runtime_map"] = _json_value(store.runtime_map_snapshot()) if store is not None and callable(getattr(store, "runtime_map_snapshot", None)) else {}
        return {"simulation": simulation_data, "world": world, "navigation": navigation_data}

