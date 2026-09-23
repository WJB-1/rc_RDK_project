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
        runtime = getattr(self._runner, "navigation_runtime", None) or getattr(self._runner, "runtime", None)
        if "tasks" not in navigation_data and runtime is not None:
            tasks = getattr(runtime, "tasks", ())
            navigation_data["tasks"] = _json_value(tasks)
        timeline = simulation_data.get("timeline", ())
        events = []
        for item in timeline:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                events.append({"timestamp": item[0], "kind": "action_completed", "message": "completed action {}".format(item[1])})
        if navigation_data.get("state") is not None:
            events.append({"timestamp": simulation_data.get("now", 0), "kind": "navigation_state", "message": "state: {}".format(navigation_data["state"])})
        navigation_data["events"] = events
        if "map" not in navigation_data:
            topology = getattr(getattr(self._runner, "world", None), "topology", None)
            if topology is not None:
                nodes = []
                for node_id in getattr(topology, "_nodes_by_id", {}):
                    node = topology.get_node(node_id)
                    nodes.append({"node_id": node.node_id, "x_mm": node.x_mm, "y_mm": node.y_mm, "node_kind": node.node_kind})
                edges = []
                for edge in getattr(topology, "_edges_by_id", {}).values():
                    edges.append({"edge_id": edge.edge_id, "from_node_id": edge.from_node_id, "to_node_id": edge.to_node_id, "length_mm": edge.length_mm, "road_kind": edge.road_kind})
                navigation_data["map"] = {"nodes": nodes, "edges": edges}
        transport = getattr(self._runner, "transport", None)
        communication = {
            "events": list(transport.events())
            if transport is not None and callable(getattr(transport, "events", None)) else []
        }
        vision_runtime = getattr(self._runner, "vision_runtime", None)
        vision = (
            vision_runtime.snapshot()
            if vision_runtime is not None and callable(getattr(vision_runtime, "snapshot", None)) else {}
        )
        return {
            "simulation": simulation_data,
            "world": world,
            "navigation": navigation_data,
            "communication": communication,
            "vision": vision,
        }
