#!/usr/bin/env python3
"""
最小调试面板启动脚本 — 不依赖摄像头、串口、模型。
只启动 Web 面板 + 导航引擎（路径规划 API + 地图可视化）。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from web import WebPushServer
from navigation.map_topology import get_topology
from navigation.map_oracle import MapOracle
from navigation.map_config import MISSION_NODES, NODE_COORDS, LANE_WIDTH_MM

topo = get_topology()
oracle = MapOracle(topo)

# 预生成 patrol_path（前端模拟器用）
patrol_path = oracle.query_shortest_path("START", list(MISSION_NODES))

web = WebPushServer(host="0.0.0.0", port=9090)

web.set_map_topology(
    nodes={name: node.to_dict() for name, node in topo.nodes.items()},
    edges=[edge.to_dict() for edge in topo.edges],
)

web.set_base_map_data({
    "patrol_path": patrol_path,
    "lane_width_mm": LANE_WIDTH_MM,
    "field_size_mm": [3200, 4400],
    "node_coords": {k: dict(v) for k, v in NODE_COORDS.items()},
})

# 启动 Flask（阻塞运行）
web.run()
