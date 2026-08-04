"""
navigation/map_config.py — 静态地图数据（节点坐标、类型、RFID）

所有可调参数已迁移至 config/settings.yaml，由 config/__init__.py 统一加载。
本模块仅保留"不经常变化的"节点拓扑元数据。
"""

from typing import Dict


# ============================================================
# 坐标系定义
# ============================================================
# Y 轴向下为正（车体初始朝向），X 轴向右为正
# 原点 (0,0): 出发区 START 中心点


# ============================================================
# 节点坐标（唯一真实数据源）
# ============================================================
NODE_COORDS: Dict[str, Dict[str, float]] = {
    "START":   {"x": 0,     "y": 0},
    "J_START": {"x": 0,     "y": 200},
    # 左纵列 (X=-400)
    "N1":      {"x": -400,  "y": 200},
    "T1_L":    {"x": -400,  "y": 1000},
    "T2_L":    {"x": -400,  "y": 1800},
    "T3_L":    {"x": -400,  "y": 2600},
    "N6":      {"x": -400,  "y": 3400},
    "N5":      {"x": -1200, "y": 3400},
    # 右纵列 (X=+400)
    "N12":     {"x": 400,   "y": 200},
    "T1_R":    {"x": 400,   "y": 1000},
    "T2_R":    {"x": 400,   "y": 1800},
    "T3_R":    {"x": 400,   "y": 2600},
    "N7":      {"x": 400,   "y": 3400},
    "N8":      {"x": 1200,  "y": 3400},
    # 外侧纵列
    "N2":      {"x": -1200, "y": 1000},
    "N3":      {"x": -1200, "y": 1800},
    "N4":      {"x": -1200, "y": 2600},
    "N11":     {"x": 1200,  "y": 1000},
    "N10":     {"x": 1200,  "y": 1800},
    "N9":      {"x": 1200,  "y": 2600},
}

NODE_TYPES: Dict[str, str] = {
    "START": "base", "J_START": "junction",
    "N1": "mission", "N2": "mission", "N3": "mission", "N4": "mission",
    "N5": "mission", "N6": "mission", "N7": "mission", "N8": "mission",
    "N9": "mission", "N10": "mission", "N11": "mission", "N12": "mission",
    "T1_L": "junction", "T1_R": "junction",
    "T2_L": "junction", "T2_R": "junction",
    "T3_L": "junction", "T3_R": "junction",
}

NODE_HAS_RFID: Dict[str, bool] = {
    "START": False, "J_START": False,
    "N1": True, "N2": True, "N3": True, "N4": True,
    "N5": True, "N6": True, "N7": True, "N8": True,
    "N9": True, "N10": True, "N11": True, "N12": True,
    "T1_L": False, "T1_R": False, "T2_L": False, "T2_R": False,
    "T3_L": False, "T3_R": False,
}

MISSION_NODES: list = [f"N{i}" for i in range(1, 13)]

JUNCTION_NODES: list = [
    "J_START",
    "T1_L", "T1_R", "T2_L", "T2_R", "T3_L", "T3_R",
]

ACTION_TYPES = ["TURN_LEFT", "TURN_RIGHT", "STRAIGHT", "STOP", "UTURN"]


# ============================================================
# 向后兼容别名（从 settings.yaml 读取，供老代码迁移期使用）
# ============================================================
def _compat():
    """延迟导入，避免循环依赖"""
    from robocup_rescue_brain.config import (
        get_track_config, get_edge_defaults,
        get_edge_tunnel, get_state_machine_config,
    )

    g = globals()
    track = get_track_config()
    g.setdefault("LANE_WIDTH_MM", track.get("lane_width_mm", 200.0))
    g.setdefault("LANE_HALF_WIDTH_MM", g["LANE_WIDTH_MM"] / 2.0)
    g.setdefault("TUNNEL_SEGMENT_LENGTH_MM", track.get("tunnel_segment_length_mm", 800.0))
    g.setdefault("STRAIGHT_SEGMENT_LENGTH_MM", track.get("straight_segment_length_mm", 800.0))
    g.setdefault("START_DEPTH_MM", track.get("start_depth_mm", 200.0))
    g.setdefault("MAIN_TRACK_SPAN_MM", track.get("main_track_span_mm", 800.0))
    g.setdefault("HALF_TRACK_SPAN_MM", track.get("half_track_span_mm", 400.0))

    de = get_edge_defaults()
    g.setdefault("EDGE_DEFAULTS", {
        "distance_mm": 800, "is_tunnel": False, "has_culvert": False,
        "speed_limit_ms": de.get("speed_limit_ms", 0.30),
    })
    dt = get_edge_tunnel()
    g.setdefault("TUNNEL_EDGE_DEFAULTS", {
        "distance_mm": 800, "is_tunnel": True, "has_culvert": False,
        "speed_limit_ms": dt.get("speed_limit_ms", 0.15),
    })

    g.setdefault("STATE_MACHINE_CONFIG", get_state_machine_config())
    g.setdefault("EXPECTED_YAW", {
        "down": 0.0, "right": 90.0, "left": -90.0, "up": 180.0,
    })


# 首次导入时执行
_compat()
