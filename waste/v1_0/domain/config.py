"""
navigation/map_config.py — 端口模型静态地图数据（节点坐标、类型、RFID）

端口模型（积木式）：
  JunctionCenter（路口方块中心）→ JunctionBoundaryPort（端口）→ RoadSegment（800净直道）

物理常量：
  BLOCK_SIZE_MM = 200   方块边长
  BLOCK_HALF_MM = 100   方块半长（端口偏移）
  STRAIGHT_SEGMENT = 800 净直道长（含隧道）
  相邻方块中心距 = 1000

字段说明：
  - 方块中心：junction-T(3岔) / junction-cross(4岔) / corner(90°拐角,2岔)
  - 端口节点：`{方块名}.P_{N/E/S/W}`，坐标 = 方块中心 ± 100mm
  - base：START 锚点（非方块）
"""

from typing import Dict


# ============================================================
# 物理常量
# ============================================================
BLOCK_SIZE_MM = 200.0
BLOCK_HALF_MM = 100.0
STRAIGHT_SEGMENT_LENGTH_MM = 800.0


# ============================================================
# 方块中心 + 端口方向定义（唯一真实数据源）
# ============================================================
# name -> (x, y, [端口方向列表])
# 方向: N(北,y-100) S(南,y+100) E(东,x+100) W(西,x-100)
_BLOCKS = {
    "J_START": (0, 300, ["N", "E", "W"]),          # 丁字路口
    "N1":      (-500, 300, ["E", "S"]),            # 拐角
    "N12":     (500, 300, ["W", "S"]),             # 拐角
    "T1_L":    (-500, 1300, ["N", "S", "E", "W"]), # 十字路口
    "T1_R":    (500, 1300, ["N", "S", "E", "W"]),  # 十字路口
    "N2":      (-1500, 1300, ["E", "S"]),          # 拐角
    "N11":     (1500, 1300, ["W", "S"]),           # 拐角
    "T2_L":    (-500, 2300, ["N", "S", "E", "W"]),
    "T2_R":    (500, 2300, ["N", "S", "E", "W"]),
    "N3":      (-1500, 2300, ["N", "S", "E"]),     # 三通
    "N10":     (1500, 2300, ["N", "S", "W"]),      # 三通
    "T3_L":    (-500, 3300, ["N", "S", "E", "W"]),
    "T3_R":    (500, 3300, ["N", "S", "E", "W"]),
    "N4":      (-1500, 3300, ["N", "S", "E"]),     # 三通
    "N9":      (1500, 3300, ["N", "S", "W"]),      # 三通
    "N5":      (-1500, 4300, ["N", "E"]),          # 拐角
    "N8":      (1500, 4300, ["N", "W"]),           # 拐角
    "N6":      (-500, 4300, ["N", "W", "E"]),      # 三通
    "N7":      (500, 4300, ["N", "E", "W"]),       # 三通
}

_DIR_OFFSET = {
    "N": (0.0, -BLOCK_HALF_MM),
    "S": (0.0, BLOCK_HALF_MM),
    "E": (BLOCK_HALF_MM, 0.0),
    "W": (-BLOCK_HALF_MM, 0.0),
}


# ============================================================
# 节点坐标（程序化生成：方块中心 + 端口）
# ============================================================
NODE_COORDS: Dict[str, Dict[str, float]] = {
    "START": {"x": 0.0, "y": 0.0},  # base 锚点，非方块
}

for _name, (_x, _y, _dirs) in _BLOCKS.items():
    NODE_COORDS[_name] = {"x": float(_x), "y": float(_y)}
    for _d in _dirs:
        _dx, _dy = _DIR_OFFSET[_d]
        NODE_COORDS[f"{_name}.P_{_d}"] = {
            "x": float(_x + _dx), "y": float(_y + _dy),
        }


# ============================================================
# 节点类型
# ============================================================
NODE_TYPES: Dict[str, str] = {
    "START": "base",
}

for _name, (_x, _y, _dirs) in _BLOCKS.items():
    n_ports = len(_dirs)
    if _name.startswith("J_"):
        _t = "junction-T"
    elif _name.startswith("T"):
        _t = "junction-cross"
    elif n_ports == 2:
        _t = "corner"
    else:
        _t = "junction-T"
    NODE_TYPES[_name] = _t
    for _d in _dirs:
        NODE_TYPES[f"{_name}.P_{_d}"] = "port"


# ============================================================
# RFID 标记（N1~N12 方块中心，端口无）
# ============================================================
NODE_HAS_RFID: Dict[str, bool] = {"START": False}
_MISSION_SET = set(f"N{i}" for i in range(1, 13))
for _name in _BLOCKS:
    NODE_HAS_RFID[_name] = _name in _MISSION_SET
    for _d in _BLOCKS[_name][2]:
        NODE_HAS_RFID[f"{_name}.P_{_d}"] = False


MISSION_NODES: list = [f"N{i}" for i in range(1, 13)]

JUNCTION_NODES: list = [
    name for name in _BLOCKS
    if name.startswith("J_") or name.startswith("T")
]

ACTION_TYPES = ["TURN_LEFT", "TURN_RIGHT", "STRAIGHT", "STOP", "UTURN"]


# ============================================================
# 向后兼容别名（从 settings.yaml 读取）
# ============================================================
def _compat():
    """延迟导入，避免循环依赖"""
    try:
        from ...config import (
            get_track_config, get_edge_defaults,
            get_edge_tunnel, get_state_machine_config,
        )
    except ImportError:
        from config import (
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
    g.setdefault("MAIN_TRACK_SPAN_MM", track.get("main_track_span_mm", 1000.0))
    g.setdefault("HALF_TRACK_SPAN_MM", track.get("half_track_span_mm", 500.0))

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
