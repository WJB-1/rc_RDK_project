# perception/algorithms/lane/constants.py
"""车道检测常量集中定义。

所有模块（line_detection / line_geometry / ground_ipm_selector / template_selector）
一律从这里取，禁止再在模块里本地定义。
"""

# ---- 线段检测 / 合并 ----
ANGLE_TOL_DEG = 3.0           # 合并线段时的最大角度差
NORMAL_DIST_TOL = 8.0         # 合并线段时的最大法向距离（像素）
MIN_LINE_LENGTH = 25.0        # 有效线段最小长度（像素）

# ---- 车道对枚举 ----
PARALLEL_ANGLE_TOL_DEG = 1.5  # 判定两条线"平行"的最大角度差（度）
Y_OVERLAP_MIN_PX = 8.0        # 线段在 Y 方向最小重叠像素
PAIR_MIN_MM = 190.0           # 车道对宽度下限
PAIR_MAX_MM = 210.0           # 车道对宽度上限
PAIR_TARGET_MM = 200.0        # 车道对宽度目标值

# ---- 六线模板 ----
TEMPLATE_LINE_X_MM = {
    "0": 99.771, "1": -99.789,
    "2": 144.157, "3": -144.721,
    "4": 170.861, "5": -171.389,
}
TEMPLATE_LINE_ORDER = ("4", "2", "0", "1", "3", "5")
TEMPLATE_LINE_WEIGHTS = {"0": 1.0, "1": 1.0, "2": 0.5, "3": 0.5, "4": 0.2, "5": 0.2}
LANE_LINE_IDS = ("0", "1")
WALL_LINE_IDS = ("2", "3", "4", "5")
ALL_LINE_IDS = ("0", "1", "2", "3", "4", "5")
TEMPLATE_LINE_COLORS = {
    "0": (0, 200, 255), "1": (0, 100, 255),
    "2": (0, 255, 0), "3": (0, 180, 0),
    "4": (255, 0, 255), "5": (180, 0, 180),
}

# ---- 车辆几何兜底默认值 ----
GROUND_BODY_LENGTH_MM = 142.0
GROUND_CAMERA_FORWARD_MM = 60.0
GROUND_CAMERA_LATERAL_OFFSET_MM = 0.0