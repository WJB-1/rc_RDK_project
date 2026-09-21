"""定义纯编排器读取的不可变经验运动标定参数。"""

# 导入不可变数据类装饰器，保证运行中不会被某个调用方原地改写标定值。
from dataclasses import dataclass

# profile.py

# 车体半长。
BODY_HALF_LENGTH_MM = 71.0

CROSS_HALF_LENGTH_MM = 100.0

# 观察区位于目标路口中心前的距离。
OBSERVATION_ZONE_FROM_CENTER_MM = 350.0 + CROSS_HALF_LENGTH_MM + BODY_HALF_LENGTH_MM

# 转弯窗口位于目标路口中心前的距离。
TURN_WINDOW_FROM_CENTER_MM = 35.0 + CROSS_HALF_LENGTH_MM + BODY_HALF_LENGTH_MM

# 前向转弯完成后车体中心相对路口中心的前向等效偏移。
TAIL_ANCHOR_FROM_CENTER_MM = CROSS_HALF_LENGTH_MM + BODY_HALF_LENGTH_MM

# 后向转弯（撤回）完成后车体中心相对路口中心的前向等效偏移。
RETRACE_ANCHOR_FROM_CENTER_MM = 35.0 + CROSS_HALF_LENGTH_MM + BODY_HALF_LENGTH_MM

# 出发固定剧本右转后的前进距离。
DEPARTURE_FORWARD_MM = 160.0  

# 启动右转落地位置距 N1 路口中心的经验距离，单位毫米。
START_TURN_LAND_FROM_N1_MM = DEPARTURE_FORWARD_MM + BODY_HALF_LENGTH_MM + CROSS_HALF_LENGTH_MM + 35.0

# 驶入下一路口中心时额外补偿的距离，单位毫米；经验值为 0 时不产生补偿。
JUNCTION_CENTER_ENTRY_EXTRA_MM = 0.0
