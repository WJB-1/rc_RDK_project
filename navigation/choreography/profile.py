"""定义纯编排器读取的不可变经验运动标定参数。"""

# 导入不可变数据类装饰器，保证运行中不会被某个调用方原地改写标定值。
from dataclasses import dataclass


@dataclass(frozen=True)
class MotionProfile:
    """描述编排器在不读取感知细节时使用的经验距离参数。

    谁调用：导航装配代码创建，`Choreographer` 在展开流程和生成动作时读取。
    谁响应：编排器以这些固定参数判断是否设置观察区并计算经验前进距离。
    输入输出：输入经过实车标定的毫米数；本对象不直接产生底盘命令。
    状态影响：不可变配置，不修改机器人、地图、任务或执行器。
    """

    # 从路口中心经验前进到普通长边观察区的固定距离，单位毫米。
    initial_observation_advance_mm: float
    # 普通道路距离下一路口中心不超过本值时属于观察区，单位毫米。
    observation_zone_max_remaining_mm: float = 500.0
    # 驶向下一路口时为经验落在路口中心而增加的补偿距离，单位毫米。
    junction_center_entry_extra_mm: float = 0.0
