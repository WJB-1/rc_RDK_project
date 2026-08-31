"""定义视觉层向导航感知适配器交付的相对观察数据契约。"""

# 导入不可变数据类装饰器，保证同一帧观察在适配期间保持稳定。
from dataclasses import dataclass
# 导入 Python 3.8 兼容的可选值和元组类型注解。
from typing import Optional, Tuple


@dataclass(frozen=True)
class RoadFeatures:
    """单帧中独立于目标检测的道路分支事实，供感知适配器解释相对位置。"""

    # 车头正前方是否检测到可继续通行的支路。
    has_forward_branch: bool
    # 车头左侧是否检测到支路。
    has_left_branch: bool
    # 车头右侧是否检测到支路。
    has_right_branch: bool
    # 上述道路结构判断的整体置信度，具体阈值由感知层约定。
    confidence: float


@dataclass(frozen=True)
class TargetDetection:
    """单帧中的一个有效目标，仍是相对车体事实，不含地图边标识。"""

    # 目标语义类型，例如 `CULVERT` 或 `OBSTACLE`。
    kind: str
    # 目标相对车体或画面的区域，例如 `LEFT`、`RIGHT` 或 `FORWARD`。
    relative_region: str
    # 当前检测的模型置信度。
    confidence: float
    # 感知层是否已通过自身质量门槛确认该检测可用。
    valid: bool


@dataclass(frozen=True)
class PerceptionFrame:
    """一次观察动作返回的完整相对视觉事实。

    谁调用：真实或虚拟感知系统通过执行器上报。
    谁响应：`NavigationRuntime` 将帧交给 `PerceptionAdapter`。
    输入输出：输入为帧时间、道路特征和目标列表；输出由适配器转换为地图更新或位置校正。
    状态影响：本对象不直接修改运行时地图。
    """

    # 帧唯一标识，用于关联观察完成中断和实际感知帧。
    frame_id: str
    # 图像或观察产生的运行环境时间。
    timestamp: float
    # 与目标列表分离传输的道路结构事实。
    road_features: RoadFeatures
    # 本帧全部有效目标，空元组表示未发现目标。
    targets: Tuple[TargetDetection, ...]
    # 调试时可选关联的图像或日志引用，不参与导航决策。
    debug_attachment: Optional[str] = None
