"""
视觉算法层 (Vision Layer)

纯视觉算法实现，无状态、无硬件依赖：
- vision/models/: 深度学习模型封装 (BiSeNet 分割, YOLO 检测)
- vision/ipm_pipeline.py: 逆透视变换 + 三段式状态机 + 质量门控

设计原则:
1. 给定输入 → 输出结果，不持有状态
2. 不直接访问摄像头、串口等硬件
3. 模型权重放在项目根目录 models/
"""

# contracts 是无依赖模块，始终可导入
from .contracts import (
    DetectionBox, CrossroadDetection, CulvertDetection,
    ObstacleDetection, CulvertReconResult, VisionTools,
)

