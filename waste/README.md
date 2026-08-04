# Waste — 废弃脚本/旧文件

已迁移到新目录结构的旧文件，保留作为历史参考。

| 旧路径 | 新路径 | 说明 |
|--------|--------|------|
| perception/camera_manager.py | hardware/camera.py | 摄像头管理（硬件驱动层） |
| perception/segmentation_engine.py | vision/models/bisenet.py | BiSeNet 语义分割（模型封装） |
| perception/detection_engine.py | vision/models/yolo_detect.py | YOLO 目标检测（模型封装） |
| perception/math_ipm_pipeline.py | vision/ipm_pipeline.py | IPM 算法 + 质量门控（视觉算法） |
