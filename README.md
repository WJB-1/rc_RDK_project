# RoboCup Rescue Brain

RDK X5 视觉与架构核心 - 公共安全赛项侦查机器人上位机系统

## 项目结构

```
robocup_rescue_brain/
├── main.py                     # 程序入口
├── config/
│   └── settings.yaml           # 全局配置文件
├── perception/                 # 【感知层】(核心视觉)
│   ├── camera_manager.py       # 双摄调度：前视(巡线)、侧视(侦察)
│   ├── lane_tracker.py         # 传统视觉：通道边缘检测与中线提取
│   ├── yolo_detector.py        # BPU推理：地标检测与人脸检测
│   └── face_recognizer.py      # BPU推理：MobileFaceNet特征提取
├── face_gallery/               # 【数据层】嫌疑人特征底库
├── navigation/                 # 【决策层】
│   ├── map_topology.py         # 场地拓扑有向图定义
│   ├── astar_planner.py        # A*路径规划算法
│   └── state_machine.py        # 状态机核心
├── communication/              # 【通讯层】
│   ├── stm32_uart.py           # 高频串口收发线程
│   └── protocol_parser.py      # 协议解析
├── hardware/                   # 【外设层】
│   └── tts_syn6288.py          # SYN6288语音合成
├── models/                     # BPU模型文件
├── utils/                      # 【工具层】
│   └── logger.py               # 统一日志系统
└── logs/                       # 日志文件
```

## 快速开始

### 1. 安装依赖

```bash
pip install -r requirements.txt
```

### 2. 运行程序

```bash
python main.py
```

## 配置说明

主要配置项位于 `config/settings.yaml`:

- **cameras**: 前视/侧视摄像头参数
- **lane_tracking**: 巡线HSV阈值、ROI区域
- **pid**: 控制参数
- **serial**: 串口通讯配置

## 核心模块接口

### CameraManager

```python
# 获取双摄帧
frames = camera_manager.get_frames()
front_frame = frames["front"]  # 前视帧
side_frame = frames["side"]    # 侧视帧
```

### LaneTracker

```python
# 处理单帧，获取偏移
offset, debug_frame = lane_tracker.process(front_frame)
# offset: [-1.0, 1.0], 负值表示偏左需右打方向
```

## 开发原则

1. **极度防御**: 视觉处理前必须判空，try-except包裹核心逻辑
2. **性能榨取**: CPU处理前先做ROI裁剪或Resize
3. **零阻塞**: 感知推理和串口收发不阻塞主线程，状态机50Hz运转
4. **单向依赖**: 决策层依赖感知层，感知层不反向依赖