# Test 目录

测试脚本按模块分类：

```
test/
├── navigation/           ← 导航/地图/状态机
│   ├── test_topology.py      赛道拓扑图单元测试
│   ├── test_map_oracle.py    Dijkstra + TSP 路径规划
│   └── test_state_machine.py Agent 状态机流转
│
├── comm/                 ← 通信协议/串口
│   ├── test_stm32_protocol.py 协议编解码测试
│   └── comm_debug_server.py   STM32 通信 C/S 可视化调试面板
│
├── vision/               ← 视觉/感知
│   ├── test_math_ipm_v4.py       IPM + 质量门控 + 路口检测 功能测试 (43项)
│   ├── test_straight_tracking.py 直道循迹集成测试 (摄像头+视觉+STM32+Web)
│   └── capture_video.py          数据集采集工具
│
├── ui/                   ← (预留) Web 渲染可视化测试
│
└── README.md
```

## 运行

```bash
cd D:/project_file/rc_compition
python robocup_rescue_brain/test/navigation/test_topology.py
python robocup_rescue_brain/test/vision/test_math_ipm_v4.py
python robocup_rescue_brain/test/vision/test_straight_tracking.py --no-serial
```

## 已废弃的目录

以下目录中的代码已迁移或不再维护，见 `waste/`：
- `map_test/` — 地图模块 → `robocup_rescue_brain/navigation/`
- `integration/` — 旧版协议 V2.0 → `robocup_rescue_brain/communication/` V2.1
- `debug/` — 临时导入调试脚本(9个) → 已删除
- `perception_test/*/core/math_ipm_pipeline.py` — → `vision/algorithms/`
- `perception_test/*/core/lane_tracker.py` — → `perception/lane_tracker.py`
- `perception_test/*/core/segmentation_engine.py` — → `vision/models/bisenet.py`
