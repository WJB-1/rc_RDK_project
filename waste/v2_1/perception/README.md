# Perception 2.1 归档脚本

本目录仅保留重构前的旧脚本，运行时代码不得导入。

- `action_interface.py`：旧导航动作到串口字节的直接发送逻辑；由 Motion 2.1 替代。
- `perception_adapter.py`：旧视觉事件直接调用导航状态机的逻辑；由 Navigation 执行器和 Perception 观察端口替代。

`lane_tracker.py` 未归档：它仍由 `VisionPipeline` 封装为当前算法实现。
