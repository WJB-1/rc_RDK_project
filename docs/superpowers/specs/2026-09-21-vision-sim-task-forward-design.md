# 视觉循迹适配与仿真任务前行设计

## 目标

恢复视觉循迹门面所需的结构化车道结果，并让仿真任务端口在不改变导航协议或导航状态的前提下，完成两段内部前行。

## 视觉循迹适配

新增 `perception.pipelines.lane.VisionPipeline` 作为兼容适配器。它接收既有 `LaneTracker`，调用其 `process(frame)`，并把偏移量、路口标志及 `last_lane_state` 中的角度、置信度、丢帧状态转换为 `FrameAnalysis` 和 `LaneMeasurement`。

有效车道输出 `LaneSampleStatus.VALID`；丢帧或缺失状态输出非有效测量，且不携带控制量。`PerceptionFacade` 保持不变，继续只依赖 `pipeline.analyze(...)`。

## 仿真任务内部动作

`SimTaskPort` 处理 `ExecuteTaskExecutionCommand` 时，严格在端口内部完成以下顺序：

1. 向 `SimWorld` 执行 `DriveExecutionCommand(300.0)`，模拟涵洞探索。
2. 通过 `SimWorld.estimate_distance_to_next_turn_window_mm()` 取得模拟视觉估计。
3. 若估计为正数，再执行该距离的第二条 `DriveExecutionCommand`。
4. 两次动作均成功后执行原任务，并向导航执行器只发布最终任务完成。

无有效估计、非正估计或任一内部前行失败时，端口发布失败终局，不执行任务完成。内部动作不成为 Navigation 的请求、不更新 NavigationStateStore，也不进入真实 MotionPort。

`SimWorld` 提供确定性的转弯窗口视觉距离估计和只读内部动作记录，用于验证。装配了 `pose_provider` 时，内部动作不得改写导航持有的位姿。

## 验证

- 视觉适配单测覆盖有效偏移/航向、丢帧无控制量以及 `PerceptionFacade` 的发布。
- 仿真端口单测覆盖 300 mm 探索、估距二次前行、无效估距失败与任务未完成。
- 运行上述测试及现有 Motion 测试；不修复本任务无关的地图拓扑仿真失败。
