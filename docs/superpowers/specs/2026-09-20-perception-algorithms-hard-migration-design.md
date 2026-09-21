# 感知算法目录硬迁移设计

## 目标

消除 `perception/algorithms` 根目录中混杂的车道、观察和基础技术实现，删除 `perception/pipelines/` 及旧算法导入路径，并确保 `test/test_correct` 迁移后继续通过。

## 范围与例外

- 删除整个 `perception/pipelines/` 目录。
- 不保留 `perception.pipelines.*` 或旧根目录算法模块的兼容重导出。
- 本次只维护 `test/test_correct`；`main.py` 和其他现有调用方若仍依赖旧路径，允许暂时不可运行。
- 保留新车道实现的行为与已验证的测试覆盖，不恢复旧的单体 `LaneTracker`。

## 目标目录

```text
perception/algorithms/
  core/
    ipm.py
    mask_utils.py
    timing.py
  lane/
    tracker.py
    config.py
    builders.py
    pipeline.py
    constants.py
    line_detection.py
    line_geometry.py
    ground_ipm_projector.py
    ground_ipm_selector.py
    new_ground.py
    template_selector.py
    types.py
    undistort.py
  observation/
    crossroad.py
    segmentation.py
    obstacle.py
    culvert.py
    quality.py
```

`lane.tracker.LaneTracker` 是唯一公开的车道跟踪入口。`core` 只放无业务状态的图像、几何与计时基础能力。`observation` 只放产出路口、障碍、涵洞和质量结论的业务算法。

## 迁移规则

1. 将所有 `test/test_correct` 车道导入迁至 `perception.algorithms.lane` 的实际模块；不再经由 `perception.pipelines`。
2. 将 `PiDiNetEngine` 对车道线检测工具的导入直接迁至 `perception.algorithms.lane.line_detection`，删除 `pidinet_lane.py` 兼容层。
3. 将车道实现所需的 `ipm`、`mask_utils`、`timing` 导入迁至 `perception.algorithms.core`。
4. 将路口、障碍、涵洞和质量判定文件迁至 `observation`，并修复其内部相对导入；不迁移 `main.py` 的调用。
5. 删除旧 `lane_analyzer.py`、旧 `perception/pipelines/`、旧根目录观察模块及旧车道兼容模块，且不留下同名转发文件。

## 验证

- `pytest test/test_correct -q` 必须全绿。
- 导入扫描确认 `test/test_correct` 不再引用 `perception.pipelines`，也不引用已删除的 `perception.algorithms.pidinet_lane`。
- 编译 `perception/algorithms` 与 `test/test_correct`。
- 真实 BPU 冒烟测试仅在具备 `hobot_dnn` 的 RDK X5 环境执行；本机缺失该运行时不作为本次失败。

## 风险控制

- 删除前以全局引用扫描列出所有旧路径消费者；范围外消费者仅记录为已知迁移后失效，不在本次修改中修复。
- 每个搬迁单元先改测试导入并验证，再删除旧文件，避免误删仍被 `test/test_correct` 使用的实现。
- 不修改车道匹配、偏航角和横向偏移量的计算公式。
