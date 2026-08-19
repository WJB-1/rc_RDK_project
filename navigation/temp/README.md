# temp/ — 兼容壳（已弃用，等待清理）

这些文件是 2026-08-19 逻辑重构时留下的 **re-export 薄壳**，用途是让旧代码
（`from navigation.map_topology import ...` 等旧路径）在重构期间不崩。

## 现状

- **所有真实实现已迁移到子包**：
  - `map_config` / `map_topology` / `line_graph` → `domain/`
  - `map_oracle` / `path_planner` → `planning/`
  - `edge_executor` → `control/`
  - `sim_scene` / `scene_generator` / `vision_checker` → `perception/simulation/`
- **所有调用方已改为新路径**（main.py / web / test / internal 均已改），
  因此这些壳**已无任何调用者**。

## 何时可删

- **现在即可删**：全库已无旧路径 import（2026-08-19 验证通过）。
- 保留此目录仅作安全网，以防有遗漏的外部调用方在删除后报错。
- 确认一次完整测试 + 端到端通过后，若无需回滚，可直接删除整个 `temp/` 目录。

## 删除前自检

```bash
grep -rn "from navigation\.\(map_\|path_planner\|line_graph\|edge_executor\|sim_scene\|scene_generator\|vision_checker\)" --include="*.py" . | grep -v navigation/temp/
# 无输出 = 可安全删除
```
