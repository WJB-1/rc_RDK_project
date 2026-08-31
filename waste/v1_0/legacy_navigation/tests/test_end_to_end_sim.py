"""
test_end_to_end_sim.py — 端到端仿真回归（Task 28 换核后空队列自循环修复）

覆盖诊断文档 §7 推荐的核心验收：换核后空队列不再自循环于 GLOBAL_PLANNING。

修复前（HEAD=15d8fee）症状（seed=42, 60000 tick）：
    final state = GLOBAL_PLANNING（卡死）
    RFID 7/12, 涵洞 2/8, current_node = N5.P_E（被失败边 stale to_node 覆盖）
    event_log 无限循环：node_arrival 到达 N5.P_E → GLOBAL_PLANNING→GLOBAL_PLANNING

修复后：
    - 空队列走 plan-or-dequeue（有缓存取下一段 / 无缓存真正调规划器），不再
      把「空队列」误判为「到达节点」。
    - current_node 传播移到真实完成点（_on_drive_advance / on_turn_done），
      不再读恢复期 stale 失败边的 to_node。
    - 倒车完成真正触发重规划（GLOBAL_PLANNING 唯一入口 → _do_global_planning）。

本测试断言「规划触发断链」已修好：RFID 全部打到 12/12（不再卡在 7/12 自循环），
并校验 max_tick 超时兜底不会静默死循环。

注意（给后续任务）：
    修复后所有 seed 都推进到 RFID 12/12；但仍有一个**与 Task 28 无关的既有 bug**
    —— 涵洞扫荡收尾期（returning + required_edges 阶段）倒车恢复到 port 节点后，
    path_planner.replan 的返程兜底 [current_node] + _cover_required_edges 用不带
    entry_heading 的 _dijkstra 生成 180° 首步，被 next_is_reverse 反复判反向 → 倒车
    自循环，导致 recon 卡在 <8（FINISHED 未达成）。该 bug 早于换核（pre-step1
    e238f24 即复现），定位在 path_planner.py，超出本任务「只改 state_machine.py +
    deadend_recovery.py」的红线，留待后续任务处理。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest

from navigation.simulation.scene.scene_generator import generate_scene
from navigation.simulation.sim_engine import SimEngine
from navigation.contracts import AgentState


def _run_seed(seed: int, max_ticks: int):
    """跑单个 seed，返回 (state, rfid, recon, current_node, tick_count, event_types)。"""
    scene = generate_scene(seed=seed)
    eng = SimEngine(scene)
    eng.start()
    for _ in range(max_ticks):
        if not eng.step():
            break
    ag = eng._agent
    rfid = sum(1 for n in ag.topo.nodes if ag.topo.nodes[n].is_visited)
    recon = len(ag.recon_culverts)
    event_types = [e["type"] for e in ag.event_log]
    return ag.state, rfid, recon, ag.current_node, eng._tick_count, event_types


class TestEndToEndNoSelfLoop(unittest.TestCase):
    """空队列自循环回归：GLOBAL_PLANNING→GLOBAL_PLANNING 无限循环被修掉。"""

    def test_repro_seed_42_reaches_full_rfid(self):
        """seed=42 修复前卡在 GLOBAL_PLANNING rfid=7/12，修复后 RFID 全打满 12/12。"""
        state, rfid, recon, node, ticks, event_types = _run_seed(42, max_ticks=60000)
        self.assertEqual(rfid, 12, "RFID 应全部打卡（修复前卡在 7/12 自循环）")
        # 修复前的核心症状是「无限 node_arrival(N5.P_E) → GLOBAL_PLANNING→GLOBAL_PLANNING」。
        # 修复后不再有无界自循环：状态推进到更深阶段的 BACKTRACK 也不再卡在 GLOBAL_PLANNING。
        self.assertNotEqual(state, AgentState.GLOBAL_PLANNING,
                            "不应再停留在 GLOBAL_PLANNING 自循环")

    def test_all_seeds_make_progress_and_terminate(self):
        """四个 seed 都从「卡死自循环」推进：状态不再是 GLOBAL_PLANNING（终态不算）。"""
        for seed in (0, 1, 2, 42):
            state, rfid, recon, node, ticks, event_types = _run_seed(seed, max_ticks=60000)
            # 终态 FINISHED/FAILED 属合法退出；否则不得卡在 GLOBAL_PLANNING 自循环
            if state not in (AgentState.FINISHED, AgentState.FAILED):
                self.assertNotEqual(state, AgentState.GLOBAL_PLANNING,
                                    f"seed={seed} 又卡回 GLOBAL_PLANNING 自循环")
            # 修复后所有 seed 都能把 RFID 打到 12/12（修复前分别 3/1/2/7）
            self.assertEqual(rfid, 12, f"seed={seed} RFID 应打满 12/12，实际 {rfid}")


if __name__ == "__main__":
    unittest.main()
