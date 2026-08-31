"""
单元测试：Agent 状态机
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import time
import unittest
from navigation.state_machine import AgentStateMachine, AgentState
from navigation.domain.topology import RaceTrackTopology
from navigation.contracts import (
    OdomUpdate, RfidEvent, CrossroadEvent, TurnAction,
    CulvertEvent, CulvertType, PlanningInfeasibleError,
)


class TestStateMachine(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.agent = AgentStateMachine(self.topo)

    def test_initial_state(self):
        self.assertEqual(self.agent.state, AgentState.IDLE)
        self.assertEqual(self.agent.current_node, "START")

    def test_current_state_initial_idle(self):
        """current_state() 初始为 IDLE，且与 agent.state 等价（只读投影）"""
        self.assertEqual(self.agent.current_state(), AgentState.IDLE)

    def test_current_state_after_start_matches_state(self):
        """start() 后 current_state() 与 agent.state 一致（本步等价语义）"""
        self.agent.start()
        self.assertEqual(self.agent.current_state(), self.agent.state)

    def test_start_transition(self):
        self.agent.start()
        self.assertIn(self.agent.state, (AgentState.GLOBAL_PLANNING, AgentState.EDGE_EXECUTING))

    def test_rfid_snap(self):
        self.agent.start()
        self.agent.on_rfid_scanned(RfidEvent(uid="N1"))
        self.assertEqual(self.agent.current_node, "N1")
        self.assertTrue(self.topo.get_node("N1").is_visited)
        self.assertIn("N1", self.agent.visited_nodes)

    def test_rfid_all_complete(self):
        self.agent.start()
        for n in [f"N{i}" for i in range(1, 13)]:
            if n in self.topo.nodes:
                self.agent.on_rfid_scanned(RfidEvent(uid=n))
                # 每个 RFID 后 state 变为 NODE_ARRIVAL，需要 tick 驱动流程
                # _on_node_arrival → _dequeue_next_edge → EDGE_EXECUTING
                self.agent.tick()
        self.assertTrue(self.topo.all_missions_completed())
        # 最后一张卡打完 + tick 后，cursor 耗尽 replan 产生回 START 路径
        # 此时状态应为 EDGE_EXECUTING（执行回 START 的边）
        # 或 FINISHED（如果当前节点 = START）
        self.assertIn(self.agent.state,
                      (AgentState.EDGE_EXECUTING, AgentState.FINISHED))

    def test_odom_update(self):
        # dx_mm=0(车体横向), dy_mm=100(车体纵向前进), dyaw_deg=0; yaw=0（朝下）时，前进对应 world Y+
        self.agent.on_odom_update(OdomUpdate(dx_mm=0, dy_mm=100, dyaw_deg=0))
        x, y, yaw = self.agent.get_position()
        self.assertAlmostEqual(y, 100.0, places=1)

    def test_odom_rotation(self):
        # yaw=90 deg（朝右），车体前进 dy_mm=100（纵向）应使 world X+100
        # world_dx = dx*cos(yaw) - dy*sin(yaw) = 0*0 - 100*1 = -100
        # world_dy = dx*sin(yaw) + dy*cos(yaw) = 0*1 + 100*0 = 0
        self.agent.yaw_deg = 90.0
        self.agent.on_odom_update(OdomUpdate(dx_mm=0, dy_mm=100, dyaw_deg=0))
        x, y, yaw = self.agent.get_position()
        self.assertAlmostEqual(x, -100.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)

    def test_crossing_detected(self):
        self.agent.start()
        self.agent.on_crossroad_detected(CrossroadEvent(distance_mm=150, duty_cycle=1.0))
        # 换核后：路口检测 → 入队 turn 任务（等价旧 state == APPROACHING → _on_approach_done）
        self.assertEqual(self.agent.current_head_kind(), "turn")

    def test_turn_action_resolution(self):
        # 测试 _determine_turn 方法的转向判定
        action = self.agent._determine_turn(0, 45)
        self.assertIn(action, (TurnAction.TURN_LEFT, TurnAction.TURN_RIGHT,
                                TurnAction.STRAIGHT, TurnAction.UTURN, TurnAction.STOP))
        # 小角度偏差应判为 STRAIGHT
        self.assertEqual(self.agent._determine_turn(0, 0), TurnAction.STRAIGHT)

    def test_calc_expected_yaw_down(self):
        # N1(-400,200) → T1_L(-400,1000): 正下方, atan2(0,800)=0°
        yaw = self.agent._calc_expected_yaw("N1", "T1_L")
        self.assertAlmostEqual(yaw, 0.0, places=1)

    def test_calc_expected_yaw_right(self):
        # START(0,0) -> N12(500,300) 并非正右方，角度约为 59.04 deg
        yaw = self.agent._calc_expected_yaw("START", "N12")
        self.assertAlmostEqual(yaw, 59.04, places=1)

    def test_calc_expected_yaw_left(self):
        # START(0,0) -> N1(-500,300) 并非正左方，角度约为 -59.04 deg
        yaw = self.agent._calc_expected_yaw("START", "N1")
        self.assertAlmostEqual(yaw, -59.04, places=1)

    def test_calc_expected_yaw_pure_right(self):
        yaw = self.agent._calc_expected_yaw("N6", "N7")
        self.assertAlmostEqual(yaw, 90.0, places=1)

    def test_calc_expected_yaw_pure_left(self):
        yaw = self.agent._calc_expected_yaw("N7", "N6")
        self.assertAlmostEqual(yaw, -90.0, places=1)

    def test_turn_determination(self):
        # diff = expected_yaw - current_yaw
        # diff > 0 → TURN_LEFT, diff < 0 → TURN_RIGHT
        self.assertEqual(self.agent._determine_turn(0, 0), TurnAction.STRAIGHT)
        self.assertEqual(self.agent._determine_turn(0, 90), TurnAction.TURN_LEFT)
        self.assertEqual(self.agent._determine_turn(0, -90), TurnAction.TURN_RIGHT)
        # 180° 掉头 = 硬逻辑错误，fail fast 抛错，绝不返回 STOP/UTURN（不伪造 90° 转向）。
        with self.assertRaises(PlanningInfeasibleError):
            self.agent._determine_turn(0, 180)
        with self.assertRaises(PlanningInfeasibleError):
            self.agent._determine_turn(0, -180)

    def test_event_log(self):
        self.agent.start()
        self.assertTrue(len(self.agent.event_log) > 0)
        self.assertEqual(self.agent.event_log[0]["type"], "startup")

    # ------------------------------------------------------------------
    # 发现 vs 侦查 语义拆分（Task 10 新增）
    # ------------------------------------------------------------------

    def test_discovered_not_recon(self):
        """发现涵洞（on_culvert_detected）只进 discovered，不进 recon"""
        self.agent.start()
        self.agent.set_culvert_targets(set())
        # 进入 EDGE_EXECUTING 后才有 current_task，直接模拟：先让状态到 EDGE_EXECUTING
        # 这里直接调用 on_culvert_detected 前需要 current_task，故先用 start 后状态判断
        self.agent.on_culvert_detected(
            CulvertEvent(culvert_type=CulvertType.SIDE, local_x_mm=0.0, local_y_mm=0.0)
        )
        # discovered 可能为空（取决于是否有 current_task），但 recon 一定为空
        self.assertEqual(self.agent.recon_culverts, set())

    def test_all_culverts_reconed_reads_recon_not_discovered(self):
        """结束条件读 recon，发现满但 recon 空时不应 True"""
        fake_targets = {1, 2, 3}
        self.agent.set_culvert_targets(fake_targets)
        # 只把 discovered 填满（模拟「看见」），recon 为空
        for eid in fake_targets:
            self.agent.runtime_map.mark_culvert_discovered(eid)
        self.assertFalse(self.agent.all_culverts_reconed())

    def test_all_culverts_reconed_true_when_recon_full(self):
        """recon 集凑满才 True"""
        fake_targets = {1, 2, 3}
        self.agent.set_culvert_targets(fake_targets)
        for eid in fake_targets:
            self.agent.runtime_map.mark_culvert_reconed(eid)
        self.assertTrue(self.agent.all_culverts_reconed())

    def test_all_culverts_reconed_empty_targets_always_true(self):
        """无目标时恒 True（实机路径 _culvert_targets 恒空）"""
        self.agent.set_culvert_targets(set())
        self.assertTrue(self.agent.all_culverts_reconed())

    def test_get_state_exports_discovered_and_recon(self):
        """NavigationState 导出 discovered/recon 两个字段"""
        self.agent.runtime_map.mark_culvert_discovered(10)
        self.agent.runtime_map.mark_culvert_reconed(20)
        nav = self.agent.get_state()
        self.assertEqual(nav.discovered_culverts, [10, 20])
        self.assertEqual(nav.recon_culverts, [20])

    def test_determine_turn_never_uturn_all_diffs(self):
        """对任意差角，_determine_turn 都不应返回 UTURN（Task 12 核心不变量）"""
        for diff in range(-180, 181, 1):
            fdiff = float(diff)
            if abs(diff) > 160:
                # 180° 掉头 = 硬逻辑错误，抛错而非返回任何转向
                with self.assertRaises(PlanningInfeasibleError):
                    self.agent._determine_turn(0.0, fdiff)
                continue
            action = self.agent._determine_turn(0.0, fdiff)
            self.assertNotEqual(action, TurnAction.UTURN,
                                f"diff={diff} 不应产生 UTURN")

    # ------------------------------------------------------------------
    # Task 15 新增：反向段检测（倒车中断，不原地掉头）
    # ------------------------------------------------------------------

    def test_next_is_reverse_not_at_START(self):
        """START 是基地端点，允许掉头重新出发，不误判为反向段（否则死循环）。"""
        from navigation.contracts import EdgeTask
        self.agent.current_node = "START"
        self.agent.yaw_deg = 180.0  # 车头朝上（回 START 后）
        # 下一段 START → J_START.P_N 朝下 yaw=0，与车头 180° 相反，但 START 可掉头
        task = EdgeTask(edge_id=0, from_node="START", to_node="J_START.P_N",
                        expected_yaw=0.0, distance_mm=200.0)
        self.agent.planner._cached_tasks = [task]
        self.agent.planner._cursor = 0
        self.assertFalse(self.agent._next_is_reverse())

    def test_next_is_reverse_on_dead_end(self):
        """赛道死胡同支路（非 START）的 180° 反向段应触发倒车。"""
        from navigation.contracts import EdgeTask
        self.agent.current_node = "N4.P_E"  # 死胡同支路末端
        self.agent.yaw_deg = -90.0           # 朝西（来路）
        task = EdgeTask(edge_id=1, from_node="N4.P_E", to_node="T3_L.P_W",
                        expected_yaw=90.0, distance_mm=800.0)  # 朝东 = 反向
        self.agent.planner._cached_tasks = [task]
        self.agent.planner._cursor = 0
        self.assertTrue(self.agent._next_is_reverse())


if __name__ == "__main__":
    unittest.main()
