"""
单元测试：Agent 状态机
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import time
import unittest
from navigation.state_machine import AgentStateMachine, AgentState
from navigation.map_topology import RaceTrackTopology
from navigation.contracts import (
    OdomUpdate, RfidEvent, CrossroadEvent, TurnAction,
)


class TestStateMachine(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.agent = AgentStateMachine(self.topo)

    def test_initial_state(self):
        self.assertEqual(self.agent.state, AgentState.IDLE)
        self.assertEqual(self.agent.current_node, "START")

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
        self.assertTrue(self.topo.all_missions_completed())
        self.assertEqual(self.agent.state, AgentState.FINISHED)

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
        self.assertEqual(self.agent.state, AgentState.APPROACHING)

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
        # START(0,0) -> N12(400,200) 并非正右方，角度约为 63.4 deg
        yaw = self.agent._calc_expected_yaw("START", "N12")
        self.assertAlmostEqual(yaw, 63.43, places=1)

    def test_calc_expected_yaw_left(self):
        # START(0,0) -> N1(-400,200) 并非正左方，角度约为 -63.4 deg
        yaw = self.agent._calc_expected_yaw("START", "N1")
        self.assertAlmostEqual(yaw, -63.43, places=1)

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
        self.assertEqual(self.agent._determine_turn(0, 180), TurnAction.UTURN)

    def test_event_log(self):
        self.agent.start()
        self.assertTrue(len(self.agent.event_log) > 0)
        self.assertEqual(self.agent.event_log[0]["type"], "startup")


if __name__ == "__main__":
    unittest.main()
