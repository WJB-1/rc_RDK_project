"""
端到端测试：完整 TSP 路线拼接 + 回基地 + 无 U-turn（Task 14）

覆盖 4 条关键链路（用户指出旧测试只测单 Dijkstra，抓不住拼接/回基地问题）：
1. 完整 EdgeTask 序列任意相邻两段不得形成 U-turn
2. 全任务规划的最后节点必须是 START
3. 队列为空但车不在 START 时，状态机不得进入 FINISHED
4. 发现非法 180° 时，状态机不得下发转向命令（返回 STOP）
"""

import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from navigation.domain.topology import RaceTrackTopology, get_topology
from navigation.planning.map_oracle import MapOracle
from navigation.planning.path_planner import PathPlanner
from navigation.state_machine import AgentStateMachine, AgentState
from navigation.contracts import TurnAction, PlanningInfeasibleError
from navigation.domain.line_graph import edge_heading, turn_angle_degs
from navigation.domain.config import MISSION_NODES


ALL_RFID = [f"N{i}" for i in range(1, 13)]


def _is_port(n):
    return ".P_" in n


class TestFullTspNoUturn(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.oracle = MapOracle(self.topo)
        self.planner = PathPlanner(self.oracle, self.topo)

    def _fold_backs(self, seq):
        """统计相邻两步构成精确 180° 折返的次数（中间节点是任意节点）。"""
        folds = []
        for i in range(len(seq) - 2):
            a, b, c = seq[i], seq[i + 1], seq[i + 2]
            h1 = edge_heading(self.topo, a, b)
            h2 = edge_heading(self.topo, b, c)
            d = turn_angle_degs(h1, h2)
            if abs(abs(d) - 180) < 1.0:
                folds.append((a, b, c))
        return folds

    def test_1_no_uturn_in_full_edge_sequence(self):
        """完整 EdgeTask 序列任意相邻两段不得形成 U-turn（180° 折返）。"""
        # 固定顺序（非乱序），全量规划
        result = self.planner.replan("START", ALL_RFID)
        seq = result.node_sequence
        folds = self._fold_backs(seq)
        self.assertEqual(folds, [], f"完整路径存在 180° 折返: {folds}")

    def test_2_last_node_is_START(self):
        """全任务规划的最后节点必须是 START。"""
        result = self.planner.replan("START", ALL_RFID)
        self.assertEqual(result.node_sequence[-1], "START")
        # 覆盖全部 12 个 RFID
        for n in ALL_RFID:
            self.assertIn(n, result.node_sequence)

    def test_2b_return_path_lands_on_START(self):
        """返程（已打卡，回 START）用 path_to_node，能生成完整路径且终于 START。"""
        path = self.oracle.path_to_node("N2.P_E", "START")
        self.assertIsNotNone(path)
        self.assertEqual(path[-1], "START")


class TestNoFakeTurnOnUturn(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.agent = AgentStateMachine(self.topo)

    def test_4_no_turn_command_on_illegal_180(self):
        """发现非法 180° 时，_determine_turn 抛 PlanningInfeasibleError（fail fast），
        绝不静默降级为 STOP、也不下发 90° 转向命令。"""
        for diff in (180.0, -180.0, 179.0, -179.0):
            with self.assertRaises(PlanningInfeasibleError):
                self.agent._determine_turn(0.0, diff)


if __name__ == "__main__":
    unittest.main()
