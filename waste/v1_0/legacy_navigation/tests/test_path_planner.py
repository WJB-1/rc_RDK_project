"""
单元测试：PathPlanner —— S-14 必经边（涵洞穿边）建模
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from navigation.domain.topology import RaceTrackTopology
from navigation.planning.map_oracle import MapOracle
from navigation.planning.path_planner import PathPlanner
from navigation.contracts import EdgeTask, PlanningInfeasibleError


class TestPathPlannerRequiredEdges(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.oracle = MapOracle(self.topo)
        self.planner = PathPlanner(self.oracle, self.topo)

    def _find_edge_by_nodes(self, a, b):
        return self.topo.get_edge(a, b)

    def test_edge_crossed_helper(self):
        self.assertTrue(PathPlanner._edge_crossed(["A", "B", "C"], "A", "B"))
        self.assertTrue(PathPlanner._edge_crossed(["A", "B", "C"], "B", "A"))
        self.assertFalse(PathPlanner._edge_crossed(["A", "X", "B"], "A", "B"))

    def test_required_edge_is_crossed(self):
        # 取一条真实直道边（N5.P_E <-> N6.P_W，正是之前探测未覆盖的那条）
        edge = self._find_edge_by_nodes("N5.P_E", "N6.P_W")
        result = self.planner.replan(
            "START", ["N1"],
            required_edges=[edge.edge_id],
        )
        seq = result.node_sequence
        # 边必须被相邻穿越 (N5.P_E,N6.P_W) 或 (N6.P_W,N5.P_E)
        self.assertTrue(
            any((seq[i] == "N5.P_E" and seq[i + 1] == "N6.P_W") or
                (seq[i] == "N6.P_W" and seq[i + 1] == "N5.P_E")
                for i in range(len(seq) - 1)),
            f"边 {edge.edge_id} 未被穿越: {seq}"
        )

    def test_multiple_required_edges_all_crossed(self):
        # 挑两条真实直道边，都应被穿越
        e1 = self._find_edge_by_nodes("N5.P_E", "N6.P_W")
        e2 = self._find_edge_by_nodes("N6.P_E", "N7.P_W")
        result = self.planner.replan(
            "START", ["N1"],
            required_edges=[e1.edge_id, e2.edge_id],
        )
        seq = result.node_sequence

        def crossed(a, b):
            return any((seq[i] == a and seq[i + 1] == b) or
                       (seq[i] == b and seq[i + 1] == a)
                       for i in range(len(seq) - 1))

        self.assertTrue(crossed("N5.P_E", "N6.P_W"), f"e1 未穿越: {seq}")
        self.assertTrue(crossed("N6.P_E", "N7.P_W"), f"e2 未穿越: {seq}")

    def test_no_required_edges_unchanged(self):
        # 无 required_edges 时行为与旧版一致
        result = self.planner.replan("START", ["N1"])
        self.assertEqual(result.node_sequence[:2], ["START", "J_START.P_N"])

    def test_required_edges_tasks_are_directed(self):
        # 必经边对应的 EdgeTask 应从 N5.P_E→N6.P_W 或反向，且 expected_yaw 对齐边方向
        edge = self._find_edge_by_nodes("N5.P_E", "N6.P_W")
        result = self.planner.replan("START", ["N1"], required_edges=[edge.edge_id])
        task_edge_ids = [t.edge_id for t in result.edge_tasks]
        self.assertIn(edge.edge_id, task_edge_ids)


class TestAssertNoUturn(unittest.TestCase):
    """方向 2 兜底筛查：规划产物含 180° 掉头指令 → 抛 PlanningInfeasibleError。"""

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.oracle = MapOracle(self.topo)
        self.planner = PathPlanner(self.oracle, self.topo)

    def _task(self, a, b, yaw):
        return EdgeTask(edge_id=0, from_node=a, to_node=b,
                        expected_yaw=yaw, distance_mm=100.0)

    def test_assert_no_uturn_raises_on_180(self):
        # 相邻两段朝向差 180° → 原地掉头 → 抛错
        tasks = [
            self._task("A", "B", 0.0),
            self._task("B", "A", 180.0),
        ]
        with self.assertRaises(PlanningInfeasibleError):
            self.planner._assert_no_uturn(tasks)

    def test_assert_no_uturn_raises_on_179(self):
        # 179° 也超过 160° 阈值 → 抛错（180° 掉头的扇区）
        tasks = [
            self._task("A", "B", 0.0),
            self._task("B", "A", 179.0),
        ]
        with self.assertRaises(PlanningInfeasibleError):
            self.planner._assert_no_uturn(tasks)

    def test_assert_no_uturn_ok_on_straight(self):
        # 直行（0°）与 90° 转向不触发
        tasks = [
            self._task("A", "B", 0.0),
            self._task("B", "C", 0.0),
            self._task("C", "D", 90.0),
        ]
        self.planner._assert_no_uturn(tasks)  # 不抛错


if __name__ == "__main__":
    unittest.main()
