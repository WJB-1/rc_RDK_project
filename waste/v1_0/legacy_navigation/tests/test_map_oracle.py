"""
单元测试：Map Oracle API
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from navigation.planning.map_oracle import MapOracle
from navigation.domain.topology import RaceTrackTopology, get_topology


class TestMapOracle(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.oracle = MapOracle(self.topo)

    def test_get_edge_properties_existing(self):
        # START -> J_START.P_N 是直接连接
        props = self.oracle.get_edge_properties("START", "J_START.P_N")
        self.assertIn("distance_mm", props)
        self.assertIn("is_tunnel", props)
        self.assertIn("speed_limit_ms", props)

    def test_get_edge_properties_undirected(self):
        # N2↔N2.P_S 是直接连接的无向边
        a = self.oracle.get_edge_properties("N2", "N2.P_S")
        b = self.oracle.get_edge_properties("N2.P_S", "N2")
        self.assertEqual(a, b)

    def test_get_edge_properties_nonexistent(self):
        with self.assertRaises(KeyError):
            self.oracle.get_edge_properties("N1", "N8")

    def test_tunnel_edge_flag(self):
        # T1_L.P_E↔T1_R.P_W 是隧道段
        props = self.oracle.get_edge_properties("T1_L.P_E", "T1_R.P_W")
        self.assertTrue(props["is_tunnel"])
        # N2.P_S↔N3.P_N 不是隧道（外侧纵列普通边）
        props2 = self.oracle.get_edge_properties("N2.P_S", "N3.P_N")
        self.assertFalse(props2["is_tunnel"])

    def test_dijkstra_same_node(self):
        dist, path = self.oracle._dijkstra("N1", "N1")
        self.assertEqual(dist, 0.0)
        self.assertEqual(path, ["N1"])

    def test_dijkstra_direct_edge(self):
        # START -> J_START = 300mm (经 J_START.P_N)
        dist, path = self.oracle._dijkstra("START", "J_START")
        self.assertEqual(path, ["START", "J_START.P_N", "J_START"])
        self.assertEqual(dist, 300.0)

    def test_dijkstra_multi_hop(self):
        dist, path = self.oracle._dijkstra("N1", "N12")
        # N1 -> N1.P_E -> J_START.P_W -> J_START -> J_START.P_E -> N12.P_W -> N12
        self.assertIsNotNone(path)
        self.assertEqual(path[0], "N1")
        self.assertEqual(path[-1], "N12")
        self.assertEqual(dist, 1000.0)

    def test_shortest_path_single_target(self):
        path = self.oracle.query_shortest_path("START", ["N1"])
        # START -> J_START.P_N -> J_START -> J_START.P_W -> N1.P_E -> N1
        # 打卡 N1（从 N1.P_E 进）后，禁止原路折返，故从 N1.P_S 离开（禁折返语义）
        self.assertEqual(path, ["START", "J_START.P_N", "J_START", "J_START.P_W",
                                "N1.P_E", "N1", "N1.P_S"])

    def test_shortest_path_multiple_targets(self):
        path = self.oracle.query_shortest_path("START", ["N1", "N12"])
        # 两个 corner 打卡，路径必须覆盖 N1 与 N12，且打卡后各自从不同端口离开（不折返）
        self.assertEqual(path[0], "START")
        self.assertIn("N1", path)
        self.assertIn("N12", path)

    def test_shortest_path_unvisited_empty(self):
        path = self.oracle.query_shortest_path("N1", [])
        self.assertEqual(path, ["N1"])

    def test_get_path_details(self):
        # N1->T1_L 经端口 N1.P_S 和 T1_L.P_N 的序列
        details = self.oracle.get_path_details(["N1", "N1.P_S", "T1_L.P_N", "T1_L"])
        self.assertEqual(len(details), 3)
        self.assertEqual(details[0]["distance_mm"], 100)   # N1 -> N1.P_S
        self.assertEqual(details[1]["distance_mm"], 800)   # N1.P_S -> T1_L.P_N
        self.assertEqual(details[2]["distance_mm"], 100)   # T1_L.P_N -> T1_L

    # ------------------------------------------------------------------
    # Task 12 新增：进入朝向约束 + 隧道同代价
    # ------------------------------------------------------------------

    def test_tunnel_same_cost_as_plain(self):
        # S-13：隧道不再 *0.6。T1_L.P_E↔T1_R.P_W 隧道长 800mm，应等于物理长度 800。
        dist, path = self.oracle._dijkstra("T1_L.P_E", "T1_R.P_W")
        self.assertEqual(dist, 800.0)

    def test_entry_heading_blocks_uturn(self):
        # 车从 N1 正下方（T1_L 方向，yaw=0）进入 N1，车头朝北（yaw=180，正上方）。
        # start=N1，第一步若「原路返回往南」就是 180° 掉头，应被禁止。
        # N1 是 corner，仅 E/S 两端口。entry_heading=180（车头朝 N），
        # 出：E 端口（yaw=90，右转 90°）或 S 端口（yaw=0，掉头 180°）。
        # 掉头应被排除，只能从 E 出。
        dist, path = self.oracle._dijkstra("N1", "N1.P_E", entry_heading=180.0)
        self.assertIsNotNone(path)
        # 从 N1 到 N1.P_E（内部半边，100mm）+ 右转代价 300mm
        self.assertEqual(dist, 400.0)

    def test_entry_heading_turn_cost_added(self):
        # entry_heading=0（车头朝南=Y+ 正下方）。N1 出发去 N1.P_S（南，yaw=0）是直行 0°，
        # 去 N1.P_E（东，yaw=90）是左转 90°。直行无代价。
        dist_straight, _ = self.oracle._dijkstra("N1", "N1.P_S", entry_heading=0.0)
        dist_left, _ = self.oracle._dijkstra("N1", "N1.P_E", entry_heading=0.0)
        self.assertEqual(dist_straight, 100.0)   # 直行：仅 100mm
        self.assertEqual(dist_left, 400.0)       # 左转：100 + 300

    def test_query_shortest_path_accepts_heading(self):
        # 新增 entry_heading 参数后，打卡 N1 仍然正确且不折返（从 N1.P_S 离开）
        path = self.oracle.query_shortest_path("START", ["N1"], entry_heading=None)
        self.assertEqual(path, ["START", "J_START.P_N", "J_START", "J_START.P_W",
                                "N1.P_E", "N1", "N1.P_S"])

    def test_no_heading_behavior_unchanged(self):
        # 确保默认（不传 entry_heading）仍等价旧版无导数障碍约束的隧道距离
        # N1→N12 = 1000mm（不含隧道），应保持
        dist, path = self.oracle._dijkstra("N1", "N12")
        self.assertEqual(dist, 1000.0)


if __name__ == "__main__":
    unittest.main()
