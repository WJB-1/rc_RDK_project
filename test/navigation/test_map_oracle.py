"""
单元测试：Map Oracle API
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from robocup_rescue_brain.navigation.map_oracle import MapOracle
from robocup_rescue_brain.navigation.map_topology import RaceTrackTopology, get_topology


class TestMapOracle(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()
        self.oracle = MapOracle(self.topo)

    def test_get_edge_properties_existing(self):
        # START -> J_START 是直接连接
        props = self.oracle.get_edge_properties("START", "J_START")
        self.assertIn("distance_mm", props)
        self.assertIn("is_tunnel", props)
        self.assertIn("speed_limit_ms", props)

    def test_get_edge_properties_undirected(self):
        # N2↔T1_L 是直接连接的无向边
        a = self.oracle.get_edge_properties("N2", "T1_L")
        b = self.oracle.get_edge_properties("T1_L", "N2")
        self.assertEqual(a, b)

    def test_get_edge_properties_nonexistent(self):
        with self.assertRaises(KeyError):
            self.oracle.get_edge_properties("N1", "N8")

    def test_tunnel_edge_flag(self):
        # T1_L↔T1_R 是隧道段
        props = self.oracle.get_edge_properties("T1_L", "T1_R")
        self.assertTrue(props["is_tunnel"])
        # N2↔N3 不是隧道（外侧纵列普通边）
        props2 = self.oracle.get_edge_properties("N2", "N3")
        self.assertFalse(props2["is_tunnel"])

    def test_dijkstra_same_node(self):
        dist, path = self.oracle._dijkstra("N1", "N1")
        self.assertEqual(dist, 0.0)
        self.assertEqual(path, ["N1"])

    def test_dijkstra_direct_edge(self):
        # START -> J_START = 200mm (直连)
        dist, path = self.oracle._dijkstra("START", "J_START")
        self.assertEqual(path, ["START", "J_START"])
        self.assertEqual(dist, 200.0)

    def test_dijkstra_multi_hop(self):
        dist, path = self.oracle._dijkstra("N1", "N12")
        # N1 -> N12 直连 = 800mm
        self.assertIsNotNone(path)
        self.assertEqual(path[0], "N1")
        self.assertEqual(path[-1], "N12")
        self.assertEqual(dist, 800.0)

    def test_shortest_path_single_target(self):
        path = self.oracle.query_shortest_path("START", ["N1"])
        # START -> J_START -> N1 (200 + 400 = 600mm)
        self.assertEqual(path, ["START", "J_START", "N1"])

    def test_shortest_path_multiple_targets(self):
        path = self.oracle.query_shortest_path("START", ["N1", "N12"])
        # START->N1=600, START->N12=600, N1<->N12=800
        # 最优顺序: START->N1->N12 或 START->N12->N1 (总长1400mm)
        self.assertEqual(path[0], "START")
        self.assertIn("N1", path)
        self.assertIn("N12", path)

    def test_shortest_path_unvisited_empty(self):
        path = self.oracle.query_shortest_path("N1", [])
        self.assertEqual(path, ["N1"])

    def test_get_path_details(self):
        # J_START→N1 (400mm) 和 N1→T1_L (800mm) 都是直接边
        details = self.oracle.get_path_details(["J_START", "N1", "T1_L"])
        self.assertEqual(len(details), 2)
        self.assertEqual(details[0]["distance_mm"], 400)   # J_START -> N1
        self.assertEqual(details[1]["distance_mm"], 800)   # N1 -> T1_L


if __name__ == "__main__":
    unittest.main()
