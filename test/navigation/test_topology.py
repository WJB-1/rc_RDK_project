"""
单元测试：赛道拓扑与节点定义
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from robocup_rescue_brain.navigation.map_topology import RaceTrackTopology, MapNode, MapEdge, get_topology
from robocup_rescue_brain.navigation.map_config import NODE_COORDS, MISSION_NODES, JUNCTION_NODES


class TestTopology(unittest.TestCase):

    def setUp(self):
        self.topo = RaceTrackTopology()

    def test_node_count(self):
        self.assertEqual(len(self.topo.nodes), 20)

    def test_all_nodes_exist(self):
        for name in NODE_COORDS:
            self.assertIn(name, self.topo.nodes)
            node = self.topo.get_node(name)
            self.assertIsInstance(node, MapNode)
            self.assertEqual(node.name, name)

    def test_node_coordinates(self):
        for name, coord in NODE_COORDS.items():
            node = self.topo.get_node(name)
            self.assertAlmostEqual(node.x_mm, coord["x"])
            self.assertAlmostEqual(node.y_mm, coord["y"])

    def test_mission_nodes_have_rfid(self):
        for name in MISSION_NODES:
            node = self.topo.get_node(name)
            self.assertTrue(node.has_rfid, f"{name} 应该有 RFID")
            self.assertEqual(node.node_type, "mission")

    def test_junction_nodes_no_rfid(self):
        for name in JUNCTION_NODES:
            node = self.topo.get_node(name)
            self.assertFalse(node.has_rfid, f"{name} 不应有 RFID")
            self.assertEqual(node.node_type, "junction")

    def test_start_node(self):
        start = self.topo.get_node("START")
        self.assertEqual(start.node_type, "base")
        self.assertFalse(start.has_rfid)
        self.assertEqual(start.x_mm, 0.0)
        self.assertEqual(start.y_mm, 0.0)

    def test_edges_are_undirected(self):
        for edge in self.topo.edges:
            self.assertTrue(self.topo.has_edge(edge.node_a, edge.node_b))
            self.assertTrue(self.topo.has_edge(edge.node_b, edge.node_a))

    def test_neighbor_symmetry(self):
        for node_name in self.topo.nodes:
            neighbors = self.topo.get_neighbor_names(node_name)
            for nb in neighbors:
                self.assertIn(node_name, self.topo.get_neighbor_names(nb))

    def test_edge_properties_positive(self):
        for edge in self.topo.edges:
            self.assertGreater(edge.distance_mm, 0)
            self.assertIsInstance(edge.is_tunnel, bool)

    def test_reset_visit_status(self):
        self.topo.get_node("N1").is_visited = True
        self.topo.reset_visit_status()
        for name in MISSION_NODES:
            self.assertFalse(self.topo.get_node(name).is_visited)

    def test_mission_progress(self):
        self.topo.reset_visit_status()
        self.assertEqual(self.topo.get_mission_progress(), (0, 12))
        self.topo.get_node("N1").is_visited = True
        self.assertEqual(self.topo.get_mission_progress(), (1, 12))

    def test_all_missions_completed(self):
        self.topo.reset_visit_status()
        self.assertFalse(self.topo.all_missions_completed())
        for name in MISSION_NODES:
            self.topo.get_node(name).is_visited = True
        self.assertTrue(self.topo.all_missions_completed())

    def test_to_dict_serializable(self):
        data = self.topo.to_dict()
        self.assertIn("nodes", data)
        self.assertIn("edges", data)
        self.assertEqual(len(data["nodes"]), 20)


class TestMapNode(unittest.TestCase):

    def test_distance_to(self):
        a = MapNode("A", 0, 0, "test", False)
        b = MapNode("B", 300, 400, "test", False)
        self.assertAlmostEqual(a.distance_to(b), 500.0)


if __name__ == "__main__":
    unittest.main()
