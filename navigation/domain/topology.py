"""定义只读端口拓扑和路口中心之间的物理巡航边派生查询。"""

# 导入不可变数据类装饰器，保证静态赛道对象不保存运行时状态。
from dataclasses import dataclass
# 导入 Python 3.8 兼容的集合与序列类型注解。
from typing import Dict, Iterable, Tuple

# 导入静态赛道事实，避免从旧导航模块导入任何运行时类或单例状态。
from .track_data import BLOCK_SPECS, PORT_OFFSETS, ROAD_SPECS, START_BRIDGE_LENGTH_MM


@dataclass(frozen=True)
class MapNode:
    """静态赛道中的物理节点，表示路口中心、端口或 START。"""

    # 节点的稳定静态标识，例如 `N6` 或 `N6.P_E`。
    node_id: str
    # 节点世界横坐标，单位毫米。
    x_mm: float
    # 节点世界纵坐标，单位毫米。
    y_mm: float
    # 节点类别，例如 `base`、`junction`、`corner` 或 `port`。
    node_kind: str


@dataclass(frozen=True)
class PhysicalEdge:
    """端口图中的静态物理边，表示内部半边、道路段或启动桥。"""

    # 可复现的物理边标识，不使用可变全局自增编号。
    edge_id: str
    # 物理边定义方向的起点，查询时允许按两端关系读取。
    from_node_id: str
    # 物理边定义方向的终点。
    to_node_id: str
    # 物理边真实长度，单位毫米。
    length_mm: float
    # 道路类别，例如 `INTERNAL`、`NORMAL` 或 `TUNNEL`。
    road_kind: str


@dataclass(frozen=True)
class CruiseEdge:
    """由端口图派生的路口中心到路口中心巡航视图，不是动作计划。"""

    # 有向巡航标识，例如 `N6->N7`。
    traversal_id: str
    # 巡航起始路口中心。
    from_junction: str
    # 巡航目标路口中心。
    to_junction: str
    # 组成巡航的中心—端口—道路—端口—中心物理边标识。
    physical_edge_ids: Tuple[str, ...]
    # 巡航道路性质，供后续编排器选择普通或隧道策略。
    road_kind: str
    # 本次边级巡航的总物理距离，单位毫米。
    length_mm: float


class TrackTopology:
    """只读静态赛道拓扑，提供节点、物理边和巡航边查询。

    谁调用：位置投影器、感知适配器、编排器、仿真器和面板。
    谁响应：本类从只读静态数据返回节点、物理边或派生 `CruiseEdge`。
    输入输出：输入为静态标识；输出为不可变对象，未知标识抛出 `KeyError`。
    状态影响：不保存访问、阻塞、涵洞或任务完成等动态事实。
    """

    def __init__(self, nodes: Iterable[MapNode], edges: Iterable[PhysicalEdge]) -> None:
        """以静态节点和物理边创建一份独立只读拓扑实例。"""

        # 按节点标识建立查询表，避免外部持有可变节点容器。
        self._nodes_by_id: Dict[str, MapNode] = {node.node_id: node for node in nodes}
        # 按物理边标识建立查询表，保证查询不依赖旧代码的自增编号。
        self._edges_by_id: Dict[str, PhysicalEdge] = {edge.edge_id: edge for edge in edges}
        # 按无向端点对建立边索引，供派生巡航路径时查找内部半边。
        self._edge_id_by_endpoints: Dict[Tuple[str, str], str] = {}
        # 遍历每条物理边，为两个方向都建立相同的静态索引。
        for edge in self._edges_by_id.values():
            # 保存定义方向的端点对。
            self._edge_id_by_endpoints[(edge.from_node_id, edge.to_node_id)] = edge.edge_id
            # 保存反向端点对，使只读查询无需假设数据表中的书写方向。
            self._edge_id_by_endpoints[(edge.to_node_id, edge.from_node_id)] = edge.edge_id

    def get_node(self, node_id: str) -> MapNode:
        """返回一个静态节点，未知标识时抛出 `KeyError`。

        谁调用：位置投影器、感知适配器、编排器和面板。
        谁响应：本类从静态节点表返回 `MapNode`。
        输入输出：输入节点标识，输出不可变节点对象。
        状态影响：只读查询，不改变拓扑。
        """

        # 未知节点没有可靠几何含义，必须显式失败而不是猜测坐标。
        if node_id not in self._nodes_by_id:
            raise KeyError(node_id)
        # 返回静态不可变节点对象。
        return self._nodes_by_id[node_id]

    def get_physical_edge(self, edge_id: str) -> PhysicalEdge:
        """返回一条静态物理边，未知标识时抛出 `KeyError`。

        谁调用：巡航派生器、仿真器和静态调试工具。
        谁响应：本类从静态物理边表返回 `PhysicalEdge`。
        输入输出：输入边标识，输出不可变物理边对象。
        状态影响：只读查询，不改变拓扑。
        """

        # 未知边不能被解释为临时动态道路，必须显式失败。
        if edge_id not in self._edges_by_id:
            raise KeyError(edge_id)
        # 返回静态不可变物理边对象。
        return self._edges_by_id[edge_id]

    def get_cruise_edge(self, from_junction: str, to_junction: str) -> CruiseEdge:
        """派生两个相邻路口中心之间的物理巡航路径，不编排动作。

        谁调用：编排器在获得规划器给出的相邻路口路线后调用。
        谁响应：本类寻找连接两端端口的道路，并返回对应物理边序列。
        输入输出：输入起终点路口，输出 `CruiseEdge`；不存在直接道路时抛出 `KeyError`。
        状态影响：只读计算，不缓存动作、不修改拓扑或机器人状态。
        """

        # 起终点必须是已知静态节点，先复用节点查询进行验证。
        self.get_node(from_junction)
        self.get_node(to_junction)
        # 查找连接两个路口本体或其端口的唯一外部道路。
        road_edge, from_endpoint, to_endpoint = self._find_connecting_road(from_junction, to_junction)
        # 创建按实际行驶方向排列的物理边标识列表。
        physical_edge_ids = []
        # 起点为路口中心且道路从端口出发时，需要先经过中心到端口的内部半边。
        if from_endpoint != from_junction:
            physical_edge_ids.append(self._get_internal_edge_id(from_junction, from_endpoint))
        # 中间道路始终属于本次巡航的物理路径。
        physical_edge_ids.append(road_edge.edge_id)
        # 终点为路口中心且道路到达端口时，需要补上端口到中心的内部半边。
        if to_endpoint != to_junction:
            physical_edge_ids.append(self._get_internal_edge_id(to_endpoint, to_junction))
        # 汇总每条物理边的长度，得到本次边级巡航总距离。
        length_mm = sum(self.get_physical_edge(edge_id).length_mm for edge_id in physical_edge_ids)
        # 返回仅包含物理解释的派生视图，动作由后续编排层决定。
        return CruiseEdge(
            traversal_id="{}->{}".format(from_junction, to_junction),
            from_junction=from_junction,
            to_junction=to_junction,
            physical_edge_ids=tuple(physical_edge_ids),
            road_kind=road_edge.road_kind,
            length_mm=length_mm,
        )

    def outgoing_cruise_edges(self, node_id: str) -> Tuple[CruiseEdge, ...]:
        """返回从指定路口中心可直接驶出的全部巡航边，并保证顺序稳定。

        谁调用：`RoutePlanner` 在有向 Dijkstra 搜索某个路口的下一跳时调用。
        谁响应：本类扫描静态道路并按 `traversal_id` 返回不可变巡航边元组。
        输入输出：输入路口或 START 标识；输出从该标识出发的全部 `CruiseEdge`。
        状态影响：只读查询，不缓存搜索状态，不修改拓扑、地图或机器人状态。
        """

        # 先验证起点存在，防止未知标识被静默当作没有出边的路口。
        self.get_node(node_id)
        # 用列表收集由每一条外部道路派生的唯一出边。
        outgoing_edges = []
        # 逐条检查静态外部道路，内部中心到端口半边不能独立成为巡航目标。
        for physical_edge in self._edges_by_id.values():
            # 跳过仅属于单个路口内部的中心到端口半边。
            if physical_edge.road_kind == "INTERNAL":
                continue
            # 若道路定义方向的起点归属当前路口，则另一端是本次巡航终点。
            if self._belongs_to_junction(physical_edge.from_node_id, node_id):
                outgoing_edges.append(self.get_cruise_edge(node_id, self._get_junction_id(physical_edge.to_node_id)))
            # 若道路定义方向的终点归属当前路口，则反向巡航同样是合法静态出边。
            elif self._belongs_to_junction(physical_edge.to_node_id, node_id):
                outgoing_edges.append(self.get_cruise_edge(node_id, self._get_junction_id(physical_edge.from_node_id)))
        # 以稳定巡航标识排序，避免静态数据书写顺序影响后续 Dijkstra 的可复现性。
        return tuple(sorted(outgoing_edges, key=lambda cruise_edge: cruise_edge.traversal_id))

    def get_cruise_edges_for_physical_edge(self, edge_id: str) -> Tuple[CruiseEdge, ...]:
        """返回经过指定外部物理道路的两个相反方向巡航边。

        谁调用：`GoalDeriver` 派生涵洞两端目标，`RoutePlanner` 校验末段是否沿涵洞边驶入时调用。
        谁响应：本类从物理道路两端解析所属路口，再派生双向 `CruiseEdge`。
        输入输出：输入外部物理道路标识；输出两个相反方向的不可变巡航边元组。
        状态影响：只读查询，不把道路误写为动态涵洞或阻塞事实。
        """

        # 取得物理边并复用公开查询的未知标识校验。
        physical_edge = self.get_physical_edge(edge_id)
        # 内部半边没有两个相邻路口，不能作为涵洞或任务接近道路。
        if physical_edge.road_kind == "INTERNAL":
            raise ValueError("内部半边不能派生双向巡航道路")
        # 将道路两个端点分别映射为所属路口中心或 START。
        first_junction = self._get_junction_id(physical_edge.from_node_id)
        second_junction = self._get_junction_id(physical_edge.to_node_id)
        # 同一路口内部道路已被前置条件排除，保留显式防护以避免返回重复巡航边。
        if first_junction == second_junction:
            raise ValueError("道路两端必须属于不同路口")
        # 按巡航标识排序，确保物理道路反查的结果不依赖原始边定义方向。
        return tuple(
            sorted(
                (
                    self.get_cruise_edge(first_junction, second_junction),
                    self.get_cruise_edge(second_junction, first_junction),
                ),
                key=lambda cruise_edge: cruise_edge.traversal_id,
            )
        )

    def _find_connecting_road(self, from_junction: str, to_junction: str) -> Tuple[PhysicalEdge, str, str]:
        """查找两个相邻路口之间的外部道路及其两端节点。"""

        # 逐条检查非内部道路，静态赛道数据中相邻路口最多存在一条直接道路。
        for edge in self._edges_by_id.values():
            # 内部半边只属于单个路口，不能作为两个路口间的道路。
            if edge.road_kind == "INTERNAL":
                continue
            # 判断边定义方向是否从起点路口归属节点通向终点路口归属节点。
            if self._belongs_to_junction(edge.from_node_id, from_junction) and self._belongs_to_junction(edge.to_node_id, to_junction):
                return edge, edge.from_node_id, edge.to_node_id
            # 判断边定义方向相反时的行驶方向，并交换端点解释。
            if self._belongs_to_junction(edge.to_node_id, from_junction) and self._belongs_to_junction(edge.from_node_id, to_junction):
                return edge, edge.to_node_id, edge.from_node_id
        # 两个路口没有直接道路时，规划器必须提供中间路口而不能让拓扑猜测路线。
        raise KeyError("{} 与 {} 之间不存在直接巡航道路".format(from_junction, to_junction))

    @staticmethod
    def _belongs_to_junction(node_id: str, junction_id: str) -> bool:
        """判断一个节点是否为指定路口中心或其边界端口。"""

        # 中心节点本身属于该路口。
        if node_id == junction_id:
            return True
        # 端口采用“路口标识.P_方向”的稳定命名，因此可用前缀无歧义判断归属。
        return node_id.startswith(junction_id + ".P_")

    @staticmethod
    def _get_junction_id(node_id: str) -> str:
        """将路口端口或中心节点标识转换为所属路口中心标识。"""

        # 端口稳定采用“路口.P_方向”命名，分隔符前的部分就是所属中心标识。
        if ".P_" in node_id:
            return node_id.split(".P_", 1)[0]
        # START 等非端口节点本身就是对应的逻辑中心。
        return node_id

    def _get_internal_edge_id(self, first_node_id: str, second_node_id: str) -> str:
        """返回路口中心与端口之间的内部半边标识。"""

        # 查询端点对索引，缺失说明静态赛道数据不完整。
        key = (first_node_id, second_node_id)
        if key not in self._edge_id_by_endpoints:
            raise KeyError("{} 与 {} 之间不存在内部半边".format(first_node_id, second_node_id))
        # 返回已验证存在的静态物理边标识。
        return self._edge_id_by_endpoints[key]


def build_default_topology() -> TrackTopology:
    """按固定比赛赛道数据创建一份新的只读端口拓扑。

    谁调用：应用入口、测试和仿真装配。
    谁响应：本函数根据 `track_data.py` 创建独立 `TrackTopology`。
    输入输出：不接收参数，返回不携带动态地图状态的新拓扑实例。
    状态影响：不读取或修改 `RuntimeMap`、机器人状态或旧导航模块。
    """

    # 用列表累积本次实例独有的静态节点对象。
    nodes = [MapNode(node_id="START", x_mm=0.0, y_mm=0.0, node_kind="base")]
    # 用列表累积本次实例独有的静态物理边对象。
    edges = []
    # 按固定中心规格生成中心节点、端口节点和中心到端口内部半边。
    for center_id, x_mm, y_mm, directions in BLOCK_SPECS:
        # 根据端口数量和名称为中心选择静态节点类别。
        center_kind = _get_center_kind(center_id, directions)
        # 写入路口或角落中心节点。
        nodes.append(MapNode(node_id=center_id, x_mm=x_mm, y_mm=y_mm, node_kind=center_kind))
        # 为该中心的每个端口生成坐标与内部半边。
        for direction in directions:
            # 从静态偏移表读取当前端口的坐标偏移。
            offset_x_mm, offset_y_mm = PORT_OFFSETS[direction]
            # 按稳定命名规则构造端口节点标识。
            port_id = "{}.P_{}".format(center_id, direction)
            # 写入不可变端口节点。
            nodes.append(MapNode(node_id=port_id, x_mm=x_mm + offset_x_mm, y_mm=y_mm + offset_y_mm, node_kind="port"))
            # 写入从中心到端口的内部半边，长度等于方块半边长度。
            edges.append(_make_edge(center_id, port_id, 100.0, "INTERNAL"))
    # 写入 START 到首个路口北端口的启动桥。
    edges.append(_make_edge("START", "J_START.P_N", START_BRIDGE_LENGTH_MM, "NORMAL"))
    # 按固定道路规格写入所有端口到端口道路段。
    for from_port_id, to_port_id, length_mm, road_kind in ROAD_SPECS:
        # 写入道路段，保留普通道路或隧道道路类别。
        edges.append(_make_edge(from_port_id, to_port_id, length_mm, road_kind))
    # 用新建节点和边创建独立只读拓扑。
    return TrackTopology(nodes=nodes, edges=edges)


def _get_center_kind(center_id: str, directions: Tuple[str, ...]) -> str:
    """根据中心名称和端口数返回静态节点类别。"""

    # 起点连接方块是三向路口。
    if center_id == "J_START":
        return "junction-T"
    # T 前缀中心是四向交叉路口。
    if center_id.startswith("T"):
        return "junction-cross"
    # 两端口中心是直角拐角。
    if len(directions) == 2:
        return "corner"
    # 其余三端口中心是三向路口。
    return "junction-T"


def _make_edge(from_node_id: str, to_node_id: str, length_mm: float, road_kind: str) -> PhysicalEdge:
    """用确定性端点命名构造一条静态物理边。"""

    # 用两个稳定端点构造可复现边标识，避免旧实现的可变计数器。
    edge_id = "edge:{}--{}".format(from_node_id, to_node_id)
    # 返回不含任何运行时阻塞、访问或涵洞字段的物理边。
    return PhysicalEdge(
        edge_id=edge_id,
        from_node_id=from_node_id,
        to_node_id=to_node_id,
        length_mm=length_mm,
        road_kind=road_kind,
    )
