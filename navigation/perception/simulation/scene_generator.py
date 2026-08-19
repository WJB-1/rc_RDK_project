"""
navigation/scene_generator.py — 随机场景生成器

职责:
1. 从普通道路边中随机选择 3 条障碍边 + 8 条涵洞边
2. 验证障碍物放置后的图连通性（START → 所有 RFID 节点 → 回 START）
3. 不通过则重试，直到生成可达场景或超过最大尝试次数
"""

import random
from typing import Set, List

from ...domain.topology import RaceTrackTopology, get_topology
from ...planning.map_oracle import MapOracle
from ...domain.config import MISSION_NODES
from .scene import SimScene, SceneGenerationError

# 物理尺寸常量 (mm)
FULL_CULVERT_LENGTH_MM = 200.0    # 涵洞全长 (正方形边长)
OBSTACLE_LENGTH_MM = 180.0        # 障碍物沿边方向长度
OBSTACLE_WIDTH_MM = 50.0          # 障碍物横跨车道宽度
STRAIGHT_EDGE_LENGTH_MM = 800.0   # 标准直道净长（端口模型下直道就是完整 800mm）


def _get_eligible_edges(topo: RaceTrackTopology) -> List[int]:
    """
    获取可放置障碍物/涵洞的直道边列表。

    端口模型下，只有「标准 800mm 净直道」可放物体：
    - 排除内部半边 (is_internal=True)
    - 排除隧道边 (is_tunnel=True)
    - 排除非 800mm 的特殊边（启动桥 200mm、顶部分叉 300mm）

    因为直道 = 独立 800mm 模块，物体放上面完整可用，绝不侵入路口方块。
    """
    eligible = []
    for edge in topo.edges:
        if edge.is_internal:
            continue
        if edge.is_tunnel:
            continue
        if edge.distance_mm != STRAIGHT_EDGE_LENGTH_MM:
            continue  # 排除启动桥 200mm 和顶部分叉 300mm
        eligible.append(edge.edge_id)
    return eligible


def _validate_connectivity(
    topo: RaceTrackTopology,
    oracle: MapOracle,
    obstacle_edge_ids: Set[int],
) -> bool:
    """
    验证障碍物封锁后的图连通性。

    检查:
    1. 从 START 出发能否到达所有 12 个 mission 节点
    2. 从最后一个可到达的 mission 节点能否回到 START
    """
    blocked = frozenset(obstacle_edge_ids)

    # 1. 从 START 到每个 mission 节点的可达性
    for node_name in MISSION_NODES:
        dist, path = oracle._dijkstra("START", node_name, blocked_edges=blocked)
        if path is None:
            return False

    # 2. 从每个 mission 节点回 START 的可达性
    for node_name in MISSION_NODES:
        dist, path = oracle._dijkstra(node_name, "START", blocked_edges=blocked)
        if path is None:
            return False

    return True


def generate_scene(seed: int = None, max_attempts: int = 100) -> SimScene:
    """
    生成一个可达的随机仿真场景。

    Args:
        seed: 随机种子（None = 使用系统时间）
        max_attempts: 最大尝试次数

    Returns:
        SimScene: 包含障碍物和涵洞位置信息

    Raises:
        SceneGenerationError: 超过最大尝试次数仍无法生成可达场景
    """
    rng = random.Random(seed)

    # 获取拓扑（单例，需要重置状态）
    topo = get_topology()
    eligible_pool = _get_eligible_edges(topo)

    if len(eligible_pool) < 11:  # 至少需要 3 + 8 = 11 条边
        raise SceneGenerationError(
            f"可放置障碍物/涵洞的边不足: {len(eligible_pool)} < 11"
        )

    for attempt in range(max_attempts):
        # 1. 随机选择 3 条障碍边
        obstacle_edge_ids = set(rng.sample(eligible_pool, 3))

        # 2. 从剩余边中选择 8 条涵洞边
        remaining_pool = [e for e in eligible_pool if e not in obstacle_edge_ids]
        culvert_edge_ids = set(rng.sample(remaining_pool, 8))

        # 3. 为每个障碍/涵洞生成随机 offset
        #    端口模型下直道 = 完整 800mm，offset ∈ [物体半长, 800-物体半长]
        #    物体完全落在直道内，不侵入两端路口方块
        obstacle_offsets = {}
        for eid in obstacle_edge_ids:
            edge = topo.get_edge_by_id(eid)
            half = OBSTACLE_LENGTH_MM / 2.0
            lo, hi = half, edge.distance_mm - half
            obstacle_offsets[eid] = round(rng.uniform(lo, hi), 1)

        culvert_offsets = {}
        for eid in culvert_edge_ids:
            edge = topo.get_edge_by_id(eid)
            half = FULL_CULVERT_LENGTH_MM / 2.0
            lo, hi = half, edge.distance_mm - half
            culvert_offsets[eid] = round(rng.uniform(lo, hi), 1)

        # 4. 验证连通性
        topo.reset_visit_status()
        oracle = MapOracle(topo)
        if _validate_connectivity(topo, oracle, obstacle_edge_ids):
            return SimScene(
                seed=seed if seed is not None else 0,
                attempt=attempt,
                obstacle_edge_ids=obstacle_edge_ids,
                culvert_edge_ids=culvert_edge_ids,
                obstacle_offsets=obstacle_offsets,
                culvert_offsets=culvert_offsets,
            )

    raise SceneGenerationError(
        f"无法在 {max_attempts} 次尝试内生成可达场景 (seed={seed})"
    )
