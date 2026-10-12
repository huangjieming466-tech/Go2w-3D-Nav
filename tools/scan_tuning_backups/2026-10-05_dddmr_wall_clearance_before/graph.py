"""在可通行节点上构建 3D 邻接图 (CSR 稀疏结构)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.spatial import cKDTree

from .config import PlannerConfig
from .perception import GroundMap


def footprint_segments_valid(gmap: GroundMap, start: np.ndarray, finish: np.ndarray,
                             cfg: PlannerConfig, start_normals: np.ndarray | None = None,
                             finish_normals: np.ndarray | None = None) -> np.ndarray:
    """Return whether oriented two-circle body segments avoid obstacles.

    Inputs are direct XYZ segment endpoints so this can validate both graph
    edges and post-processed paths.
    """
    if (cfg.footprint_offset <= 1e-6 or gmap.obstacle_tree_xy is None or
            len(gmap.obstacles) == 0):
        return np.ones(len(start), dtype=bool)

    radius = cfg.footprint_radius if cfg.footprint_radius > 0 else cfg.robot_radius
    if radius <= 0:
        return np.ones(len(start), dtype=bool)

    start = np.asarray(start, dtype=np.float64)
    finish = np.asarray(finish, dtype=np.float64)
    count = len(start)
    if start_normals is None:
        _, start_idx = gmap.kdtree.query(start, k=1)
        start_normals = gmap.normals[start_idx]
    if finish_normals is None:
        _, finish_idx = gmap.kdtree.query(finish, k=1)
        finish_normals = gmap.normals[finish_idx]

    samples = max(2, int(cfg.footprint_edge_samples))
    fractions = np.linspace(0.0, 1.0, samples)
    valid = np.ones(count, dtype=bool)
    batch_size = 50000

    for begin in range(0, count, batch_size):
        end = min(begin + batch_size, count)
        starts, finishes = start[begin:end], finish[begin:end]
        normals_a = start_normals[begin:end]
        normals_b = finish_normals[begin:end]
        direction = finishes[:, :2] - starts[:, :2]
        length = np.linalg.norm(direction, axis=1)
        heading = np.zeros_like(direction)
        nonzero = length > 1e-9
        heading[nonzero] = direction[nonzero] / length[nonzero, None]

        blocked = np.zeros(end - begin, dtype=bool)
        for fraction in fractions:
            base = starts * (1.0 - fraction) + finishes * fraction
            normal = normals_a * (1.0 - fraction) + normals_b * fraction
            normal_norm = np.linalg.norm(normal, axis=1, keepdims=True)
            normal /= np.where(normal_norm > 1e-9, normal_norm, 1.0)

            shift = np.column_stack((heading * cfg.footprint_offset,
                                     np.zeros(end - begin)))
            centres = np.concatenate((base + shift, base - shift), axis=0)
            bases = np.concatenate((base, base), axis=0)
            surface_normals = np.concatenate((normal, normal), axis=0)
            try:
                distance, obstacle_idx = gmap.obstacle_tree_xy.query(
                    centres[:, :2], k=1, workers=-1
                )
            except TypeError:
                # SciPy versions used by some ARM robot images do not expose
                # the workers argument.
                distance, obstacle_idx = gmap.obstacle_tree_xy.query(
                    centres[:, :2], k=1
                )
            close = distance <= radius
            if np.any(close):
                relative = gmap.obstacles[obstacle_idx[close]] - bases[close]
                dz = np.einsum("ij,ij->i", relative, surface_normals[close])
                collision = (dz > cfg.max_step) & (dz < cfg.robot_height)
                hit_index = np.flatnonzero(close)[collision] % (end - begin)
                blocked[hit_index] = True

        valid[begin:end] = ~blocked

    return valid


def _footprint_edge_valid(gmap: GroundMap, a: np.ndarray, b: np.ndarray,
                          cfg: PlannerConfig) -> np.ndarray:
    """Validate graph edges with the same footprint used for output paths."""
    return footprint_segments_valid(
        gmap, gmap.nodes[a], gmap.nodes[b], cfg, gmap.normals[a], gmap.normals[b]
    )


@dataclass
class NavGraph:
    """CSR 形式的导航图: indptr/indices/weights."""

    indptr: np.ndarray
    indices: np.ndarray
    weights: np.ndarray
    nodes: np.ndarray
    lethal: np.ndarray
    cost: np.ndarray
    kdtree: cKDTree
    config: PlannerConfig

    @property
    def n_nodes(self) -> int:
        return len(self.nodes)

    @property
    def n_edges(self) -> int:
        return len(self.indices) // 2

    def neighbors(self, i: int):
        beg, end = self.indptr[i], self.indptr[i + 1]
        return self.indices[beg:end], self.weights[beg:end]

    def __repr__(self) -> str:  # pragma: no cover
        return f"<NavGraph nodes={self.n_nodes} edges≈{self.n_edges}>"


def build_graph(gmap: GroundMap, cfg: PlannerConfig | None = None,
                verbose: bool = False) -> NavGraph:
    """半径近邻建图, 并按坡度/台阶/致命代价过滤不可行边.

    边权 w = 3D 长度 * (1 + cost_weight * 平均代价)
    """
    cfg = cfg or gmap.config
    radius = cfg.connection_radius if cfg.connection_radius > 0 else 2.6 * cfg.voxel_size
    nodes, lethal, cost = gmap.nodes, gmap.lethal, gmap.cost
    tree = gmap.kdtree

    pairs = tree.query_pairs(radius, output_type="ndarray")
    if len(pairs) == 0:
        raise RuntimeError("建图失败: 没有任何邻接边, 请增大 connection_radius")
    a, b = pairs[:, 0], pairs[:, 1]

    delta = nodes[b] - nodes[a]
    d_xy = np.linalg.norm(delta[:, :2], axis=1)
    d_z = np.abs(delta[:, 2])
    length = np.linalg.norm(delta, axis=1)

    # 1) 边坡度约束: 水平位移极小的"垂直边"按台阶处理
    tan_max = np.tan(np.deg2rad(cfg.max_slope_deg))
    slope_ok = d_z <= np.maximum(tan_max * d_xy, cfg.max_step)
    # 2) 台阶约束.  Do not let a large connection radius silently relax the
    # robot's vertical capability: that used to create metre-scale "steps"
    # when bridging sparse stair maps.  Dense ramps remain connected through
    # their short local edges, each of which stays within max_step.
    step_ok = d_z <= cfg.max_step
    # 3) 致命节点不可进入
    free_ok = (~lethal[a]) & (~lethal[b])
    valid = slope_ok & step_ok & free_ok & (length > 1e-9)

    # The oriented footprint must be checked before graph search. Otherwise
    # DDMR can publish a centre-line route that SCAN later rejects at a corner.
    if cfg.footprint_offset > 1e-6 and np.any(valid):
        valid_indices = np.flatnonzero(valid)
        footprint_ok = _footprint_edge_valid(gmap, a[valid_indices], b[valid_indices], cfg)
        valid[valid_indices] &= footprint_ok

    a, b, length = a[valid], b[valid], length[valid]
    d_xy, d_z = d_xy[valid], d_z[valid]
    mean_cost = 0.5 * (cost[a] + cost[b])
    w = length * (1.0 + cfg.cost_weight * mean_cost)

    # Keep ordinary local mesh edges first.
    keep = np.ones(len(a), dtype=bool)
    if cfg.max_neighbors > 0 and len(a):
        for src in (a, b):
            order = np.lexsort((length, src))          # 先按节点分组, 组内按长度升序
            grouped = src[order]
            starts = np.searchsorted(grouped, grouped, side="left")
            rank = np.arange(len(grouped)) - starts
            keep[order] &= rank < cfg.max_neighbors

    # Dense points on a single tread are closer than the following tread, so
    # a pure k-nearest graph loses all climbing links.  Retain a bounded set
    # of short, height-changing links for every node without opening long
    # cross-room shortcuts.
    if cfg.transition_neighbors > 0 and len(a):
        transition = (
            (d_z >= cfg.transition_min_height)
            & (d_xy <= cfg.transition_radius)
        )
        transition_keep = np.zeros(len(a), dtype=bool)
        for src in (a, b):
            candidate = np.flatnonzero(transition)
            if len(candidate) == 0:
                continue
            order = candidate[np.lexsort((length[candidate], src[candidate]))]
            grouped = src[order]
            starts = np.searchsorted(grouped, grouped, side="left")
            rank = np.arange(len(grouped)) - starts
            transition_keep[order] |= rank < cfg.transition_neighbors
        keep |= transition_keep

    a, b, w, length = a[keep], b[keep], w[keep], length[keep]

    n = len(nodes)
    rows = np.concatenate([a, b])
    cols = np.concatenate([b, a])
    vals = np.concatenate([w, w])
    csr = coo_matrix((vals, (rows, cols)), shape=(n, n)).tocsr()
    csr.sort_indices()

    if verbose:
        print(f"[graph] 节点 {n} 个, 双向边 {csr.nnz} 条 (半径 {radius:.2f} m)")

    return NavGraph(indptr=csr.indptr.astype(np.int64),
                    indices=csr.indices.astype(np.int64),
                    weights=csr.data.astype(np.float64),
                    nodes=nodes, lethal=lethal, cost=cost, kdtree=tree, config=cfg)
