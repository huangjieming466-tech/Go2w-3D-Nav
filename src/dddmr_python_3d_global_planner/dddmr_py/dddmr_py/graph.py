"""在可通行节点上构建 3D 邻接图 (CSR 稀疏结构)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.spatial import cKDTree

from .config import PlannerConfig
from .perception import GroundMap


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

    try:
        pairs = tree.query_pairs(radius, output_type="ndarray")
    except TypeError:
        # SciPy 1.3 on the Foxy Jetson image only returns a set here.
        pairs = np.asarray(list(tree.query_pairs(radius)), dtype=np.int64).reshape(-1, 2)
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
