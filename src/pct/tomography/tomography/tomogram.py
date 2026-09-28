import numpy as np
from collections import deque
try:
    import cupy as cp
except ImportError:
    cp = None

try:
    from scipy import ndimage
except ImportError:
    ndimage = None

from .kernels import *


class Tomogram(object):
    def __init__(self, cfg):
        self.use_gpu = False
        if cp is not None:
            try:
                # Importing CuPy is not enough: verify that the CUDA runtime
                # can actually enumerate a device in this shell.
                self.use_gpu = cp.cuda.runtime.getDeviceCount() > 0
            except Exception:
                self.use_gpu = False
        if not self.use_gpu and ndimage is None:
            raise RuntimeError("CPU tomography requires scipy")
        self.resolution = cfg.map.resolution
        self.ground_h = cfg.map.ground_h
        self.ground_min_h = cfg.map.ground_min_h
        self.normal_ground_min_z = cfg.map.normal_ground_min_z
        self.slice_dh = cfg.map.slice_dh
        self.ground_max_h = cfg.map.ground_max_h
        self.half_trav_k_size = int(cfg.trav.kernel_size / 2)
        self.interval_min = cfg.trav.interval_min
        self.interval_free = cfg.trav.interval_free
        self.step_stand = 1.2 * self.resolution * np.tan(cfg.trav.slope_max)
        self.step_cross = cfg.trav.step_max
        self.standable_th = int(cfg.trav.standable_ratio * (2 * self.half_trav_k_size + 1) ** 2) - 1
        self.cost_barrier = float(cfg.trav.cost_barrier)
        self.safe_margin = cfg.trav.safe_margin
        self.inflation = cfg.trav.inflation
        self.connectivity_seed = cfg.trav.connectivity_seed
        self.connectivity_enable = bool(cfg.trav.connectivity_enable)
        self.connectivity_threshold = float(cfg.trav.connectivity_threshold)
        self.connectivity_output_cost = float(cfg.trav.connectivity_output_cost)
        self.half_inf_k_size = int((self.safe_margin + self.inflation) / self.resolution)

    def initKernel(self):
        self.tomography_kernel = tomographyKernel(
            self.resolution, 
            self.map_dim_x, 
            self.map_dim_y,
            self.n_slice_init,
            self.slice_h0,
            self.slice_dh
        )

        self.trav_kernel = travKernel(
            self.map_dim_x,
            self.map_dim_y,
            self.half_trav_k_size,
            self.interval_min,
            self.interval_free,
            self.step_cross, 
            self.step_stand, 
            self.standable_th, 
            self.cost_barrier
        )

        self.inflation_kernel = inflationKernel(
            self.map_dim_x,
            self.map_dim_y,
            self.half_inf_k_size
        )

        xp = cp if self.use_gpu else np
        self.inf_table = xp.zeros(
            (2 * self.half_inf_k_size + 1, 2 * self.half_inf_k_size + 1), 
            dtype=xp.float32
        )
        for i in range(self.inf_table.shape[0]):
            for j in range(self.inf_table.shape[1]):
                dist = np.sqrt(
                    (self.resolution * (i - self.half_inf_k_size)) ** 2 + \
                    (self.resolution * (j - self.half_inf_k_size)) ** 2
                )
                self.inf_table[i, j] = np.clip(
                    1 - (dist - self.inflation) / (self.safe_margin + self.resolution),
                    a_min=0.0, a_max=1.0
                )

    def initBuffers(self):
        xp = cp if self.use_gpu else np
        shape = (self.n_slice_init, self.map_dim_x, self.map_dim_y)
        self.layers_g = xp.zeros(shape, dtype=xp.float32)
        self.layers_c = xp.zeros(shape, dtype=xp.float32)
        self.grad_mag_sq = xp.zeros(shape, dtype=xp.float32)
        self.grad_mag_max = xp.zeros(shape, dtype=xp.float32)
        self.trav_cost = xp.zeros(shape, dtype=xp.float32)
        self.inflated_cost = xp.zeros(shape, dtype=xp.float32)

    def initMappingEnv(self, center, map_dim_x, map_dim_y, n_slice_init, slice_h0):
        xp = cp if self.use_gpu else np
        self.center = xp.array(center, dtype=xp.float32)
        self.map_dim_x = int(map_dim_x)
        self.map_dim_y = int(map_dim_y)
        self.n_slice_init = int(n_slice_init)
        self.slice_h0 = float(slice_h0)
        
        self.initBuffers()
        if self.use_gpu:
            self.initKernel()

    def clearMap(self):
        self.layers_g *= 0.
        self.layers_c *= 0.
        self.layers_g += -1e6
        self.layers_c += 1e6

        self.grad_mag_sq *= 0.
        self.grad_mag_max *= 0.
        self.trav_cost *= 0.
        self.inflated_cost *= 0.

    def point2map(self, points, ground_flags=None):
        if not self.use_gpu:
            return self._point2map_cpu(points, ground_flags)

        points = cp.asarray(points)
        finite = ~cp.isnan(points).any(axis=1)
        points = points[finite]
        if ground_flags is None:
            ground_flags = cp.ones(points.shape[0], dtype=cp.float32)
        else:
            ground_flags = cp.asarray(ground_flags, dtype=cp.float32)[finite]
        self.clearMap()

        # Tomogram
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()
        
        self.tomography_kernel(
            points, ground_flags, self.center,
            self.layers_g, self.layers_c,
            size=(points.shape[0])
        )

        # A cell with only a high return, such as a ceiling, has no valid
        # floor evidence and must not become a walkable surface.
        self.layers_g[(self.layers_g < self.ground_min_h) |
                      (self.layers_g > self.ground_max_h)] = -1e6
        valid_ground = self.layers_g > -1e5
        ground_for_gradient = cp.where(valid_ground, self.layers_g, self.ground_h)

        diff_x_sq = cp.maximum(
            (ground_for_gradient[:, 1:-1, :] - ground_for_gradient[:, :-2, :]) ** 2,
            (ground_for_gradient[:, 1:-1, :] - ground_for_gradient[:,  2:, :]) ** 2
        )
        diff_y_sq = cp.maximum(
            (ground_for_gradient[:, :, 1:-1] - ground_for_gradient[:, :, :-2]) ** 2,
            (ground_for_gradient[:, :, 1:-1] - ground_for_gradient[:, :,  2:]) ** 2
        )
        self.grad_mag_sq[:, 1:-1, 1:-1] = diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
        self.grad_mag_max[:, 1:-1, 1:-1] = cp.maximum(diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])
        
        interval = (self.layers_c - self.layers_g)

        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_map = cp.cuda.get_elapsed_time(start_gpu, end_gpu)

        # Traversability
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()

        self.trav_kernel(
            interval, self.grad_mag_sq, self.grad_mag_max,
            self.trav_cost,
            size=(self.n_slice_init * self.map_dim_x * self.map_dim_y)
        )

        self.trav_cost[~valid_ground] = self.cost_barrier

        self.inflation_kernel(
            self.trav_cost, self.inf_table,
            self.inflated_cost,
            size=(self.n_slice_init * self.map_dim_x * self.map_dim_y)
        )

        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_trav = cp.cuda.get_elapsed_time(start_gpu, end_gpu)

        # Layer Simplification
        start_gpu = cp.cuda.Event()
        end_gpu = cp.cuda.Event()
        start_gpu.record()

        idx_simp = [0]
        if self.layers_g.shape[0] > 1:
            l_idx, m_idx = 0, 1
            diff_h = self.layers_g[1:] - self.layers_g[:-1]
            while m_idx < self.n_slice_init - 2:
                mask_l_g = self.layers_g[m_idx] - self.layers_g[l_idx] > 0
                mask_l_t = self.inflated_cost[l_idx] > self.inflated_cost[m_idx]
                mask_u_g = diff_h[m_idx] > 0
                mask_t = self.inflated_cost[m_idx] < self.cost_barrier
                unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
                if cp.any(unique):
                    idx_simp.append(m_idx)
                    l_idx = m_idx
                m_idx += 1
            idx_simp.append(m_idx)

        trav_grad_x = (self.inflated_cost[idx_simp][:, 2:, :] - self.inflated_cost[idx_simp][:, :-2, :])
        trav_grad_y = (self.inflated_cost[idx_simp][:, :, 2:] - self.inflated_cost[idx_simp][:, :, :-2])
        
        end_gpu.record()
        end_gpu.synchronize()
        gpu_t_simp = cp.cuda.get_elapsed_time(start_gpu, end_gpu)
        
        gpu_t_all = gpu_t_map + gpu_t_trav + gpu_t_simp
        #print("CuPy GPU time (ms):", gpu_t_all)

        layers_t = self.inflated_cost[idx_simp].get()
        layers_g = cp.where(
            self.layers_g[idx_simp] > -1e6, 
            self.layers_g[idx_simp],
            cp.nan
        ).get()
        layers_c = cp.where(
            self.layers_c[idx_simp] < 1e6, 
            self.layers_c[idx_simp], 
            cp.nan
        ).get()
        layers_t = self.apply_stair_connectivity(layers_t, layers_g)
        trav_gx = np.zeros_like(layers_g)
        trav_gx[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
        trav_gy = np.zeros_like(layers_g)
        trav_gy[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]

        t_gpu = {
            't_map': gpu_t_map, 
            't_trav': gpu_t_trav, 
            't_simp': gpu_t_simp, 
        }
        
        return layers_t, trav_gx, trav_gy, layers_g, layers_c, t_gpu

    def apply_stair_connectivity(self, layers_t, layers_g):
        if not self.connectivity_enable:
            return layers_t
        """Reject isolated horizontal slabs while retaining reachable stair treads.

        PCT evaluates each vertical slice independently.  For stairs that
        makes a nearby floor slab look as valid as the treads.  This pass
        links cells in 3D only through adjacent XY cells whose support-height
        delta is no greater than the configured Go2 step limit.
        """
        # A stair tread is often locally expensive because its edge creates a
        # large height gradient. Use every non-barrier support cell to build
        # the geometric staircase graph, then lower the cost only after it is
        # proven connected to the seed platform.
        valid = np.isfinite(layers_g) & (layers_t < self.cost_barrier)
        if not np.any(valid):
            return layers_t

        indices = np.argwhere(valid)
        if self.connectivity_seed is None:
            seed = indices[np.argmin(layers_g[valid])]
        else:
            seed_xyz = np.asarray(self.connectivity_seed, dtype=np.float32)
            xyz = np.column_stack((
                (indices[:, 1] - self.map_dim_x // 2) * self.resolution + float(self.center[0]),
                (indices[:, 2] - self.map_dim_y // 2) * self.resolution + float(self.center[1]),
                layers_g[valid],
            ))
            seed = indices[np.argmin(np.sum((xyz - seed_xyz) ** 2, axis=1))]

        allowed = np.zeros_like(valid, dtype=bool)
        queue = deque([tuple(int(v) for v in seed)])
        allowed[tuple(seed)] = True
        n_layer, n_x, n_y = valid.shape
        while queue:
            layer, x_idx, y_idx = queue.popleft()
            z_here = layers_g[layer, x_idx, y_idx]
            for next_layer in range(max(0, layer - 1), min(n_layer, layer + 2)):
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        # Adjacent vertical slices at the same XY cell are
                        # alternate representations of a tread. They must be
                        # connected; only the exact same 3D cell is skipped.
                        if dx == 0 and dy == 0 and next_layer == layer:
                            continue
                        nx, ny = x_idx + dx, y_idx + dy
                        if not (0 <= nx < n_x and 0 <= ny < n_y):
                            continue
                        if not valid[next_layer, nx, ny] or allowed[next_layer, nx, ny]:
                            continue
                        if abs(float(layers_g[next_layer, nx, ny] - z_here)) > self.step_cross:
                            continue
                        allowed[next_layer, nx, ny] = True
                        queue.append((next_layer, nx, ny))

        filtered = layers_t.copy()
        filtered[~allowed] = self.cost_barrier
        filtered[allowed] = np.minimum(filtered[allowed], self.connectivity_output_cost)
        return filtered

    def _point2map_cpu(self, points, ground_flags=None):
        """CPU fallback for machines without CUDA/CuPy.

        It preserves the PCT map representation and uses SciPy filters for
        the local traversability and inflation operations.
        """
        points = np.asarray(points, dtype=np.float32)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if ground_flags is None:
            ground_flags = np.ones(points.shape[0], dtype=bool)
        else:
            ground_flags = np.asarray(ground_flags, dtype=bool)[finite]
        self.clearMap()

        ix = np.rint((points[:, 0] - float(self.center[0])) / self.resolution).astype(np.int64)
        iy = np.rint((points[:, 1] - float(self.center[1])) / self.resolution).astype(np.int64)
        valid = (ix >= -self.map_dim_x // 2) & (ix < self.map_dim_x // 2 + self.map_dim_x % 2)
        valid &= (iy >= -self.map_dim_y // 2) & (iy < self.map_dim_y // 2 + self.map_dim_y % 2)
        ix = ix[valid] + self.map_dim_x // 2
        iy = iy[valid] + self.map_dim_y // 2
        z = points[valid, 2]
        cell = ix * self.map_dim_y + iy
        flat_g = self.layers_g.reshape(self.n_slice_init, -1)
        flat_c = self.layers_c.reshape(self.n_slice_init, -1)

        for s_idx in range(self.n_slice_init):
            height = self.slice_h0 + s_idx * self.slice_dh
            below = (z <= height) & ground_flags
            above = z > height
            if np.any(below):
                np.maximum.at(flat_g[s_idx], cell[below], z[below])
            if np.any(above):
                np.minimum.at(flat_c[s_idx], cell[above], z[above])

        # Keep the CPU fallback consistent with the CUDA implementation.
        self.layers_g[(self.layers_g < self.ground_min_h) |
                      (self.layers_g > self.ground_max_h)] = -1e6
        valid_ground = self.layers_g > -1e5
        ground_for_gradient = np.where(valid_ground, self.layers_g, self.ground_h)

        diff_x_sq = np.maximum(
            (ground_for_gradient[:, 1:-1, :] - ground_for_gradient[:, :-2, :]) ** 2,
            (ground_for_gradient[:, 1:-1, :] - ground_for_gradient[:, 2:, :]) ** 2,
        )
        diff_y_sq = np.maximum(
            (ground_for_gradient[:, :, 1:-1] - ground_for_gradient[:, :, :-2]) ** 2,
            (ground_for_gradient[:, :, 1:-1] - ground_for_gradient[:, :, 2:]) ** 2,
        )
        self.grad_mag_sq[:, 1:-1, 1:-1] = diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
        self.grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
            diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :]
        )

        interval = self.layers_c - self.layers_g
        stand_sq = self.step_stand ** 2
        cross_sq = self.step_cross ** 2
        kernel = 2 * self.half_trav_k_size + 1
        for s_idx in range(self.n_slice_init):
            cost = np.full((self.map_dim_x, self.map_dim_y), self.cost_barrier, dtype=np.float32)
            free = interval[s_idx] >= self.interval_min
            cost[free] = np.maximum(0.0, 20.0 * (self.interval_free - interval[s_idx, free]))
            smooth = self.grad_mag_sq[s_idx] <= stand_sq
            cost[smooth] += 15.0 * self.grad_mag_sq[s_idx, smooth] / max(stand_sq, 1e-9)
            cross = (~smooth) & (self.grad_mag_max[s_idx] <= cross_sq)
            standable = ndimage.convolve(
                smooth.astype(np.int16), np.ones((kernel, kernel), dtype=np.int16), mode='constant'
            )
            usable = cross & (standable >= self.standable_th)
            cost[usable] = cost[usable] + 20.0 * self.grad_mag_max[s_idx, usable] / max(cross_sq, 1e-9)
            self.trav_cost[s_idx] = np.where(free, cost, self.cost_barrier)
            self.trav_cost[s_idx][~valid_ground[s_idx]] = self.cost_barrier

            # The exact GPU kernel applies a radial score. A maximum filter
            # retains the same conservative obstacle inflation on CPU.
            self.inflated_cost[s_idx] = ndimage.maximum_filter(
                self.trav_cost[s_idx], size=2 * self.half_inf_k_size + 1, mode='constant'
            )

        idx_simp = [0]
        if self.layers_g.shape[0] > 1:
            l_idx, m_idx = 0, 1
            diff_h = self.layers_g[1:] - self.layers_g[:-1]
            while m_idx < self.n_slice_init - 2:
                mask_l_g = self.layers_g[m_idx] - self.layers_g[l_idx] > 0
                mask_l_t = self.inflated_cost[l_idx] > self.inflated_cost[m_idx]
                mask_u_g = diff_h[m_idx] > 0
                mask_t = self.inflated_cost[m_idx] < self.cost_barrier
                unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
                if np.any(unique):
                    idx_simp.append(m_idx)
                    l_idx = m_idx
                m_idx += 1
            idx_simp.append(m_idx)

        layers_t = self.inflated_cost[idx_simp]
        layers_g = np.where(self.layers_g[idx_simp] > -1e6, self.layers_g[idx_simp], np.nan)
        layers_c = np.where(self.layers_c[idx_simp] < 1e6, self.layers_c[idx_simp], np.nan)
        trav_gx = np.zeros_like(layers_g)
        trav_gy = np.zeros_like(layers_g)
        trav_gx[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
        trav_gy[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]
        t_cpu = {'t_map': 0.0, 't_trav': 0.0, 't_simp': 0.0}
        layers_t = self.apply_stair_connectivity(layers_t, layers_g)
        trav_gx = np.zeros_like(layers_g)
        trav_gy = np.zeros_like(layers_g)
        trav_gx[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
        trav_gy[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]
        return layers_t, trav_gx, trav_gy, layers_g, layers_c, t_cpu
