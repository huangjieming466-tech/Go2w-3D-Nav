from .scene import Scene


class SceneSuperfixed(Scene):
    """PCT profile for the corrected Super-LIO static map."""

    def __init__(self):
        super().__init__()
        self.pcd.file_name = 'go2w_mid360_map_fixed_oriented_normals.pcd'

        self.map.resolution = 0.15
        self.map.ground_h = -2.50
        self.map.ground_min_h = -3.00
        self.map.ground_max_h = 11.00
        # The PCD carries signed normals: upward-facing support surfaces are
        # retained, while downward-facing ceiling/slab undersides are rejected.
        self.map.normal_ground_min_z = 0.20
        self.map.slice_dh = 0.10

        self.trav.kernel_size = 3
        self.trav.interval_min = 0.30
        self.trav.interval_free = 0.45
        self.trav.slope_max = 0.95
        self.trav.step_max = 0.26
        self.trav.standable_ratio = 0.0
        self.trav.safe_margin = 0.05
        self.trav.inflation = 0.05
        self.trav.connectivity_enable = False
