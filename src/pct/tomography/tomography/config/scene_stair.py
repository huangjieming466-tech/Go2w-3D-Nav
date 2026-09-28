from .scene import Scene


class SceneStair(Scene):
    """PCT profile for a gravity-aligned, labelled staircase capture."""

    def __init__(self):
        super().__init__()
        self.pcd.file_name = 'stair_floor0_support_candidates.pcd'
        self.map.resolution = 0.10
        self.map.ground_h = 0.0
        self.map.ground_min_h = -0.20
        self.map.ground_max_h = 20.0
        self.map.normal_ground_min_z = 0.70
        self.map.slice_dh = 0.12
        self.trav.kernel_size = 7
        self.trav.interval_min = 0.55
        self.trav.interval_free = 0.70
        self.trav.slope_max = 0.45
        self.trav.step_max = 0.20
        self.trav.standable_ratio = 0.15
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.20
        self.trav.inflation = 0.12
        self.trav.connectivity_seed = None
        self.trav.connectivity_enable = True
        self.trav.connectivity_threshold = 20.0
        self.trav.connectivity_output_cost = 15.0
