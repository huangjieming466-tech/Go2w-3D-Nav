from .scene_fastlio import SceneFastlio


class SceneFastliostair(SceneFastlio):
    """Permissive stair-tread profile for inspecting PCT support candidates.

    A 0.7 m support window cannot fit on a typical stair tread.  This profile
    deliberately uses a 0.3 m local window and minimal inflation so it is for
    map validation only, not a final Go2 motion-safety configuration.
    """

    def __init__(self):
        super().__init__()
        self.map.normal_ground_min_z = 0.55
        # Reject isolated upward-facing returns in open clearance while
        # retaining a sparse stair tread supported by nearby coplanar points.
        self.map.ground_support_radius = 0.22
        self.map.ground_support_min_neighbors = 4
        self.map.ground_support_height_tolerance = 0.12
        self.map.ground_support_normal_dot = 0.85
        self.map.slice_dh = 0.10

        self.trav.kernel_size = 3
        self.trav.interval_min = 0.30
        self.trav.interval_free = 0.45
        self.trav.slope_max = 0.95
        self.trav.step_max = 0.26
        self.trav.standable_ratio = 0.0
        self.trav.safe_margin = 0.05
        self.trav.inflation = 0.05
