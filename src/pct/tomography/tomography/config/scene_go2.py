from .scene import Scene


class SceneGo2(Scene):
    """Baseline parameters for a Go2 corridor map from Elevator-LIO."""

    def __init__(self):
        super().__init__()
        # Floor-leveled map: floor is approximately z=0 and the radar origin
        # is approximately 1.2 m above it.
        self.pcd.file_name = '8f_to_9f.pcd'

        self.map.resolution = 0.10
        self.map.ground_h = 0.0
        self.map.ground_min_h = -0.30
        self.map.ground_max_h = 100.0
        self.map.normal_ground_min_z = 0.70
        # Keep vertical sampling fine enough to identify the walking surface.
        # A very large value makes the tomography kernel select the highest
        # return in each XY cell, which can be the ceiling.
        self.map.slice_dh = 0.15

        self.trav.kernel_size = 7
        # Stair traversal must remain stricter than the earlier diagnostic
        # configuration, otherwise adjacent slabs are all treated as routes.
        self.trav.interval_min = 0.45
        self.trav.interval_free = 0.60
        self.trav.slope_max = 0.45
        self.trav.step_max = 0.18
        self.trav.standable_ratio = 0.15
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.25
        self.trav.inflation = 0.15
        # Set this to a known lower-floor point in map coordinates when the
        # capture origin is known. None uses the lowest valid support surface.
        self.trav.connectivity_seed = None
        self.trav.connectivity_threshold = 20.0
