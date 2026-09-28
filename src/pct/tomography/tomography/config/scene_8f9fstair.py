from .scene import Scene


class Scene8f9fstair(Scene):
    """Stair-candidate profile for the valid, pre-divergence 8F-to-9F map."""

    def __init__(self):
        super().__init__()
        self.pcd.file_name = '8f_to_9f_fastlio_valid_gravity_oriented_normals.pcd'
        # The valid map spans roughly 69 x 63 m and two floors.  A 0.15 m
        # grid keeps its 3-D tomogram within practical GPU memory.
        self.map.resolution = 0.15
        self.map.ground_h = -15.80
        self.map.ground_min_h = -16.00
        self.map.ground_max_h = 4.20
        self.map.normal_ground_min_z = 0.55
        self.map.slice_dh = 0.15

        self.trav.kernel_size = 3
        self.trav.interval_min = 0.30
        self.trav.interval_free = 0.45
        self.trav.slope_max = 0.95
        self.trav.step_max = 0.26
        self.trav.standable_ratio = 0.0
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.05
        self.trav.inflation = 0.05
        self.trav.connectivity_enable = False
