from .scene_superfixed import SceneSuperfixed


class SceneSuperfiltered(SceneSuperfixed):
    """Looser stair connectivity profile for the filtered Super-LIO map."""

    def __init__(self):
        super().__init__()
        # Keep signed-normal ceiling rejection, but retain sparse tread cells.
        self.map.normal_ground_min_z = 0.20
        self.map.ground_support_radius = 0.24
        self.map.ground_support_min_neighbors = 3
        self.map.ground_support_height_tolerance = 0.14
        self.map.ground_support_normal_dot = 0.75

        self.trav.interval_min = 0.18
        self.trav.interval_free = 0.32
        self.trav.slope_max = 1.20
        self.trav.step_max = 0.45
        self.trav.safe_margin = 0.02
        self.trav.inflation = 0.02
