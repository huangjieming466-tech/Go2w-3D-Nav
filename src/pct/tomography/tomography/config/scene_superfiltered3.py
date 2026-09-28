from .scene_superfixed import SceneSuperfixed


class SceneSuperfiltered3(SceneSuperfixed):
    """Intermediate stair connectivity profile for validation."""

    def __init__(self):
        super().__init__()
        self.map.normal_ground_min_z = 0.20
        self.map.ground_support_radius = 0.0
        self.map.ground_support_min_neighbors = 1

        self.trav.interval_min = 0.10
        self.trav.interval_free = 0.25
        self.trav.slope_max = 1.20
        self.trav.step_max = 0.50
        self.trav.safe_margin = 0.02
        self.trav.inflation = 0.02
