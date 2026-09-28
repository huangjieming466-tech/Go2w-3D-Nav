from .scene_superfixed import SceneSuperfixed


class SceneSuperfiltered2(SceneSuperfixed):
    """Aggressive connectivity test while preserving signed-normal filtering."""

    def __init__(self):
        super().__init__()
        self.map.normal_ground_min_z = 0.05
        self.map.ground_support_radius = 0.0
        self.map.ground_support_min_neighbors = 1

        self.trav.interval_min = 0.05
        self.trav.interval_free = 0.15
        self.trav.slope_max = 1.50
        self.trav.step_max = 0.80
        self.trav.safe_margin = 0.01
        self.trav.inflation = 0.01
