from typing import Optional


class ScenePCD():
    file_name: Optional[str] = None


class SceneMap():
    resolution = 0.10
    ground_h = 0.0
    ground_min_h = -0.15
    ground_max_h = 0.8
    # Negative disables PCD normal-based ground selection.
    normal_ground_min_z = -2.0
    # Optional local-consistency test for normal-selected support points.
    # Disabled by default to preserve the original PCT behavior.
    ground_support_radius = 0.0
    ground_support_min_neighbors = 1
    ground_support_height_tolerance = 0.0
    # Cosine of the maximum normal-angle mismatch. Negative disables it.
    ground_support_normal_dot = -1.0
    # Optional planar region growing from normal-selected ground points.  It
    # only reclassifies measured nearby points; it never synthesizes a floor
    # in an unobserved gap.  Disabled by default.
    ground_plane_expand_radius = 0.0
    ground_plane_expand_min_neighbors = 5
    ground_plane_expand_max_distance = 0.0
    ground_plane_expand_min_normal_z = 0.0
    slice_dh = 0.5


class SceneTrav():
    kernel_size = 7
    interval_min = 0.50
    interval_free = 0.65
    slope_max = 0.36
    step_max = 0.20
    standable_ratio = 0.20
    cost_barrier = 50.0

    safe_margin = 0.4
    inflation = 0.2
    # Keep only traversable cells that are connected through Go2-safe steps.
    # None selects the lowest valid component; set [x, y, z] for deterministic
    # operation on a known start floor.
    connectivity_seed = None
    # This is an optional post-filter.  Keep it disabled unless a capture has
    # been verified; selecting the lowest component is unsafe for stairs.
    connectivity_enable = False
    connectivity_threshold = 20.0
    connectivity_output_cost = 15.0


class Scene():
    pcd: ScenePCD = ScenePCD()
    map: SceneMap = SceneMap()
    trav: SceneTrav = SceneTrav()
