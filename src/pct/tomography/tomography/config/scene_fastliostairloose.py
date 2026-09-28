from .scene_fastlio import SceneFastlio


class SceneFastliostairloose(SceneFastlio):
    """Export the Fast-LIO scene under a separate tomogram file name.

    All mapping and traversability settings intentionally come from
    ``SceneFastlio``.  This lets stair experiments follow the single Fast-LIO
    configuration without silently overriding values in a child profile.
    """

    def __init__(self):
        super().__init__()
        # Export under a separate name so the strict-filter tomogram remains
        # available for side-by-side comparison.
        self.pcd.file_name = 'stair_fastlio_gravity_oriented_normals_loose.pcd'
