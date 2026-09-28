from .scene_fastliostairloose import SceneFastliostairloose


class Scene8f9fstairoriented(SceneFastliostairloose):
    """Loose stair profile for the 8F-to-9F Elevator-LIO oriented map."""

    def __init__(self):
        super().__init__()
        self.pcd.file_name = '8f_9f_stair_oriented_normals.pcd'

        # Keep the inherited fastliostairloose traversability settings, while
        # matching this map's observed vertical extent (-1.46 to 9.14 m).
        self.map.ground_h = -1.60
        self.map.ground_min_h = -2.00
        self.map.ground_max_h = 9.50
