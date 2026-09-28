# DDMR Python ROS 2 Interface

`dddmr_python_ros` wraps the vendored `dddmr_py` static PCD global planner.

## Topics

- `/clicked_point` (`geometry_msgs/PointStamped`): RViz target input.
- `/move_base_simple/goal` (`geometry_msgs/PoseStamped`): alternative target input.
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`): updates the static start.
- `/global_path` (`nav_msgs/Path`): DDMR global path.
- `/initial_path` (`nav_msgs/Path`): identical path for SCAN Planner Mode 3.
- `/dddmr/map_points`, `/dddmr/traversable`, `/dddmr/lethal`, `/dddmr/obstacles`:
  visualization point clouds.

For online operation, use `use_odom_start:=true` and set `odom_topic` to the
LIO odometry topic whose coordinates share the PCD map frame.
