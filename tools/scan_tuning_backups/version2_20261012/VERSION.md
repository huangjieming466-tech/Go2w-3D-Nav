# SCAN Planner Version 2 Snapshot

Created: 2026-10-12

This snapshot is the active V2 baseline immediately before the dynamic/static
occupancy experiment (V3).

Restore the V2 local planner state by copying `plan_env/` back to
`src/scan_planner_ros2/src/planner/plan_env/`, `planner.yaml` back to
`src/scan_planner_ros2/src/planner/plan_manage/config/planner.yaml`, and
`run.launch.py` back to
`src/scan_planner_ros2/src/planner/plan_manage/launch/run.launch.py`. The
captured report is the V2 documentation baseline. Other workspace packages and
user changes are not part of this snapshot.

V2 relevant state:

- `fsm.planning_horizon` and `manager.planning_horizon`: `4.5 m`.
- `grid_map.obstacles_inflation_z_up`: `0.0 m`.
- `grid_map.obstacles_inflation_z_down`: `0.15 m`.
- `grid_map.p_miss`: `0.22`.
- Clipped free-ray handling and the existing V2 Mode 3 tracking changes are
  included.

Version 3 adds transient-occupancy aging and static-hit persistence metadata;
it must not overwrite this snapshot.
