# SCAN And DDDMR Tuning Report

## Scope

This report records the navigation work starting with SCAN Mode 3 wall
crossing, through stair traversal, Go2W collision footprint alignment, and
dynamic-obstacle residual handling.

Test map:

```text
/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd
```

Robot horizontal envelope used by both planners:

```text
Go2W body: about 0.70 m long x 0.43 m wide x 0.50 m high
two circles: radius 0.22 m, front/rear offset 0.13 m
result: 0.70 m long x 0.44 m wide
```

## Runtime Chain

```text
Super-LIO current scan and local odometry in world
    -> Open3D localization gives world-to-map alignment
    -> DDDMR plans on offline PCD in map
    -> map_path_to_world_bridge converts /initial_path to /initial_path_world
    -> SCAN Mode 3 receives local world odometry, current local cloud,
       and /initial_path_world
    -> SCAN publishes /cmd_vel
```

SCAN stays in Super-LIO's continuous `world` frame. DDDMR is map-frame global
planning; its path must be converted to `world` before SCAN consumes it.

## Change Summary: Before And After

| Problem | Original source behavior | Current behavior | Validation state |
| --- | --- | --- | --- |
| Mode 3 cuts wall corners | Uses one distant local target; intermediate DDDMR turn points are not retained as a route constraint | Uses the ordered DDDMR segment within a `3.5 m` horizon as B-spline initialization and route-fitness reference | PCD corner replay verified; continue real-wall tests |
| Stair collision | Collision query near body/LIO origin can treat a next tread or overhead stair geometry as blocking | Query is offset upward and uses a narrow lower vertical band | Real robot stair ascent/descent verified |
| Rear legs close to wall | SCAN's front/rear footprint did not match Go2W; DDDMR used a single circular check and could invalidate a route during post-processing | SCAN and DDDMR use the same two-circle envelope; final DDDMR route is revalidated | Offline route validation verified; real clearance remains to be tuned conservatively |
| Dynamic-obstacle residual | Only observed hit/miss rays change occupancy; endpoints outside the update box could be discarded before contributing a free ray | Out-of-range returns are clipped to the local box as free-ray endpoints; free-ray evidence is stronger | Source behavior fixed; controlled disappear-and-clear test remains pending |
| Safe curve, unsafe execution at a corner | Controller and FSM progress local trajectories using elapsed wall-clock time even when actual robot motion lags | Original time-based controller is retained, but every raw controller `cmd_vel` is now checked against SCAN's current inflated GridMap before being forwarded | Near-wall command-veto simulation verified; real robot retest required |

## Problem 1: SCAN Cut Through Walls

### Symptom

DDDMR's global path turned around walls correctly, but SCAN chose one distant
look-ahead point and generated a direct minimum-snap chord. The local chord
could cross a wall instead of following DDDMR's intermediate turn points.

### Root Cause

Mode 3 treated the global route mainly as a target selector. It did not retain
the local section of the global route as a reference curve during initialization
and rebound optimization.

In the original implementation, this allowed the minimum-snap initialization
to connect the current point directly to a distant look-ahead target. At an
outside corner, that straight chord can cross a wall even if DDDMR's complete
global path already turns around it.

### Changes

- Add `fsm.follow_reference_path: true`.
- Reduce both planner horizons from `7.5 m` to `3.5 m`.
- Resample the DDDMR route segment inside that horizon and use it to initialize
  SCAN's local B-spline.
- Retain the DDDMR segment's intermediate Z values instead of linearly
  flattening Z from start to endpoint.
- Enable route-fitness cost during rebound optimization.

Relevant files:

```text
src/scan_planner_ros2/src/planner/plan_manage/src/scan_replan_fsm.cpp
src/scan_planner_ros2/src/planner/plan_manage/src/planner_manager.cpp
src/scan_planner_ros2/src/planner/bspline_opt/src/bspline_optimizer.cpp
src/scan_planner_ros2/src/planner/plan_manage/config/planner.yaml
```

### Verification

On the stair4 PCD route from `[-0.751, 2.456, 3.650]` to
`[-2.5007, 20.7320, 3.6449]`:

| Test | Result |
| --- | --- |
| Old single-endpoint approach | Maximum 3D deviation from global route: `2.87 m` at `3.5 m` horizon |
| Reference-path approach | `67` consecutive local trajectories |
| New maximum XY deviation | `0.392 m` at the main turn/rebound |
| Typical straight section deviation | about `0.05-0.085 m` |

This removed the observed global-path shortcut through walls. It does not make
SCAN ignore genuine local obstacles; SCAN can still depart from the reference
route when a current local obstacle requires avoidance.

## Problem 2: Stair Z Collision And Mid-Stair Failure

### Symptom

SCAN sometimes classified the next stair tread or the space above the robot as
an obstacle. It then reported that the start or first control points were in an
obstacle and stopped part way up a stair.

### Approach

The collision query must be evaluated at a body-height reference, while the
command controller remains planar. The vertical occupancy band must exclude
the next tread above the body without removing real side obstacles.

Before this adjustment, the source collision query used the body/LIO-origin
height and its vertical inflation band directly. On the stair map, points from
the next tread or overhead geometry could then fall inside the checked band,
causing a safe stair direction to be reported as occupied.

### Current Parameters

```yaml
grid_map.collision_query_z_offset: 0.30
grid_map.obstacles_inflation_z_up: 0.0
grid_map.obstacles_inflation_z_down: 0.15
```

Interpretation:

- `collision_query_z_offset` shifts only collision evaluation upward from the
  LIO/body origin. It does not command a Z motion.
- `z_up=0.0` avoids inflating obstacle voxels below the query point upward into
  the robot's stair path.
- `z_down=0.15` keeps a narrow lower collision band for local hazards.

### Result

Real stair tests reported normal up/down traversal after this adjustment. These
values should remain unchanged while evaluating horizontal wall clearance or
dynamic obstacle behavior.

## Problem 3: Go2W Rear Legs Close To Walls

### Symptom

A center-line route could look clear in RViz while the rear of Go2W was close
to a wall or a person. The original SCAN two-circle offset was too short for
the 0.70 m Go2W body.

Before the DDDMR changes, its graph and route validation used one circular
horizontal footprint. Its optional smoothing, shortcutting, resampling, and
ground projection stages were also not checked again against the final Go2W
envelope. Therefore a graph route could be safe while its published route was
not.

### SCAN Footprint Change

```yaml
grid_map.double_cylinder_radius: 0.22
grid_map.double_cylinder_offset: 0.13
optimization.dist0: 0.35
optimization.lambda_collision: 3.0
optimization.lambda_fitness: 1.0
```

The circle centers are at `Q +/- offset` along heading. Therefore:

```text
width  = 2 * radius = 0.44 m
length = 2 * (radius + offset) = 0.70 m
```

`radius` defines hard lateral collision; `offset` extends the front/rear
envelope. `dist0` is the optimizer's preferred clearance, not the hard body
width.

### DDDMR Alignment

DDDMR originally used only one circular `robot_radius` check. New optional
parameters align it with SCAN:

```text
footprint_radius:=0.22
footprint_offset:=0.13
footprint_edge_samples:=3
```

DDDMR validates each graph edge against the same oriented two-circle model.
The check accounts for obstacles between `max_step` and `robot_height` along
the local surface normal.

### Important Follow-Up Found During Validation

The original DDDMR graph edges were safe, but its later shortcut, resampling,
smoothing, and ground projection could make the rear circle collide. On the
stair4 route, the smoothed 240-point output had two unsafe segments near
`[5.86, 11.72, 3.78]`.

The final output is now validated after post-processing. If it is unsafe,
DDDMR publishes the original 69-point safe graph route instead of a sparse
4-point shortcut. The verified route has zero invalid two-circle segments.

This is important for SCAN Mode 3: a 4-point path caused SCAN to select a
distant corner as a local endpoint. The 69-point output provides enough
intermediate route points for the `3.5 m` local horizon.

## Problem 4: Dynamic Obstacles Leave Occupancy Residuals

### Source Behavior

SCAN's local grid has no occupancy TTL. A cell is cleared only when:

1. a later lidar ray passes through it and supplies a `miss` update;
2. it leaves the local update range; or
3. it leaves the 10 m sliding-map window.

If a person leaves but the old location receives no later free ray because of
occlusion, blind areas, or sparse returns, its occupied state can remain.

The unmodified cloud projection path also dropped a return outside
`local_update_range` with `continue`. That direction then supplied neither an
occupied endpoint nor a clipped free-space ray, so it could not clear old
nearby occupancy as the sensor viewpoint changed. The original per-voxel
hit/miss counters used short integer types, which can overflow under dense
LiDAR free-ray updates.

### Code Changes

- Preserve whether a projected endpoint is a measured hit or a free-space-only
  clipped endpoint.
- For a valid return outside the local update box, clip the ray endpoint just
  inside the box and raycast it as free space rather than dropping the ray.
- Ignore zero-length ray setup failures.
- Use `int` rather than `short`/`char` for dense raycast counters.

### Super-LIO Rear Coverage

The endpoint Super-LIO launch now derives its rear 2 m / 90 degree filter from
`save_map`:

| Mode | Rear 2 m / 90 degree filter |
| --- | --- |
| Navigation: `save_map:=false` | disabled |
| Mapping: `save_map:=true` | enabled |

The original `lio.sensor.blind: 0.2` near-field filter remains in both modes.
This improves SCAN's ability to observe and clear rear local cells while not
saving navigation-time PCDs.

### First Fusion Adjustment

Only the free-ray probability changed:

```yaml
grid_map.p_hit: 0.85       # unchanged
grid_map.p_miss: 0.22      # changed from 0.30
grid_map.p_occ: 0.80       # unchanged
grid_map.p_min: 0.12
grid_map.p_max: 0.98
```

In log odds:

```text
p_hit  = +1.735
old miss = -0.847
new miss = -1.266
occupied threshold = +1.386
```

A saturated dynamic cell needs two explicit free-ray frames to clear instead
of three. At 5 Hz this is approximately `0.4 s` instead of `0.6 s`. New
obstacle confirmation remains unchanged. This is intentionally not a time
decay mechanism: cells with no free-ray observation are still retained.

### Not Yet Implemented

- Per-voxel TTL or age-based decay.
- A recovery/retreat behavior when SCAN starts inside its occupied grid.
- A dynamic-obstacle test in which a person-shaped target disappears from the
  exact same sensor ray path.

These should be a separate change. A global TTL would risk deleting walls or
stair edges after intermittent returns; any future TTL should apply only to
near-field local cells that are no longer repeatedly hit.

## Problem 5: Safe Local Trajectory, But Unsafe Time-Based Execution At Corners

### Symptom

During a real corner test, RViz showed a local B-spline that followed the
DDDMR route around the wall. The robot could nevertheless command forward or
sideways motion toward the far side of the wall. This is distinct from the
earlier local-planner shortcut problem: the displayed planned curve was
already on the safe side of the wall, but the executed motion departed from
it.

This behavior is most likely when the robot is delayed by turning, a dynamic
obstacle, stair posture adjustment, wheel slip, or a command/odometry delay.
It does not necessarily appear on a straight route. On a straight route, a
future trajectory target is normally still in the same forward direction, so
time lead mostly makes the robot chase too aggressively rather than cut across
a wall.

### Root Cause

The previous closed-loop controller advanced the trajectory cursor only by
wall-clock time after receiving a B-spline. It used a control law equivalent
to:

```text
command velocity = planned velocity at t
                 + Kp * (planned position at t - actual odometry position)
```

The B-spline contains a theoretical time schedule as well as a geometric
path. If the robot falls behind that schedule at a corner, the selected
`planned position at t` can already be around the corner or behind a wall.
The full XY position-error term then points directly from the robot's actual
location toward that later point. The controller did not independently query
the grid map before publishing this correction velocity.

The planner still replans from actual odometry and can therefore publish a
safe curve. This explains the apparent contradiction of a safe RViz path with
an unsafe executed command.

The original FSM had a second, independent wall-clock cursor. It used
`now - local_data.start_time` to decide whether enough of the local trajectory
had elapsed to roll-replan, which portion to collision-check, and which old
trajectory velocity to use when seeding the next trajectory. Therefore
changing only the controller would still leave an inconsistent, time-ahead
replanning path after a delay or a turn-in-place event.

### Odom-Projected Controller Trial: Reverted

The closed-loop controller was temporarily changed to use actual XY odometry
to determine trajectory progress:

1. Search a bounded forward section of the current B-spline.
2. Project the current odometry position onto its nearest sampled segment.
3. Keep progress monotonic within the currently received trajectory.
4. Select the command target a short look-ahead time after that projected
   progress, rather than after elapsed wall-clock time.
5. Apply position correction only along the trajectory tangent. A transverse
   error can no longer directly create a chord velocity through a corner.

The experimental controller parameters were:

```yaml
closed_loop_controller.progress_search_ahead: 3.0  # trajectory-time search bound, s
closed_loop_controller.progress_sample_dt: 0.05    # B-spline sampling interval, s
```

### Experimental Progress Parameter Reference

| Parameter | Node | Unit | Current value | Effect | Trade-off |
| --- | --- | --- | --- | --- | --- |
| `progress_search_ahead` | `closed_loop_controller` | s of B-spline parameter time | `3.0` | From the current projected progress, only search this far forward for the closest odom projection. | Removed from active YAML after real-test regression. |
| `progress_sample_dt` | `closed_loop_controller` | s | `0.05` | Segment sampling resolution for the controller projection; at `0.20 m/s`, roughly `1 cm` between nominal samples. | Removed from active YAML after real-test regression. |

`time_forward: 0.8` is not new. It remains the controller's target look-ahead
after current projected progress. At `0.20 m/s`, it corresponds to about
`0.16 m` of nominal forward look-ahead. It is deliberately separate from the
`progress_search_*` parameters: one chooses the target ahead of the robot;
the other finds where the robot currently is on the trajectory.

Relevant file:

```text
src/scan_planner_ros2/src/planner/plan_manage/src/closed_loop_controller.cpp
```

### Trial Result And Remaining Safety Layer

An isolated corner replay of the controller change using the stair4 crop
reduced the initial local trajectory/reference-route maximum XY deviation from
approximately `0.97-1.04 m` in the previous controller run to `0.302 m`; the
observed worst sample in that run was `0.336 m`. The `scan_planner` unit tests
passed `2/2` after the controller build.

The following real-robot results invalidated this controller change as an
active solution: unstable stair ascent/descent, early stop in the middle of a
stair flight, S-shaped motion on an unobstructed straight, and repeated
rotation. The FSM-only projection trial also caused lateral oscillation on the
stairs. Both projection implementations were removed; the active controller
is restored to the version used with the Mode 3 reference-path wall fix.

### FSM Progress Trial: Reverted

The FSM was briefly changed to use the same odom-projected progress for rolling
replanning, collision-check start time, and previous-trajectory derivatives.
The change compiled and unit tests passed, but the first real test caused the
robot to oscillate laterally while climbing in the middle of a stair flight.
This is unsafe and worse than the previous direct stair traversal.

The FSM change and its two experimental parameters
`fsm.progress_search_ahead` / `fsm.progress_sample_dt` have therefore been
removed from the active source and YAML. The active version retains only the
original time-based controller together with the Mode 3 reference-path wall
fix. A future FSM design must preserve the established stair replan cadence
and explicitly test stair traversal before being enabled.

The isolated replay was therefore insufficient: a geometric trajectory metric
did not exercise real controller/robot coupling across repeated replans and
stairs. Any future progress-based controller must first be validated against
recorded real odometry and command traces, including a stair traversal.

### Controlled Odom-Freeze Reproduction: 2026-09-30

The controller fault was then reproduced without DDDMR. The same controlled
L-wall map used a safe ordered reference route:

```text
[3.5, 4.8] -> [3.5, 6.5] -> [4.5, 6.5] -> [8.0, 8.0]
```

The direct chord from the actual start to the final point crosses the wall,
while the supplied route goes around it. The simulator froze body odometry for
45 seconds after initial trajectory generation and temporarily raised the FSM
replan threshold to prevent a new plan from resetting the controller clock.
This is intentionally a controlled reproduction of wheel slip, posture delay,
or a temporarily blocked robot; it is not an active robot parameter.

Result:

```text
safe local B-spline minimum wall clearance: 0.583 m
actual frozen odom:                         (3.50, 4.80)
controller wall-risk command at t=45.03 s: world velocity (0.037, 0.036) m/s
robot collision radius:                     0.220 m
```

The velocity ray from the still-wall-side odometry intersects the L wall. No
planner error or local-trajectory collision occurred. This directly confirms
the real-robot symptom: the active time-driven controller can publish an
unsafe `cmd_vel` even while the displayed B-spline itself is safe.

The reproduction infrastructure is deliberately simulation-only:

```text
go2_kinematic_sim.freeze_after_sec
go2_kinematic_sim.freeze_duration_sec
run.launch.py fsm_replan_threshold
tools/run_scan_l_wall_reference_test.sh
tools/scan_l_wall_probe.py --detect-cmd-wall
```

All new parameters default to inactive values. They do not change real-world
SCAN launch behavior.

### Implemented CmdVel GridMap Safety Layer: 2026-09-30

The odometry-projection controller experiment was intentionally not retained:
it degraded real stair traversal and straight-line motion. Instead, the active
time-based controller remains unchanged and its output is now guarded at the
last safe boundary before the robot bridge.

```text
closed_loop_controller -> /cmd_vel_raw
scan_planner_node command safety -> /cmd_vel -> go2_twist_bridge
```

For each raw twist, SCAN propagates the **actual current odometry** for
`execution_safety.horizon` seconds in `execution_safety.dt` steps. Every
predicted pose is queried against the existing inflated GridMap using exactly
the same Go2W two-circle footprint, collision Z offset, and out-of-map rule as
the local planner. A predicted occupied pose causes:

1. `/cmd_vel` to receive a zero Twist; the bridge never receives the unsafe
   raw command.
2. `/planning/cmd_collision` to publish `true` for observation in RViz or
   terminal tools.
3. A collision lasting `execution_safety.replan_delay` to request the normal
   SCAN FSM replan, rate-limited by `execution_safety.replan_cooldown`.

Zero commands which may be interleaved by the controller do not reset a
collision episode immediately. The episode releases only after `0.25 s` (or
two GridMap sampling intervals) without a predicted collision; this prevents
zero commands from suppressing the replan debounce.

Active parameters:

```yaml
execution_safety.enabled: true
execution_safety.horizon: 1.0
execution_safety.dt: 0.05
execution_safety.replan_delay: 0.15
execution_safety.replan_cooldown: 0.50
```

The `1.0 s` horizon is deliberately a final execution guard, not a substitute
for the 3.5 m route-constrained local planner. At the current `0.20 m/s`
speed limit it looks about `0.20 m` ahead, stopping before a footprint contact;
the normal FSM replan then selects a new local trajectory.

Controlled near-wall validation used the L-wall map with the body at
`[4.55, 4.00]` facing the wall at `x=5.0`, then injected a raw
`linear.x=0.20 m/s` command. Recorded result:

```text
collision_true=36
raw_max_vx=0.200 m/s
safe_max_vx=0.000 m/s
```

This verifies the command veto and topic boundary. The next real-robot test
must verify that a sustained veto during `EXEC_TRAJ` produces both a zero
`/cmd_vel` and a new local B-spline rather than trapping the robot in place.

Implementation note: all command-safety timestamps are initialized from the
node clock. This avoids mixing ROS time and system time when hardware launches
with `use_sim_time:=false`.

### Mode 3 Odom-Ordered Route Progress: 2026-09-30

Real-robot recording exposed a separate handoff defect: DDDMR starts its
graph route at the nearest traversable voxel rather than the exact current
body pose. In one recorded run the map-frame offset was `0.24 m`. SCAN used
the current odom for B-spline start position but still inserted this already
behind route-start node into every local reference segment. Replans then
alternated their initial heading, and the closed-loop controller repeatedly
entered its turn-in-place branch.

Mode 3 now advances `reference_progress_index_` from current odometry before
constructing its local reference segment. It consumes only consecutive route
points within `fsm.reference_reach_tolerance` and caps a single update at
`fsm.reference_progress_max_advance=0.60 m`. This is a forward-only ordered
operation, not a global nearest-point search, so it cannot jump across a
nearby U-wall branch. The local segment starts at the actual odom and the
first unreached DDDMR point. The current progress anchor itself is not added
as a local waypoint: it is a graph handoff marker and can sit slightly behind
the exact odom position. This restores the reference construction used by the
previously verified 3.5 m stair traversal while retaining the newer ordered
anti-wall-jump progress rule.

## Current Parameters

```yaml
fsm.thresh_replan: 0.5
fsm.planning_horizon: 3.5
fsm.follow_reference_path: true
grid_map.resolution: 0.05
grid_map.sliding_map_size_x: 10.0
grid_map.sliding_map_size_y: 10.0
grid_map.sliding_map_size_z: 5.0
grid_map.local_update_range_x: 5.0
grid_map.local_update_range_y: 5.0
grid_map.local_update_range_z: 2.5
grid_map.double_cylinder_radius: 0.22
grid_map.double_cylinder_offset: 0.13
grid_map.collision_query_z_offset: 0.30
grid_map.obstacles_inflation_z_up: 0.0
grid_map.obstacles_inflation_z_down: 0.15
grid_map.p_hit: 0.85
grid_map.p_miss: 0.22
grid_map.p_occ: 0.80
grid_map.max_ray_length: 5.0
optimization.lambda_collision: 3.0
optimization.lambda_fitness: 1.0
optimization.dist0: 0.35
closed_loop_controller.time_forward: 0.8
```

DDDMR launch parameters used for the stair4 map:

```text
use_source_normals:=true voxel_size:=0.10 normal_k:=12
normal_ground_min_z:=0.10 max_slope_deg:=38.0 max_roughness:=0.08
robot_radius:=0.22 robot_height:=0.50 footprint_radius:=0.22
footprint_offset:=0.13 footprint_edge_samples:=3 max_step:=0.30
inflation_radius:=0.05 connection_radius:=1.50 transition_radius:=1.50
transition_neighbors:=16 max_neighbors:=32
```

## Commands

Endpoint Super-LIO navigation mode:

```bash
ros2 launch super_lio go2w_mid360.py rviz:=false save_map:=false
```

Local DDDMR:

```bash
ros2 launch dddmr_python_ros dddmr_pcd_planner.launch.py pcd_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd frame_id:=map launch_rviz:=true use_odom_start:=true odom_topic:=/baselink2map_kalman use_source_normals:=true voxel_size:=0.10 normal_k:=12 normal_ground_min_z:=0.10 max_slope_deg:=38.0 max_roughness:=0.08 robot_radius:=0.22 robot_height:=0.50 footprint_radius:=0.22 footprint_offset:=0.13 footprint_edge_samples:=3 max_step:=0.30 inflation_radius:=0.05 connection_radius:=1.50 transition_radius:=1.50 transition_neighbors:=16 max_neighbors:=32
```

Local SCAN Mode 3:

```bash
ros2 launch scan_planner run.launch.py is_real_world:=true lio_backend:=super_global navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_sim_time:=false planning_horizon:=3.5 follow_reference_path:=true cmd_vel_topic:=/cmd_vel
```

PCD simulation:

```bash
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop use_pcd_map:=true pcd_map_file:=/home/hjj/Go2w_3D_Nav/src/Super-LIO/PCD/map_2026-09-24_16-35-38/8f_9f_stair4_dense_oriented_normals.pcd init_x:=-0.751 init_y:=2.456 init_z:=4.121 map_size_x:=40.0 map_size_y:=40.0 map_size_z:=10.0 planning_horizon:=3.5 follow_reference_path:=true sim_sensing_horizon:=5.0 sim_sensing_rate:=10.0 sim_vertical_fov:=59.0 sim_min_raylength:=0.1 sim_lidar_pitch:=13.0 launch_rviz:=true use_sim_time:=false
```

Before publishing a simulated goal, wait for the DDDMR log:

```text
DDDMR map ready
```

The 454122-point map can take more than 30 seconds to cold-load on this host.

## Verification And Limits

### Controlled L-Wall Simulation: 2026-09-30

A synthetic, watertight L wall was generated independently of the stair PCD:

```text
vertical wall: x=5.0, y=0.0..6.0, z=0.0..1.6 m
horizontal wall: y=6.0, x=5.0..10.0, z=0.0..1.6 m
robot collision radius: 0.22 m
```

The SCAN-only test deliberately started at `[3.5, 4.8, 0.3]` and supplied the
ordered reference route `[3.5, 4.8] -> [3.5, 6.5] -> [4.5, 6.5] -> [8.0, 8.0]`.
The direct start-to-goal chord crosses the vertical wall, so this is not a
trivially safe direct-route test. Across five SCAN replans, the minimum local
trajectory-to-wall XY clearance was `0.580 m`; no `SEARCH_ERR` or planning
failure was logged. This validates that the active Mode 3 local planning does
not cut this controlled corner when it receives a safe ordered global route.

The simulated robot advanced only into the first turn section during the
65-second run, so this result is a collision/route-following validation, not
a complete arrival-time validation.

The matching end-to-end DDDMR-to-SCAN L-wall run did not pass: DDDMR published
a 26-point route whose segment crossed the vertical wall (global minimum
clearance `0.200 m`, less than the `0.220 m` robot radius). SCAN then later
generated local trajectories with zero wall clearance. This is a DDDMR wall
occupancy or graph-edge validation defect; it cannot be used to judge the
SCAN reference-path fix. DDDMR must be corrected and the combined test rerun
before claiming end-to-end wall safety.

| Item | Status |
| --- | --- |
| SCAN wall shortcut fix with a safe reference route | controlled L-wall simulation verified |
| Controller safety under odom lag | near-wall GridMap cmd_vel veto verified; real-robot replan behavior pending |
| DDDMR-to-SCAN end-to-end L-wall route | failed: DDDMR path crosses wall; DDDMR fix pending |
| Stair vertical collision tuning | real robot traversal verified |
| DDDMR two-circle graph and output route | offline verified on stair4 map |
| SCAN local trajectory with new 69-point DDDMR path | pending complete simulation after DDDMR warm-up |
| Dynamic person disappearance and occupancy clearing | pending controlled simulation and real test |
| Recovery after entering an occupied cell | not implemented |

### DDDMR-to-SCAN L-Wall Freeze Simulation: 2026-10-05

The end-to-end test used the same synthetic L-wall PCD, DDDMR `/initial_path`,
SCAN `navi_mode=3`, simulated LiDAR/GridMap, closed-loop controller, command
safety filter, and the Go2 kinematic simulator. The near-corner case started
at `(3.5, 4.8, 0.3)` and targeted `(8.0, 8.0, 0.3)`. The simulated body was
held still for 45 seconds after startup to model delayed motion feedback.

The previous DDDMR edge checker queried only the nearest obstacle in XY. At a
wall, this could return its floor-level sample, so higher samples in the body
band were missed. Its fixed three-point edge sampling could also skip narrow
geometry. `graph.py` now samples segments at a spacing no larger than half the
voxel/radius scale and checks all nearby obstacle heights when the nearest
sample itself is below the body band. The old file is preserved in
`tools/scan_tuning_backups/2026-10-05_dddmr_wall_clearance_before/graph.py`.

The corrected DDDMR run used:

```text
robot_radius=0.35, footprint_radius=0.22, footprint_offset=0.13, footprint_edge_samples=12, connection_radius=0.60, transition_radius=0.60, inflation_radius=0.15
```

DDDMR published 57 route poses, length `5.58 m`, with minimum centerline wall
clearance `0.240 m`. SCAN's initial local A* reported that 8/21 direct samples
were occupied, then rebound optimization succeeded and emitted a B-spline with
minimum wall clearance `0.337 m`. This shows the local optimizer recovered
from its blocked straight initialization.

Over the simulated run, odometry's minimum center clearance was `0.254 m`
against the `0.220 m` radius. The command safety layer published zero collision
vetoes. An independent integration of the final filtered Twist commands also
did not enter the wall. The robot approached the corner closely, but did not
penetrate it in this kinematic test.

The diagnostic's two-metre constant-velocity ray flagged some commands near
the corner, while the actual simulated odometry and command-integrated path
remained clear. That ray ignores the controller's changing velocity and the
curved B-spline, so it is a warning metric rather than proof of a collision.
This L-wall run therefore does not reproduce the reported real-robot
cmd_vel-through-wall event. It does show that DDDMR graph-edge validation was
one independent source of unsafe paths; that source is now fixed in this
working tree. Real body dynamics, tracking lag, and the exact field U-corner
remain unverified by this kinematic model.

Reproduce in one command:

```text
START_X=3.5 START_Y=4.8 FREEZE_AFTER=2.0 FREEZE_DURATION=45.0 bash tools/run_l_wall_mode3_test.sh
```

## Backups

```text
tools/scan_tuning_backups/2026-09-29_pre_wall_test/
tools/scan_tuning_backups/2026-09-29_route_following/
tools/scan_tuning_backups/2026-09-29_go2w_footprint/
tools/scan_tuning_backups/2026-09-29_dynamic_footprint/
tools/scan_tuning_backups/2026-09-29_fast_free_space/
tools/scan_tuning_backups/2026-09-30_pre_restore_upstream_mode3/
tools/scan_tuning_backups/2026-09-30_cmd_vel_grid_safety_before/
tools/scan_tuning_backups/2026-10-05_dddmr_wall_clearance_before/
tools/scan_tuning_backups/2026-10-05_fullchain_lwall_before/
```

Each directory contains a before/after archive or result record for its scope.
The `2026-09-30_cmd_vel_grid_safety_before/files.tar.gz` archive is the exact
pre-safety-layer state of the four affected SCAN files:

```text
plan_manage/include/plan_manage/scan_replan_fsm.h
plan_manage/src/scan_replan_fsm.cpp
plan_manage/config/planner.yaml
plan_manage/launch/run.launch.py
```

To disable the new guard temporarily without restoring files, launch with
`execution_safety_enabled:=false`. This is for A/B diagnosis only; it
reconnects `/cmd_vel_raw` to the normal forwarding path and removes the final
execution collision veto.

## SCAN Version 0 vs Version 1: 2026-10-07

Compared the saved post-rollback V0 patch with the pre-comparison working-tree
snapshot used as V1:

- `tools/scan_tuning_backups/2026-10-06_pre_comparison/version0.patch`
- `tools/scan_tuning_backups/2026-10-06_pre_comparison/working_tree.patch`

The 0.5 m DDDMR waypoint thinning is present in both versions. Both retain the
exact final waypoint, so this change alone does not explain why V0 behaved
better. V1 also changed `fsm.thresh_replan` from 1.0 m to 0.5 m; that is a
separate setting from waypoint spacing and doubles the nominal replanning
frequency at the same speed.

The most behaviorally significant V1-only changes were:

| Area | V0 | V1 snapshot | Potential effect |
| --- | --- | --- | --- |
| Mode 3 tracking | Original global-trajectory lookahead/local-target logic | Explicit local DDDMR reference segment, ordered progress, and route-fitness cost | More strongly constrains local avoidance to the global route; can increase optimizer/replan pressure near corners or dynamic obstacles |
| Command output | No grid-map command veto | Forward-simulates cmd_vel for 1.0 s, publishes zero on predicted collision, then requests replanning | Stale/over-conservative occupancy can directly stop motion and cause repeated recovery |
| Replanning | `fsm.thresh_replan=1.0` | `0.5` | More frequent trajectory replacement and possible command/trajectory churn |
| Controller yaw lookahead | `time_forward=0.8 s` | `0.0 s` | Changes desired yaw from a forward point to approximately the instantaneous trajectory tangent |
| Clearance objective | `dist0=0.2`, collision weight 1.0 | `dist0=0.45`, collision weight 3.0 | V1 asks for more clearance and may become infeasible in narrow doors/platforms |
| Timing limits | Planner/controller limits around 0.75 m/s in the saved V0 patch | Around 0.20 m/s in V1 snapshot | Changes trajectory time scale and tracking response; confirm effective runtime parameters before attributing behavior |

V1 also changed occupancy raycasting: it replaced the `continue` for valid
returns outside the local update box with a ray clipped to that box and marked
as free-space evidence; it added free rays for invalid/out-of-range depth
pixels, widened per-voxel counters, and changed the ray sequence counter. These
are separate map-clearing changes and should be evaluated independently from
the V1 route-following/controller changes.

The snapshots bundle several changes, so they establish candidates, not a
single proven cause. Based on code paths and the reported V0 improvement, the
leading candidates are V1's command veto, stricter route-following/fitness, and
more frequent replanning. The 0.5 m waypoint thinning is not a V0/V1 delta.
Because V1 used `dist0=0.45` while V0 used `0.2`, increasing V0 directly to
0.45 would also reintroduce one of the more conservative V1 settings; clearance
should be changed incrementally and independently.

## SCAN Occupancy Clearing Audit: 2026-10-07

This audit describes the current V0 source and makes no code changes.

The local map uses per-voxel log odds. A measured endpoint contributes a hit;
voxels traversed between the sensor origin and an endpoint contribute misses.
The code applies one hit-or-miss log-odds update to each cached voxel per scan.
With current V0 parameters (`p_hit=0.85`, `p_miss=0.30`, occupied threshold
`p_occ=0.80`, upper clamp probability `p_max=0.98`), a saturated occupied cell
needs about three miss-only scan updates to fall below the occupied threshold.
V1's `p_miss=0.22` would make each observed miss stronger, clearing a saturated
cell in about two miss-only updates. Neither value helps if no free ray reaches
the cell. There is no age/timeout decay for occupied cells that receive no
further observations. Within one scan, ties between hit and miss counts are
classified as a hit by the current `>=` comparison.

Two current-source behaviors can prevent that evidence from arriving:

1. In `GridMap::cloudCallback`, a point outside `local_update_range` is dropped
   when its range is still within `max_ray_length` (`grid_map.cpp:918-924`).
   This discards the entire ray, even if the ray crosses previously occupied
   voxels inside the update region. The V1 snapshot changed this to clip the
   ray to the local boundary and use it as free-space evidence.
2. `raycastProcess()` returns when there are no projected returns, and the
   cloud callback does not request an occupancy update when no accepted points
   remain. PointCloud2 has no explicit no-return rays, so directions with no
   return cannot clear old cells in the current implementation.

The rolling map clears cells only when they leave the sliding-map bounds. It
does not expire obstacles merely because they have become occluded or are no
longer in the current scan. Consequently, an old dynamic obstacle can remain
occupied indefinitely if later rays do not cross its voxel. Blindness alone
cannot prove the space is free; any temporal decay must be designed so it does
not erase persistent walls or static objects without fresh free-space evidence.

The current V0 raycast also stores `raycast_num_` as `char`, and per-voxel
traversal flags are chars. The sequence can wrap and reuse old flag values;
this is a secondary correctness risk for long-running clearing, distinct from
the dropped-ray issue. The pre-comparison V1 snapshot widened the sequence
counter, but left the per-voxel flags as chars.

No obstacle-clearing code was changed during this audit. The highest-confidence
next fix to test is restoring clipped free rays for out-of-local-range returns,
while keeping V0 trajectory/controller behavior unchanged. A timeout/decay
policy should be a separate experiment, because it trades dynamic-obstacle
recovery against accidentally forgetting static obstacles.

### Blind / Out-of-View Persistence: Design Note

When an obstacle leaves the LiDAR field of view, enters the minimum-range blind
zone, or becomes occluded, the current PointCloud2 contains no negative
measurement for the old voxel. Raycasting cannot distinguish “obstacle moved”
from “sensor cannot currently observe it.” A global short TTL on every occupied
voxel would solve stale people but can also erase a static wall while the robot
turns away from it.

There is an additional safety consideration: unobserved cells start below the
occupancy clamp and have zero inflated occupancy, so current collision queries
treat unknown as traversable. Simply expiring stale occupied cells back to
unknown therefore permits planning through them unless unknown-space handling
is changed too.

Candidate approaches, ordered from lower to higher scope:

1. Restore clipped free rays first. This only clears cells with actual
   free-space evidence and is the safest fix for dropped rays.
2. Add a transient-obstacle layer with per-voxel last-hit time and a bounded
   TTL. Keep persistent/static occupancy in a separate layer; do not age both
   layers identically. A minimal persistence classifier can use repeated hits
   over a time window, but a moving/person tracker is more reliable because a
   stationary person may otherwise be misclassified as static.
3. Decide how expired/unknown cells affect collision checks. Options are to
   keep unknown blocked near the robot and inside the local planning horizon,
   or to use a trusted static map as the persistent layer while allowing
   transient cells to expire. Current SCAN does neither automatically.

Do not clear an occupied voxel immediately when it crosses into a blind
sector. First test an explicit transient TTL in simulation with both a moving
obstacle and a static wall/corner, including turns that temporarily hide the
wall. Select the TTL from measured sensor/update latency and verify that the
robot never receives a command through an unobserved wall. A single-layer
global timeout is not considered safe without those tests.

### Clipped Free-Ray Experiment: 2026-10-07

Implemented the first, evidence-based clearing change while preserving the V0
planner/controller parameters:

- `cloudCallback()` no longer discards a valid return just because its endpoint
  lies outside `local_update_range`. It clips the endpoint to the first local
  range boundary (or `max_ray_length`, whichever comes first) and labels that
  clipped endpoint as a miss, not an obstacle hit.
- `raycastProcess()` carries the endpoint hit/miss label through map-boundary
  and maximum-range clipping. Voxels traversed from the sensor to the clipped
  endpoint receive the existing free-space updates.
- Depth-image projection keeps its prior endpoint-as-hit behavior.

The `plan_env` library and the dependent `scan_planner` package compiled
successfully on 2026-10-07; `git diff --check` passed. This is a build check,
not yet a ROS-bag or robot behavior test. Expected improvement: a stale voxel
inside the local update region can now be cleared when a current measured ray
continues beyond it to a farther return, even if that return itself is outside
the local update box. It does not clear voxels in unobserved directions,
behind nearer returns, or in the LiDAR blind zone; those still need a separate
transient-obstacle/unknown-space policy.

With the current planner settings (`max_ray_length=5.0 m`, horizontal local
update ranges `x/y=5.0 m`, `z=2.5 m`), most ordinary horizontal returns within
the ray limit were already inside the local box. Thus this patch may have
little visible effect on typical same-height moving obstacles; its direct
coverage is rays that actually cross the local-box boundary (for example,
vertically displaced returns). It should not be presented as a fix for
out-of-view or blind-zone persistence.

Pre-change source backup: `tools/scan_tuning_backups/2026-10-07_before_clipped_free_rays/plan_env_before.tar.gz`.

### Upstream Mode 3 Collision Fix Port Audit: 2026-10-07

The original SCAN-Planner repository contains upstream commit `b654557`
(`fix: mode3 collision`). The ROS 2 community branch predates it. Community
PR [#11](https://github.com/wuyi2121/SCAN-Planner/pull/11) ports that change
to the `ros2-community` branch; GitHub currently shows the PR as open. The PR
reports a successful Humble simulator run, but no hardware validation.

Neither saved V0 nor V1 contains the complete port:

- Both versions already have the 0.5 m reference-waypoint thinning, retain the
  final waypoint, and reject a reference path received before odometry. These
  are overlapping pieces, not proof that the complete fix was included.
- The upstream fix seeds the Mode 3 global trajectory on the supplied route,
  rather than starting its global trajectory from the current odometry state;
  it also requires at least two reference points. V0/V1 still call the
  waypoint planner from `odom_pos_` and `odom_vel_`.
- Upstream `planFromCurrentTraj()` preserves the supplied reference-path global
  trajectory during Mode 3 replans and only re-optimizes the local rebound
  trajectory, with a three-stage fallback. V0 and V1 still regenerate a global
  trajectory from current odometry to `end_pt_`.
- Upstream `getLocalTarget()` projects the current start onto that global
  trajectory and advances by accumulated path length. V0 retains its
  `last_progress_time_`-based progress. V1 adds a separate ordered DDDMR
  waypoint-segment tracker and route-fitness cost, but that is not the same
  upstream global-trajectory projection/replan fix.
- The upstream port also sets the horizon to 3.5 m and
  `obstacles_inflation_z_down` to 0.1 m. V0 already uses a 3.5 m horizon, but
  its stair-specific vertical setting differs; matching a parameter does not
  mean the algorithm fix is present.

Conclusion: the user's recollection is correct at the full-fix level: neither
V0 nor V1 includes the complete upstream Mode 3 collision fix. V0/V1 share a
few individual pieces, while V1 additionally contains our own route-following
experiments. This is separate from the latest clipped-free-ray mapping change.

The upstream Mode 3 fix was ported onto V0 on 2026-10-07, preserving the local
0.5 m thinning/Z alignment, 3.5 m horizon, stair collision settings, and
clipped-free-ray work:

- Build the Mode 3 global reference trajectory from the supplied route's first
  point and remaining ordered waypoints, with zero initial route velocity and
  acceleration; reject paths with fewer than two points.
- During Mode 3 replans, keep the DDDMR global trajectory and reseed local
  rebound optimization from the active local trajectory at its current time.
  Retry in three stages: previous-trajectory initialization, polynomial
  initialization, then randomized polynomial initialization.
- Select the Mode 3 local target by projecting onto the global reference
  trajectory and walking forward by accumulated arc length up to the local
  planning horizon. Clamp the target velocity to the planner's configured
  maximum. Modes 1 and 2 retain their existing target-selection/replan path.

The adapted ROS 2 code compiled successfully with `colcon build` for
`scan_planner`; `git diff --check` passed. No simulator or hardware behavior
claim is made here. The user elected to validate directly on the real robot.
The exact pre-port FSM source is backed up at
`tools/scan_tuning_backups/2026-10-07_before_mode3_fix/scan_replan_fsm.cpp.tar.gz`
(SHA-256 `0da526c3b1d8f7e9f9b60e160361f493633722f166c008b9ed97619918df5b57`).

## Follow-Up Idea: Fuse MID-360S And Unitree L2 In Super-LIO: 2026-10-07

The host was connected to the robot and both LiDAR streams were inspected
read-only. This is a feasibility note only; no nodes, network settings, or
Super-LIO code were changed.

### Live Topic Findings

| Stream | Observed type/frame | Measured rate | Point fields |
| --- | --- | ---: | --- |
| `/livox/lidar` (MID-360S) | `sensor_msgs/PointCloud2`, `livox_frame` | about 10.9 Hz | `x/y/z/intensity/tag/line/timestamp` |
| `/utlidar/cloud` (Unitree L2) | `sensor_msgs/PointCloud2`, `utlidar_lidar` | about 16.8 Hz | `x/y/z/intensity/ring/time` |
| `/go2/imu` | `sensor_msgs/Imu`, `go2_imu` | publishing | consumed by current `super_lio_node` |
| `/utlidar/imu` | `sensor_msgs/Imu`, `utlidar_imu` | publishing | no subscriber observed |

`/utlidar/cloud_deskewed` was present in graph discovery, but no sample arrived
within the 5-second read window. The two point clouds expose different
per-point time fields, so a parser can in principle preserve timing, but the
units/semantics of L2's `time` field still need validation. Sequential header
reads showed the L2 timestamps roughly six minutes behind the host/MID-360/Go2
IMU timestamps. This was not a simultaneous clock-offset measurement; measure
the offset and drift together before deciding whether clock correction is
possible or hardware/device time synchronization is required.

### Current Super-LIO Input Contract

Current `go2w_mid360.yaml` selects `lio.sensor.lidar_type: 8`, subscribes to
`/livox/lidar`, and uses `/go2/imu`. In the ROS wrapper, `lidar_type=1`
subscribes to `livox_ros_driver2/msg/CustomMsg`; the `else` branch subscribes
to `sensor_msgs/msg/PointCloud2`. The `LIVOX_PC2=8` parser expects the Livox
PointXYZRTLT fields, including `tag` and a `timestamp` field measured as
nanoseconds relative to the first point. It does not parse the L2 `ring/time`
layout. Thus the existing configuration already uses the PointCloud2 path,
while the legacy CustomMsg path remains in the source.

### Earlier CustomMsg-to-PointCloud2 Adaptation

This compatibility change was made earlier during the MID-360 driver-path
troubleshooting. The `rviz_MID360s_launch.py` path used by this setup publishes
the Livox PointXYZRTLT data as `sensor_msgs/PointCloud2`, while the original
Super-LIO Livox subscription path consumed `livox_ros_driver2/msg/CustomMsg`.
Super-LIO was extended with a second subscription/parser path for that Livox
PointCloud2 layout and a new `LIVOX_PC2=8` sensor type; the original CustomMsg
path was retained for drivers that still publish it. The Go2W configuration
was switched to `lidar_type: 8` and `/livox/lidar`, which is the active setup
verified in the current source and runtime graph.

This adaptation is specific to Livox PointXYZRTLT fields (`tag`, `line`, and
per-point `timestamp`); it is not a generic PointCloud2 parser. The available
Git history does not isolate this edit as a later commit because the adapted
source is already present in the imported workspace snapshot. The historical
reason and change are recorded here from the prior troubleshooting and current
configuration/source evidence.

### Feasibility And Required Work

Geometrically, both LiDARs can contribute to one LIO state if each sensor is
calibrated to the same body IMU frame. The current Super-LIO frontend is still
single-LiDAR: one LiDAR subscription, one `T_imu_lidar` extrinsic, and one scan
buffer synchronized against the IMU. Dual-LiDAR LIO therefore needs per-sensor
parsers, independent queues and extrinsics, a common scan-time policy, and
per-point deskew using the shared IMU trajectory before combining observations
for state estimation. Merely concatenating the clouds or remapping both topics
to one input would lose the sensor-specific timing/extrinsics and can create
motion-dependent double surfaces.

Recommended validation order, before implementation:

1. Capture simultaneous header and point-time samples from both LiDARs and
   `/go2/imu`; establish time offset, drift, and L2 `time` units.
2. Confirm both sensor-to-IMU 6-DoF extrinsics and frame axis conventions.
3. Verify each sensor stream independently with the same IMU and calibration.
4. Only then implement and benchmark a dual-LiDAR measurement frontend against
   the current MID-360-only odometry/map baseline.

Official references: [Unitree L2 SDK](https://github.com/unitreerobotics/unilidar_sdk2)
provides L2 point-cloud and IMU interfaces; the [MID-360 protocol](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
documents point timestamps and device synchronization options. These establish
sensor interfaces, not successful dual-LiDAR operation in this workspace.

## Wall Clearance And Dynamic Clearing Follow-Up: 2026-10-08

This pass addresses the reported wall-hugging and stale local obstacle cells.
It does not change the SCAN trajectory/controller algorithm or add a blind-zone
timeout.

### DDDMR Collision-Graph Fixes

Two defects were found in the active `dddmr_py` source:

- The obstacle query was limited to `inflation_radius`, but hard collision was
  tested against `robot_radius`. When `inflation_radius < robot_radius` (the
  current stair launch uses `0.05 m` and `0.22 m` respectively), points in the
  hard-collision annulus were never queried and could not mark a node lethal.
  The query now covers `max(inflation_radius, robot_radius)`.
- A* validated graph-edge endpoints but not the space between them. In the
  synthetic L-wall test, 11 of 72 raw path edges crossed lethal cells. The
  retained graph edges are now sampled along their centerline and rejected if
  they pass through lethal cells or leave the nearby surface support.

### Verification

The focused regression test for `robot_radius > inflation_radius` passes. The
DDDMR self-test suite passes 8/8, including the existing multilayer/stair,
clearance, and path-safety checks. On the same L-wall Mode 3 simulation, before
edge validation the DDDMR/SCAN centerline clearances reported by the test probe
were `0.237 m` / `0.249 m`; after edge validation they were `0.286 m` /
`0.295 m`. Simulated odometry clearance was `0.295 m`, goal error was
`0.059 m`, and no command-to-wall risk was reported. This is a deterministic
synthetic test, not a real robot safety certification.

The actual `8f_9f_stair4_dense_oriented_normals.pcd` also planned the tested
corner route successfully after edge validation: 22 output poses, with zero
raw or post-processed segments failing the segment-feasibility check. Map
preprocessing took `12.1 s` and peaked at about `943 MB` on this host, so the
additional swept-edge check has a measurable resource cost for this map.

Follow-up regression check on 2026-10-08 found that the full edge sweep is too
conservative for this stair map: with identical map/config, disabling the
edge sweep changed the largest graph component from `10,309` to `20,508`
nodes. At the user's request, the graph-edge sweep was removed while retaining
the hard-collision query correction. On the actual stair4 map, the tested
route succeeds with 21 output poses and a 1.919 m path; graph has 715,259
undirected edges and 21,278 non-lethal nodes. The DDDMR suite currently has one
known failure in `test_path_is_continuous_and_safe`: a resampled point lies
nearest to a lethal node when edge validation is disabled. This rollback
restores graph connectivity but does not claim every interpolated route is
collision-free; validate the route in RViz and keep SCAN active for local
collision handling.

The hard-collision query correction also makes `robot_radius=0.22` effective
even when `inflation_radius=0.05`; previously the query was accidentally
clipped to `0.05 m`. This explains additional red/unreachable regions under the
existing launch values. Do not treat the old behavior as a valid physical
footprint model. A proper follow-up should replace the circular body model
with a validated Go2W footprint and test route connectivity and collision
clearance together.

### SCAN Clearance Parameter Trial

A temporary A/B configuration (`dist0=0.45 m`, collision weight `2.0`,
controller `max_vy=0.18 m/s`, DDDMR `inflation_radius=0.25 m`) was tested without
overwriting the normal parameter YAML. Raising these values alone did not
increase the SCAN spline clearance in the first run. The later clearance gain
came from rejecting unsafe DDDMR graph edges, not from claiming that these
larger parameter values solved wall hugging. Keep the normal SCAN values unless
a doorway-specific replay shows the parameter change helps without making the
route infeasible.

The run launch now accepts optional `planner_config` and `controller_config`
YAML overlays, layered on top of the package defaults. Defaults are unchanged.

### Dynamic Occupancy Tuning

`grid_map.p_miss` was reduced from `0.30` to `0.22`. With the existing
`p_max=0.98` and `p_occ=0.80`, this takes about two observed free-ray updates to
clear a saturated occupied voxel, versus about three updates at `0.30`. The
hit/miss ray logic still decides whether a cell is observed free; this change
does not expire a voxel that receives no ray because it is occluded, out of
view, or inside the sensor blind region. The clipped free-ray change from
2026-10-07 remains in place. The final static L-wall run with `p_miss=0.22`
completed with `0.300 m` minimum SCAN/odometry clearance, `0.032 m` goal error,
and no command-to-wall risk. This confirms that the faster miss update did not
erase the tested static wall; it is not a moving-obstacle or blind-zone test.

Do not add a global occupancy TTL yet. An unobserved cell currently becomes
traversable when its occupancy evidence expires, so a short timeout could erase
a static wall while the robot turns away. Next dynamic validation should use a
moving-person replay and a static wall in the same scene, record the old hit
cells and `/grid_map/occupancy_inflate`, and verify both that stale dynamic
cells clear after free rays and that unobserved/static walls remain blocked.

The active DDDMR map still uses a circular `robot_radius`, while SCAN uses a
two-cylinder body envelope. Their orientation-dependent footprints are not yet
identical; a later DDDMR footprint update should be tested against both the
doorway and stair-corner maps before changing the real-robot parameters.

### Mode 3 Stair Tracking Trial (2026-10-12)

The reported remaining stair issue is primarily on descent; ascent is currently
acceptable. The initial `5.5 m` horizon trial was longer than useful; the user
tested it and selected `4.5 m`. The active configuration is now `4.5 m` for
both FSM and manager.

- The launch file exposes one `planning_horizon` argument and applies it to
  both parameters; its default is `4.5`, so this default applies to all
  navigation modes unless overridden.
- Replace mode 3's unrestricted nearest-point search over the entire global
  trajectory with a forward-only projection from the last accepted progress.
  The allowed arc-length advance is bounded by elapsed time and
  `manager.max_vel`, with about `0.10 m` projection sampling. This specifically
  targets branch jumps between nearby stair flights and prevents progress from
  moving backward. Progress and its timestamp reset when a new reference path
  is accepted.
- Emit a throttled INFO line every 2 seconds with projected path time, current
  projection gap, and the allowed progress window, so this can be diagnosed at
  normal log verbosity on the robot.
- A temporary `obstacles_inflation_z_down=0.10 m` trial was reverted after
  reviewing the operator's clarification that a person had been behind the dog.
  The current value remains `0.15 m`; `obstacles_inflation_z_up=0.0`,
  `collision_query_z_offset=0.30 m`, and the horizontal double-cylinder
  footprint are unchanged. Do not attribute those A* occupied samples to stair
  inflation without voxel-level evidence.

The horizon extends the local target and planner optimization horizon. No
vertical collision parameter remains changed in this trial.

Mode 3 previously projected its local planning start against the entire global
reference trajectory on every replan. At stairs, nearby flights can be close in
3D while belonging to different parts of the route; unrestricted nearest-point
projection can therefore jump to a different path branch. The projection now
advances monotonically from its last accepted time and searches only a forward
arc-length window derived from elapsed time and `manager.max_vel`, sampled at
approximately `0.10 m`. The reference progress resets when a new waypoint path
is accepted. A throttled info message reports projection time, distance gap,
and allowed window. The relevant source/configuration files are
`plan_manage/src/scan_replan_fsm.cpp`,
`plan_manage/include/plan_manage/scan_replan_fsm.h`,
`plan_manage/config/planner.yaml`, and `plan_manage/launch/run.launch.py`.

The latest local logs inspected were
`~/.ros/log/scan_planner_node_100510_1791777899071.log`,
`scan_planner_node_77327_1791777365207.log`, and
`scan_planner_node_53181_1791775075883.log`. They repeatedly report A* frontier
exhaustion. In representative failures both endpoints are `occ=0`, while
14-19 of 21 straight-line samples are occupied; examples include
`[15.245,-2.177,2.056] -> [15.895,-1.677,2.356]` (`dz=+0.30 m`, 17/21
occupied) and `[0.657,17.081,0.842] -> [1.157,17.731,1.142]` (`dz=+0.30 m`,
15/21 occupied). Other traces include descending candidate segments. This
confirms occupied cells along the search interpolation, but the log does not
identify whether each cell came from stair treads, another obstacle, or stale
occupancy. The operator reports that a person's return was initially behind
the robot and the robot then planned backward into that area. This is consistent
with stale dynamic occupancy: `p_miss=0.22` only reduces occupancy when a ray
actually traverses the old voxel; a person occluded by the robot or outside the
sensor field of view produces no clearing ray there. Thus `p_miss` and vertical
inflation changes do not solve this failure mode. Keep vertical inflation at
`0.15 m` and investigate selective aging/clearing of dynamic candidates while
ensuring static walls remain occupied. This recommendation was implemented as
the separate V3 experiment below; the vertical inflation value remains `0.15 m`.

The ROS 2 `scan_planner` package previously built successfully using
`colcon build --base-paths src/scan_planner_ros2/src/planner/plan_manage
--packages-select scan_planner --symlink-install`, and
`ros2 launch scan_planner run.launch.py --show-args` listed
`planning_horizon:=5.5`. Rebuild after the current `4.5 m` config change. The
`4.5 m` horizon has not yet been validated on the stair map or robot. The
obstacle log diagnosis is consistent with stale person occupancy, not confirmed
stair inflation.
For the first controlled descent, keep global and local paths visible and record
the projection INFO lines, local target Z, and `/cmd_vel`. If the local planner
still stalls, diagnose the exact occupied voxels/clearance before changing
`collision_query_z_offset` or vertical inflation, since increasing the lookahead
alone cannot make an infeasible stair footprint traversable.

## SCAN Version 3: Dynamic/Static Occupancy Aging (2026-10-12)

Version 2 was snapshotted before this experiment at
`tools/scan_tuning_backups/version2_20261012/`. The snapshot includes the full
`plan_env/` directory, the active planner YAML, and this report. Restore those
files to return to V2; do not restore unrelated workspace changes.

Version 3 keeps the V2 planner/controller settings, including the `4.5 m`
planning horizon, `p_miss=0.22`, clipped free rays, and Mode 3 progress logic.
It changes occupancy handling only:

- A voxel that receives a hit-dominant occupancy update is tracked as
  transient evidence unless it has already been promoted to persistent static
  occupancy. Evidence is counted once per voxel per fused scan.
- A transient candidate is promoted to static after at least five hit scans
  spanning at least `0.5 s`. Static-marked voxels are not removed by timeout.
- An occupied transient candidate with no new hit for `3.0 s` is reset to
  unknown and its inflation contribution is removed. If a free ray clears it
  first, its transient evidence is discarded immediately.
- The aging pass runs on the existing 20 Hz occupancy timer and iterates only
  the transient-candidate hash map, not every voxel in the local map. A
  throttled INFO summary reports expired transient cells, static promotions,
  pending candidates, and TTL.
- Sliding-map resets clear the associated transient/static metadata. A
  confirmed static cell can still be cleared by sufficient observed free-ray
  evidence, and its static mark is then removed; “persistent” means no blind
  timeout, not immunity to explicit sensor evidence.

Parameters are in `planner.yaml`:

```yaml
grid_map.dynamic_obstacle_aging_enabled: true
grid_map.dynamic_obstacle_ttl_sec: 3.0
grid_map.static_obstacle_confirm_time_sec: 0.5
grid_map.static_obstacle_confirm_scans: 5
```

The classifier is temporal, not semantic: a person standing still in one voxel
for long enough can be promoted to static, while a static wall seen for too few
scans can remain transient and eventually expire. Therefore V3 is a controlled
hardware trial, not a guarantee that static/dynamic identity is always correct.
The required validation keeps a static wall in view, hides it briefly during a
turn, moves a person behind the robot, then checks that the person returns clear
after TTL while the wall remains blocked. Record the V3 INFO summaries,
`/grid_map/occupancy`, `/grid_map/occupancy_inflate`, planner success, and
`/cmd_vel`. Stop the test if a static wall disappears or the robot plans through
unknown space. The exact V2 rollback files are already saved above.
