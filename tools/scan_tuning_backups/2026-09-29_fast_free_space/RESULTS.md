# Faster Free-Space Clearing And Rear Footprint Validation

## Occupancy Fusion

Only `grid_map.p_miss` changed:

```text
old: 0.30, log odds -0.84730
new: 0.22, log odds -1.26567
```

`p_hit=0.85` and `p_occ=0.80` remain unchanged. A saturated obstacle now
needs two consecutive free-ray updates to become non-occupied instead of
three. At 5 Hz this is about 0.4 s instead of 0.6 s. This is not a TTL and
does not clear cells that receive no free-ray observation.

## DDDMR And SCAN Footprint

With the Go2W footprint `radius=0.22`, `offset=0.13`, the initial route's
69 raw graph edges all passed the oriented two-circle check. The previous
post-processing pipeline produced two unsafe sampled segments near
`[5.86, 11.72, 3.78]`. DDDMR now validates post-processed output and falls
back to the 69-point raw graph path when smoothing or projection violates the
footprint.

The resulting route is 23.863 m and has zero invalid double-circle segments.
Publishing raw graph points, instead of a sparse 4-point shortcut, also keeps
enough intermediate reference points for SCAN Mode 3's 3.5 m local horizon.

## Simulation Status

The native PCD simulator started with the updated miss log odds. DDDMR's cold
load of the 454122-point stair map can take more than 24 s on this host, so a
short bounded run published its goal before DDDMR was ready and therefore
produced zero local trajectories. It is an initialization timing limitation,
not a planner result. A full route simulation must wait for the DDDMR log line
`DDDMR map ready` before publishing the goal.

## Rollback

- `before_fast_free_space.tar.gz`
- `after_fast_free_space.tar.gz`
