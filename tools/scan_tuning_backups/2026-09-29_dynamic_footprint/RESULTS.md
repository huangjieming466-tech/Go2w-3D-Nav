# Navigation Rear Coverage And Global Footprint

## Scope

This change implements only the requested Super-LIO rear coverage and DDDMR
footprint alignment. Grid occupancy expiration and local recovery are not part
of this revision.

## Super-LIO

`go2w_mid360.py` now derives the rear near-filter from `save_map`:

| Launch mode | `lio.sensor.rear_near_filter_enable` |
| --- | --- |
| `save_map:=false` navigation | `false` |
| `save_map:=true` mapping | `true` |

The YAML `lio.sensor.blind: 0.2` setting remains active in both modes. Thus
navigation retains the original 0.2 m near-field blind range but no longer
removes the rear 90 degree sector within 2 m.

The launch update was deployed to both the endpoint source and installed share
path on `unitree@192.168.20.219`. Endpoint backups use suffix
`20260929_151720`.

## DDDMR

New optional parameters check every graph edge using an oriented two-circle
footprint. Defaults are zero and retain the legacy single-circle behavior.

```text
footprint_radius:=0.22
footprint_offset:=0.13
footprint_edge_samples:=3
```

The resulting horizontal envelope is `2 * (0.13 + 0.22) = 0.70 m` long and
`2 * 0.22 = 0.44 m` wide. This matches SCAN's configured Go2W footprint
closely (`0.70 m x 0.43 m`).

## Verification

- Python syntax checks passed for modified launch and DDDMR files.
- DDDMR demo map built with the new footprint: `16640` nodes, `154794` edges.
- Static stair4 map loaded with the new footprint: `30372` nodes, `21601`
  traversable nodes, `217603` obstacle points.

`tests/test_planner.py` ran. Five of seven existing demo assertions passed;
the two existing failures concern its low-clearance and lethal-point
expectations and occur with the legacy model too.

## Rollback

- Before archive: `before_dynamic_footprint.tar.gz`
- After archive: `after_dynamic_footprint.tar.gz`
