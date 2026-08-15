# 2026-08-15 localization mismatch evidence

This directory preserves the ROS data and runtime evidence from the stopped
August 15 field session.  It is intended for independent localization and
controller analysis; the conclusions below are only an index into the raw
evidence.

## Safety outcome

- The follower stayed paused except for a brief start request that immediately
  returned `HOLD:OFF_BAND`.
- The maximum absolute `linear.x` and `angular.z` on `/cmd_vel_raw`,
  `/cmd_vel_gated`, and `/cmd_vel` were all zero in both included bags.
- Shutdown was performed with the follower paused, the base taken out of auto
  mode, and `/cmd_vel` confirmed zero before all ROS processes were stopped.
- `blackbox_20260815_212633.bag` closed cleanly.  The earlier active bag was
  copied and reindexed; its original NUC file was not modified.

## Sessions

### `210339_wrong_localization`

The operator reported that the chair was at field label WP10.  Localization
remained around `(-4.95, -3.59)`, approximately 2.15 m from the closest point
in the 1,886-point dense route.  Both the safety band and v8 drivable mask
checks were false.  A start request produced `HOLD:OFF_BAND` and was paused.

The bag contains only stationary diagnostics after initialization, so it does
not contain the initial registration score.  The included bag is an indexed
copy of the original `.bag.active` file.

### `212633_wp0_relocalization`

The stack was restarted with `global_only=true`.  The GPU search accepted a
global hypothesis and started around `(0.060, 0.170)`, 0.05 m from dense route
WP43.  The corresponding localization log records refined score 0.937 and a
verified `TRACKING` transition.

After the operator moved the chair to field label WP0, a manual experiment
seeded dense route WP0 `(-7.900, -2.800)` at 10 degrees.  Registration
candidates repeatedly proposed corrections 0.85--1.67 m from the seed, with
fitness about 0.079--0.084 and inlier ratio about 0.65--0.68.  Candidate
consensus was unstable during the observed verification window.  The bag
contains the full diagnostic sequence, including two `CONSENSUS_READY` events
and the later displaced poses.  The final pose was `(-9.305, -2.921)`, 1.41 m
from dense route WP0.

Do not equate the field labels WP0/WP10 with the dense JSON indexes without a
separate mapping.  The active route has 1,886 dense points, and a known August
12 bag also began near dense WP43.  The field-label-to-route-index mapping is
an unresolved part of this incident.

## Contents

| Path | Purpose |
| --- | --- |
| `bags/blackbox_20260815_210339_reindexed.bag` | Indexed copy of the first stationary/off-band session |
| `bags/blackbox_20260815_212633.bag` | Cleanly closed global-search and WP0 reseed session |
| `derived/session_summary.json` | Counts, bounds, first/last poses, command maxima, route identity, and diagnostic counts |
| `derived/pose.csv` | Pose, yaw, closest dense waypoint, and route distance for every pose |
| `derived/localization_diagnostics.csv` | State, reason, fitness, inlier ratio, prediction deltas, backend, and reset count |
| `derived/commands.csv` | Raw, gated, and final velocity commands |
| `derived/follower_status_events.csv` | Follower status changes only |
| `derived/rosbag_info_*.txt` | ROS message schemas, topic counts, duration, and compression |
| `derived/runtime_manifest.txt` | NUC commit, dirty-tree inventory, and launch selections |
| `derived/sha256sum.txt` | Hashes of source bags, configs, masks, and primary logs |
| `logs/` | NUC launch logs captured after shutdown |
| `config/` | Exact uncommitted v6+v8 trim route, safety band, and v8 mask YAML used by the run |

The v8 PGM itself is already tracked at `routes/route_2d_map_v8.pgm`; its hash
in `derived/sha256sum.txt` is
`3d183e8320f68c5deafd72c8f45a5780dade29ff43ca323664f27f4e3ed2eaa8`.

## Reproducing the extraction

On a ROS Noetic machine:

```bash
rosbag info bags/blackbox_20260815_212633.bag
rostopic echo -b bags/blackbox_20260815_212633.bag -p \
  /fast_lio_icp/pose > pose_raw.csv
rostopic echo -b bags/blackbox_20260815_212633.bag -p \
  /fast_lio_icp/localization_diagnostics > diagnostics_raw.csv
```

Start analysis with `derived/session_summary.json`, then use the derived CSVs
for plots and the bags whenever full ROS message fidelity or time alignment is
required.  Git LFS is required to fetch the bag payloads.
