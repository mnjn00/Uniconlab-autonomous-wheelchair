<p align="center">
  <img src="docs/assets/hero-route.png" alt="Hanyang University Unicon Lab autonomous wheelchair" width="100%" />
</p>

<h1 align="center">Unicon Lab Autonomous Wheelchair</h1>

<p align="center">
  Hanyang University · Unicon Lab, Robotics<br />
  A fail-closed ROS 1 Noetic stack for map-based localization, route following,
  and independent motion-safety supervision on a real powered wheelchair.
</p>

<p align="center">
  <a href="http://wiki.ros.org/noetic"><img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros" alt="ROS Noetic" /></a>
  <a href="https://releases.ubuntu.com/20.04/"><img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge&logo=ubuntu" alt="Ubuntu 20.04" /></a>
  <img src="https://img.shields.io/badge/LiDAR-Livox%20MID--360-00A8E1?style=for-the-badge" alt="Livox MID-360" />
  <img src="https://img.shields.io/badge/localization-field%20validated-16a34a?style=for-the-badge" alt="Localization field validated" />
  <img src="https://img.shields.io/badge/operation-supervised%20only-f59e0b?style=for-the-badge" alt="Supervised operation only" />
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache%202.0-f59e0b?style=for-the-badge" alt="Apache 2.0" /></a>
</p>

<p align="center">
  <a href="#two-stacks-in-one-repository">Two stacks</a> ·
  <a href="#what-has-actually-been-driven">Field results</a> ·
  <a href="#the-vehicle">Vehicle</a> ·
  <a href="#running-it">Running it</a> ·
  <a href="#verification">Verification</a> ·
  <a href="#safety-boundary">Safety boundary</a>
</p>

> [!IMPORTANT]
> The chair drives itself, and it does so **only with an operator holding the joystick**.
> Moving the joystick drops the base out of auto mode and the follower holds within one control
> cycle. No unsupervised operation and no passenger transport is authorized; the formal release
> gates in [`contracts/wp0/A16-release-authority.yaml`](contracts/wp0/A16-release-authority.yaml)
> remain unapproved and are not changed by any field result below.

## Two stacks in one repository

Reading the code without knowing this wastes a lot of time, so it is the first thing here.

| | `src/static_livox_localization/` | `src/wheelchair_*/` |
| --- | --- | --- |
| Status | **This is what drives the chair** | Designed architecture, not deployed |
| Localization | FAST-LIO odometry + GICP against a prior map | Adapter, confidence guard, topology guard |
| Command owner | `waypoint_follower.py` | `move_base` |
| Deployed to the NUC | Yes, by `tools/push_to_nuc.sh` | No |
| Formal WP0 contracts | Not bound to them | Bound, frozen, hash-checked |

The `wheelchair_*` packages carry the contract and guard design and most of the test suite. The
`static_livox_localization` package grew in the field, against a real chair on a real campus route,
and it is the one on the vehicle. Both are maintained; only the second one moves wheels today.

## What has actually been driven

Two complete autonomous runs of the 383 m campus route on **2026-07-31**, and three recorded
sessions on **2026-08-02**. Localization was the open question and is now closed:

| Measured while driving, both 07-31 runs pooled | Value | Gate |
| --- | --- | --- |
| Samples | 154 | — |
| Tracking state | `TRACKING` for every one | — |
| `inlier_ratio` | 0.9666 – 1.0000 (mean 0.9949) | > 0.20 |
| `fitness` | 0.0137 – 0.0329 (mean 0.0219) | < 0.28 |
| Localization holds published or suppressed | 0 | — |

Both runs initialized with **no manual seed**, reaching `TRACKING` through
`WAITING_INITIALIZATION` → `VERIFYING`, with FAST-LIO stationary drift of 0.016 m and 0.014 m
against a 0.15 m health limit. The two runs agree to within 0.0006 on both means, which is what
makes this a measurement rather than one lucky lap. For reference the same gates were set from
07-29 figures where the inlier ratio ran 0.44 – 0.79; the worst sample on 07-31 beats that day's
best, on the same route and the same map.

One excursion is recorded rather than smoothed over: parked at the goal twenty minutes after the
second run ended, the fix fell to inlier 0.124 – 0.262 and crossed the gate four times with fitness
still inside its ceiling. Nothing was driving. It is the only time the fix came near failing, so the
end of the route is its weakest geometry.

The recordings are in [`blackbox/`](blackbox/) under Git LFS, checksummed in per-session manifests
and verified by `tests/test_blackbox_archive.py`. A truncated bag opens fine and plays short, which
is the failure that check exists for.

## The vehicle

A powered wheelchair with a Livox MID-360 on the front of the left armrest.

| Measurement | Value | How it was obtained |
| --- | --- | --- |
| Sensor forward of the wheel axle | 0.500 m | Operator measurement, 2026-07-31 |
| Sensor left of the wheel axle | 0.200 m | Operator measurement, 2026-07-31 |
| Sensor height above ground | 0.725 m | Operator measurement; the 0727 map independently gives 0.775 m ± 0.018 |
| Ground-blind radius | ~5.9 m | `0.725 / tan(7°)` from the MID-360's lower field of view |

That blind radius is why drop safety does not come from the live scan. Kerbs and drop-offs sit
inside it, so they are avoided by keeping the chair inside a **safety band** precomputed from the
merged map along the route. The scan is used for what the sensor can see: obstacles and people.

Frames follow REP-103 in
[`wheelchair_hardware.urdf.xacro`](src/wheelchair_description/urdf/wheelchair_hardware.urdf.xacro):
`base_footprint` at the axle midpoint on the ground, `imu_link`/`body` at FAST-LIO's IMU origin,
`lidar_link` at the Livox optical origin derived from it through the built-in extrinsic.

## Command path

Deliberately narrow, with each stage able to stop the chair if the one above it misbehaves:

```text
waypoint_follower.py          route following, band containment, obstacle policy
  -> /cmd_vel_raw
safety_gate.py                independent stopping envelope and swept footprint
  -> /cmd_vel_gated
tip_guard.py                  rate limiter and staleness fail-safe, last stage
  -> /cmd_vel
wheel_cmd / uart              UART-level watchdog
```

If the follower misbehaves or dies the gate stops the chair; if the gate dies, tip_guard's staleness
check stops it; if that dies, the UART watchdog does.

### Obstacle policy

`obstacle_clusters.py` clusters the accumulated scan and tracks each object in the odom frame -
motion is only a question in a frame that does not move with the chair. The follower then answers
two situations differently:

- **Watched standing still** → gone around, decided 5 m out so the chair drifts past instead of
  driving up to it and stopping first. Every sidestep is vetted against the safety band.
- **Moving, or not yet watched long enough** → waited out where it stands. It resumes on its own
  once the corridor is clear, with no timer to reset. Stepping around a person is a manoeuvre into
  where they are about to be.

Distances are measured from an object's own returns, not from its bounding box. On 2026-07-31 a
wall crossing the scan diagonally was published as a box whose near face read 0.69 m dead ahead
while its nearest return inside the corridor was 2.13 m, and the chair held for 16 minutes on
evidence that did not exist.

### Diagnostic mode

`SAFETY_POLICIES=false` switches off everything that is a judgement about the world - band
containment, the raw corridor scan, the localization hold, the geofence, the slope limit - so that
one run can measure one thing without another guard ending the measurement first and looking, from
outside a stationary chair, exactly like the thing it was meant to detect.

What never switches off is the joystick override and the checks it rests on: `MANUAL_MODE` is the
override, and `BASE_STALE` is whether the channel reporting it is still alive. Suppressed policies
are still evaluated and published as `WOULD_HOLD:` on `/waypoint_follower/status`, because that run
is the only place their thresholds can be calibrated against the real thing.

## Running it

Deployment is one command from a checkout with the map volume attached:

```bash
./tools/push_to_nuc.sh /Volumes/<map-volume>/merged_0707_0725_v1
```

It verifies the map bundle, syncs and builds the workspace with whichever catkin tool owns the build
space, and installs the field scripts with checksum verification. Those scripts live in `$HOME`
outside the checkout, which is how a stale copy once kept launching the previous week's route.

At the chair:

```bash
./trial_0727.sh    # brings the whole stack up, leaves the follower PAUSED
./go.sh            # starts the drive
./stop.sh          # stops it, as does moving the joystick
```

`go.sh` refuses rather than starts if the follower is absent, the object tracker is silent, or
localization is not `TRACKING`, and it checks all three before commanding anything - a check after
the first command runs while the chair is already moving. `stop.sh` checks nothing, because a stop
that refuses on a failed precondition is not a stop.

A black box records pose, diagnostics, commands, wheel status, follower state and the object summary
for the whole session.

## Verification

```bash
python3 -m pytest -q                              # host suite
python3 scripts/validate_wp0_contracts.py --root . # frozen WP0 contracts
```

Pinned Ubuntu 20.04 / Noetic:

```bash
docker build -f tools/noetic/Dockerfile -t wheelchair-noetic-validation .
docker run --rm --network none wheelchair-noetic-validation bash -lc \
  'source /opt/ros/noetic/setup.bash && catkin_make && catkin_make run_tests && python3 -m pytest -q'
```

Around 700 host tests pass. Several suites are deliberately evidence-bound and fail without the
external dataset - a missing GLIM artifact, target-NUC run, or physical gate stays blocked rather
than being replaced by a simulated claim.

The tests that matter most here are the ones written against something that actually went wrong:
`test_mount_geometry` (one sensor height living in six files), `test_localization_field_envelope`
(gates sized outside the measured envelope), `test_object_profile` and `test_cluster_guard`
(measuring an obstacle from returns rather than a box), `test_drive_policy` (the joystick override
surviving every switched-off policy), and `test_blackbox_archive` (a recording nobody can repeat).

## Repository layout

```text
src/
├── static_livox_localization/   # THE DEPLOYED STACK: FAST-LIO + GICP localization,
│                                #   follower, safety gate, clustering, field scripts
├── wheelchair_interfaces/       # frozen ROS messages, actions, and ABI contracts
├── wheelchair_perception/       # canonical sensor products; no motion permission
├── wheelchair_navigation/       # localizer adapter, route manager, move_base integration
├── wheelchair_route_safety/     # independent route and geofence authority
├── wheelchair_decision/         # deterministic mission FSM and ExecuteRoute action
├── wheelchair_safety/           # gate plus collision/slope/localization/topology authorities
├── wheelchair_hardware/         # exact driver contract; disabled by default
├── wheelchair_bringup/          # explicit sim/replay/shadow/hardware profiles
├── wheelchair_description/      # Gazebo and NUC-derived URDFs, not interchangeable
└── wheelchair_gazebo/           # simulation-only scenarios and evidence collectors
```

| Path | Purpose |
| --- | --- |
| `routes/` | Route waypoints, safety bands, corridor masks and previews |
| `blackbox/` | Field recordings (Git LFS) with per-session manifests |
| `contracts/wp0/` | Frozen ownership, schema, evidence, hazard, and authority contracts |
| `data/` | Committed candidate map, metadata, and directional routes |
| `tools/` | Deployment, field startup, route/band generation, offline analysis |
| `docs/` | Operator, mapping, interfaces, simulator, safety, and field reports |

`wheelchair_description` keeps two non-interchangeable entry points: the Gazebo-only
`wheelchair.urdf.xacro` and the NUC-derived `wheelchair_hardware.urdf.xacro`. Hardware launch and
calibration must never fall back to the simulation description.

## Safety boundary

Field operation to date is **supervised**: an operator rides the chair with a hand on the joystick
for the whole run, on a known route, on a campus footpath, with no passenger other than the
operator. That is the only mode any result in this repository supports.

The formal release gates are unchanged and remain unapproved:

```yaml
software_release_candidate_authorized: false
hardware_motion_authorized: false
passenger_operation_authorized: false
campus_operation_authorized: false
physical_authority: false
```

They stay false while any of these is unknown or unmeasured: the exact real-driver topic, type,
MD5, sign, units, rate, timeout, mode and watchdog behaviour; physical e-stop and joystick priority
and latency; footprint, payload, battery, surface, braking and stopping envelope; fingerprinted
target-NUC resource, thermal and endurance qualification; inert HIL repetitions and segregated
closed-course no-passenger evidence; surveyed route, corridor and exclusions with written campus
approval; and a separately reviewed passenger-operation protocol.

See [`contracts/wp0/A16-release-authority.yaml`](contracts/wp0/A16-release-authority.yaml),
[`contracts/wp0/A14-hazard-log.yaml`](contracts/wp0/A14-hazard-log.yaml), and
[`docs/safety_case.md`](docs/safety_case.md) for the complete claim boundary.

## Documentation

- [Operator runbook](docs/operator_runbook.md)
- [2026-08-02 field report](docs/2026-08-02-field-report.md)
- [Replay and offline mapping](docs/replay_and_mapping.md)
- [Interfaces and ownership](docs/interfaces.md)
- [Simulator fidelity](docs/simulator_fidelity.md)
- [Safety case](docs/safety_case.md)

## License

Apache License 2.0. See [LICENSE](LICENSE).
