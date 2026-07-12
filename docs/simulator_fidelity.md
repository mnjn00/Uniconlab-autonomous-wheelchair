# Simulator fidelity and evidence labels

Gazebo Classic is an algorithm-regression environment, not a wheelchair, campus, or passenger qualification environment. Normative claim limits are in `contracts/wp0/A13-simulator-fidelity.yaml`. Every metric or release statement must carry exactly one tag, and promotion above its evidence is a release rejection.

## Evidence tags

From least to greatest authority:

1. `UNIT_ONLY`
2. `REPLAY_CONSISTENCY`
3. `SIMULATION_ONLY`
4. `TARGET_NUC_SOFTWARE`
5. `HIL_INERT`
6. `CLOSED_COURSE_NO_PASSENGER`
7. `CAMPUS_APPROVED`
8. `PASSENGER_APPROVED`

These are cumulative gates, not labels an operator may choose. Current physical authority is blocked. `TARGET_NUC_SOFTWARE` still does not authorize motion; neither HIL nor closed-course evidence authorizes campus or passengers. A software RC is not motor, passenger, campus, braking, curb/grade, or wet-weather approval.

## What current software evidence can support

| Claim | Maximum current tag | Supported statement | Limitation / required replacement evidence |
|---|---|---|---|
| ABI, graph, state, no bypass, watchdog zero | `SIMULATION_ONLY` | software topology and fail-closed transitions | actual base graph, driver, timeout, and physical stop require verified audit/HIL |
| `move_base` path/collision in scripted worlds | `SIMULATION_ONLY` | deterministic algorithm regression and scripted metrics | world/map/model bias prohibits campus success or incident probability; needs surveyed closed course |
| native localization against Gazebo truth | `SIMULATION_ONLY` | simulated error/recovery behavior | idealized geometry/noise and no real truth; needs surveyed reference and hardware bag |
| Livox conversion/replay | `REPLAY_CONSISTENCY` | ABI, ordering, count and deterministic conversion after source validation | no truth; Gazebo is not proven to model Livox scan pattern, motion distortion, reflectivity, rain, multipath or occlusion |
| collision/TTC/dynamic obstacles | `SIMULATION_ONLY` | equations, states, injected faults, scripted stops | pedestrian intent/classification and real blind zones/reflectivity require instrumented closed course |
| caster/contact/friction/slip/curb | `SIMULATION_ONLY` | coarse controller regression only | Gazebo contact/friction/caster dynamics do not establish stability, curb traversal, grade or stopping |
| braking latency/distance | `SIMULATION_ONLY` | software zero-command publication timing only | driver, motors, brake, battery, payload, surface, temperature and slope require WP7 measurements |
| slope policy | `SIMULATION_ONLY` | IMU math, state and stale/calibration fault logic | gravity alignment, vibration, slip, grade and cross-slope stability require calibration/closed course |
| payload/battery scenarios | `SIMULATION_ONLY` | parameterized software behavior | mass distribution, center of gravity, voltage, torque, derating, acceleration and braking are unmeasured |
| e-stop/manual/joystick/watchdog | `SIMULATION_ONLY` | logical topic, latch, reset and timeout behavior | physical priority/circuit/latency, native timeout, sign and units require repeated inert tests |
| target-NUC resources | `UNIT_ONLY` | method/schema only | workstation or Gazebo-host observations cannot qualify the NUC; run all A12 profiles on fingerprinted target |
| committed map/route/geofence | `UNIT_ONLY` | hashes, schema and internal consistency | candidates are unsurveyed and grant no corridor, slope, clearance, direction or campus authority |
| passenger safety | `UNIT_ONLY` | only the statement “not authorized” | requires every earlier gate, institutional/campus approval, and a distinct passenger protocol |

## Landed simulation surfaces

Run from a sourced Noetic workspace:

```bash
roslaunch wheelchair_bringup sim_bringup.launch world:=empty
roslaunch wheelchair_bringup sim_bringup.launch world:=road_free_space
roslaunch wheelchair_bringup sim_bringup.launch world:=sidewalk_obstacles
roslaunch wheelchair_bringup sim_bringup.launch world:=static_dynamic_obstacles
```

`sim_bringup.launch` fixes `/use_sim_time=true`, starts Gazebo/navigation/safety, and relays only `/cmd_vel_safe` to `/wheelchair_base_controller/cmd_vel`. The automated suite interface is:

```bash
python3 scripts/run_gazebo_rc_suite.py --config src/wheelchair_gazebo/config/scenarios.yaml --output EVIDENCE_DIR/gazebo-rc-report.json
```

The CLI always requires a live ROS 1/Gazebo graph; a missing Noetic command, package, `/clock`, canonical topic, metrics collector, or evidence artifact fails closed as `PLATFORM_UNAVAILABLE` or failure. Use the suite's actual report and artifact hashes; do not state a scenario passed merely because it launched. Expected software evidence includes deterministic/seeded scenario reports, command-boundary and collision outcomes, timestamps, configuration/world/model/release hashes, and exactly one evidence tag per metric. Missing reports remain pending. No test results are asserted in this document.

## Model and algorithm caveats

- Geometry, sensor noise, obstacle motion, contact, friction, caster behavior, wheel slip, curb impacts, payload/center of gravity, battery/thermal derating, and latency are simplified or unqualified.
- The simulated lidar has no demonstrated equivalence to Livox nonrepetitive scanning, timing, intensity/tag/line fields, motion distortion, rain, multipath, glare, material response, or real occlusion/blind zones.
- Gazebo truth can expose software regression but cannot validate real localization probability, map datum, extrinsics, time synchronization, corridor clearance, or campus route safety.
- Collision-free scripted runs do not predict pedestrian behavior or incident rates.
- Publishing zero quickly in software does not measure wheel deceleration, brake engagement, stopping distance, or physical e-stop/manual priority.
- Developer-workstation latency, CPU, RAM, disk, swap, and thermal observations cannot be relabeled `TARGET_NUC_SOFTWARE`.
- Current unknown driver, extrinsic, braking, physical override/e-stop, surveyed-route, and NUC facts remain hardware blockers regardless of simulation volume.

Store generated evidence outside source/user data and bind it by hash in the release manifest. Do not commit multi-GB bags, user data, secrets, or fabricated result summaries.
