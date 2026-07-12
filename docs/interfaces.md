# Runtime interfaces

This is the ROS 1 Noetic software-RC contract summarized from `contracts/wp0/A03-topic-tf-time-ownership.yaml` and `src/wheelchair_interfaces/`. The only autonomous command path is `/cmd_vel_nav` → `safety_gate` → `/cmd_vel_safe` → a profile-selected inert sink. The current release has no authorized motor endpoint.

## Topics and ownership

| Topic | Message | Sole producer | Consumer(s) | Time/rate contract and failure |
|---|---|---|---|---|
| `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | selected sensor adapter | collision supervisor | 10 Hz nominal; 0.30 s TTL; stale is blind STOP |
| `/sensors/imu/data` | `sensor_msgs/Imu` | selected sensor adapter | slope supervisor, selected localization adapter | 200 Hz nominal; 0.10 s TTL; stale/unknown is STOP |
| `/odom` | `nav_msgs/Odometry` | verified base or simulation adapter | collision supervisor, localization guard, hardware adapter | ≥20 Hz; 0.20 s TTL; stale is STOP |
| `/localization/candidate` | `wheelchair_interfaces/LocalizationCandidate` | selected native Noetic localization adapter | independent localization guard | ≥10 Hz; 0.25 s TTL; candidate is untrusted; stale is LOST/STOP |
| `/localization/status` | `wheelchair_interfaces/LocalizationStatus` | independent localization guard | safety gate, route safety | ≥10 Hz; 0.25 s TTL; LOST is STOP |
| `/safety/localization` | `wheelchair_interfaces/SafetySignal` | independent localization guard | safety gate | ≥10 Hz; 0.25 s TTL; unknown/stale is STOP |
| `/route/active` | `wheelchair_interfaces/ActiveRoute` | decision | route safety, navigation route manager | 2 Hz heartbeat; 0.75 s TTL; untrusted request; stale is STOP |
| `/route/progress` | `wheelchair_interfaces/RouteProgress` | navigation route manager | decision | ≥5 Hz; 0.50 s TTL; progress only; stale causes hold/cancel |
| `/route_safety/geofence_status` | `wheelchair_interfaces/GeofenceStatus` | `wheelchair_route_safety` | safety gate, diagnostics | 20 Hz; 0.25 s TTL; stale is STOP |
| `/safety/geofence` | `wheelchair_interfaces/SafetySignal` | `wheelchair_route_safety` | safety gate | 20 Hz; 0.25 s TTL; unknown/stale is STOP |
| `/safety/collision_status` | `wheelchair_interfaces/CollisionStatus` | collision supervisor | safety gate, diagnostics | 10 Hz; 0.30 s TTL; stale/blind is STOP |
| `/safety/collision` | `wheelchair_interfaces/SafetySignal` | collision supervisor | safety gate | 10 Hz; 0.30 s TTL; unknown/stale is STOP |
| `/safety/slope_status` | `wheelchair_interfaces/SlopeStatus` | slope supervisor | safety gate, diagnostics | 50 Hz; 0.10 s TTL; unknown/stale is STOP |
| `/safety/slope` | `wheelchair_interfaces/SafetySignal` | slope supervisor | safety gate | 50 Hz; 0.10 s TTL; unknown/stale is STOP |
| `/decision/motion_intent` | `wheelchair_interfaces/MotionIntent` | decision | safety gate | ≥10 Hz; 0.30 s TTL; stale causes hold/STOP |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | `move_base` | safety gate, collision supervisor | ≥10 Hz while active; 0.30 s TTL; stale is exact zero |
| `/hardware/driver_status` | `wheelchair_interfaces/DriverStatus` | verified hardware adapter | safety gate | ≥20 Hz; 0.15 s TTL; currently unauthorized; stale disarms/stops |
| `/safety/mode` | `wheelchair_interfaces/SafetySignal` | verified hardware adapter | safety gate | ≥20 Hz; 0.15 s TTL; currently unauthorized; unknown/stale is STOP |
| `/safety/driver` | `wheelchair_interfaces/SafetySignal` | verified hardware adapter | safety gate | ≥20 Hz; 0.15 s TTL; currently unauthorized; unknown/stale is STOP |
| `/safety/estop` | `std_msgs/Bool` | verified I/O or operator source | safety gate | event plus heartbeat when available; `true` latches; loss never clears |
| `/safety/estop_reset` | `std_msgs/Bool` | guarded operator request | safety gate | rising-edge event; rejection keeps latch set |
| `/safety/arm` | `std_msgs/Bool` | operator request | safety gate | one-shot request; accepted only when every required input and topology check is clear; reset never arms |
| `/safety/mission_cancelled` | `std_msgs/Bool` | mission/operator authority | safety gate | must be true for guarded e-stop reset |
| `/safety/state` | `wheelchair_interfaces/SafetyState` | safety gate | operator/diagnostics | latched structured state; inspect `armed`, `estop_latched`, `reason_mask`, ages, counters, hashes |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | safety gate only | collision supervisor and profile-selected inert/verified adapter | 50 Hz including exact zero; downstream last-command hold forbidden |
| actual driver topic | manifest-defined | direct verified driver or verified adapter | physical driver | no default; missing/unknown contract blocks launch before endpoint creation |

`/move_base` is `move_base_msgs/MoveBaseAction`: `move_base` is the sole server and decision the sole client. Failure cancels and holds.

## TF ownership

| Transform | Sole owner | Contract |
|---|---|---|
| `map` → `odom` | selected localization adapter | ≤0.25 s old and exact map identity |
| `odom` → `base_footprint` | selected base or simulation source | ≤0.25 s old |
| `base_footprint` → `base_link` | `robot_state_publisher` | fixed, measured calibration required for hardware |
| `base_link` → `lidar_link` | `robot_state_publisher` | fixed, measured calibration required for hardware |
| `base_link` → `imu_link` | `robot_state_publisher` | fixed, measured calibration required for hardware |

Duplicate authority, an unknown calibration, stale TF, or a jump outside explicit relocalization is STOP. Frame IDs have no leading slash.

## Time and graph rules

Safety subscribers use queue size 1, latest-only/drop-old behavior, source stamp plus callback receipt-age validation, and a 50 ms future tolerance. Zero, regressing, impossible, stale, or mismatched timestamps fail closed. `/use_sim_time` is fixed before process start: Gazebo owns simulation clock; replay has exactly one `rosbag play --clock` publisher; native/shadow uses wall time. Clock reversal/reset disarms and requires explicit re-arm. A duplicate publisher, TF authority, or command bypass is STOP/disarm. `SafetySignal` evidence is accepted only when sequence, evaluation stamp, source, reason mask, and policy SHA match its structured status.
