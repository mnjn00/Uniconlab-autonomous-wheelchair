---
schema_version: 1
artifact_id: A02-interface-abi-v1
owner: wheelchair_interfaces owner
reviewer_role: independent safety architect
status: frozen_contract
source_plan_sha256: bd1b9454bc34f68714e6b986e80466535f817a42c25f662db0990adc79ca601e
provenance: approved RALPLAN Appendix A
canonical_abi_sha256: 2f9185da216397708649931207018f4fc8ed79ea1b2c4d3494afafa0891daed0
---

# ROS 1 interface ABI v1

## Canonical inventory and hash algorithm

The inventory is closed and ordered exactly as listed below. No other `.msg` or `.action` file is part of ABI v1.

1. `msg/SafetyReason.msg`
2. `msg/SafetySignal.msg`
3. `msg/CollisionStatus.msg`
4. `msg/SlopeStatus.msg`
5. `msg/LocalizationCandidate.msg`
6. `msg/LocalizationStatus.msg`
7. `msg/ActiveRoute.msg`
8. `msg/GeofenceStatus.msg`
9. `msg/MotionIntent.msg`
10. `msg/RouteProgress.msg`
11. `msg/MissionState.msg`
12. `msg/DriverStatus.msg`
13. `msg/SafetyState.msg`
14. `action/ExecuteRoute.action`

Canonicalization is byte-exact and locale-independent:

1. Encode every inventory path as UTF-8 exactly as printed, with no leading directory or `./`.
2. Source content is the UTF-8 payload of its corresponding fenced block below, excluding fence lines. It uses LF (`0x0a`) only, has no BOM, has no trailing spaces or tabs, and ends with exactly one LF.
3. For each entry in inventory order, append `path_bytes`, one NUL byte (`0x00`), the content length as an unsigned 64-bit big-endian integer, then `content_bytes`.
4. Concatenate entries without separators or metadata and calculate lowercase hexadecimal SHA-256. The result must equal front-matter `canonical_abi_sha256`.
5. Generation additionally fails for a missing, duplicate, differently cased, or unlisted `.msg`/`.action` source. Constants and field order are ABI. Any change requires schema v2 and a reviewed ADR.

ROS 1 dependencies are exactly `std_msgs`, `geometry_msgs`, and `actionlib_msgs` (plus generated self-references). Custom messages contain no arrays and no unbounded binary payloads. IDs, source, and policy strings are UTF-8 without control characters and are runtime-capped at 64 bytes. SHA fields are exactly 64 lowercase hexadecimal bytes. Human result messages are capped at 256 bytes. Every float must be finite. For diagnostic distances, TTC, scores, and ages, `-1.0` is the only unavailable sentinel and can never support CLEAR/OK.

## Canonical sources

### `msg/SafetyReason.msg`
```text
uint64 ESTOP=1
uint64 STALE_CMD=2
uint64 MODE=4
uint64 GEOFENCE=8
uint64 COLLISION=16
uint64 LOCALIZATION=32
uint64 DRIVER=64
uint64 INVALID_CMD=128
uint64 CLOCK=256
uint64 STALE_INTENT=512
uint64 INTERNAL_FAULT=1024
uint64 STARTUP=2048
uint64 SENSOR_STALE=4096
uint64 COLLISION_BLIND=8192
uint64 COLLISION_TTC=16384
uint64 COLLISION_DISTANCE=32768
uint64 SLOPE=65536
uint64 IMU_UNCALIBRATED=131072
uint64 ROUTE_MANIFEST=262144
uint64 GRAPH_TOPOLOGY=524288
uint64 TF=1048576
uint64 BACKPRESSURE=2097152
uint64 DEADLINE_MISS=4194304
uint64 MANUAL_OVERRIDE=8388608
uint64 HARDWARE_UNVERIFIED=16777216
uint64 MAP_MISMATCH=33554432
uint64 COLLISION_OCCLUDED=67108864
uint64 LOCALIZATION_INCONSISTENT=134217728
uint64 RESOURCE=268435456
uint64 CORRUPT_DATA=536870912
uint64 RESET_REJECTED=1073741824
uint64 INPUT_UNKNOWN=2147483648
uint64 ROUTE_STATE=4294967296
uint64 ODOM_STALE=8589934592
uint64 IMU_STALE=17179869184
uint64 LIDAR_STALE=34359738368
uint64 POLICY_MISMATCH=68719476736
uint64 mask
```
Bits 0 through 36 map in declaration order. Bits 37 through 63 are reserved and must be zero; unknown/reserved bits mean `INTERNAL_FAULT`.

### `msg/SafetySignal.msg`
```text
uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 STOP=2
std_msgs/Header header
uint32 sequence
uint8 state
uint64 reason_mask
string source
string policy_sha256
```
`header.stamp` is evaluation time in the active ROS clock; `header.frame_id` is the evaluation frame or empty only for frame-free mode/driver signals.

### `msg/CollisionStatus.msg`
```text
uint8 STATE_UNKNOWN=0
uint8 STATE_CLEAR=1
uint8 STATE_CAUTION=2
uint8 STATE_STOP=3
uint8 VISIBILITY_UNKNOWN=0
uint8 VISIBILITY_FULL=1
uint8 VISIBILITY_PARTIAL=2
uint8 VISIBILITY_BLIND=3
uint8 MOTION_NONE=0
uint8 MOTION_STATIC=1
uint8 MOTION_DYNAMIC=2
uint8 MOTION_AMBIGUOUS=3
std_msgs/Header header
time evaluation_stamp
uint32 sequence
uint8 state
uint8 visibility
uint8 obstacle_motion
uint64 reason_mask
string source
string policy_id
string policy_sha256
float32 input_age_s
float32 transform_age_s
float32 odom_age_s
float32 command_age_s
float32 coverage_fraction
float32 forward_speed_mps
float32 angular_speed_rps
float32 closing_speed_mps
float32 nearest_x_m
float32 nearest_y_m
float32 nearest_distance_m
float32 time_to_collision_s
float32 reaction_distance_m
float32 braking_distance_m
float32 uncertainty_margin_m
float32 required_stop_distance_m
float32 clear_distance_m
float32 recommended_max_linear_mps
uint32 obstacle_point_count
uint32 consecutive_clear_frames
```
`header.stamp` is cloud source time in `base_footprint`; `evaluation_stamp` is completion time.

### `msg/SlopeStatus.msg`
```text
uint8 STATE_UNKNOWN=0
uint8 STATE_CLEAR=1
uint8 STATE_SLOW=2
uint8 STATE_STOP=3
uint8 CAL_UNCALIBRATED=0
uint8 CAL_CALIBRATING=1
uint8 CAL_VALID=2
uint8 CAL_INVALID=3
std_msgs/Header header
time evaluation_stamp
uint32 sequence
uint8 state
uint8 calibration_state
uint64 reason_mask
string source
string policy_id
string policy_sha256
string calibration_sha256
float32 input_age_s
float32 transform_age_s
float32 gravity_norm_mps2
float32 pitch_rad
float32 roll_rad
float32 pitch_rate_rps
float32 roll_rate_rps
float32 acceleration_residual_mps2
float32 orientation_disagreement_rad
float32 recommended_max_linear_mps
```
`header.stamp` is IMU source time and `header.frame_id` is `base_link` after measured transform.

### `msg/LocalizationCandidate.msg`
```text
uint8 RAW_UNINITIALIZED=0
uint8 RAW_INITIALIZING=1
uint8 RAW_OK=2
uint8 RAW_DEGRADED=3
uint8 RAW_LOST=4
geometry_msgs/PoseWithCovarianceStamped pose
uint8 raw_state
uint32 reset_count
string map_id
string map_sha256
string source
float32 raw_score
```
The candidate is never safety authority. `raw_score=-1.0` is unavailable and cannot contribute to OK.

### `msg/LocalizationStatus.msg`
```text
uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 OK=2
uint8 DEGRADED=3
uint8 LOST=4
uint8 RELOCALIZING=5
std_msgs/Header header
time evaluation_stamp
uint32 sequence
uint8 state
uint64 reason_mask
uint32 reset_count
string source
string map_id
string map_sha256
string policy_sha256
string zone_id
float32 pose_age_s
float32 transform_age_s
float32 position_std_m
float32 yaw_std_rad
float32 scan_residual_m
float32 inlier_ratio
float32 innovation_nis
float32 ambiguity_ratio
float32 position_jump_m
float32 yaw_jump_rad
uint32 consecutive_good_samples
bool independent_check_passed
```
`header` is the evaluated candidate pose header in `map`; unavailable metrics forbid OK.

### `msg/ActiveRoute.msg`
```text
uint8 DIRECTION_NONE=0
uint8 DIRECTION_OUTBOUND=1
uint8 DIRECTION_RETURN=2
std_msgs/Header header
uint32 activation_sequence
uint8 direction
string mission_id
string route_id
string map_id
string map_sha256
string route_manifest_sha256
string safety_manifest_sha256
```
This is an untrusted request, not authority.

### `msg/GeofenceStatus.msg`
```text
uint8 UNKNOWN=0
uint8 INSIDE=1
uint8 MARGIN=2
uint8 OUTSIDE=3
uint8 MANIFEST_ERROR=4
std_msgs/Header header
time evaluation_stamp
uint32 sequence
uint8 state
uint64 reason_mask
string source
string manifest_id
string manifest_sha256
string route_id
string segment_id
string zone_id
float32 pose_age_s
float32 transform_age_s
float32 position_uncertainty_m
float32 minimum_signed_clearance_m
float32 required_boundary_margin_m
```
`header` is the localization pose header in `map`. Only INSIDE with sufficient clearance may map to CLEAR; MARGIN is STOP in v1.

### `msg/MotionIntent.msg`
```text
uint8 HOLD=0
uint8 PROCEED=1
uint8 SLOW=2
uint8 STOP=3
std_msgs/Header header
uint32 sequence
uint8 behavior
uint64 reason_mask
string mission_id
float32 max_linear_mps
float32 max_angular_rps
```
Intent cannot raise immutable hard caps.

### `msg/RouteProgress.msg`
```text
uint8 INACTIVE=0
uint8 ACTIVE=1
uint8 AT_STOP=2
uint8 COMPLETE=3
uint8 INVALID=4
std_msgs/Header header
uint32 sequence
uint8 state
string mission_id
string route_id
string map_id
string segment_id
uint32 waypoint_index
float32 along_track_m
float32 cross_track_error_m
float32 distance_remaining_m
```
Progress is diagnostic/decision input and never safety authority.

### `msg/MissionState.msg`
```text
uint8 DISARMED=0
uint8 LOCALIZING=1
uint8 READY=2
uint8 NAVIGATING=3
uint8 PAUSED_OBSTACLE=4
uint8 PAUSED_SAFETY=5
uint8 GOAL_REACHED=6
uint8 ABORTED=7
uint8 FAULT=8
std_msgs/Header header
uint32 sequence
uint8 state
uint64 reason_mask
string mission_id
string route_id
string map_id
```
Every state except NAVIGATING emits HOLD or STOP intent.

### `msg/DriverStatus.msg`
```text
uint8 UNKNOWN=0
uint8 MANUAL=1
uint8 AUTO_DISABLED=2
uint8 AUTO_READY=3
uint8 FAULT=4
std_msgs/Header header
uint32 sequence
uint8 state
uint64 reason_mask
string source
string contract_id
string contract_sha256
bool enabled
bool manual_override_active
bool physical_estop_asserted
bool watchdog_verified
float32 heartbeat_age_s
float32 command_timeout_s
float32 measured_linear_mps
float32 measured_angular_rps
```
Only fresh AUTO_READY with consistent booleans may generate CLEAR; unavailable measured velocities are `-1.0`.

### `msg/SafetyState.msg`
```text
uint8 DISARMED=0
uint8 CLEAR=1
uint8 STOPPED=2
uint8 LATCHED=3
uint8 FAULT=4
std_msgs/Header header
uint32 sequence
uint8 state
uint64 reason_mask
bool armed
bool estop_latched
geometry_msgs/Twist requested_command
geometry_msgs/Twist output_command
float32 command_age_s
float32 intent_age_s
float32 geofence_age_s
float32 collision_age_s
float32 localization_age_s
float32 slope_age_s
float32 mode_age_s
float32 driver_age_s
uint32 deadline_miss_count
uint32 dropped_input_count
string release_manifest_sha256
```
Unknown ages are `-1.0` and preclude CLEAR. `header.stamp` is gate evaluation time in `base_footprint`.

### `action/ExecuteRoute.action`
```text
uint8 DIRECTION_OUTBOUND=1
uint8 DIRECTION_RETURN=2
string mission_id
string route_id
uint8 direction
string map_id
string map_sha256
string route_manifest_sha256
string safety_manifest_sha256
---
uint8 SUCCEEDED=0
uint8 REJECTED=1
uint8 CANCELED=2
uint8 ABORTED=3
uint8 FAULT=4
bool success
uint8 result_code
uint64 reason_mask
string message
---
wheelchair_interfaces/RouteProgress progress
wheelchair_interfaces/MissionState mission_state
```
A missing or mismatched hash is REJECTED before publishing ActiveRoute or a move_base goal. Cancellation emits HOLD before returning its result.
