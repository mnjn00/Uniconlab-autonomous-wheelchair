---
schema_version: 1
artifact_id: A01-adr-noetic-localization
owner: WP0 software architecture owner
reviewer_role: independent safety architect
status: accepted
source_plan_sha256: bd1b9454bc34f68714e6b986e80466535f817a42c25f662db0990adc79ca601e
provenance: approved RALPLAN Option A decision
---

# ADR: Noetic runtime localization with offline ROS 2/GLIM

## Decision

Use **Option A**:

1. The deployment NUC remains Ubuntu 20.04, ROS 1 Noetic, catkin, and a single native ROS 1 runtime graph.
2. A digest-pinned offline ROS 2 environment may read the ROS 2 sqlite bag, validate the exact Livox source IDL, produce the canonical ROS 1 bag, run GLIM, and export hash-pinned map/trajectory/provenance artifacts.
3. ROS 2 and GLIM are not runtime dependencies and cannot publish command, mode, safety, driver, or TF authority into the deployment graph.
4. Runtime uses exactly one qualified native Noetic localizer. Its `LocalizationCandidate` is untrusted. A separately owned localization guard alone publishes `/localization/status` and `/safety/localization`.
5. Exactly one selected Noetic localization adapter owns `map -> odom`. Duplicate authority, stale TF, policy/map mismatch, or uncertain localization is STOP and disarms motion.
6. The singular command topology remains `/cmd_vel_nav -> safety_gate -> /cmd_vel_safe -> verified hardware boundary`. Localization cannot bypass or weaken it.

## Supersession

The earlier Ubuntu 24.04 / ROS 2 Jazzy / Nav2 / Gazebo Harmonic / `ros_gz` / `gz_ros2_control` deployment direction is **superseded** by the later explicit Ubuntu 20.04 + ROS 1 Noetic constraint. It has no execution or release authority and cannot be revived for implementation convenience. A live ROS 2 bridge is not part of this release.

## Selection order

Qualify an existing `base_model` native localizer first if it satisfies the candidate ABI, single-TF ownership, independent confidence, reset, recovery, timing, and resource contracts. Otherwise qualify AMCL only with qualified 2D scan and wheel odometry. Otherwise evaluate native Noetic Cartographer localization. No candidate is selected by familiarity alone.

## Consequences

- Canonical bag conversion and map provenance are release artifacts.
- Current bags, maps, routes, loop closure, replay, and Gazebo outputs remain candidate/software evidence; none proves absolute localization or physical accuracy.
- One ROS clock domain and one localization TF authority reduce mixed-middleware failure modes.
- A simpler native runtime localizer is acceptable only when it passes frozen confidence, accuracy, replay, timing, and target-NUC gates.
- `hardware_motion_authorized` and `passenger_operation_authorized` remain false.

## Rejected alternatives

- **Option B, live ROS 2 GLIM sidecar/bridge:** rejected as default because clock, QoS, lifecycle, TF duplication, stale-pose, resource, and host-distro failures become runtime safety dependencies.
- **Option C, native Noetic wrapper around GLIM core:** deferred because it creates the largest dependency, maintenance, licensing, build, and resource burden.

## Revisit trigger

Revisit only through a new reviewed ADR after Option A fails frozen confidence, accuracy, timing, or target-NUC resource gates **after** source clocks, extrinsics, data defects, and policy defects are corrected. Threshold relaxation is not a valid trigger response.

## Rollback

Restore one previously signed release bundle containing binaries, native localizer, map, route-safety manifest, collision/slope/localization policies, and hashes. Restart DISARMED with all positive permissions UNKNOWN; require successful localization and a separate explicit arm. Never auto-resume a mission.
