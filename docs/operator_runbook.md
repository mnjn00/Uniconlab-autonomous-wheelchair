# Software-only operator runbook

All commands run from the repository root after a Noetic catkin build:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

This runbook has no physical-motion procedure. Never connect `/cmd_vel_safe` to an inferred base topic. Do not use `hardware_enabled`: the current driver manifest and release authority deliberately reject it before creating a motor endpoint.

## Startup profiles

### Simulation

```bash
roslaunch wheelchair_bringup sim_bringup.launch world:=sidewalk_obstacles
```

Other landed worlds are `empty`, `road_free_space`, and `static_dynamic_obstacles`. Gazebo owns `/clock`, and the only controller relay is `/cmd_vel_safe` → `/wheelchair_base_controller/cmd_vel` inside simulation.

### Replay

In terminal 1, set simulated time before nodes start and select the inert replay profile:

```bash
roslaunch wheelchair_bringup bringup.launch use_sim_time:=true hardware_profile:=replay
```

In terminal 2, start the verified normalized ROS 1 bag paused; this is the sole replay clock publisher:

```bash
rosbag play --clock --pause ARTIFACT/normalized.bag
```

Inspect state first, then press Space in the `rosbag play` terminal. Never loop or seek backward during a qualification run. A clock reset/regression disarms and requires a clean restart and explicit re-arm. The normalized bag contains no `/clock` topic.

### Hardware-disabled/read-only shadow

```bash
roslaunch wheelchair_bringup bringup.launch hardware_profile:=hardware_shadow hardware_enable:=false
```

The default is `src/wheelchair_hardware/config/driver-unverified.yaml`. The shadow adapter only observes `/cmd_vel_safe` and must not create a real driver publisher. Current code publishes its status on `/driver_status`, while the safety gate contract consumes `/hardware/driver_status`; therefore shadow remains fail-closed/DISARMED until that mismatch is corrected. Do not add an ad-hoc relay or remap.

## Preflight and observation

Before any software scenario:

1. Confirm the intended profile and clock owner; do not switch `/use_sim_time` after startup.
2. Confirm `/cmd_vel_nav` has only `move_base` as producer and `/cmd_vel_safe` only `safety_gate`. Confirm no real driver topic/node exists in sim, replay, or shadow.
3. Confirm exactly one TF owner for `map`→`odom` and `odom`→`base_footprint`.
4. Confirm `/safety/state` is DISARMED, output command is finite zero, and inspect `reason_mask`, all input ages, deadline/drop counters, `estop_latched`, and release hash:

```bash
rostopic echo -n 1 /safety/state
rostopic echo -n 1 /diagnostics
rostopic info /cmd_vel_nav
rostopic info /cmd_vel_safe
rosnode list
rosrun tf tf_monitor
```

5. Confirm map, route, policy, driver, and release hashes match the selected bundle. Missing/empty/mixed hashes are STOP, not an operator override.
6. In replay, confirm only `rosbag play --clock` publishes `/clock`. In simulation, confirm Gazebo is the clock owner.

Unknown driver, measured extrinsics/time offsets, braking/stopping envelope, actual NUC fingerprint/resources, physical manual/e-stop behavior, route survey, or campus approval blocks hardware. A developer workstation cannot substitute for the target NUC.

## Arm, stop, reset, and manual handling

These ROS requests are for simulation/replay software exercises only; never use them as a physical operating procedure.

To request arm after every required structured safety signal, topology, driver/mode state, localization, route state, and command stream is fresh and CLEAR:

```bash
rostopic pub -1 /safety/arm std_msgs/Bool 'data: true'
rostopic echo -n 1 /safety/state
```

Acceptance is shown only by `armed: true` with reason mask zero. A request does not force authority.

To inject a software e-stop and verify the latch/zero:

```bash
rostopic pub -1 /safety/estop std_msgs/Bool 'data: true'
rostopic echo -n 1 /safety/state
```

Do not reset until the mission is cancelled, the source reports e-stop false, driver state is MANUAL or AUTO_DISABLED, measured linear/angular motion is stationary, topology is valid, and every other safety input is CLEAR. Then issue distinct events:

```bash
rostopic pub -1 /safety/mission_cancelled std_msgs/Bool 'data: true'
rostopic pub -1 /safety/estop std_msgs/Bool 'data: false'
rostopic pub -1 /safety/estop_reset std_msgs/Bool 'data: true'
rostopic echo -n 1 /safety/state
```

A successful reset remains DISARMED. `RESET_REJECTED` means leave the latch set, inspect the failed prerequisite, and fix the source; never repeatedly publish reset. Arm, if appropriate for the software scenario, is a separate later request. Manual/joystick state always disarms autonomy; do not publish synthetic mode/driver messages to defeat it. Physical manual priority, e-stop circuit, and latency are unknown and block hardware.

## Fault triage

1. Treat nonzero `reason_mask`, `estop_latched`, missing state, stale age, clock jump, TF duplicate, deadline/backpressure count, or nonzero output after a stop as a blocking fault.
2. Cancel the mission; do not restart or re-arm while the cause is present.
3. Use `contracts/wp0/A04-safety-reason-registry.yaml` to decode the mask. Inspect the matching status topic and `/diagnostics`; preserve source/evaluation timestamps, sequence, reason, policy/release hashes, and graph identity.
4. For localization LOST: cancel, remain stationary, perform the explicit localization reset/relocalization workflow, and require READY/DISARMED. Never resume the old goal.
5. For graph/hash/map/route/policy mismatch: stop the profile and restore one complete bundle. Never hot-reload safety policy while armed.
6. For recorder backpressure or resource/deadline faults: stop non-safety workload or roll back. Never widen TTLs, speed caps, or queues.
7. For any suspected physical/manual/e-stop/driver fault: keep hardware disabled. Software evidence cannot clear it.

## Incident export

The separate bounded recorder writes `incident-<trigger_timestamp_ns>-<event>-<sequence>.json` to the configured `output_dir` in `src/wheelchair_bringup/config/observability.yaml`; it is not part of the safety callback and grants no authority. Export an existing immutable incident JSON to a dedicated evidence directory outside user bag storage, then audit it:

```bash
python3 scripts/audit_incident_evidence.py EVIDENCE/incident-<trigger_timestamp_ns>-<event>-<sequence>.json --report EVIDENCE/audit-report.json
```

Retain the incident JSON and `audit-report.json` together. A missing/truncated file, hash mismatch, non-monotonic timestamps, unbounded payload, missing pre/post window, or audit failure is a blocker. Do not edit evidence, include secrets/serial numbers, or commit user data or multi-GB bags.

## Release install and atomic rollback

Only the complete AC0-AC6 matrix can create a clean software-only manifest. Ordinary CI pytest/build JUnits are incomplete software evidence, never release reports; missing full-bag, target-NUC, live, hardware, or passenger gates produce no clean release. `hardwareMotionAuthorized` and `passengerOperationAuthorized` remain `false` in every gate report and manifest.

Create one strict JSON gate report at each exact path below. Each report has exactly `artifactType`, `schemaVersion`, `gateId`, `status`, `claimTag`, `hardwareMotionAuthorized`, `passengerOperationAuthorized`, `sourceRevision`, `configurationDigest`, `bundleDigest`, `releaseInputDigest`, and `result`; its type/version is `wheelchair-ac-gate-report`/`1`, claim is `SOFTWARE_ONLY`, status is `PASS`, both authority flags are `false`, and `result` is `{"passed":true,"cases":N}` for `N > 0`.

```bash
REPORTS=(
  evidence/contracts/abi-v1-report.json
  evidence/topology/command-graph-report.json
  evidence/route-safety/anti-widening-report.json
  evidence/safety/collision-ttc-report.json
  evidence/safety/slope-policy-report.json
  evidence/localization/confidence-holdout-report.json
  evidence/conversion/determinism-and-corruption-report.json
  evidence/mission/fsm-contract-report.json
  evidence/safety/gate-permission-matrix.json
  evidence/performance/target-nuc-60min-report.json
  evidence/simulation/fidelity-claim-report.json
  evidence/release/rollback-drill-report.json
  evidence/hardware/hardware-gate-negative-report.json
  evidence/release/passenger-authority-negative-report.json
)
```

Provide a separately verifier-approved parent rollback binding in `parent-rollback.json`; free-form parent IDs and `parent_state` are rejected. It has exactly this shape (all `sha256` values are lowercase 64-hex strings, every inventory list is non-empty, sorted, and has unique paths):

```json
{
  "parentReleaseBindingSha256": "<target-release-binding>",
  "parentReleaseBindingReceiptSha256": "<sha256 of canonical parentReleaseBindingSha256+inventory>",
  "inventory": {
    "binaries": [{"path": "binaries/item", "sha256": "<sha256>"}],
    "maps": [{"path": "maps/item", "sha256": "<sha256>"}],
    "routes": [{"path": "routes/item", "sha256": "<sha256>"}],
    "policies": [{"path": "policies/item", "sha256": "<sha256>"}],
    "drivers": [{"path": "drivers/item", "sha256": "<sha256>"}]
  },
  "restartReceipt": {
    "state": "DISARMED",
    "permissions": "UNKNOWN",
    "localizationRequired": true,
    "missionResume": false,
    "parentReleaseBindingSha256": "<target-release-binding>",
    "inventoryDigest": "<sha256 of canonical inventory>"
  }
}
```

Generate and independently verify only after all exact reports and that receipt exist inside the release root:

```bash
python3 scripts/generate_release_manifest.py --root RELEASE --output RELEASE/release-manifest.json \
  $(printf -- ' --report %s' "${REPORTS[@]}") \
  --rollback-parent "$(<RELEASE/parent-rollback.json)" \
  --blocker 'hardware motion qualification has not passed' \
  --blocker 'passenger operation qualification has not passed'
python3 scripts/verify_release_manifest.py RELEASE/release-manifest.json --root RELEASE
```

Install is dry-run unless `--apply` is supplied:

```bash
python3 scripts/install_noetic_rc.py --manifest RELEASE/release-manifest.json --source RELEASE --prefix /tmp/wheelchair-rc-sandbox
python3 scripts/install_noetic_rc.py --manifest RELEASE/release-manifest.json --source RELEASE --prefix /tmp/wheelchair-rc-sandbox --apply
```

Before a non-idempotent rollback, stop the graph and write separate current-state evidence; this is not the parent restart receipt and cannot be the string `DISARMED`:

```json
{
  "state": "DISARMED",
  "permissions": "UNKNOWN",
  "localizationRequired": true,
  "missionResume": false,
  "currentReleaseBindingSha256": "<current-release-binding>",
  "targetReleaseBindingSha256": "<target-release-binding>",
  "hardwareMotionAuthorized": false,
  "hardwareEnabled": false,
  "evidenceBindingSha256": "<sha256 of this object excluding evidenceBindingSha256>"
}
```

```bash
python3 scripts/rollback_noetic_rc.py --prefix /tmp/wheelchair-rc-sandbox \
  --target TARGET_RELEASE_BINDING_SHA256 --disarmed-evidence current-state.json
python3 scripts/rollback_noetic_rc.py --prefix /tmp/wheelchair-rc-sandbox \
  --target TARGET_RELEASE_BINDING_SHA256 --disarmed-evidence current-state.json --apply
```

Rollback verifies both manifests, the target release ID against `parentReleaseBindingSha256`, the hash-bound inventory and receipt, and separate current-state DISARMED/UNKNOWN/no-resume evidence before changing `current`. It restarts only DISARMED with permissions UNKNOWN; localization must be re-qualified and missions never auto-resume. Neither tool modifies drivers, ROS/system services, bag data, armed state, or paths outside the selected prefix.
