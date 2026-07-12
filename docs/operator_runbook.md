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

Only the complete AC0-AC6 matrix can create an authoritative software-only manifest. Ordinary CI pytest/build JUnits are incomplete software evidence, never gate reports. Simulation, replay, CI, and generated evidence do not grant hardware-motion or passenger-operation authority; missing external full-bag, live Gazebo, target-NUC, hardware, or passenger qualification fails closed.

Every gate report is a strict `wheelchair-ac-gate-report` schema version `2` object with exactly these top-level keys:

```json
{
  "artifactType": "wheelchair-ac-gate-report",
  "schemaVersion": 2,
  "gateId": "WP0-ABI-001",
  "status": "PASS",
  "claimTag": "SOFTWARE_ONLY",
  "hardwareMotionAuthorized": false,
  "passengerOperationAuthorized": false,
  "sourceRevision": "<prepared sourceRevision>",
  "configurationDigest": "<prepared configurationDigest>",
  "bundleDigest": "<prepared bundleDigest>",
  "releaseInputDigest": "<prepared releaseInputDigest>",
  "result": {
    "passed": true,
    "executedCommands": ["<non-empty command actually run>"],
    "environment": {"os": "<non-empty OS>", "architecture": "<non-empty architecture>"},
    "tool": {"name": "<non-empty tool name>", "version": "<non-empty tool version>"},
    "durationSeconds": 1,
    "metrics": {"interfacesChecked": 1},
    "invariants": {"abiCompatible": true},
    "artifacts": [{"path": "evidence/raw/abi-output.txt", "sha256": "<lowercase-64-hex>"}]
  }
}
```

`durationSeconds` and the sole gate metric must be positive. `executedCommands` and `artifacts` must be non-empty; artifact references are `{path, sha256}`, hash-verified against files below the release root, unique, and sorted lexicographically by `path`. For each report, replace the example metric and invariant with that gate's exact pair below; no extra result fields are allowed.

Before generating reports, prepare the bindings from the release root. Keep `BINDINGS` outside `RELEASE`; report generators copy its four values exactly into every report.

```bash
umask 077
BINDINGS="$(mktemp "${TMPDIR:-/tmp}/wheelchair-release-bindings.XXXXXX")"
python3 scripts/generate_release_manifest.py --root RELEASE --prepare-bindings --bindings-output "$BINDINGS"
```

`evidence/**` is excluded only from Git source-dirtiness detection and the release-input digest to avoid release-input circularity. It is still hashed in `qualification_evidence`, and every required report and every referenced artifact remains hash-bound qualification evidence in the manifest. Do not use a JUnit as a substitute for one of these reports.

```bash
REPORTS=(
  evidence/contracts/abi-v1-report.json
  evidence/topology/command-graph-report.json
  evidence/route-safety/anti-widening-report.json
  evidence/safety/collision-ttc-report.json
  evidence/safety/slope-policy-report.json
  evidence/safety/gate-permission-matrix.json
  evidence/conversion/determinism-and-corruption-report.json
  evidence/localization/confidence-holdout-report.json
  evidence/localization/glim-offline-input-report.json
  evidence/localization/glim-offline-reproduction-report.json
  evidence/localization/glim-offline-comparison-report.json
  evidence/mission/fsm-contract-report.json
  evidence/performance/target-nuc-60min-report.json
  evidence/simulation/fidelity-claim-report.json
  evidence/release/rollback-drill-report.json
  evidence/hardware/hardware-gate-negative-report.json
  evidence/release/passenger-authority-negative-report.json
)
```

| Gate | Metric | Invariant |
| --- | --- | --- |
| `WP0-ABI-001` | `interfacesChecked` | `abiCompatible` |
| `WP1-TOPOLOGY-001` | `commandPathsChecked` | `singleCommandPath` |
| `WP1-GEOFENCE-001` | `routeBoundsChecked` | `routeNotWidened` |
| `WP1-COLLISION-001` | `collisionScenarios` | `ttcStopsEnforced` |
| `WP1-SLOPE-001` | `slopeScenarios` | `slopePolicyEnforced` |
| `WP1-CONTROL-001` | `permissionCases` | `unauthorizedCommandsDenied` |
| `WP2-CONVERSION-001` | `conversionCases` | `deterministicAndCorruptionSafe` |
| `WP3-LOCALIZATION-001` | `holdoutFrames` | `lowConfidenceHeld` |
| `WP3-GLIM-INPUT-001` | `offlineInputFrames` | `offlineInputPinned` |
| `WP3-GLIM-REPRODUCTION-001` | `reproductionRuns` | `offlineReproducible` |
| `WP3-GLIM-COMPARISON-001` | `comparisonFrames` | `offlineComparisonWithinTolerance` |
| `WP4-MISSION-001` | `fsmTransitions` | `missionContractEnforced` |
| `WP6-TIMING-001` | `measuredSeconds` | `targetNucDurationMet` |
| `WP6-SIMCLAIM-001` | `simulationCases` | `simulationClaimBounded` |
| `WP6-ROLLBACK-001` | `rollbackDrills` | `rollbackDisarmed` |
| `WP0-HWGATE-NEG-001` | `deniedHardwareRequests` | `hardwareMotionDenied` |
| `WP0-PASSENGER-NEG-001` | `deniedPassengerRequests` | `passengerOperationDenied` |

The parent rollback argument is a strict reference, not an inline inventory. All hashes are lowercase 64-hex. `parentManifestPath` is a non-empty safe relative path (not absolute and containing no `..`) below the installed parent release root. `parentInventoryDigest` is the canonical digest of the parent's complete manifest `hashes` object.

```json
{
  "parentReleaseBindingSha256": "<parent-release-binding>",
  "parentManifestSha256": "<sha256 of parent manifest file>",
  "parentManifestPath": "release-manifest.json",
  "parentInventoryDigest": "<canonical sha256 of parent manifest hashes>",
  "restartReceipt": {
    "path": "receipts/restart-disarmed.json",
    "sha256": "<sha256 of referenced receipt file>",
    "parentReleaseBindingSha256": "<parent-release-binding>",
    "parentInventoryDigest": "<same parent inventory digest>"
  }
}
```

The referenced restart receipt exists under that parent root, hashes to `restartReceipt.sha256`, and has exactly this fail-closed shape:

```json
{
  "state": "DISARMED",
  "permissions": "UNKNOWN",
  "localizationRequired": true,
  "missionResume": false,
  "parentReleaseBindingSha256": "<parent-release-binding>",
  "parentInventoryDigest": "<parent inventory digest>",
  "hardwareMotionAuthorized": false,
  "passengerOperationAuthorized": false
}
```

An authoritative manifest requires a clean Git source revision of exactly 40 lowercase hexadecimal characters and an explicit non-empty HMAC signing-key file outside `RELEASE`. The key file is required for generation, independent verification, install dry-run and apply, and rollback dry-run and apply. A missing, empty, or wrong key is a hard refusal. Never put the key or its contents in artifacts, logs, or the repository.

```bash
umask 077
KEY_FILE="$(mktemp "${TMPDIR:-/tmp}/wheelchair-release-key.XXXXXX")"
trap 'rm -f "$KEY_FILE" "$BINDINGS"' EXIT
# Write the non-empty key from the approved secret manager without echoing it.
# The key file must remain outside RELEASE and mode 0600.

python3 scripts/generate_release_manifest.py --root RELEASE --output RELEASE/release-manifest.json \
  $(printf -- ' --report %s' "${REPORTS[@]}") \
  --rollback-parent "$(<RELEASE/parent-rollback.json)" \
  --blocker hardware_motion_unqualified \
  --blocker passenger_operation_unqualified \
  --release-signing-key "$KEY_FILE"

python3 scripts/verify_release_manifest.py RELEASE/release-manifest.json --root RELEASE \
  --release-signing-key "$KEY_FILE"

python3 scripts/install_noetic_rc.py --manifest RELEASE/release-manifest.json --source RELEASE \
  --prefix /tmp/wheelchair-rc-sandbox --release-signing-key "$KEY_FILE"
python3 scripts/install_noetic_rc.py --manifest RELEASE/release-manifest.json --source RELEASE \
  --prefix /tmp/wheelchair-rc-sandbox --release-signing-key "$KEY_FILE" --apply
```

Before a non-idempotent rollback, stop the graph and write separate current-state evidence; it is not the parent restart receipt and cannot be the string `DISARMED`:

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
  --target TARGET_RELEASE_BINDING_SHA256 --disarmed-evidence current-state.json \
  --release-signing-key "$KEY_FILE"
python3 scripts/rollback_noetic_rc.py --prefix /tmp/wheelchair-rc-sandbox \
  --target TARGET_RELEASE_BINDING_SHA256 --disarmed-evidence current-state.json \
  --release-signing-key "$KEY_FILE" --apply
```

Rollback independently verifies both signed manifests, the target binding, safe parent-manifest reference and hash, parent inventory digest, restart receipt reference/hash/bindings, and separate current-state DISARMED/UNKNOWN/no-resume evidence before changing `current`. It restarts only DISARMED with permissions UNKNOWN; localization must be re-qualified and missions never auto-resume. Neither tool modifies drivers, ROS/system services, bag data, armed state, or paths outside the selected prefix.
