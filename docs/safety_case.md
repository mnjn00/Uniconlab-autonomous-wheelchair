# Software-RC safety case

## Scope and authority

**Claim S0:** this repository may support a software release candidate for Ubuntu 20.04, ROS 1 Noetic simulation, replay, and read-only shadow operation. Evidence: hash-bound WP0 contracts, fail-closed software components, and the release manifest workflow.

**Claim S1:** autonomous commands have one guarded route: `/cmd_vel_nav` → safety gate → `/cmd_vel_safe`. Route/geofence, collision, slope, localization, topology, mode, driver, and e-stop authorities are independent inputs; missing, stale, malformed, mismatched, or unknown evidence yields finite zero and DISARMED.

**Claim S2:** offline ROS 2/Livox conversion and GLIM can establish conversion/replay consistency only. They cannot establish absolute localization, stopping, route, or physical safety.

**Authority boundary:** a software RC is **not** motor actuation, passenger, campus, road-traffic, curb/grade, wet-weather, braking-distance, or functional-safety approval. `hardware_motion_authorized`, `passenger_operation_authorized`, and `campus_operation_authorized` remain false. `hardware_enabled` must fail before a real motor endpoint exists. Unknown driver topic/type/MD5/sign/units/rate/timeout, physical manual/e-stop priority and latency, measured sensor extrinsics/time offsets, braking/stopping envelope, payload/battery/surface behavior, surveyed route, and target-NUC fingerprint/resources block hardware. No software, replay, Gazebo, workstation, map, or GLIM result can waive those gates.

## Hazards, controls, and remaining gates

| Hazard | Software claim/control | Residual fact that blocks physical authority |
|---|---|---|
| H01 forged/widened geofence | independent immutable route-safety owner; exact map/route/policy hashes; mismatch/outside/unknown stops | footprint, localization uncertainty, and route survey |
| H02 collision clear while blind/occluded/stale | conservative envelope, coverage/TTC evidence, stale and blind STOP, clear hysteresis | real blind zones, pedestrian behavior, reflectivity, calibrated braking |
| H03 Livox time/data corruption | source/IDL/hash/count/time checks, canonical record index, transactional output, deterministic verification | full source bag, actual source IDL/clocks and alignment evidence |
| H04 acceleration mistaken for slope | gravity/orientation agreement, stationary calibration, residual and stale STOP | vibration, slip, gravity alignment, grade/cross-slope stability |
| H05 confidently wrong localization | independent guard, map identity, residual/inlier/NIS/ambiguity checks, LOST/cancel/disarm | source bag has no odom/TF/truth; campus accuracy unmeasured |
| H06 CPU/thermal/I/O starvation | queue depth one, no safety-callback I/O, deadline/backpressure STOP | actual deployment NUC fingerprint and required profiles are absent |
| H07 wrong driver contract or held-last command | no default motor topic; exact verified manifest and graph required; hardware launch blocked | real API, polarity, units, timeout, mode, odom and stop behavior unknown |
| H08 e-stop/manual override failure | latched logical stop, guarded reset, no automatic re-arm | independent physical circuit/priority/latency unverified |
| H09 simulation/replay overclaim | every claim carries an evidence tag; promotion above evidence is rejected | cumulative HIL, closed-course, campus and passenger reviews absent |
| H10 mixed/unsurveyed map and route | hash-bound atomic bundle, candidate label, validation, refuse mission and roll back | datum, corridor, exclusions, crossings, directions and survey absent |
| H11 physical envelope exceeds simulation | simulation policies grant zero physical authority | stopping/stability across payload, battery, surface, temperature, grade and weather unknown |
| H12 ROS/NUC common-mode failure | process/clock/watchdog fault logic publishes zero | independently measured physical stop path and institutional review absent |

High or catastrophic hazards remain STOP/BLOCKED when evidence is missing. Controls do not reduce residual physical risk to an accepted level.

## State, stop, reset, and recovery claims

- Startup, process restart, clock reset, release install, and rollback begin DISARMED with positive permissions UNKNOWN.
- E-stop assertion latches stop and disarms. Loss of the e-stop source never clears the latch.
- Reset is a guarded request, not an arm request. It requires e-stop physically/logically clear, manual or auto-disabled state, stationary measured motion, mission cancelled, valid topology, and every other safety input clear. A rejected reset records `RESET_REJECTED` and remains latched.
- Arming is a separate explicit request and succeeds only on a later all-clear evaluation. No old mission resumes automatically.
- LOST localization cancels the goal and stops. Relocalization is stationary and explicit and returns READY/DISARMED.
- Mixed, unsigned, corrupt, partially installed, or hash-mismatched releases/maps/routes/policies are refused; rollback restores one complete prior bundle and remains DISARMED.

## Evidence and data governance

Normative limits are `contracts/wp0/A13-simulator-fidelity.yaml`, `A14-hazard-log.yaml`, `A15-evidence-inventory.yaml`, and `A16-release-authority.yaml`. `contracts/wp0/A17-verification-matrix.yaml` expects, among others:

- `evidence/contracts/abi-v1-report.json`
- `evidence/topology/command-graph-report.json`
- `evidence/route-safety/anti-widening-report.json`
- `evidence/safety/collision-ttc-report.json`, `slope-policy-report.json`, and `gate-permission-matrix.json`
- `evidence/localization/confidence-holdout-report.json`
- `evidence/conversion/determinism-and-corruption-report.json`
- `evidence/mission/fsm-contract-report.json`
- `evidence/performance/target-nuc-60min-report.json`
- `evidence/simulation/fidelity-claim-report.json`
- `evidence/release/rollback-drill-report.json`
- `evidence/hardware/hardware-gate-negative-report.json`

These are expected destinations, not observed results. A missing artifact remains pending/blocked and must not be reported as a pass. The target-NUC report is explicitly blocked by the unknown fingerprint; slope is simulation-policy-only; source conversion is blocked on source IDL/hash; hardware evidence is negative-gate-only. No test result or target-NUC result is asserted here.

The external Livox bag and user manual are not committed. Do not commit user data, source recordings, credentials, serial numbers, or a multi-GB bag. Commit only reviewed, minimal, non-sensitive manifests/reports and candidate map/route artifacts with provenance and hashes.
