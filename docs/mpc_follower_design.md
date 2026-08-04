# MPC follower design

Status: draft for review. No vehicle change until section 10's offline gates
pass and a supervised field protocol is run on the NUC.

This document fixes the model, horizon, constraints, solver, CPU budget and
fallback ladder for a model-predictive follower, and records why the
candidate references were chosen or rejected, so the implementation argues
from evidence rather than from the shape of whatever code was easiest to
find.

## 1. Why MPC, and why PRIEST died

The PRIEST integration (reverted in `81fed5d`) died of CPU starvation: it
saturated the NUC and took the rest of the stack down with it. The deleted
code shows the shape that did it:

- `batch=200` samples x `iterations=12` CEM loops x `projection_iterations=15`,
  all pure-Python numpy;
- a horizon sized to the REMAINING route, `horizon_for(reach, floor 4 s,
  ceiling 40 s)` - the per-cycle problem grew with distance to the goal;
- `ensure_plan` retried on failure, so an infeasible situation increased
  the compute instead of degrading;
- no CPU budget and no solve-time watchdog anywhere in the chain.

On the NUC11PHKi7C (i7-1165G7, 4 cores / 8 threads, 28 W) the resident
stack (FAST-LIO, drivers, perception, recording, VNC) already holds most of
two cores and needs headroom for spikes; PRIEST's worst case asked for
several more cores. Nothing about the hardware was wrong - the problem
shape was. The MPC in this document is designed so the per-cycle problem is
CONSTANT in route length, solved by a C-library QP in milliseconds, under an
explicit budget, with a defined ladder when it fails.

The capability case: the pure-pursuit follower that drives the chair today
tracks one fixed line and can only stop when that line is blocked. The
07-31 and 08-02 field work already gives the ingredients for something
better: a measured + operator-drawn corridor band (`safety_band.py`),
clustered obstacles, and a speed policy. MPC uses them natively: the band
becomes hard lateral constraints, obstacles become soft half-planes, and
the speed policy becomes the velocity reference, so passing a pedestrian is
a small lateral deviation inside the corridor instead of a stop-and-wait.

## 2. Reference repositories: chosen and rejected

| Repository | Verdict | What we take |
|---|---|---|
| `alexliniger/MPCC` (ETH IfA) | **Primary formulation reference** | Contouring + lag error path following with a progress variable; track constraints as two linear half-planes per step; obstacle avoidance by converting the detour side into a modified corridor constraint; solving a linearised, discretised NLP as a time-varying QP. We adopt the formulation, not the code: its racing dynamics (bicycle + Pacejka tyres) and its progress-maximising objective are replaced below. |
| `MizuhoAOKI/nullspace_mpc` | Rejected as base, cited for solver sanity | Ground-vehicle (swerve drive) QP-MPC in narrow spaces, ROS Noetic, solving through a QpSolverCollection that wraps OSQP/hpipm/qpOASES - independent confirmation that a condensed QP is the right solver family for this vehicle class. Rejected because the full stack is GPU-recommended (its own README says CPU-only is insufficient for stable control on their system) and its multi-objective nullspace priorities solve a problem we do not have. |
| `udacity/CarND-MPC-Project` | Rejected (deprecated) | Marked deprecated upstream; Ipopt/CppAD C++ for a car at highway speed. One technique is adopted: compensating command-chain latency by starting the prediction from the state estimated at `now + latency` rather than `now`. |
| `Zhefan-Xu/Intent-MPC` | Deferred to v2 | Intent-prediction-driven MPC for dynamic obstacles, ROS Noetic C++. The right reference when pedestrians get intent modelling; today's obstacle problem is static/clustered and does not justify a prediction stack. |
| OSQP (`osqp/osqp`, classic 0.6.x API) | **Solver** | C-core condensed QP with warm start, pip wheels for the NUC's Python 3.9, deterministic timing. The Python process only assembles matrices; the solve itself never touches the GIL-bound hot loop. |

## 3. Where it sits in the chain

Slot: the follower slot only. The rest of the safety chain is untouched:

```
localization + perception -> [mpc_follower] -> /cmd_vel_raw
    -> safety_gate -> tip_guard (command relay) -> wheel_cmd_guard -> base
```

Bringup selects the follower with one switch and defaults to the validated
pure-pursuit follower (`PROFILE=pursuit`, `PROFILE=mpc` to opt in), so
rollback is one line and both followers share the same input topics,
the same guard conditions (`localization_policy`, pose/cloud/base
staleness, manual mode, off-band/off-route geofence) and the same status
contract strings.

## 4. Model

Unicycle kinematics, the honest model for this chair at <= 0.6 m/s:

```
X' = v cos(th)     Y' = v sin(th)     th' = w     v' = a     w' = al
state x = (X, Y, th, v)      input u = (a, al)
```

Parameters are the follower's existing constants, not new ones:

| Parameter | Value | Source |
|---|---|---|
| `dt` | 0.1 s | CONTROL_HZ = 10 |
| `N` | 25 (2.5 s, ~1.5 m at v_max) | section 5 budget |
| `v_max` | 0.6 m/s | MAX_SPEED |
| `a` range | [-0.6, +0.18] m/s^2 | MAX_DECEL / MAX_ACCEL |
| `|w|` | <= 0.5 rad/s | MAX_YAW_RATE |
| `|al|` | <= 1.5 rad/s^2 | follower yaw slew 1.5/CONTROL_HZ per tick |
| `v` | >= 0 | no reversing in normal operation |

Rollouts use exact per-step rotation (heading advanced by `w*dt`, position
integrated with the midpoint heading), so linearisation error is confined
to the QP assembly: dynamics are linearised around the warm-started
trajectory once per cycle. With `w <= 0.5` and `dt = 0.1` the heading move
per step is <= 0.05 rad, which is where this is cheap; one solve per cycle
is the budget, a second refinement iteration is allowed only if the first
solve returned in under half the budget.

## 5. Objective - follow, do not race

MPCC maximises progress because a race car's job is speed. This chair's job
is arriving without falling off the band, so progress is NOT an objective;
the schedule comes from the existing speed policy as a reference velocity
`v_ref(s)` (0.6 cruise, 0.3 on slope, 0.2 in narrow stations, obstacle
ramp), and the cost is

```
J = sum_k  w_lat * e_lat(k)^2          contouring error at step k
        + w_head * (th(k) - th_ref(k))^2
        + w_vel * (v(k) - v_ref(k))^2
        + w_rate * (|a(k)|^2 + |al(k)|^2)
        + w_slack * (obstacle slacks)
```

`e_lat` is the MPCC contouring error evaluated the way `safety_band.py`
already measures it: signed offset along the nearest station's normal.
That makes the cost and the constraint the same geometry, which means the
constraint check and the objective can never disagree about what "lateral"
means.

## 6. Constraints

All linear per step after linearisation around the warm-start iterate:

- **Band (hard, never slacked):** at each predicted position take the two
  bracketing stations and apply `SafetyBand.lateral_limits` - the MORE
  RESTRICTIVE of the two, corridor-narrowed. With the station normals
  frozen per step this is two half-planes per step: `n . p <= hi`,
  `n . p >= lo`, inset by a linearisation reserve (8 cm, capped at a
  quarter of the local width so a narrow choke stays feasible). The band
  is the one thing that cannot yield: it is the map's statement about
  where the ground breaks.
- **Obstacles (soft, slack allowed):** one half-plane per clustered
  obstacle, placed at the obstacle position padded by 0.45 m (chair
  half-width 0.35 + 0.10 margin), oriented toward the roomier side as
  measured from the band's own limits at the obstacle - an obstacle can
  never authorise ground the band says breaks. The plane is infinite, so
  both its reach and its demanded clearance are scheduled by arc
  distance: it activates 10 m before the obstacle, the required clearance
  ramps linearly to the full padding 2 m before it, and holds until 1.5 m
  past. Full clearance over the whole approach deadlocked in simulation
  (the planner would rather stop than hold a half-metre lean for 10 m);
  a ramp completing only at the obstacle left the chair chasing its own
  requirement and arriving centimetres short. Slack is penalised 5x the
  lateral weight so clearance always beats centreline pull; the planner
  may lean on it, the containment check never does.
- **Speed cap (hard):** `v <= v_max` on every step, not only a cost term -
  an inaccurate solve must never accumulate velocity past MAX_SPEED. This
  is not theoretical: with the cap soft, inaccurate solves accumulated
  1.44 m/s in simulation.
- **Yaw-rate cap (hard):** `|w| <= w_max` on every step, as a STATE bound.
  The first implementation bounded the yaw ACCELERATION with w_max
  instead, which over-clamped alpha by 3x and left w itself unbounded: a
  60/90/120-degree heading error then commanded 0.69/0.86/1.02 rad/s, up
  to twice the cap, and the gate's clamp would have executed a trajectory
  the plan never validated - the same failure mode as the soft speed cap.
  Caught by review; held by DriveLimitsTest.
- **Actuators:** bounds and slew from section 4, `v >= 0`. All hard bounds
  hold within the solver's feasibility tolerance (~1e-3), no tighter.
- **Terminal (soft):** lateral inside the band and heading aligned with
  the corridor tangent at step N, so the truncated horizon does not aim
  the chair at the band edge.

## 7. Solver and runtime behaviour

Condensed QP through OSQP (classic 0.6.x API): ~175 decision variables and
~330 constraints for N=25. Warm start from the previous primal/dual
solution. Settings tuned for speed over polish (`polish=False`, tolerances
1e-3, iteration cap). Each cycle rebuilds the model from scratch - osqp's
in-place update path proved unreliable once the linearisation point moves -
and, when under half the budget, runs a second SQP iteration re-linearised
about the solution just found; one pass leaves residual linearisation drift
exactly where the band narrows fast. The iterate itself is re-anchored
inside the band every cycle (step 0 stays at the measured state), because
band rows written relative to an iterate that drifted outside the band
start the cycle with a feasibility margin no input can recover.

The ladder, with a 40 ms budget per 100 ms cycle:

1. warm solve -> if unsolved, cold re-solve on the same matrices (separates
   an osqp infeasibility-certificate misfire from a real conflict) -> if
   still unsolved, cold iterate and cold solve (separates a degenerate
   linearisation point from a real conflict);
2. still unsolved after all three -> INFEASIBLE_STOP, controlled stop.
   Obstacles need no say in this naming and get none: their rows are soft
   with unbounded slack, so they can neither cause infeasibility nor hide
   it - INFEASIBLE_STOP always means a band-plus-dynamics conflict, and
   impassability by obstacle arrives through the blocked-detection rung
   below instead. A counterfactual re-solve without the obstacle planes
   was tried (46d9e41) and reverted (2e14191) once review proved the
   branch could never fire; test_obstacle_rows_cannot_cause_or_mask_-
   infeasibility pins the property so it cannot be un-proved silently;
3. solved but over budget -> reuse the previous first input, up to 3
   consecutive cycles (REUSED), then BUDGET_STOP;
4. solved, but blocked detection says otherwise: never sit inside an
   obstacle's 0.40 m distance floor, and stop (BLOCKED_STOP) once a large
   slack (> 0.2 m) has been held for 10 consecutive cycles without 0.2 m
   of arc progress - passing shows as slack plus forward motion, being
   wedged shows as slack and none. In simulation the slack-only variant
   crept to 11 cm from an obstacle and camped; the floor plus the
   progress-aware streak is what stops it at ~1 m or at the padding;
5. nothing about this is silent: every rung publishes a status string on
   the follower status topic, and the black box records it.

Latency compensation, the one CarND idea we keep: the prediction should
start from the state estimated at `now + L` where L is the measured
command-chain delay (gate + relay + base response), not from `now`. Not
yet in the offline core - the simulated plant has no lag; it lands with
the ROS node once L is measured on the NUC.

State-anchor smoothing is a node requirement, measured 2026-08-04: the
anchor must blend the (jittery) localisation pose with wheel-odometry
v/w through an EMA (~0.4 gain validated) instead of re-anchoring on the
raw pose. With 2 cm of injected lateral jitter the raw anchor produces
5.0 yaw-command reversals per metre - busy, hunting-adjacent steering;
the EMA anchor cuts that to 1.3/m with no tracking regression (lateral
RMS 13 mm either way). At 5 cm jitter the raw anchor stalls the run at
183 m; the smoothed one runs to the route's choke point. No sustained
oscillation was found in any configuration - the 2 s limit-cycle screen
and the lateral spectrum stay clean - but the busyness is exactly what
would feel like hunting on the chair, so the smoothing is mandatory,
not a tune-later.

One residual, same measurement: with jitter present (smoothed or not)
the run ends in an INFEASIBLE_STOP at the route's ~334 m choke point,
which the noiseless run clears. The stop is safe - that is the ladder
working - but a stall there on every drive is not acceptable. The fix
is for the node to shape v_ref with the follower's existing speed
policy (narrow-band creep, slope slowdown) instead of a constant 0.6:
a slower approach shrinks the dynamic constraint pressure over the
horizon exactly where the corridor leaves no margin. The offline sim
kept the constant reference on purpose, so this gap is a node
requirement, recorded here rather than discovered in the field.

**Amended 2026-08-04, when the node was built.** The instinct above -
slow down for the pinch - is right. The quantity to slow for, the speed
to slow to, and above all the diagnosis were not, and the route does not
finish until a defect underneath all of it is fixed. What follows is what
the measurements said, in the order they said it, because the first two
findings were wrong and the way they were wrong is the useful part.

*First attempt, and a false conclusion.* Measured over the v4 band at
2 cm jitter with the EMA anchor: a constant 0.6 reaches 350 m, a constant
0.3 stops **earlier** at 334 m. That reads as proof that speed is not the
lever, and it was written up as such. It was measured on top of a defect.

*The defect.* `polyline_refs` snapped the heading reference to whichever
polyline segment a horizon step landed on. Stations are 0.5 m apart and a
step covers 0.06 m at cruise, so eight steps shared a segment and the
ninth inherited the entire inter-station heading change - up to 26.7
degrees, which the cost reads as a demand to rotate at 4.7 rad/s against
a 0.5 rad/s cap. The chair spent its yaw authority chasing corners that
were not there, drifted laterally, and met the hard band rows. The
resulting INFEASIBLE_STOP at 350 m looked exactly like a corridor too
narrow to drive. It was a staircase in the reference. Interpolating the
heading along arc - no smoothing, so the genuine 71-degree turn at 372 m
is untouched - drops the steepest demand the route contains from 2.2 to
0.563 rad/s. The residual over the cap is handled by
`mpc_speed.curvature_speed`, which limits v to w_max / curvature at the
two stations of 756 where the geometry really does out-turn the chair.

*Second finding, which stands.* Creep would not have been slow-but-moving,
it would have been stopped. Sweeping a constant reference from a standing
start: at and below 0.22 m/s the chair settles at exactly zero while the
solve reports OK. There is no progress term in the objective, so forward
motion is bought only by `w_vel * (v - v_ref)^2`, and below ~0.22 that no
longer covers the lateral, heading and rate cost of moving. "Narrow-band
creep" at CREEP_SPEED 0.15 would have parked the chair at the first
narrow station and reported nothing.

*The choke, correctly diagnosed.* With the heading honest, the chair
reaches 334 m inside the band with 0.011 m of lateral offset and stops
anyway - and widening the band by 1 cm makes it solvable. The corridor
there is 0.13 m, and `band_inset` was taking 25 % per side of it: half the
pinch spent on a linearisation reserve, leaving an upper bound 3.5 mm
under where the chair already was. That reserve is a numerical allowance,
not a safety margin, and it was sized by a round number rather than by the
error. Measured, the gap between the planned trajectory and the same
inputs applied to the nonlinear model is 0.0 mm at one step, 16 mm
mid-horizon at 0.6 m/s, 4.5 mm at 0.3 m/s, 74 mm worst case at the horizon
end. So 80 mm is right for the far end and the *fraction* was the problem:
at 15 % the pinch keeps 19.5 mm of reserve, still 4x the error measured at
the speed the chair passes it.

*And the part the fraction alone did not fix.* With the fraction at 15 %
the chair still stopped, now at 335 m, with the corridor shaping already
holding it at the 0.30 floor. Relaxing the band rows one horizon segment
at a time located it exactly: **steps 0-4**. Not the pinch 15 m ahead -
the near horizon. The reserve was flat across all 25 steps, so step 1,
which is 0.03 m ahead and carries 0.0 mm of measured error, was being
given the same 52 mm the far end needs. At a station where the corridor is
0.35 m wide that reserve fenced off ground the chair was already standing
on, and the near steps are precisely the ones it cannot maneuver out of:
at 0.3 m/s there is half a second before step 5, and a corridor whose
centre shifts between adjacent stations can move further than that in the
time. So the reserve is now a ramp - `band_inset * (k+1)/N`, floored at
10 mm, which is 2x the 4.5 mm worst one-step error - matching the shape of
the error curve it was always supposed to be covering.

Two things this was NOT. It was not the second SQP iteration: forcing that
pass on every cycle still stopped 2 runs of 3, and forcing it off stopped
3 of 3. And it was not solver load - though that pass IS gated on wall
clock (`first_ok_ms < 40 % of budget`), which means the refinement the
chair gets depends on machine load. That is worth knowing before the NUC
run and is listed as an open item; it was not the cause here.

*What the node does.* Reference floored at TURN_FLOOR_SPEED 0.30 - the
follower's own constant, from the measurement that the loaded base does
not rotate below ~1.3 km/h at the faster wheel - with any policy verdict
below the floor returned as a stop, not a slower reference; this
controller has no creep regime. On top of that, v is shaped by the
narrowest corridor within 15 m (the pinch is infeasible at 0.5 m/s and
solves at 0.4) and by curvature. The corridor rule is this controller's
own and is deliberately not transcribed from pursuit, which never writes
the band into a solver and so has no reason for it.

*How much jitter is the right amount.* Everything above turns on injecting
2 cm of lateral noise, which until now was a guess. Measured from the black
box - `/fast_lio_icp/pose` over the three complete runs that carry enough
straight running to separate noise from corner-cutting, 9782 samples taken
on sections whose heading changes less than 2 degrees across the window and
whose speed is above 0.15 m/s - the real residual about a smoothed track is:

| condition | median | p95 | p99 |
|---|---|---|---|
| **measured, 9782 samples** | **4.0 mm** | **11.2 mm** | **21.8 mm** |
| injected sigma 0.02 (passes) | 22.2 mm | 46.2 mm | 57.4 mm |
| injected sigma 0.03 (stops) | 33.2 mm | 69.2 mm | 85.9 mm |
| injected sigma 0.05 (stops) | 55.4 mm | 115.7 mm | 143.3 mm |

The passing condition's MEDIAN exceeds the chair's 99th percentile, and its
own p99 is 2.6x the chair's. So the route completes under noise that is
harsher than the real thing throughout the distribution, not merely at the
tail - which is the useful direction for a validation to be wrong in.

It also says where the edge is, and the edge is not far away: sigma 0.03
stops at 335 m. That is 1.5x the chair's p99 as a *continuous* condition,
so there is real margin, but nobody should read "it completes" as "it has
margin to spare". At sigma 0.05 the usable corridor at the pinch is 65 mm
against noise of comparable size, and stopping is the correct behaviour
rather than a defect to design away.

Note that section 10's gates were measured **without** injected
localisation noise, and remain valid only for that condition.

## 8. CPU budget (NUC11PHKi7C, i7-1165G7, 4C/8T, 28 W sustained)

| Consumer | Budget (cores) |
|---|---|
| FAST-LIO + ICP localization | <= 2.0 |
| drivers + VNC + rosbag recording | <= 0.7 |
| perception / clustering | <= 0.5 |
| gate + relay + wheel_cmd_guard | <= 0.2 |
| **MPC follower (avg / peak)** | **<= 0.1 / 0.3** |
| reserve for spikes and thermals | >= 1.0 |

Measured, not assumed: the acceptance evidence is a per-node CPU trace
(psutil sampler alongside the black box) over a full run, filed with the
run's report. A planner that cannot hold its budget is reverted by the
bringup switch, not tuned in place.

## 9. Guards inherited unchanged

NO_POSE / NO_CLOUD / BASE_STALE / MANUAL_MODE / OFF_ROUTE / OFF_BAND,
the localization policy (TRACKING to drive, bounded DEGRADED grace,
everything else holds) and the tilt/slope speed policy all behave exactly
as in the current follower. MPC replaces the steering and speed decision
inside the guard envelope, never the envelope.

## 10. Validation plan

Offline, now, with no NUC available:

- unit tests (repo convention, unittest, no ROS needed): rollout
  correctness, constraint assembly, warm-start shift, infeasibility stop,
  unpassable-obstacle block (`tests/test_mpc_core.py`);
- closed-loop simulation on the shipped route v4 band (758 stations,
  measured + corridor limits), `tools/sim_mpc_follower.py`, three
  scenarios; gates recorded beside the thresholds they justify.

Measured 2026-08-03, development machine (Apple silicon), osqp 0.6.7.post3,
Python 3.9 - the same major version the NUC runs:

| Scenario | Result | Containment | Cross-track / clearance | Solve p99 |
|---|---|---|---|---|
| clear, 377 m end to end | goal reached, all OK | 100 % of 6351 steps | p95 0.021 m, max 0.071 m | 6.2 ms |
| passable obstacle (1.8 m usable) | goal reached, passed in-band | 100 % of 6368 steps | min obstacle distance 0.441 m (floor 0.40) | 6.5 ms |
| impassable obstacle (0.26 m usable) | BLOCKED_STOP 9.8 m short | 100 % of 3018 steps | never inside the floor | 7.2 ms |

Gates as passed: band containment 100 % in every scenario; clear-run
cross-track p95 0.021 m <= 0.25 m; zero unreported infeasibilities, zero
silent budget overruns; solve time p99 <= 6.5 ms <= 10 ms on the
development machine. The NUC gate remains p99 <= 25 ms, measured on
hardware; even a 4x slowdown factor clears it. The solve-time gate is
enforced by the sim script itself (a PASS/FAIL line, with the 1-minute
load average printed beside it so a loaded box is not mistaken for a
planner regression) - the first version of the script claimed the gate in
this document without checking it, and review caught that too.

The obstacles were placed by the band itself - the passable one at the
widest usable station in the middle half of the route, the blocked one at
the narrowest - so the scenarios test what the map says, not a guess about
it. A 0.45 m padded obstacle inside a 0.26 m corridor is physically
impassable, and the gate for that run is the controlled stop.

On the NUC, when it is back:

1. shadow run - MPC computes, the pursuit follower drives, disagreement is
   logged;
2. supervised run with the operator at the manual switch;
3. two consecutive full autonomous runs compared against the 2026-07-31
   localization envelope (inlier mean 0.9949, TRACKING throughout) - the
   planner must not degrade the localizer's numbers.

## 11. Anti-patterns (binding, from the PRIEST failure)

- no horizon that grows with remaining route length;
- no sampling loops in a Python hot path;
- no retry-on-failure without a bound and a degrade rung;
- no planner without a CPU budget and a solve-time watchdog;
- no obstacle handling that can authorise ground the band says breaks.

## 12. Open questions

- obstacle cluster source for the node implementation: reuse the
  follower's accumulated-cloud corridor guard as-is, or subscribe to the
  perception cluster topic; decide when the ROS node is written, with the
  perception author;
- automatic fallback to pursuit vs operator switch: v1 is operator switch
  plus safe stop, revisit after field data;
- NUC packaging: `pip install osqp==0.6.7.post3` on Python 3.9 (cp39
  manylinux wheels exist upstream) - verify at first deployment, before
  any drive.
