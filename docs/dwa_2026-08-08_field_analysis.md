# DWA profile, 2026-08-08: why it drove 48 m of 380 m

Two runs, first time `PROFILE=dwa` was driven with a person in the chair.

| | 19:30:13 | 19:58:00 |
|---|---|---|
| duration | 1009 s | 726 s |
| route covered | waypoint 43 -> 228 (8 -> 44 m) | 43 -> 248 (8 -> 48 m) |
| net displacement | 0 m over 127 m of track | 39 m over 92 m |
| samples actually commanding | 1006 (~100 s) | 1224 (~122 s) |
| `HOLD:MANUAL_MODE` | 3465 | 3867 |
| `HOLD:DWA_BLOCKED` | 1810 (181 s, one 180 s block) | 799 (79 s, longest 77 s) |
| localisation holds | `LOST` 277 | `DEGRADED_TIMEOUT` 1236 |

Everything below is measured from `/tmp/bags/blackbox_20260808_19{3013,5800}.bag`
against `20260803_route_v5`. The teammate report of "S-curves and yaw
saturation" is confirmed; the cause it proposed is not the one the data shows.

## What was ruled out first

- **Not the command ramp.** Commanded `v` had median 0.300 and p90 0.600 -
  the chair was driving at proper speed. The `v=0.02` line quoted in the
  report is a single sample after a stop, not the norm. `target_w` reverses
  sign 43 and 52 times against the ramp's 35 and 49: the ramp *reduced* the
  oscillation it was blamed for.
- **Not ambiguous arc-length lookup.** The route never comes within 3 m of a
  point more than 20 m away along itself (0 of 2004 waypoints), so nearest-
  point projection is unambiguous everywhere.
- **Not the band veto.** Replayed on the recorded poses, a median 99 of 126
  candidates pass band containment. Candidates were available; the score
  chose badly among them.
- **Not plant lag.** Adding a 0.3 s yaw time constant and 0.2 s pose latency
  to a closed-loop replay raises reversals from 76 to 123 but never produces
  saturation. Lag is an aggravator, not the cause.

## Defect 1: the score never looked at where the chair pointed

`cost = 3.0*path + 2.0*obstacle - 1.0*progress`, with `path` the mean
distance from the rollout to the route. Decomposed at saturating instants,
the winner and the straight candidate differ in `progress` by **0.00 m** -
the whole decision was the path term, and a hard turn scored closer to the
line than going straight.

A position-only cost driving a saturating actuator is a bang-bang regulator.
Its signature is in the data: `|target_w|` median exactly `MAX_YAW_RATE`,
saturated 50 % and 49 % of commanding samples, sign reversing every 1.8 s -
a 1.6 m wavelength at 0.45 m/s. At the reversal instants cross-track error
was only 0.19 m while **heading** error was 19-21 degrees. The chair was
limit-cycling about its heading, on a line it was already sitting on.

Pure pursuit does not do this because its lookahead geometry has the angle
term built in. The DWA score had no equivalent.

Fix: `W_HEADING = 2.0` on the mean heading error against the corridor's own
tangent over the rollout. The weight is not sensitive - anything from 0.5 up
behaves the same; 2.0 is the middle of that plateau. Plus `W_STEER = 1.0` so
reversing the steer is not free (above ~2.0 the chair cuts corners; at 4.0 a
closed-loop replay lost a third of its progress).

## Defect 2: standing still was a candidate, and it won

A stationary rollout is a single point. Sitting on the line its path cost is
exactly zero, and `W_PROGRESS` 1.0 could not outweigh `W_PATH` 3.0. **The
chair scored a reward for not moving.** That is `DWA_BLOCKED`: 180 s in one
continuous block in run 1, at 51 degrees of heading error, and 77 s in run 2
at 76 degrees, where 99 % of blocked samples were past 60 degrees. A healthy
fix, an admissible corridor, and the planner electing to stay put.

Fix: only moving candidates compete. A stop is a refusal - what the caller
does when `plan()` returns a reason instead of a command - never a choice.
`BLOCKED` is gone from the status vocabulary; `OFF_BAND` / `OBSTACLE` /
`NO_CANDIDATE` remain and are strictly more informative. The follower
publishes `HOLD:DWA_<status>` unchanged.

## The failure loop, closed

Oscillate -> heading error grows -> past ~50 degrees standing still outscores
every arc -> stuck until a person takes the joystick -> released -> repeat.
346 s and 387 s of manual driving in runs that covered 44 m and 48 m.

## Measured effect of the fix

Replayed against the recorded field poses:

| | run 1 | run 2 |
|---|---|---|
| yaw saturation, before | 9 % | 28 % |
| yaw saturation, after | 3 % | 3 % |
| median `\|w\|`, before -> after | 0.25 -> 0.10 | 0.10 -> 0.05 |
| stop commands | 115/505 | 64/519 |

Closed-loop, with 0.3 s yaw lag and 0.2 s pose latency, seeding the heading
error the field runs deadlocked at:

| initial heading error | before | after |
|---|---|---|
| 51 deg | drives, 44 reversals | drives, 29 reversals |
| 76 deg | **BLOCKED for all 1200 steps** | drives to 68 m, 0 blocked |
| 100 deg | BLOCKED | drives to 68 m, 0 blocked |

Worst-case cross-track over the full route improves 0.26 m -> 0.13 m.

## What is NOT fixed, and must not be read as fixed

- **`OFF_BAND` 23 % in run 1** persists under replay. The chair was genuinely
  at or outside the corridor edge - cross-track reached 2.51 m - partly from
  manual driving. No scoring change reaches that; it is a starting-state
  problem.
- **Residual reversals (~0.25/s in replay)** are mostly the route's own
  bends and the 0.05 rad/s yaw grid, not oscillation: RMS excess over the
  curvature the route actually demands is 0.07-0.11 rad/s against a route
  demand peaking at 0.39 rad/s. Counting reversals alone overstates this.
- **Yaw time constant and pose latency are still unmeasured.** 0.3 s and
  0.2 s are plausible values used to stress the replay, not measurements.
  `LATENCY_S` still defaults to 0.
- **Simulation only.** The chair has not been driven since. `PROFILE` still
  defaults to `pursuit`.
- **The braking-horizon idea was tried and dropped.** Admitting a candidate
  on the prefix that fits its own braking distance, rather than a flat 1.7 s,
  changed nothing once the two defects above were fixed. It would have
  relaxed a safety criterion for no gain, so the band veto is untouched.

## Addendum, 2026-08-11: two more defects, found by reading the code

Neither is a new measurement. The chair has not been driven since 08-08 and
everything above stands as recorded.

**The profile never asked whether the obstacle was moving.** `DwaFollower`
replaces `step()` whole, and the parked-or-moving decision lived in the body
of the pursuit `step()` rather than beside the guards that were extracted for
exactly this reason. So the profile handed its planner the nearest threat
whatever the tracker said about it, and a rollout scorer given a walking
person picks the arc that clears them by `OBSTACLE_FLOOR_M` and drives past -
where the pursuit profile stands and waits. The `mpc` profile had it too.
Both now call `WaypointFollower.avoidance_for`, and a `WAIT` never reaches
the planner at all.

**The obstacle reached the planner as one point.** Measuring an object's
distance from its own returns instead of its bounding box was the 07-31 fix;
the shape was the half left behind. A wall spanning the corridor arrived as a
single point, and an arc clearing that point by 0.41 m was admitted while
passing through the rest of the wall. `cluster_guard.object_points` publishes
every lateral slice the object occupies - the same measurement
`profile_reach` takes the minimum of - for the nearest few objects rather
than only the nearest one, because the single-object argument holds for a
distance and fails for a shape.

**And the tests were checking a trajectory the planner never scored.**
`dwa_core.rollout` translated then rotated; `DwaPlanner._rollouts` rotates
then translates. They disagreed by one full heading step at the sample
spacing, and `stays_in_band(rollout(state, *planner.plan(...)))` is how
`tests/test_dwa_band.py` verifies containment - so the check leaned in the
direction that hides a candidate leaving the band on its first step. Both now
take the same step. No node calls `rollout`, so nothing the chair runs
changed; what changed is that the test means what it says.

### What is still not fixed

`OFF_BAND` remains a stop only a person can clear, and the entry above is
right that no scoring change reaches it - wrong only in implying nothing
does. It is not a scoring problem: every rollout point is tested, so a chair
a centimetre outside the corridor has no admissible candidate at all,
because driving back in starts outside. A bounded recovery for it is written
and tested and deliberately **not** merged. It is the only change in this
stack that turns a stop into motion, and it is waiting on one complete run of
the route to be worth its risk. That run is what this profile needs before
anything else.
