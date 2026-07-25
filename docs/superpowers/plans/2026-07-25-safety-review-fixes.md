# Safety Review Fixes (Top 4) Implementation Plan

> **For agentic workers:** Steps use checkbox (`- [ ]`) syntax for tracking. Executed inline in this session.

**Goal:** Fix the four highest-priority safety/reliability issues found in the 2026-07-25 code review of the localization/safety stack, before the collected driving data is remapped into a new map.

**Architecture:** Four independent, narrowly-scoped patches across `waypoint_follower.py`, `tip_guard.py`, and `auto_initial_pose.py` in `src/static_livox_localization/`. Each patch is self-contained and committed separately so any one can be reverted without affecting the others.

**Tech Stack:** Python 3 / rospy (ROS Noetic). Existing "surface" tests in `test/` are pure text-assertion tests (no rospy import) and can run under plain pytest.

---

## Pre-existing state (not in scope, noted for context)

`test_tip_guard_surface.py` and `test_waypoint_follower_surface.py` already fail against the current uncommitted code (constant renames `TRIP_PITCH_RAD`→`TRIP_DEV_RAD`, `RELEASE_PITCH_RAD`→`RELEASE_DEV_RAD`, and `MAX_SPEED`/`SLOPE_SPEED` value changes from the "double cruise speed" work never propagated to tests). These failures pre-date this plan and are not caused by it. Task steps below only assert the specific strings each task touches, not full-suite green.

---

### Task 1: DEGRADED localization timeout stop

**Files:**
- Modify: `src/static_livox_localization/scripts/waypoint_follower.py`
- Test: `src/static_livox_localization/test/test_waypoint_follower_surface.py`

- [ ] **Step 1: Add the timeout constant and state field**

In `waypoint_follower.py`, near the other timing constants (after `POSE_STALE_S = 1.0` / `BASE_STALE_S = 1.5`):

```python
DEGRADED_STOP_S = 3.0
```

In `WaypointFollower.__init__`, alongside the other tracking fields (after `self.tracking_state = ""`):

```python
        self.degraded_since = None
```

- [ ] **Step 2: Track DEGRADED duration and add the timeout hold**

In `step()`, the current DEGRADED handling only caps speed:

```python
        if self.tracking_state == "DEGRADED":
            allowed = min(allowed, SLOPE_SPEED)
```

Replace the state-reason chain so a DEGRADED state that persists past `DEGRADED_STOP_S` becomes a hold, same tier as `LOCALIZATION_LOST`. In the `reason` if/elif chain (where `elif self.tracking_state == "LOST": reason = "LOCALIZATION_LOST"` lives), add tracking of `degraded_since` and a new branch immediately after it:

```python
        if self.tracking_state == "DEGRADED":
            if self.degraded_since is None:
                self.degraded_since = now
        else:
            self.degraded_since = None
```

Add this block right after the existing `elif self.tracking_state == "LOST": reason = "LOCALIZATION_LOST"` line, as a new `elif`:

```python
        elif self.tracking_state == "DEGRADED" and self.degraded_since is not None and \
                (now - self.degraded_since).to_sec() > DEGRADED_STOP_S:
            reason = "LOCALIZATION_DEGRADED_TIMEOUT"
```

(The DEGRADED-duration tracking block itself must run unconditionally each cycle, before the `reason` if/elif chain, so it also resets when the state clears mid-hold.)

Leave the existing `if self.tracking_state == "DEGRADED": allowed = min(allowed, SLOPE_SPEED)` speed-cap later in the function as-is — it still applies for the first `DEGRADED_STOP_S` seconds before the hold kicks in.

- [ ] **Step 3: Add a text assertion test**

Append to `test_waypoint_follower_surface.py`:

```python
def test_degraded_localization_times_out_to_a_hold():
    text = follower_text()
    assert "DEGRADED_STOP_S" in text
    assert '"LOCALIZATION_DEGRADED_TIMEOUT"' in text
    assert "self.degraded_since" in text
```

- [ ] **Step 4: Run the new test**

Run: `pytest src/static_livox_localization/test/test_waypoint_follower_surface.py::test_degraded_localization_times_out_to_a_hold -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/static_livox_localization/scripts/waypoint_follower.py src/static_livox_localization/test/test_waypoint_follower_surface.py
git commit -m "fix: stop the chair after DEGRADED localization persists past 3s

Previously DEGRADED only capped speed to SLOPE_SPEED and never held,
so the follower kept driving on uncorrected dead-reckoning pose
(map_T_odom_ frozen at last accepted correction) for up to
lost_after_s=8s before LOST forced a stop. Band containment safety
depends on pose_xy being trustworthy, which it is not during a
sustained correction failure streak."
```

---

### Task 2: Remove the duplicate hill-assist boost in tip_guard.py

**Files:**
- Modify: `src/static_livox_localization/scripts/tip_guard.py`
- Test: `src/static_livox_localization/test/test_tip_guard_surface.py`

- [ ] **Step 1: Remove the BOOST_* constants**

Delete this block (it duplicates `climb_boost`'s purpose but bypasses the accel governor and has no slope gate):

```python
# Hill assist: closed-loop speed makeup. The command chain is open-loop,
# so on a steep ramp the actual speed sags below the commanded one and
# the chair stalls; commanding harder in one step is what pitches the
# nose up. Instead an integrator slowly raises the output while the
# MEASURED speed lags the desired one - torque arrives gradually, and
# the boost freezes and decays the moment the gyro shows any nose-lift
# tendency, so climbing power and tipping tendency can never coexist.
BOOST_MAX = 0.45
BOOST_GAIN_PER_S = 0.10
BOOST_DECAY_PER_S = 0.60
BOOST_FREEZE_RATE_RAD_S = math.radians(4.0)
SPEED_MEASURE_WINDOW_S = 0.4
```

Keep `SPEED_MEASURE_WINDOW_S` — it is still used by `_odom_track`/`measured_speed` computation in `on_odom`, which `climb_boost` also depends on. Only remove `BOOST_MAX`, `BOOST_GAIN_PER_S`, `BOOST_DECAY_PER_S`, `BOOST_FREEZE_RATE_RAD_S`.

- [ ] **Step 2: Remove the `self.boost` field**

In `__init__`, delete the duplicate block:

```python
        self.climb_boost = 0.0
        self.baseline_pitch = None
        self.measured_speed = 0.0
        self.boost = 0.0
        self._odom_track = deque()
```

Replace with (drops the duplicate `self.measured_speed = 0.0` and `self.boost`):

```python
        self.climb_boost = 0.0
        self.baseline_pitch = None
        self._odom_track = deque()
```

(`self.measured_speed = 0.0` stays where it already is, earlier in `__init__` right after `self.fused_pitch = 0.0` — do not duplicate it.)

- [ ] **Step 3: Remove the `update_boost` method entirely**

Delete:

```python
    def update_boost(self, dt, desired):
        """Hill-assist integrator - only with a verified IMU axis, only
        while genuinely lagging, frozen/decayed on any nose-lift sign."""
        nose_lift = self.pitch_rate * (1.0 if desired >= 0 else -1.0)
        unsafe = (self.tripped or not self.axis_config_ok or
                  nose_lift > BOOST_FREEZE_RATE_RAD_S or
                  abs(self.pitch_rate) > CAUTION_RATE_RAD_S)
        if unsafe or desired <= 0.05:
            self.boost = max(0.0, self.boost - BOOST_DECAY_PER_S * dt)
            return
        lag = desired - self.measured_speed
        if lag > 0.05:
            self.boost = min(BOOST_MAX,
                             self.boost + BOOST_GAIN_PER_S * lag * dt / 0.3)
        elif lag < -0.02:
            self.boost = max(0.0, self.boost - BOOST_DECAY_PER_S * dt)
```

- [ ] **Step 4: Remove the call site and the output-line bypass**

In `spin()`, delete this line (the call into the removed method):

```python
            self.update_boost(dt, 0.0 if (self.tripped or stale) else desired)
```

And simplify the final output line from:

```python
            out.linear.x = self.current_speed + (
                self.boost if self.current_speed > 0.02 else 0.0)
```

to:

```python
            out.linear.x = self.current_speed
```

Also update the status string, which currently includes `boost=%.2f`:

```python
            self.status_pub.publish(String(
                data="%s pitch=%.1f dev=%.1f rate=%.1f budget=%.2f "
                     "v=%.2f boost=%.2f" % (
                    state, math.degrees(self.fused_pitch),
                    math.degrees(self.deviation()),
                    math.degrees(self.pitch_rate), self.accel_budget,
                    self.measured_speed, self.boost)))
```

to:

```python
            self.status_pub.publish(String(
                data="%s pitch=%.1f dev=%.1f rate=%.1f budget=%.2f "
                     "v=%.2f" % (
                    state, math.degrees(self.fused_pitch),
                    math.degrees(self.deviation()),
                    math.degrees(self.pitch_rate), self.accel_budget,
                    self.measured_speed)))
```

- [ ] **Step 5: Add a text assertion test guarding against regression**

Append to `test_tip_guard_surface.py`:

```python
def test_no_duplicate_unbounded_boost_bypassing_the_accel_governor():
    text = guard_text()
    assert "def update_boost" not in text
    assert "self.boost" not in text
    assert "BOOST_MAX" not in text
    assert "out.linear.x = self.current_speed\n" in text
```

- [ ] **Step 6: Run the new test**

Run: `pytest src/static_livox_localization/test/test_tip_guard_surface.py::test_no_duplicate_unbounded_boost_bypassing_the_accel_governor -v`
Expected: PASS

- [ ] **Step 7: Run the full tip_guard surface file and confirm no NEW failures**

Run: `pytest src/static_livox_localization/test/test_tip_guard_surface.py -v`
Expected: same pass/fail set as before this task, minus nothing new broken (the pre-existing `TRIP_PITCH_RAD`/`RELEASE_PITCH_RAD` failures noted above are unrelated to this task and unaffected by it).

- [ ] **Step 8: Commit**

```bash
git add src/static_livox_localization/scripts/tip_guard.py src/static_livox_localization/test/test_tip_guard_surface.py
git commit -m "fix: remove duplicate hill-assist boost that bypassed the accel governor

update_boost()/self.boost duplicated climb_boost's purpose (closed-loop
speed makeup comparing desired vs measured_speed) but added its result
directly onto current_speed in the final output line, skipping
accel_budget/HARD_DECEL/LAUNCH_ACCEL ramp-limiting entirely, and had no
on_slope gate (so it could fire on flat ground at launch, exactly the
jerky-start scenario climb_boost's own comment says it exists to avoid).
climb_boost alone (already governor- and slope-gated) is kept."
```

---

### Task 3: auto-init retry loop on full failure

**Files:**
- Modify: `src/static_livox_localization/scripts/auto_initial_pose.py`
- Test: `src/static_livox_localization/test/test_auto_initial_pose_surface.py`

- [ ] **Step 1: Add a retry-count argument**

In `main()`'s `argparse` block, after `--top`:

```python
    parser.add_argument("--retries", type=int, default=2,
                        help="extra full re-collection attempts after the "
                             "first, if no candidate passes verification")
```

- [ ] **Step 2: Wrap submap collection + scoring + verification in a retry loop**

The current `main()` body does: collect submap once -> score once -> try top-N candidates once -> give up. Restructure so that block is a function called from a retry loop.

Extract the existing body from `collector = SubmapCollector(args.window_s)` down through the final `rospy.logerr("no candidate passed verification"); return 4` into a new function `attempt(args, map_points, tree, candidates)` that returns `0` on success and a nonzero code on failure (same codes as today: `2` no submap, `3` low score, `4` no candidate verified). Keep all existing logic and log messages identical — including the literal string `"failed verification, trying next"`, which `test_auto_init_falls_back_to_next_candidate_on_rejection` asserts on.

Then `main()` becomes:

```python
    for attempt_num in range(1, args.retries + 2):
        rospy.loginfo("auto-init attempt %d/%d", attempt_num, args.retries + 1)
        result = attempt(args, map_points, tree, candidates)
        if result == 0:
            return 0
        if attempt_num <= args.retries:
            rospy.logwarn(
                "attempt %d failed (code %d) - recollecting and retrying",
                attempt_num, result)
    rospy.logerr("auto-init failed after %d attempt(s)", args.retries + 1)
    return result
```

Each call to `attempt()` must construct a fresh `SubmapCollector` internally (so retries capture new data rather than reusing a stale/thin submap) — keep the `collector = SubmapCollector(args.window_s)` line inside `attempt()`, not hoisted to `main()`.

- [ ] **Step 3: Add a text assertion test**

Append to `test_auto_initial_pose_surface.py`:

```python
def test_auto_init_retries_with_a_fresh_submap_on_full_failure():
    text = script_text()
    assert "--retries" in text
    assert "def attempt(" in text
    assert "recollecting and retrying" in text
    # each retry must build its own collector, not reuse one across attempts
    attempt_def = text.index("def attempt(")
    assert "SubmapCollector(args.window_s)" in text[attempt_def:]
```

- [ ] **Step 4: Run the new test**

Run: `pytest src/static_livox_localization/test/test_auto_initial_pose_surface.py -v`
Expected: all tests in this file PASS, including the pre-existing ones (this task does not remove any string they assert on) and the new one.

- [ ] **Step 5: Commit**

```bash
git add src/static_livox_localization/scripts/auto_initial_pose.py src/static_livox_localization/test/test_auto_initial_pose_surface.py
git commit -m "fix: retry auto-init with a fresh submap instead of giving up once

A single bad 2s capture window (person crossing the FOV, chair still
settling, sensor warm-up) could doom the entire unattended startup,
requiring manual RViz seeding. This matches the 2026-07-24 field
failure log exactly. Now recollects and retries (default: 2 extra
attempts) before falling back to the manual-seed message."
```

---

### Task 4: pure pursuit global resync on position jump

**Files:**
- Modify: `src/static_livox_localization/scripts/waypoint_follower.py`
- Test: `src/static_livox_localization/test/test_waypoint_follower_surface.py`

- [ ] **Step 1: Add a resync-distance constant**

Near `GEOFENCE_M = 3.5`:

```python
NEAREST_RESYNC_M = 2.0
```

- [ ] **Step 2: Compare windowed vs. global nearest distance and resync on divergence**

In `pure_pursuit_target()`, the current windowed search is:

```python
        d = np.linalg.norm(self.waypoints - self.pose_xy, axis=1)
        if not self.route_locked:
            self.nearest_index = int(np.argmin(d))
            self.route_locked = True
        window_end = min(self.nearest_index + 15, len(self.waypoints))
        self.nearest_index = int(
            self.nearest_index + np.argmin(d[self.nearest_index:window_end]))
```

`d` (the full-array distance vector) is already computed here. Add a global-vs-windowed divergence check right after the windowed update:

```python
        d = np.linalg.norm(self.waypoints - self.pose_xy, axis=1)
        if not self.route_locked:
            self.nearest_index = int(np.argmin(d))
            self.route_locked = True
        window_end = min(self.nearest_index + 15, len(self.waypoints))
        windowed_index = int(
            self.nearest_index + np.argmin(d[self.nearest_index:window_end]))
        global_index = int(np.argmin(d))
        if d[global_index] + NEAREST_RESYNC_M < d[windowed_index]:
            rospy.logwarn(
                "waypoint_follower: position diverged from windowed search "
                "(wp %d, %.1fm) vs global nearest (wp %d, %.1fm) - resyncing",
                windowed_index, d[windowed_index], global_index, d[global_index])
            self.nearest_index = global_index
        else:
            self.nearest_index = windowed_index
```

This only resyncs when the global nearest point is meaningfully closer than what the forward-only window found (e.g. after a counter-motion reverse or a localization re-acquisition jump); on ordinary forward driving the two agree and behavior is unchanged.

- [ ] **Step 3: Add a text assertion test**

Append to `test_waypoint_follower_surface.py`:

```python
def test_pure_pursuit_resyncs_globally_when_position_jumps_backward():
    text = follower_text()
    assert "NEAREST_RESYNC_M" in text
    assert "global_index = int(np.argmin(d))" in text
    assert "resyncing" in text
```

- [ ] **Step 4: Run the new test**

Run: `pytest src/static_livox_localization/test/test_waypoint_follower_surface.py::test_pure_pursuit_resyncs_globally_when_position_jumps_backward -v`
Expected: PASS

- [ ] **Step 5: Run the full waypoint_follower surface file and confirm no NEW failures**

Run: `pytest src/static_livox_localization/test/test_waypoint_follower_surface.py -v`
Expected: same pass/fail set as after Task 1, plus this new test passing.

- [ ] **Step 6: Commit**

```bash
git add src/static_livox_localization/scripts/waypoint_follower.py src/static_livox_localization/test/test_waypoint_follower_surface.py
git commit -m "fix: let pure pursuit resync globally after a position jump

nearest_index only ever searched forward within a 15-waypoint window
(monotonic, never decreasing), so a backward displacement - from
tip_guard's counter-motion reverse or a post-LOST re-acquisition pose
jump - left it referencing a stale, too-far-advanced waypoint with no
way to recover. Now falls back to a full-array nearest search whenever
the global nearest point is materially closer than the windowed one."
```

---

## Final steps

- [ ] **Push the branch**

```bash
git push origin work/safety-fixes-20260725
```

- [ ] **Sync the four fixed files back to the NUC's live checkout on the same branch** (the NUC is where the robot actually runs; GitHub is the backup/review copy)
