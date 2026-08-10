# Unified Collision Height Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unify the `obstacle_clusters` height and the `safety_gate` height at the shared ground-relative collision range of 0.15 m through 1.50 m.

**Architecture:** Define collision-height bounds in the existing shared `cloud_points.py` module and import them from both consumers. Filter cluster input before grid clustering so overhead-only returns cannot create control objects, while collision-relevant lower returns remain available to classification and corridor profiling.

**Tech Stack:** Python 3, ROS 1 `rospy`, NumPy, pytest

## Global Constraints

- The common collision-height range is exactly 0.15 m through 1.50 m above ground.
- The upper value is a maximum collision height, not a minimum object height.
- Do not change classification thresholds, stopping distance, route geometry, or costmap range.
- Open a Draft PR and do not merge it without a later explicit user instruction.
- The PR title and body must explicitly say that the `obstacle_clusters` height and the `safety_gate` height were unified.

---

### Task 1: Share and enforce collision-height bounds

**Files:**
- Modify: `src/static_livox_localization/scripts/cloud_points.py`
- Modify: `src/static_livox_localization/scripts/safety_gate.py`
- Modify: `src/static_livox_localization/scripts/obstacle_clusters.py`
- Test: `src/static_livox_localization/test/test_cluster_pipeline.py`
- Test: `src/static_livox_localization/test/test_waypoint_follower_surface.py`

**Interfaces:**
- Produces: `cloud_points.COLLISION_MIN_HEIGHT_M: float`
- Produces: `cloud_points.COLLISION_MAX_HEIGHT_M: float`
- Consumes: both safety consumers import the two constants instead of defining independent collision-height bounds.

- [ ] **Step 1: Add failing contract tests for one shared height source**

Add assertions that `cloud_points.py` defines `COLLISION_MIN_HEIGHT_M = 0.15` and `COLLISION_MAX_HEIGHT_M = 1.5`, and that both `safety_gate.py` and `obstacle_clusters.py` import and use those names rather than independent numeric ceilings.

- [ ] **Step 2: Add a failing pipeline test for overhead-only returns**

Construct a dense forward point set whose ground-relative heights are all between 1.60 m and 1.90 m, run the real `ObstacleClusters.step()`, and assert that the published summary has an empty `objects` list.

- [ ] **Step 3: Add a failing pipeline test for a vertically mixed object**

Construct a dense object containing points below and above 1.50 m. Assert that clustering still reports an object from the retained lower points and that its reported vertical size does not include the filtered overhead points.

- [ ] **Step 4: Run the new tests and verify RED**

Run:

```bash
python3 -m pytest \
  src/static_livox_localization/test/test_cluster_pipeline.py \
  src/static_livox_localization/test/test_waypoint_follower_surface.py -q
```

Expected: the shared-constant contract and overhead-only pipeline tests fail because `obstacle_clusters.py` still uses `REL_Z = (0.15, 2.4)`.

- [ ] **Step 5: Implement the shared constants and consumer imports**

Define the two constants in `cloud_points.py`. Replace `safety_gate.py`'s independent minimum/maximum collision-height values and `obstacle_clusters.py`'s `REL_Z` values with imports of the shared constants. Keep filtering before clustering and classification.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run the Step 4 command again. Expected: all selected tests pass.

- [ ] **Step 7: Run the complete static-localization test suite**

Run:

```bash
python3 -m pytest src/static_livox_localization/test -q
```

Expected: all tests pass with no new warnings or errors.

- [ ] **Step 8: Commit the implementation**

Stage only the five implementation/test files and commit with:

```bash
git commit -m "fix: unify obstacle cluster and safety gate heights"
```

### Task 2: Publish the review-only change

**Files:**
- No production files added beyond Task 1.

**Interfaces:**
- Consumes: the tested branch from Task 1.
- Produces: a GitHub Draft PR targeting `main`.

- [ ] **Step 1: Verify the intended diff and repository state**

Run `git status -sb`, `git diff origin/main...HEAD --check`, and inspect `git diff --stat origin/main...HEAD`. Confirm only the design, plan, shared constants, consumer wiring, and regression tests are present.

- [ ] **Step 2: Push the feature branch**

Push `agent/unify-obstacle-cluster-safety-gate-height` to `origin` with upstream tracking.

- [ ] **Step 3: Open a Draft PR targeting main**

Use this required title:

```text
fix: unify obstacle_clusters and safety_gate collision heights
```

The body must state that the `obstacle_clusters` height and the `safety_gate` height are now unified at 0.15-1.50 m, explain the previous 2.40 m mismatch and overhead false stops, list the tests run, and state that the PR must not be merged without explicit approval.

- [ ] **Step 4: Confirm Draft status**

Read the created PR metadata and verify `isDraft` is true before reporting completion.
