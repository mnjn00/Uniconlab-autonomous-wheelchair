# Unify Obstacle Cluster and Safety Gate Collision Height

## Goal

Unify the ground-relative obstacle height used by `obstacle_clusters` and
`safety_gate` at 0.15 m through 1.50 m. This prevents overhead-only returns
that the safety gate intentionally ignores from re-entering the motion-control
path through clustered-object avoidance.

The pull request must explicitly state that it unifies the
`obstacle_clusters` height and the `safety_gate` height. It will be opened as a
draft and must not be merged without a later, explicit instruction from the
user.

## Current Problem

Both nodes consume the same motion-compensated point cloud, but they apply
different ground-relative height limits:

- `safety_gate` accepts collision points from 0.15 m through 1.50 m.
- `obstacle_clusters` accepts points from 0.15 m through 2.40 m.

As a result, an object made entirely of returns between 1.50 m and 2.40 m can
be clear to `safety_gate` while still becoming a cluster. `cluster_guard` and
the follower judge that cluster by its horizontal corridor reach and can slow
or stop the wheelchair without rechecking its height.

## Design

Define the collision-height bounds once in the existing shared point-cloud
module:

```python
COLLISION_MIN_HEIGHT_M = 0.15
COLLISION_MAX_HEIGHT_M = 1.50
```

Both `safety_gate` and `obstacle_clusters` will import these constants. The
safety gate will pass them to its raw-point filter, while obstacle clustering
will apply them before `cluster_grid()`, `classify()`, `lateral_profile()`, and
summary generation.

This is a control-plane filter, not a change to the semantic classification
rules. Existing `person`, `vehicle`, `obstacle`, and `outside_band` labels keep
their current meaning for the points that remain.

## Required Behaviour

- Returns below 0.15 m remain excluded as ground or irrelevant low returns.
- Returns from 0.15 m up to 1.50 m remain eligible for collision clustering.
- An object whose returns are all above 1.50 m produces no control cluster.
- An object spanning above and below 1.50 m remains detectable from its lower,
  collision-relevant returns.
- The 1.50 m upper bound is a maximum collision height, not a minimum object
  height.
- `safety_gate` and `obstacle_clusters` cannot silently drift to different
  collision-height constants.

## Testing

Add regression tests that first fail against the current 2.40 m cluster
ceiling and then pass after the shared constants are adopted:

1. Assert both consumers reference the same shared 0.15 m and 1.50 m bounds.
2. Feed an overhead-only point set above 1.50 m and verify no object summary is
   generated for motion control.
3. Feed a vertically mixed object and verify its points below 1.50 m still
   produce a cluster.
4. Run the focused static-localization test suite to catch changes to the
   existing rider mask, classification, tracking, and corridor profile.

## Scope Boundaries

This change does not alter localization, route geometry, stopping-distance
calculation, class-specific motion policy, or the costmap obstacle range. It
does not add a new runtime tuning parameter or an overhead visualization topic.
Those are separate features and are not needed to fix this mismatch.

## Operational Safety

The threshold assumes the documented seated occupant height of 1.35 m plus a
0.15 m clearance margin. Field deployment still requires low-speed validation
with an operator and emergency stop available. The pull request is review-only
until the user explicitly authorizes a merge.
