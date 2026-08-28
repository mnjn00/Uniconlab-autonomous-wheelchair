# Isolated candidate-precheck integration

Base: 950cbc4404db51cdac5f7936c296dc2db8172c4f.
Ported: candidate preselection and ramped-command geometry from 4ae732f.

Only five runtime Python files change: dwa_core, gpu_dwa_backend,
dwa_follower, person_bypass_dwa_follower, trajectory_safety_gate.
Existing rejected-yaw retry, static-object permits, person qualification,
ROS timestamp fixes, localization, maps, perception and motor code remain.
No distance/velocity settings are changed. The independent gate's verdict
logic remains the 950cbc4 version; shared point filtering is factored out.

Both CPU and GPU planners inspect candidates in cost order. Active static
threat permits enable the additional raw-cloud swept-footprint veto, using
the predicted ramped command. No passing candidate means GATE_TRAJECTORY,
not relaxed safety. Missing/stale sensor data rejects candidates. A valid
cloud containing no obstacle-height points is not confused with missing data.

NUC verification: 41 focused tests, 43 follower-policy tests, 55 motion/cluster
tests, and 26 runtime/hybrid tests passed (the last set repeats 6 runtime
surface tests). Actual CuPy candidate rejection/reselection passed. A
read-only live sample of 24,098 points took 3.08ms for six candidate checks;
this is one sample, not a worst-case timing guarantee.

Deployment uses a separate directory and direct Python entrypoints with the
existing runtime environment. Only follower and gate restart, initially
PAUSED. Existing source directories and route assets are not overwritten.
Field avoidance is not validated by these offline tests.
