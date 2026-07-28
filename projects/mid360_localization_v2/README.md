# Mid-360 fixed-map localization

This package is a new native localizer. It does not reuse
`static_livox_localization`, publish motor commands, or claim that an
uncalibrated match is safe.

## Selected sensor and upstream algorithms

- **IMU:** the ICM40609 built into the Livox Mid-360, published at
  `/livox/imu`. LiDAR and IMU share one device and clock path. The 2026-07-27
  comparison in `docs/debug_bags.md` found no VN-100 gyro advantage at the
  chair's dynamics, while the VN accelerometer scale was still unverified.
- **Local odometry:** upstream HKU-MARS FAST-LIO2. This package consumes its
  `/Odometry` and deskewed `/cloud_registered_body` outputs; it does not
  reimplement the iterated Kalman filter.
- **Global initialization:** upstream `hdl_global_localization` using
  FPFH-TEASER. The node refuses a silent FPFH-RANSAC fallback.
- **Continuous map registration:** upstream CPU FastVGICP from `fast_gicp`.
  The code here only assembles timestamped submaps, supplies initial guesses,
  applies acceptance gates, and converts frames.

Pinned upstream revisions and the two compatibility patches are in
`external/`.

## Transform convention

FAST-LIO reports the built-in IMU body pose:

```text
odom_T_body
```

Registration estimates `map_T_body`. The pose needed by navigation is
calculated, not renamed:

```text
map_T_base_footprint = map_T_body * body_T_base_footprint
```

The node obtains `body_T_base_footprint` from TF at the cloud timestamp. A
measured static transform must therefore exist in the hardware URDF/TF tree.
If it does not, pose publication stops with `BASE_EXTRINSIC_MISSING`. The
simulation-only URDF dimensions are not accepted as hardware calibration.

The node also converts FAST-LIO odometry to canonical `/odom` and publishes
the sole `odom -> base_footprint` TF for this localization-only configuration.
The pinned FAST-LIO compatibility patch adds `publish/tf_en`; the launch sets
it false so FAST-LIO cannot simultaneously publish `camera_init -> body` and
create a second parent/loop through the static base-to-body calibration. Do
not run a second owner of `odom -> base_footprint`.

## Build on the ROS1 Noetic NUC

Install ROS dependencies, `vcstool`, OpenSSL, PCL, OpenCV and OpenMP first.
Then run:

```bash
bash projects/mid360_localization_v2/tools/bootstrap_localization_noetic.sh \
  /home/u20/localization_ws
source /home/u20/localization_ws/devel/setup.bash
```

`hdl_global_localization` builds its pinned TEASER++ dependency because the
bootstrap passes `-DENABLE_TEASER=ON`. It also links
`wheelchair_interfaces`, `wheelchair_navigation`, and this package into the
new workspace, so the reset-epoch-safe adapter is built without changing the
deployed workspace. The FAST-LIO compatibility patch switches the ROS1
message namespace to the official
`livox_ros_driver2/CustomMsg` and adds the TF publication switch needed for
single ownership.

## First run with the merged map

Compute the immutable identity after `merged_0707_0725.ply` is copied to the
NUC:

```bash
sha256sum /data/maps/merged_0707_0725.ply
```

Make sure the measured `base_footprint <-> body` static TF is already
published. With the driver and FAST-LIO running, perform the read-only input
check:

```bash
rosrun wheelchair_lidar_localization localization_preflight.py \
  --map /data/maps/merged_0707_0725.ply \
  --map-sha256 <64-character-sha256>
```

It verifies the immutable map, exact topic message types, source-stamped
Mid-360 IMU rate and acceleration units, and the required measured TF. Then
keep the chair stationary and run:

```bash
roslaunch wheelchair_lidar_localization mid360_localization.launch \
  map_path:=/data/maps/merged_0707_0725.ply \
  map_id:=merged_0707_0725 \
  map_sha256:=<64-character-sha256> \
  start_driver:=true
```

Initialization requires three mutually consistent global results followed by
FastVGICP refinement. Consensus compares full 3D translation and rotation,
and `map -> odom` must remain aligned with gravity; a repeatable upside-down
or rolled FPFH alias is therefore still rejected. Loss never resumes
automatically. After stopping the chair and canceling the mission, use RViz
`2D Pose Estimate` to publish a fresh `/initialpose`. Both this localizer and
the existing safety adapter see the same request, so the reset counter cannot
bypass relocalization authorization. The arrow coordinates are not trusted
as a pose seed; FPFH-TEASER still performs the global search.

Every asynchronous reset or source-time regression advances an internal
state epoch. A registration result from an older epoch is discarded. Native
pose `header.seq` is also set to the corresponding `reset_count`, and the
`fast_lio_icp` adapter requires that binding before combining the split pose
and diagnostic topics.


Native outputs:

- `/fast_lio_icp/pose` — untrusted `map -> base_footprint` pose candidate
- `/fast_lio_icp/localization_diagnostics` — map identity, raw state and
  registration metrics
- `/fast_lio_icp/aligned` — debug cloud in `map`
- `/odom` and `odom -> base_footprint` — canonical FAST-LIO odometry

Connect the first two to the existing `wheelchair_navigation`
`fast_lio_icp` adapter and independent localization guard only after replay
calibration. A successful native match alone does not authorize motion.
