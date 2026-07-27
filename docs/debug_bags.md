# Debug bags: recording and dual-IMU comparison

Debug bags exist to answer one question at a time on the real chair. The
2026-07-27 rollback (`999028a`) was performed precisely so a drive could be
recorded with the rolled-back stack while the VN-100 runs beside it,
influencing nothing: both IMUs then see the same motion, and "which sensor
is better" stops being an argument and becomes a measurement.

Keep bags on the external SSD or the NUC. Never commit a bag, its absolute
paths, or credentials to this repository.

## 1. Recording

On the NUC, with the stack already running:

```bash
./record_debug_bag.sh                 # 10 minutes
DURATION=5m ./record_debug_bag.sh     # something else
```

`tools/record_debug_bag.sh` records every live topic (minus large
reconstructible previews), stops on its own, reindexes after a power cut,
and writes a summary with a dropped-message check next to the bag. Read its
header before first use; it is the source of truth for exclusions.

Drive protocol that makes the data analysable:

1. sit still for at least a minute before moving,
2. drive the route, hill included,
3. sit still for at least a minute at the end, undisturbed - no bumping,
   no folding, nobody leaning on the chair.

The stationary bookends are what separate sensor drift from motion-induced
error. A bookend that is noisy on both sensors at once is physical
vibration, not noise; the 2026-07-27 bag's final 35 s was contaminated this
way and its bias-drift numbers are unusable.

Joystick driving is fine and expected here. The hardware joystick does not
pass through ROS: in the 2026-07-27 bag the chair drove 165.8 m (wheel
odometry path, 79.2 m max displacement, peaks of -1.07/+0.95 m/s) while
`/cmd_vel*`, `/mode_cmd` and `/joy` were never even advertised, and
`/wheel_cmd` sat advertised but silent. A joystick bag therefore contains
sensors only, never commands; command visibility comes for free only on
autonomous runs, where the black box in `tools/start_wheelchair_localization.sh`
already records the whole command chain.

## 2. Comparing the two IMUs

Install the reader offline, never on the deployment NUC:

```bash
python3 -m pip install rosbags numpy
```

Then one command reads any bag that has both IMUs and the wheel odometry:

```bash
python3 tools/compare_imu_bag.py /path/to/debug.bag
python3 tools/compare_imu_bag.py debug.bag --yaw-deg -2.80   # the default
```

The default yaw is the measured VN-100 <- MID360 extrinsic (SVD over 150 s
of shared gyro, residual 0.0144 rad/s; lever arm 14.4 cm back, 2.2 cm
across, 6.7 cm down). How to read the report:

- **accel units**: the Livox IMU reports acceleration in g, the VN-100 in
  m/s^2. Feeding one to a config tuned for the other silently breaks tilt.
  The rolled-back VN integration handled this (FAST-LIO rescales by the
  measured mean-accel norm at init; `tip_guard` reads angular velocity
  only) - the report exists to keep that checked rather than assumed.
- **stationary bias/noise**: compare the two sensors on the same still
  air. Bias is in deg/s with z also in deg/h; noise carries an ARW proxy.
- **bias drift** between bookends is only meaningful if the final bookend
  was calm; the tool flags it when both sensors get noisy at once.
- **moving agreement**: the gyro difference after yaw alignment, and each
  IMU's yaw rate against wheel odometry as an independent third source.
  When both IMUs disagree with the wheels by the same amount, the wheels
  are slipping and the IMUs are not the problem.
- **accel differences while moving** are dominated by the lever arm
  (omega^2 r reaches ~0.30 m/s^2 at the chair's max yaw rate) and must not
  be read as sensor error until that is compensated.

## 3. Findings from the 2026-07-27 bag

Bag `debug_20260727_193008.bag`: 357 s, 466,616 messages, zero dropped.
Stack in the rolled-back configuration (`VN_IMU=0`, FAST-LIO on
`/livox/imu`), VN-100 driver started separately and driving nothing.

- The chair moved, by joystick (see section 1): 165.8 m driven between
  50.2 s and 323.1 s of the bag, 55.5 % of samples moving.
- Localization held: `TRACKING` 318/340 diagnostics (93.5 %), `DEGRADED`
  22 (6.5 %). Measured `inlier_ratio` mean 0.365, min 0.169, against the
  configured `min_inlier_ratio: 0.20` in
  `src/static_livox_localization/config/moving_localization.yaml` - the
  floor is breached only at the worst moments, not at the operating point.
- **No VN-100 gyro advantage is demonstrated at wheelchair dynamics.**
  Both run at 200 Hz. After yaw alignment the moving gyro difference RMS
  is 0.69/0.92/0.67 deg/s (x/y/z). Stationary z-bias is +116.7 deg/h
  (Livox) vs +93.2 deg/h (VN) in the first bookend; z-noise 0.27 vs
  0.28 deg/s - both sit on the same environmental floor (both rise to
  1.88 deg/s together in the vibrated final bookend). Against wheel
  odometry yaw rate the two are indistinguishable: RMS 7.59 deg/s and
  correlation 0.877 each, i.e. the residual is wheel slip, common to both.
  Max yaw rate seen was 120 deg/s, far inside either sensor's range.
- Units, measured: `/livox/imu` stationary |g| = 0.9994 (g); `/vectornav/IMU`
  |g| = 9.5435 (m/s^2, but 2.7 % below 9.80665). Treat the VN-100
  accelerometer scale as unverified before any use that depends on tilt
  magnitude rather than tilt direction.
- Header stamps: `/livox/imu` trails `/vectornav/IMU` by ~4.2 ms (median).
- `/Odometry` (FAST-LIO) publishes pose but its twist is identically zero;
  nothing in `src/static_livox_localization` consumes twist, so this is
  harmless today and a trap for anything that later does.

## 4. What this changes, and what still needs a drive

Committed now, on the strength of this bag: the recorder already covered
everything (no change needed), the comparison is repeatable
(`tools/compare_imu_bag.py`), and the measurements above are on record.

Still open, each needing its own validation drive rather than a batch:

- a clean stationary bias-drift comparison (calm final bookend) before the
  VN-100 fusion is attempted again;
- moving validation of any re-fusion, since the earlier one was verified
  stationary-only and then rolled back with everything else;
- the four defects the rollback re-introduced (permissive safety-band
  brackets, `tip_guard` climb-boost bleed-through, unvalidated uart drive
  mode, hardcoded VNC password) plus fail-closed behaviour outside
  `TRACKING` - re-check individually, never re-apply as a set.
