# Replay and offline mapping

These workflows are non-destructive and workstation/offline only. Keep the source ROS 2 bag outside the repository and immutable. Do not commit user data, absolute private source paths, credentials, or a multi-GB bag. Outputs remain candidate evidence until every required hash, calibration, survey, and physical gate is separately satisfied.

Use distinct, empty output directories on a filesystem with adequate free space. Never point an output at the source.

## 1. Verify and stage the ROS 2 Livox source

Full pinned verification checks sqlite3/CDR metadata and records, expected `/livox/lidar` and `/livox/imu` types/counts, total count/duration, source discovery, and SHA-256:

```bash
python3 scripts/verify_rosbag2_manifest.py SOURCE_ROSBAG2 --hash --output WORK/source-verification.json
python3 scripts/stage_rosbag2_source.py SOURCE_ROSBAG2 WORK/staged --output-json WORK/staging-result.json
```

Staging refuses an existing destination and a destination inside the source. It copies `metadata.yaml` and creates absolute symlinks to the unchanged source sqlite segment; `WORK/staged/rosbag2_source_manifest.json` binds the staged view to hashes and file identity. A source that changes during staging is rejected and the partial staging directory is removed. `--no-pinned-expectations` is for unrelated internally consistent fixtures only and cannot qualify this dataset.

The verified source contains 144,484 records over 688.225098527 s: 6,882 `livox_ros_driver2/msg/CustomMsg` records and 137,602 `sensor_msgs/msg/Imu` records. The immutable 2,813,546,496-byte physical sqlite has SHA-256 `f3773d88c9391e25e70b2019d0ffa1a9b2d48beebbb2afcbb9af5566ae8f4ae5`; its pre-existing declared-name alias was not mutated. The generated staging manifest has SHA-256 `bd90d38f16e8dd539c3dee0cd19c17aa4e78a5c932dfee7fd08f9847df1cb745`.

## 2. Normalize to canonical ROS 1

Install the exact offline dependency only in the offline conversion environment, never on the deployment NUC:

```bash
python3 -m pip install --requirement tools/offline/requirements.lock
```

Provide reviewed source IDL files, alignment JSON, source frame names, a real frame-evidence SHA, and provenance values; do not substitute guessed extrinsics or hashes:

```bash
python3 scripts/normalize_livox_bag.py \
  WORK/staged/rosbag2_source_manifest.json WORK/normalized \
  --alignment INPUT/alignment.json \
  --custom-msg-idl INPUT/CustomMsg.msg \
  --custom-point-idl INPUT/CustomPoint.msg \
  --lidar-source-frame SOURCE_LIDAR_FRAME \
  --imu-source-frame SOURCE_IMU_FRAME \
  --frame-evidence-sha256 FRAME_EVIDENCE_SHA256 \
  --ros-distribution SOURCE_ROS_DISTRIBUTION \
  --livox-driver-revision SOURCE_DRIVER_REVISION \
  --owner OWNER --reviewer REVIEWER
python3 scripts/verify_normalized_bag.py WORK/normalized
```

The normalizer validates frozen source IDL hashes, point layout/finite values/count/order, Livox `timebase + offset_time`, header residual, IMU quaternion/covariances, source/storage time, alignment schema, frames, and record ordering. It writes the directory transaction atomically or a sibling `.conversion_error.json`; it never accepts partial output. The accepted directory contains `normalized.bag`, `records.jsonl`, `conversion_report.json`, and `normalization_manifest.yaml`. Fusion remains unqualified unless the alignment is explicitly verified (p99 residual ≤2 ms and drift ≤0.5 ms/min) and IMU orientation is available.

### Recorded full-bag result

`artifacts/software_rc/full-bag-normalization.json` is the canonical shareable evidence; the 3.1 GB normalized bag remains external and uncommitted. Three independent full conversions produced byte-identical bag, records, report, and manifest files and the same semantic stream. An independent verifier returned `ok` with exact hashes and counts: 6,882 clouds, 137,602 IMUs, 144,484 total records, and 137,594,880 points.

All 6,882 clouds retain legitimate source-order interleaving, totaling 910,296 adjacent offset decreases. No point was sorted, dropped, or repaired. This passes WP2 zero-loss deterministic normalization for ingestion and replay consistency only.

Alignment remains identity with zero offset and `verified: false`. The source recording ROS distribution and storage-plugin package revision, lidar-IMU calibration, extrinsics, odom, TF, commands, and independent truth remain unknown. The observed local Livox checkout revision `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` is not proof of the recording revision. The pinned rosbags 0.10.11 Jazzy typestore was only a compatible CDR decoding parameter; it does not show that the bag was recorded on Jazzy. Fusion, localization, hardware, campus, and passenger qualification remain blocked, and both hardware-motion and passenger-operation authority remain false.

## 3. Offline GLIM repeatability

The normalizer and verifier now share the reviewed `wheelchair.normalized_livox/v1`
manifest ABI (`output.bag_path`, `output.sha256`, `output.format`, and exact topic
metadata). The explicit offline exporter converts that immutable ROS 1 artifact
into a hash-bound `wheelchair.glim_rosbag2_input/v1` directory; GLIM never reads
an edited or ad-hoc manifest.

```bash
python3 scripts/export_glim_rosbag2.py \
  --input-manifest WORK/normalized/normalization_manifest.yaml \
  --output-dir WORK/glim-input
python3 scripts/run_glim_repro.py \
  --ros2-manifest WORK/glim-input/glim_rosbag2_manifest.json \
  --config INPUT/glim.json \
  --output-dir WORK/glim-repro \
  --image REGISTRY/GLIM_IMAGE@sha256:IMAGE_DIGEST
python3 scripts/compare_glim_runs.py \
  --repro-dir WORK/glim-repro \
  --output WORK/glim-comparison.json
```

`run_glim_repro.py` requires the Dockerfile-pinned full GLIM source revision, immutable image digest, one thread and fixed seed by default, network disabled, read-only container root, read-only inputs, and three isolated runs. Each run must produce `trajectory.csv`, `occupancy.pgm`, and `occupancy.yaml`; manifests bind command, input, config, image, artifacts, timing, and resources. `compare_glim_runs.py` checks timestamp coverage, pairwise SE(2)-aligned trajectory consistency, occupancy IoU, and loop residual. Its label is `REPLAY_CONSISTENCY_NOT_TRUTH`; repeatability and loop closure do not prove map or localization accuracy.

## 4. Export a candidate 2D map and directional routes

Export from reviewed numeric cloud and trajectory arrays. Use only measured footprint values when evidence exists; `simulation` is permitted solely for software artifacts. Use a real safety-manifest SHA instead of the zero default for any qualification attempt:

```bash
python3 scripts/export_glim_2d_map.py \
  --cloud WORK/cloud.npy \
  --trajectory WORK/trajectory.npy --trajectory-has-time \
  --output-dir WORK/map-candidate \
  --map-name map --route-name hanyang_routes \
  --footprint-source simulation \
  --footprint-width WIDTH_M --footprint-length LENGTH_M \
  --clearance-margin MARGIN_M \
  --split-index REVIEWED_INTERIOR_INDEX \
  --safety-manifest-sha256 SAFETY_MANIFEST_SHA256
```

The exporter projects against declared gravity, constructs a trinary occupancy grid, creates separate outbound/return routes at an explicit interior split, computes grade and loop-consistency diagnostics, binds hashes, writes via staged atomic replacement, and sets every hardware speed and authorization to zero/false. Outputs are `map.pgm`, `map.yaml`, `map.metadata.json`, and `hanyang_routes.yaml` with default names.

Validate map/route binding, geometry, occupancy clearance, direction split, hashes, candidate labels, and recomputed grade:

```bash
python3 scripts/validate_hanyang_route.py \
  --map-yaml WORK/map-candidate/map.yaml \
  --route WORK/map-candidate/hanyang_routes.yaml \
  --metadata WORK/map-candidate/map.metadata.json \
  --required-margin MARGIN_M
```

`valid` means internal candidate consistency only. Warnings prevent `candidate_qualified`; `physically_qualified`, `surveyed`, and `approved` remain false. Promotion requires measured extrinsics/time, surveyed datum/corridors/exclusions/crossings/directions, measured footprint and stopping uncertainty, independent truth, closed-course evidence, and campus approval.

## 5. Noetic replay of a verified normalized bag

Set every consumer to simulation time before starting the sole paused clock publisher:

```bash
roslaunch wheelchair_bringup bringup.launch use_sim_time:=true hardware_profile:=replay
rosbag play --clock --pause WORK/normalized/normalized.bag
```

The bag must contain no `/clock`. Preserve absolute storage time; do not rewrite stamps. Source regression, a normalized header more than 50 ms ahead of replay clock, duplicate clock/TF owner, seek/backward clock, or hash mismatch blocks qualification. Replay may establish deterministic coverage, LOST/recovery behavior, and timing consistency, but the source lacks odom, TF, command, mode, driver, and independent truth, so it cannot establish absolute localization, route, control, braking, or hardware accuracy.
