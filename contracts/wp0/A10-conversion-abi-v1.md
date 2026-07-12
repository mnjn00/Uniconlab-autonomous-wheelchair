---
schema_version: 1
artifact_id: A10-conversion-abi-v1
owner: WP0-SW-data-localization-owner
reviewer: WP0-architect-and-critic
status: candidate
provenance:
  source: Stage-2-approved-plan-Appendix-C
  source_directory: /home/mnjn/projects/rosbag-glim-setup/livox_ros_driver2/msg/
  runtime_boundary: offline-ros2-glim-only
---

# A10 — ROS 2 Livox/IMU to canonical ROS 1 bag conversion ABI v1

## Authority and boundary

This contract applies only to the digest-pinned offline ROS 2/GLIM artifact pipeline. It grants no runtime ROS 2 dependency, hardware-motion authority, passenger authority, or localization-accuracy claim. Unknown physical facts and unverified alignment remain blocking for fused-map qualification.

The converter accepts exactly one ROS 2 bag directory. Before decoding any row it MUST validate `metadata.yaml`, every sqlite file and SHA-256, the storage plugin and version, serialization format `cdr`, ROS distribution, `livox_ros_driver2` revision, topic names/types/counts/duration, and the source IDL evidence below against the input manifest. Only staging mismatches with code `E_SOURCE_DISCOVERY` and audited reason `zero_byte_expected_populated_suffixed_segment` or `pre_existing_source_symlink_alias` are accepted; every other mismatch, discovery ambiguity, missing evidence, or validation failure fails closed.

## Source IDL discovery evidence

The authoritative discovery location is:

`/home/mnjn/projects/rosbag-glim-setup/livox_ros_driver2/msg/`

The recorded evidence is hash-only; this artifact does not invent or claim to reproduce uninspected source bytes:

| Source object | SHA-256 |
|---|---|
| `CustomMsg` source bytes | `f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96` |
| `CustomPoint` source bytes | `b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59` |
| `CustomMsg.msg` name + NUL + bytes + trailing NUL + `CustomPoint.msg` name + NUL + bytes + trailing NUL | `8d51083a4570d6e81f3193c9b8c39e16d2d5fb2d776dd198a997c7c5c6f4aac7` |

The composite input is the literal UTF-8 basename `CustomMsg.msg`, one `0x00` byte, the exact `CustomMsg.msg` file bytes, one trailing `0x00` byte, then the literal UTF-8 basename `CustomPoint.msg`, one `0x00` byte, the exact `CustomPoint.msg` file bytes, and one trailing `0x00` byte, with no additional newline insertion, path prefix, normalization, or sorting. The source contract is conceptually `CustomPoint(offset_time:uint32, x/y/z:float32, reflectivity/tag/line:uint8)` and `CustomMsg(header, timebase:uint64, point_num:uint32, lidar_id:uint8, rsvd:uint8[3], points:CustomPoint[])`; only the hash-pinned source bytes determine exact field order and syntax. Any type, field, order, bound, byte, or hash mismatch is `E_SOURCE_IDL` and requires ABI review, never converter adaptation.

Rows are read in ascending `(storage_timestamp_ns, sqlite_row_id)`. This order is authoritative and MUST NOT be sorted, repaired, or otherwise changed.
The digest-pinned source evidence contains 6,882 Livox clouds. Every cloud preserves acquisition order but has interleaved, non-monotonic `uint32 offset_time` values; across the clouds there are 910,296 adjacent decreases, where an adjacent decrease is an index `i > 0` for which `offset_time[i] < offset_time[i-1]`. This is an observed sensor ordering property, not corruption.

## Time model

Four times remain distinct:

1. **Livox source time:** `source_time_ns = CustomMsg.timebase`.
2. **Per-point source time:** `point_time_ns[i] = timebase + offset_time[i]`. Every offset MUST be an exact `uint32`, addition MUST not overflow unsigned 64-bit nanoseconds, and the minimum/maximum point times MUST be computed over every point rather than inferred from the first/last point. Adjacent offset decreases are valid and MUST be counted, not rejected or repaired.
3. **IMU source time:** the exact decoded `sensor_msgs/msg/Imu.header.stamp`.
4. **Storage time:** the exact rosbag2 sqlite record timestamp. It becomes the ROS 1 bag record timestamp without conversion. Equal storage timestamps retain sqlite row order.

The original Livox header stamp, raw source times, storage time, per-cloud minimum/maximum offsets, minimum/maximum point times, and adjacent-decrease count are retained in `records.jsonl`; the exact raw offsets and their acquisition order are retained in the canonical PointCloud2 payload and covered by its record hash. The Livox header stamp MUST be within 1 ms of `timebase` for qualification and is never substituted for it. The normalized cloud header stamp is aligned `timebase`; the normalized IMU header stamp is its original source stamp plus only its signed fixed offset.

The signed alignment artifact MUST validate against `time-alignment-schema.json`. Only `identity` and one constant `fixed_offset_ns` per sensor are permitted. Data-dependent, per-row, per-point, segmented, interpolated, or clock-warp alignment is forbidden. The repository default is identity, zero offsets, and `verified:false`. Qualification requires calibration evidence, p99 cross-sensor residual no greater than 2 ms, and drift no greater than 0.5 ms/min over the bag. Unverified alignment permits ingestion tests only and cannot qualify fused localization or map accuracy.

The output bag MUST NOT contain `/clock`. Replay has exactly one clock publisher, `rosbag play --clock`; all consumers set `/use_sim_time=true` before playback starts, playback starts paused, and absolute bag storage time is preserved. A source stamp regression or a normalized header more than 50 ms ahead of replay clock blocks localization qualification. Backward replay-clock movement is a stop/disarm condition, not a reason to rewrite stamps.

## Frame mapping

The only canonical outputs are:

| Input | Output topic | Output frame |
|---|---|---|
| Livox `CustomMsg` | `/sensors/lidar/points` | `lidar_link` |
| ROS 2 `sensor_msgs/msg/Imu` | `/sensors/imu/data` | `imu_link` |

The normalization manifest records every exact source-frame-to-canonical-frame mapping and its evidence. An alias is allowed only when evidence proves both names denote the same physical frame. Conversion performs no rotation, translation, axis swap, unit conversion, or coordinate transform. Empty, unknown, contradictory, or mismatched frames abort the transaction with `E_FRAME_MAPPING`.

## Canonical PointCloud2 ABI

Each accepted `CustomMsg` produces exactly one ROS 1 `sensor_msgs/PointCloud2`: `height=1`, `width=point_num`, little-endian, `is_dense=true`, `point_step=24`, and `row_step=24*width`. No padding exists beyond the fields below.

| Offset | Field | PointField datatype / count | Required value |
|---:|---|---|---|
| 0 | `x` | `FLOAT32 / 1` | source metres, unchanged |
| 4 | `y` | `FLOAT32 / 1` | source metres, unchanged |
| 8 | `z` | `FLOAT32 / 1` | source metres, unchanged |
| 12 | `intensity` | `FLOAT32 / 1` | exact numeric conversion of reflectivity 0..255; no scaling |
| 16 | `offset_time` | `UINT32 / 1` | source nanoseconds after cloud header stamp |
| 20 | `line` | `UINT8 / 1` | source byte unchanged |
| 21 | `tag` | `UINT8 / 1` | source byte unchanged |
| 22 | `reflectivity` | `UINT8 / 1` | source byte unchanged |
| 23 | `lidar_id` | `UINT8 / 1` | message `lidar_id`, copied to every point |

`point_num` MUST equal the decoded array length. Reserved `rsvd` bytes are copied into the corresponding record-index entry. Coordinates and derived intensity MUST be finite and representable. Every point and exact offset value is emitted once in source acquisition order. The converter MUST NOT sort, drop, clamp, repair, or reject points solely because an adjacent offset decreases.

## Canonical IMU ABI

Each accepted source IMU produces exactly one ROS 1 `sensor_msgs/Imu`. Orientation quaternion, angular velocity in rad/s, linear acceleration in m/s², and all three 3x3 covariance arrays are copied bit-for-bit after CDR decode. The converter MUST NOT synthesize a quaternion, gravity, bias, zero covariance, or `covariance[0] = -1`.

All numeric fields MUST be finite. When orientation is declared available, a nonzero quaternion norm outside `1 ± 0.01` is malformed and aborts conversion. An explicitly unavailable orientation and its source sentinel are preserved exactly and set fusion qualification false; it is not corruption and is not repaired. Covariance contents and availability semantics must be verifiable from the source. Unverifiable covariance is reported and blocks fusion qualification; structural or nonfinite covariance aborts conversion.

## Transaction and deterministic artifacts

Conversion is one transaction. Work may exist only under an unaccepted temporary name. The final artifacts become accepted together only after all source rows, counts, duration, ordering, semantic hashes, and artifact hashes validate. On any error the process exits nonzero, emits a canonical error report, and leaves no accepted `normalized.bag`, manifest, or record index. It MUST NOT silently drop, duplicate, repair, clamp, sort, reorder, interpolate, substitute, or partially publish any row or point. A reviewed waiver creates a new derived-dataset ID and cannot claim zero-loss normalization.

The accepted bag is ROS bag v2 with compression `none`, fixed 768 KiB chunk threshold, connection order `/sensors/lidar/points` then `/sensors/imu/data`, fixed caller IDs, and no volatile metadata. Accepted artifacts are:

- `normalized.bag` and its SHA-256;
- `normalization_manifest.yaml` with `schema_version: 1`, `artifact_id: wheelchair.normalized_livox/v1`, owner/reviewer/status/provenance, source ABI and artifact hashes, storage/container/tool/revision hashes, parameters, frame mappings, and qualification flags. Its `output` object contains relative `bag_path: normalized.bag`, the bag `sha256`, `format: rosbag1-v2`, canonical topic/type/count records for `/sensors/lidar/points` then `/sensors/imu/data`, compression/chunk settings, counts, first/last source and storage times, clock statistics, and `offset_time_regression_statistics`. Legacy artifact IDs or `bag_sha256` aliases are not accepted;
- canonical UTF-8 `records.jsonl`, sorted keys and no insignificant whitespace, exactly one line per source row, containing sqlite row ID, topic, storage/source/min/max-point nanoseconds, point count, reserved bytes, input payload SHA-256, decoded canonical payload SHA-256, and, for each cloud, `minimum_offset_time`, `maximum_offset_time`, and `adjacent_offset_decrease_count`;
- `conversion_report.json`, containing exact counts, errors, gaps, semantic-stream SHA-256, and the same aggregate `offset_time_regression_statistics` as the manifest.

`offset_time_regression_statistics` contains `cloud_count`, `point_count`, `clouds_with_adjacent_decreases`, and `adjacent_offset_decrease_count`. The converter computes these values from each source cloud while preserving order. The independent verifier MUST unpack the accepted PointCloud2 payloads, recompute every per-cloud minimum, maximum, and adjacent-decrease count, aggregate those counts, and require exact equality with both report and manifest. For the digest-pinned source, `cloud_count` and `clouds_with_adjacent_decreases` are both 6,882 and `adjacent_offset_decrease_count` is 910,296.

Three runs with identical pinned inputs MUST produce identical bag bytes, manifest bytes, record-index bytes, semantic-stream hash, and report bytes.

## Stable failures

A validator MUST return the applicable stable code below and the source row/topic when available. Multiple errors are ordered by source row then this table order. Every code is fatal to acceptance unless explicitly described as a qualification-only result.

| Code | Condition |
|---|---|
| `E_SCHEMA_VERSION` | unsupported artifact or manifest schema |
| `E_UNKNOWN_FIELD` | extra manifest/schema field |
| `E_SOURCE_DISCOVERY` | missing/ambiguous source directory, metadata, sqlite, topic, or type evidence |
| `E_SOURCE_MANIFEST` | file hash, storage plugin/version, CDR, ROS distribution, driver revision, topic/type/count/duration mismatch |
| `E_SOURCE_IDL` | either source hash, composite hash, or decoded IDL contract mismatch |
| `E_CDR_DESERIALIZE` | payload cannot be decoded exactly |
| `E_TOPIC_TYPE` | row topic/type differs from the manifest |
| `E_POINT_COUNT` | `point_num` differs from points array length |
| `E_NONFINITE` | nonfinite coordinate, IMU field, or covariance |
| `E_POINT_TIME_OVERFLOW` | `timebase + offset_time` overflows |
| `E_SOURCE_TIME_REGRESSION` | source time regresses where monotonicity is required |
| `E_STORAGE_ORDER` | storage timestamp/row order regresses |
| `E_HEADER_TIME_RESIDUAL` | Livox header differs from timebase by more than 1 ms; conversion may be recorded but qualification is blocked |
| `E_ALIGNMENT_SCHEMA` | alignment artifact is absent, unsigned/untrusted, unknown-field, or schema-invalid |
| `E_ALIGNMENT_UNQUALIFIED` | verified residual/drift evidence is absent or exceeds 2 ms / 0.5 ms/min; ingestion only |
| `E_FRAME_MAPPING` | frame is empty, unknown, contradictory, or not an evidence-backed alias |
| `E_POINT_LAYOUT` | canonical 24-byte layout/value requirement cannot be met exactly |
| `E_IMU_MALFORMED` | structural IMU error or declared quaternion has invalid norm |
| `E_IMU_COVARIANCE` | covariance availability/content cannot be verified; fusion qualification is false |
| `E_CLOCK_FUTURE` | normalized header exceeds replay clock by more than 50 ms; qualification blocked |
| `E_COUNT_DURATION` | final topic/total counts or duration differ from source manifest |
| `E_OUTPUT_DETERMINISM` | any required artifact or semantic hash differs across identical runs |
| `E_TRANSACTION` | atomic finalization/no-partial-output invariant cannot be guaranteed |

Golden evidence MUST cover distinct interleaved point offsets with exact acquisition order/value preservation, all-point minimum/maximum times, per-cloud and aggregate adjacent-decrease counts, distinct line/tag/reflectivity/lidar ID values, equal-storage-time cloud ordering, header/timebase residual, nontrivial IMU quaternion/covariances, fixed alignment, and replay-clock ages. Corrupt evidence MUST cover IDL mismatch, CDR failure, point-count mismatch, non-`uint32` offset, NaN, point-time overflow, source/storage regression, frame mismatch, malformed quaternion, aggregate-statistics mismatch, and final count mismatch, each with its stable code and no accepted bag. An adjacent offset decrease is explicitly excluded from corrupt evidence.
