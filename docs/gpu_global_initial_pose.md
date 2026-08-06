# GPU global initial-pose search

The RTX 2060 is used only for the wide coarse pass.  Exact cKDTree scoring,
local refinement, ambiguity rejection, and ICP verification remain the
authorities that may accept a pose.

## Search contract

1. Build a sorted sparse voxel-support map on the GPU.
2. Expand every 3 m mapping-trajectory sample laterally by 10 m at 1 m steps.
3. Score every position at 30 degree yaw steps in chunks of 1,024 poses.
4. Send the best 256 GPU hypotheses back through the historical CPU cKDTree
   metric.
5. Apply the existing diverse shortlist, local refinement, ambiguity gate,
   minimum score, and ICP verification without weakening them.

The field launcher sets `AUTO_INIT_REQUIRE_GPU=true` by default.  A missing
driver, CUDA library, allocation, or kernel therefore terminates automatic
initialisation without publishing a pose.  Set it to `false` explicitly only
to run the old trajectory-only CPU search.

Useful overrides:

```bash
AUTO_INIT_REQUIRE_GPU=true \
AUTO_INIT_GPU_LATERAL_RADIUS_M=10.0 \
AUTO_INIT_GPU_LATERAL_STEP_M=1.0 \
~/start_wheelchair_localization.sh
```

An isolated branch workspace can be exercised without replacing the live
catkin package:

```bash
LOCALIZATION_WS="$HOME/gpu_global_initial_pose_ws" \
  "$HOME/gpu_global_initial_pose_trial/tools/start_wheelchair_localization.sh"
```

CUDA component-wheel library directories are discovered below
`~/.local/lib/python3.8/site-packages/nvidia/*/lib` by the launcher.  The ROS
and NVIDIA system installations are not modified.

## Measured on the deployed NUC and map

Date: 2026-08-05.  RTX 2060 6 GB, driver 570.133.07, CuPy 12.3.0, CUDA runtime
12.2, 2,696,359-point runtime PCD.

- Sparse map: 1.7 million support keys, 0.45 m voxels.
- Full coarse pass: 778 trajectory samples, 16,338 expanded XY positions,
  196,056 XY/yaw poses, 1,800 scan points.
- GPU coarse plus CPU exact re-rank: 4.93 seconds in the integrated test.
- A known map-derived pose was the unique GPU winner and remained the exact
  cKDTree winner with score 1.0.

These are implementation gates, not field localisation evidence.  Replay
several stationary bags from different starts before deploying this branch.

## Deliberately not moved to the GPU

- MPC/OSQP, FAST-LIO, clustering, the safety band, and drive policy remain on
  the CPU.
- GICP and point-cloud preprocessing are the next GPU candidates, but belong
  in a separate branch.  First record moving_icp wall time and CPU, MPC solve
  p99, GPU power/temperature, and battery draw.  A FastVGICPCuda/NDTCuda change
  must preserve the current transform, convergence, and failure contracts and
  must be built by `push_to_nuc.sh` on Ubuntu 20.04/Noetic.
