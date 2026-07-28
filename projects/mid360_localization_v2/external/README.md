# Localization upstream dependencies

`localization_noetic.repos` pins the exact revisions used by the ROS1 Noetic
localization workspace. The project calls these implementations through their
published ROS/C++ interfaces instead of copying their algorithms.

- [HKU-MARS FAST-LIO](https://github.com/hku-mars/FAST_LIO), GPL-2.0:
  FAST-LIO2 LiDAR-inertial odometry and the official Mid-360 configuration.
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2),
  BSD-3-Clause: Mid-360 `CustomMsg` point cloud and built-in IMU transport.
- [hdl_global_localization](https://github.com/koide3/hdl_global_localization),
  BSD: FPFH-TEASER global pose candidate service.
- [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus), MIT: robust
  registration solver, built through the pinned koide3 fork required by
  `hdl_global_localization`.
- [fast_gicp](https://github.com/koide3/fast_gicp), BSD-3-Clause: CPU
  FastVGICP fixed-map refinement.

The compatibility patches are deliberately small and are checked against the
pinned commits before application. Review the upstream license files imported
by `vcstool` before distributing binaries.
