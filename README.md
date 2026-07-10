# ROS1 Noetic Wheelchair Simulation Scaffold

Simulation/development scaffold for an electric wheelchair platform targeting Ubuntu 20.04, ROS1 Noetic, Gazebo Classic 11, catkin, `move_base`, `costmap_2d`, and a mandatory safety gate between navigation and the base command interface.

This repository intentionally does not claim medical-device certification or real-world safety approval. Treat every parameter and model as a starting point for simulation and development only.

## Stack target

- Ubuntu 20.04
- ROS1 Noetic (`roslaunch`, `rosparam`, catkin)
- Gazebo Classic 11 with `gazebo_ros` and `gazebo_ros_control`
- ROS navigation stack: `move_base`, `costmap_2d`, `dwa_local_planner`
- Command path: `move_base` -> `/cmd_vel_nav` -> `wheelchair_safety/safety_gate.py` -> `/cmd_vel_safe` -> base-controller interface

ROS2 Jazzy, Gazebo Harmonic/Gazebo Sim, Nav2, `ros_gz`, `gz_ros2_control`, ROS2 launch files, and ROS2 launch testing are intentionally not used.

## Ubuntu 20.04 / ROS1 Noetic bootstrap

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-navigation \
  ros-noetic-dwa-local-planner \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-xacro \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-topic-tools \
  python3-catkin-tools python3-pytest
```

## Environment check

```bash
cd /home/mnjn/projects/wheelchair_ros1_noetic_sim
./scripts/check_ros1_noetic_env.sh
```

The script reports `roscore`, `roslaunch`, `catkin_make`, `xacro`, and Gazebo availability. Missing tools are expected on non-Noetic hosts.

## Build

```bash
cd /home/mnjn/projects/wheelchair_ros1_noetic_sim
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Launch commands

Display only:

```bash
roslaunch wheelchair_description display.launch
```

Gazebo Classic 11 simulation with navigation and safety gate:

```bash
roslaunch wheelchair_bringup sim_bringup.launch world:=sidewalk_obstacles
```

Other world choices:

```bash
roslaunch wheelchair_bringup sim_bringup.launch world:=empty
roslaunch wheelchair_bringup sim_bringup.launch world:=road_free_space
```

Navigation-only launch:

```bash
roslaunch wheelchair_navigation navigation.launch use_sim_time:=true
```

Safety gate only:

```bash
roslaunch wheelchair_safety safety.launch
```

## Static/unit verification on any Python 3 host

```bash
cd /home/mnjn/projects/wheelchair_ros1_noetic_sim
python3 -m pytest -q
```

The tests validate URDF/xacro invariants, image-derived dimensions, navigation/safety parameters, safety-gate priority logic, speed caps, e-stop latch/reset, stale watchdog behavior, and the no-bypass command wiring invariant.

## Hardware-porting notes

- Intel NUC + RTX 2060/2080 class GPU: keep ROS1 Noetic on Ubuntu 20.04 for this scaffold; install NVIDIA drivers compatible with the target kernel and Gazebo Classic rendering.
- 3D LiDAR: publish a `sensor_msgs/PointCloud2` stream on `/lidar/points` or adjust `wheelchair_navigation/config/costmap_common.yaml` observation sources.
- Base controller: the hardware driver must consume `/cmd_vel_safe` or a relay from `/cmd_vel_safe`. Do not subscribe the base driver directly to `/cmd_vel_nav`, `/cmd_vel_raw`, or `/cmd_vel`.
- Odometry: publish `/odom` and `odom -> base_footprint` TF, or configure the base driver/controller to provide equivalent transforms.
- E-stop: wire hardware and software e-stop states to `/safety/estop`; reset is explicit through `/safety/estop_reset` after the unsafe condition clears.


## Real campus route data

The real Livox/IMU rosbag used for the Hanyang Aegimun ↔ Engineering Center loop is stored locally at `/home/mnjn/다운로드/livox`. The bag is multi-GB and is intentionally not committed to Git. Its metadata is committed at `data/hanyang_aegimun_loop/livox_rosbag_metadata.yaml`.

Committed route/map artifacts:

- `data/hanyang_aegimun_loop/map.yaml` / `map.pgm`: 2D occupancy map extracted from the GLIM output.
- `data/hanyang_aegimun_loop/map.metadata.json`: map and grade statistics.
- `data/hanyang_aegimun_loop/hanyang_aegimun_loop.waypoints.yaml`: recorded closed-loop route waypoints.

## Existing wheelchair platform reference

The platform manual is local-only at `/home/mnjn/다운로드/20230725_wheel_manual (2).pdf`. It documents the ROS1 workflow: `base_model bringup.launch`, `base_model navigation.launch`, `rviz_wheel.launch`, and `base_model automode.py` (`a` = auto, `m` = manual). Keep the NUC on Ubuntu 20.04 + ROS1 Noetic; do not replace it with the ROS2/Gazebo Sim experiment stack.
