# Wheelchair description profiles

The two descriptions are deliberately separate because they have different
evidence and must never be selected interchangeably.

| Profile | Model | Launch | Authority |
| --- | --- | --- | --- |
| Simulation | `wheelchair.urdf.xacro` | `display.launch` or Gazebo `spawn.launch` | Nominal dimensions, simulated sensors and Gazebo plugins only |
| Physical NUC wheelchair | `wheelchair_hardware.urdf.xacro` | `hardware_description.launch` | Active NUC chassis copied from `base_model`, corrected with measured 7/27 sensor offsets |

The hardware copy preserves the active NUC model's chassis parts and records
its source SHA-256 in the Xacro header. Its `base_footprint` is the drive-axle
midpoint, while the legacy chassis `base_link` remains 0.260 m forward. The
Livox built-in IMU/body and LiDAR frames use the offset fitted from physical
in-place rotations plus the configured built-in-IMU extrinsic.

Do not use the simulation model on the wheelchair. Its LiDAR and IMU placement
is explicitly nominal, and its Gazebo plugins publish simulation-only sensor
topics. Do not use the hardware model to claim Gazebo fidelity: the remaining
legacy chassis dimensions were copied from the NUC and still require a physical
dimension survey before they can be treated as certified geometry.

The hardware launch publishes description TF only. The wheel odometry publisher
must publish `odom -> base_footprint`; publishing axle-centred odometry as
`odom -> base_link` recreates the 0.260 m frame disagreement this profile fixes.
