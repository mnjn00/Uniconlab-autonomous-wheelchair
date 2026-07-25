"""Body-frame to lidar-frame correction, shared by the motion nodes.

FAST-LIO publishes /cloud_registered_body in the IMU BODY frame. Since the
VN-100 swap that is no longer the lidar frame: the measured extrinsic puts
the lidar 14.5 cm forward, 6.8 cm above and 2.80 deg in yaw from the body
origin. Every geometry constant in the follower and the gate (sensor
height, corridor half-width, stop distances) was tuned in the lidar/chair
frame, so the cloud is rotated back into that frame rather than each
constant being re-derived.

Leaving the constants alone silently put the ground plane 6.8 cm off, read
obstacles 14.5 cm farther away than they were, and skewed the forward
corridor by 2.80 deg - 0.17 m of lateral displacement at 3.4 m, enough to
miss an object inside the chair's own half-width.

Deliberately free of ROS imports so it is unit-testable without a ROS
install; see test/test_body_frame_geometry.py.

Values mirror config/fastlio_mid360_vn100.yaml extrinsic_T/extrinsic_R.
Running on the lidar's built-in IMU (VN_IMU=0) needs the config/mid360.yaml
values instead: offset (-0.011, -0.02329, 0.04412), yaw 0.
"""

import math

import numpy as np

LIDAR_IN_BODY_XYZ = (0.14465, -0.01507, 0.06811)
LIDAR_IN_BODY_YAW_RAD = math.radians(2.797)

# built-in IMU fallback, for when the stack is launched with VN_IMU=0
LIDAR_IN_BODY_XYZ_BUILTIN = (-0.011, -0.02329, 0.04412)
LIDAR_IN_BODY_YAW_RAD_BUILTIN = 0.0


def body_to_lidar(points, offset_xyz=LIDAR_IN_BODY_XYZ,
                  yaw_rad=LIDAR_IN_BODY_YAW_RAD):
    """Express body-frame points (N,3) in the lidar (chair-aligned) frame.

    The extrinsic gives the lidar pose in the body frame, so this applies
    its inverse: p_lidar = Rz(-yaw) @ (p_body - offset).
    """
    if points is None or not len(points):
        return points
    shifted = np.asarray(points) - np.asarray(offset_xyz, dtype=np.float64)
    c, s = math.cos(-yaw_rad), math.sin(-yaw_rad)
    out = np.empty_like(shifted)
    out[:, 0] = c * shifted[:, 0] - s * shifted[:, 1]
    out[:, 1] = s * shifted[:, 0] + c * shifted[:, 1]
    out[:, 2] = shifted[:, 2]
    return out
