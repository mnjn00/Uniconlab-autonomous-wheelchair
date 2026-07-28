"""Body-frame to lidar-frame correction, shared by the motion nodes.

FAST-LIO publishes /cloud_registered_body in the IMU BODY frame. Since the
VN-100 swap that is no longer the lidar frame: the measured extrinsic puts
the lidar 14.5 cm forward and 6.8 cm above the body origin with a full
three-axis rotation. Every geometry constant in the follower and the gate
(sensor height, corridor half-width, stop distances) was tuned in the
lidar/chair frame, so the exact configured transform is inverted here
rather than each constant being re-derived.

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

import numpy as np

LIDAR_IN_BODY_XYZ = (0.14465, -0.01507, 0.06811)
LIDAR_TO_BODY_ROTATION = (
    (0.998785, -0.048796, -0.006840),
    (0.048776, 0.998805, -0.003054),
    (0.006981, 0.002716, 0.999972),
)

LIDAR_IN_BODY_XYZ_BUILTIN = (-0.011, -0.02329, 0.04412)
LIDAR_TO_BODY_ROTATION_BUILTIN = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

_PROFILES = {
    "vn100": (LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION),
    "builtin": (
        LIDAR_IN_BODY_XYZ_BUILTIN,
        LIDAR_TO_BODY_ROTATION_BUILTIN,
    ),
}


def lidar_extrinsics(profile):
    try:
        return _PROFILES[profile]
    except KeyError:
        raise ValueError("unknown body-frame profile: %s" % profile)


def body_to_lidar(points, offset_xyz=LIDAR_IN_BODY_XYZ,
                  lidar_to_body_rotation=LIDAR_TO_BODY_ROTATION):
    """Express body-frame points (N,3) in the lidar (chair-aligned) frame.

    The extrinsic gives the lidar pose in the body frame, so this applies
    its inverse: p_lidar = R^-1 @ (p_body - offset).
    """
    if points is None or not len(points):
        return points
    shifted = np.asarray(points, dtype=np.float64) - np.asarray(
        offset_xyz, dtype=np.float64)
    rotation = np.asarray(lidar_to_body_rotation, dtype=np.float64)
    if shifted.ndim != 2 or shifted.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if rotation.shape != (3, 3):
        raise ValueError("lidar-to-body rotation must have shape (3, 3)")
    return np.linalg.solve(rotation, shifted.T).T
