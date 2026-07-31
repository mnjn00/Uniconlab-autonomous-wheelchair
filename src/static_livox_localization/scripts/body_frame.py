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

# Where the chair's centre sits in the body frame FAST-LIO reports.
#
# The sensor is mounted at the front of the LEFT armrest, so the point every
# pose is about is neither the middle of the chair nor on its centreline. That
# was measured rather than assumed, from the 2026-07-27 route recording
# itself: both spin-in-place bookends were fitted jointly for
#
#     p_body(map) = centre(map) + R(yaw) . r
#
# over 215 poses covering 371 deg and 798 deg, giving r = (0.517 forward,
# 0.173 left) with a 22.5 mm residual - the 798 deg spin alone fits to 7.5 mm.
# Four clean spins in the 0707/0725 mapping runs bracket the same value, so the
# mount did not move between mapping and recording. The chair's wheels are
# symmetric about its centre, so a true spin in place turns about that centre
# and the fitted r IS the mounting offset; a spin with unequal wheel speeds
# would have degraded the residual instead.
#
# Between-spin spread across all six measurements is about +-0.03 m, which is
# the honest uncertainty - five times smaller than the 0.173 m error that
# ignoring this leaves behind. tools/measure_mount_offset.py re-derives it from
# any bag containing a spin; run on the same recording it reports
# (-0.514, -0.171) by averaging the two spins separately rather than fitting
# them jointly, so the two routes to the number agree to 3 mm.
#
# Re-measure if the sensor is ever remounted. Nothing detects that on its own.
CHAIR_CENTRE_IN_BODY_XYZ = (-0.517, -0.173, 0.0)

REFERENCE_BODY = "body"
REFERENCE_CHAIR_CENTRE = "chair_centre"

_PROFILES = {
    "vn100": (LIDAR_IN_BODY_XYZ, LIDAR_TO_BODY_ROTATION),
    "builtin": (
        LIDAR_IN_BODY_XYZ_BUILTIN,
        LIDAR_TO_BODY_ROTATION_BUILTIN,
    ),
}


def _body_T_lidar(profile):
    offset, rotation = lidar_extrinsics(profile)
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=np.float64)
    matrix[:3, 3] = np.asarray(offset, dtype=np.float64)
    return matrix


def pose_correction(pose_profile, route_profile):
    """Express a pose given in `pose_profile`'s body frame in the body frame
    the route was captured in.

    FAST-LIO reports the pose of its IMU body frame, so the origin moves
    when the inertial source changes: 15.5 cm along the chair's forward
    axis and 2.80 deg in heading between the two profiles here. A route is
    a recording of that origin's path, so a route captured on one profile
    and driven on another is compared against the wrong point. Simulating
    the follower's own steering loop over the 2026-07-27 route, the
    mismatch costs 7 cm of mean cross-track (0.092 m -> 0.164 m), against a
    kerb clearance budget of 0.45 m.

        map_T_body_route = map_T_body_pose . body_pose_T_lidar
                                           . (body_route_T_lidar)^-1

    Identity when the profiles match, so this is inert on a route driven
    with the sensor it was recorded on.
    """
    return _body_T_lidar(pose_profile) @ np.linalg.inv(
        _body_T_lidar(route_profile))


def reference_correction(route_reference):
    """body_T_reference for the point a route is expressed about.

    Composed AFTER pose_correction, so it is applied in the body frame and
    therefore rotates with the chair. Applying it in the map frame instead
    would put the centre in the wrong place at every heading but due east.

    Identity for a body-referenced route, so routes recorded before this
    was measured keep exactly the behaviour they were validated with, and
    the reference has to be declared rather than guessed - driving a
    chair-centred route as a body-centred one displaces it 0.173 m sideways
    with nothing to reveal the mistake.
    """
    matrix = np.eye(4)
    if route_reference == REFERENCE_BODY:
        return matrix
    if route_reference == REFERENCE_CHAIR_CENTRE:
        matrix[:3, 3] = np.asarray(CHAIR_CENTRE_IN_BODY_XYZ, dtype=np.float64)
        return matrix
    raise ValueError("unknown route reference point: %s" % route_reference)


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
