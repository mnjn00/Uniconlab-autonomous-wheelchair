"""One mount height, agreed on by everything that consumes it.

The Livox sits 0.725 m above the ground on the front of the left armrest,
measured 2026-07-31 and corroborated by the 0727 map: sampling the merged
cloud under 77 flat patches of the recorded trajectory puts the optical
origin 0.775 m up (sd 0.018 m, three sampling radii agreeing to 8 mm).

It was 0.30 m in five places before that, and 0.30 m was never measured -
it was a placeholder that the URDF then anchored both sensor joints to. The
cost was not cosmetic. Obstacle height is computed as `z + sensor_height`,
so understating the mount by 0.425 m slid the whole detection band up with
it: the gate's 0.15-1.9 m window was really looking at 0.575-2.325 m above
ground, and nothing shorter than a 57 cm bollard existed as far as the chair
was concerned. The same error put the ground-blind radius at 2.4 m in the
follower's docstring when 0.725/tan(7 deg) makes it 5.9 m.

A constant that wrong for that long, in that many files, is what this test
is for. Sensor height belongs to the vehicle, not to whichever module last
needed a number.
"""

import re
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
HARDWARE_URDF = (ROOT / "src" / "wheelchair_description" / "urdf"
                 / "wheelchair_hardware.urdf.xacro")

MOUNT_HEIGHT_M = 0.725
# Livox optical origin relative to imu_link, built-in IMU profile.
LIDAR_ABOVE_IMU_M = 0.04412


def constant(path, name):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(r"^%s\s*=\s*([0-9.]+)" % re.escape(name), text,
                      re.MULTILINE)
    assert match, "%s does not define %s" % (Path(path).name, name)
    return float(match.group(1))


@pytest.mark.parametrize("path,name", [
    (SCRIPTS / "safety_gate.py", "SENSOR_HEIGHT_M"),
    (SCRIPTS / "obstacle_clusters.py", "SENSOR_HEIGHT_M"),
    (ROOT / "tools" / "digital_twin_rviz_pub.py", "SENSOR_HEIGHT"),
])
def test_every_consumer_uses_the_measured_mount_height(path, name):
    assert constant(path, name) == MOUNT_HEIGHT_M


def test_the_follower_default_matches_too():
    """It is a parameter here, but the default is what the vehicle runs -
    the bringup does not pass ~sensor_height."""
    text = (SCRIPTS / "waypoint_follower.py").read_text(encoding="utf-8")
    match = re.search(r'get_param\("~sensor_height",\s*([0-9.]+)\)', text)
    assert match and float(match.group(1)) == MOUNT_HEIGHT_M
    bringup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")
    assert "sensor_height" not in bringup


def test_the_urdf_puts_the_lidar_at_the_same_height():
    """TF and the perception constants describe one physical mount. When
    they disagree, RViz shows the obstacle boxes at a height the guard is
    not using, and the disagreement looks like a rendering quirk."""
    text = HARDWARE_URDF.read_text(encoding="utf-8")
    lidar = re.search(
        r'<joint name="lidar_joint".*?origin xyz="([-0-9. ]+)"', text,
        re.DOTALL)
    assert lidar, "lidar_joint is gone from the hardware URDF"
    assert float(lidar.group(1).split()[2]) == MOUNT_HEIGHT_M


def test_the_urdf_imu_height_follows_from_the_lidar_through_the_extrinsic():
    """imu_link is not independently measured. It is the lidar height minus
    the built-in IMU extrinsic, and deriving it any other way reintroduces
    the two-numbers-for-one-mount problem this file exists to stop."""
    text = HARDWARE_URDF.read_text(encoding="utf-8")
    imu = re.search(
        r'<joint name="imu_joint".*?origin xyz="([-0-9. ]+)"', text,
        re.DOTALL)
    assert imu, "imu_joint is gone from the hardware URDF"
    assert abs(float(imu.group(1).split()[2])
               - (MOUNT_HEIGHT_M - LIDAR_ABOVE_IMU_M)) < 1e-6


def test_the_rider_exclusion_box_reaches_below_the_ground():
    """Raw lidar z, so it moved when the mount height did. If its floor sits
    above the ground plane the rider's feet and the footrest fall outside
    the box and cluster as an obstacle travelling along in front of the
    chair - which, being stationary relative to the chair, the tracker would
    then happily call parked."""
    text = (SCRIPTS / "obstacle_clusters.py").read_text(encoding="utf-8")
    match = re.search(r"RIDER_EXCLUDE_Z\s*=\s*\(([^,]+),", text)
    assert match, "RIDER_EXCLUDE_Z is gone"
    floor = match.group(1).strip()
    assert "SENSOR_HEIGHT_M" in floor, (
        "the exclusion floor is a bare number again: %s" % floor)


def test_the_blind_radius_in_the_docstring_follows_from_the_mount():
    """The band exists because the sensor cannot see near ground. How near
    is a function of the mount height, and quoting a figure from a mount
    height that was never real understates what the band is carrying."""
    import math
    text = (SCRIPTS / "waypoint_follower.py").read_text(encoding="utf-8")
    quoted = re.search(r"cannot see ground within ~([0-9.]+) m", text)
    assert quoted, "the follower no longer states its ground-blind radius"
    expected = MOUNT_HEIGHT_M / math.tan(math.radians(7.0))
    assert abs(float(quoted.group(1)) - expected) < 0.15, (
        "docstring says %s m, geometry says %.1f m"
        % (quoted.group(1), expected))
