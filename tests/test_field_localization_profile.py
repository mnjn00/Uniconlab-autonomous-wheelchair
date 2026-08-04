import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
PACKAGE = ROOT / "src" / "static_livox_localization"
CANONICAL_SHA256 = "3639f5942101e67d8f62baf533017475146ebb681f4a8482ecaf0f2a7cec6536"
RUNTIME_SHA256 = "ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431"
RUNTIME_NAME = "merged_0707_0725_0p20m_xyzi.pcd"
# The shipped pair is chair-centred. A sensor-referenced route applies every
# clearance about a point 0.2 m left of the chair, which under-protects the
# right side by exactly that much; see body_frame.CHAIR_CENTRE_IN_BODY_XYZ.
ROUTE_NAME = "20260803_route_v5_waypoints.json"
BAND_NAME = "20260803_route_v5_safety_band.json"
# Superseded pairs. Deployment naming one of these while the bringup launches
# the other is how a record ends up describing a drive that did not happen,
# and the old files staying on disk is what lets it pass unnoticed.
SUPERSEDED = (
    "20260802_route_v4_waypoints.json",
    "20260802_route_v4_safety_band.json",
    "20260727_new_route_waypoints.json",
    "20260727_new_route_safety_band.json",
    "20260727_chair_centred_waypoints.json",
    "20260727_chair_centred_safety_band.json",
)


def shell_default(source, name):
    match = re.search(
        rf'^{name}="\$\{{{name}:-([^}}]+)\}}"$',
        source,
        flags=re.MULTILINE,
    )
    assert match is not None
    return match.group(1)


def test_field_startup_uses_one_hash_pinned_runtime_map_for_auto_init_and_icp():
    startup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8"
    )
    config = yaml.safe_load(
        (PACKAGE / "config" / "moving_localization.yaml").read_text(encoding="utf-8")
    )
    launch = ET.parse(PACKAGE / "launch" / "moving_localization.launch")
    args = {item.attrib["name"]: item.attrib["default"] for item in launch.findall("arg")}

    assert shell_default(startup, "MAP").endswith("/" + RUNTIME_NAME)
    assert shell_default(startup, "MAP_SHA256") == RUNTIME_SHA256
    assert config["map_path"].endswith("/" + RUNTIME_NAME)
    assert config["map_sha256"] == RUNTIME_SHA256
    assert args["map_path"].endswith("/" + RUNTIME_NAME)
    assert args["map_sha256"] == RUNTIME_SHA256
    assert 'map_path:="$MAP"' in startup
    assert 'map_sha256:="$MAP_SHA256"' in startup
    assert 'auto_init_map:="$MAP"' in startup
    assert "auto_initialization_verified false" in startup
    assert "rosparam get /fast_lio_icp/auto_initialization_verified" in startup
    assert 'auto_init_global_only:=true' in startup
    assert "/fast_lio_icp/auto_initialization_stable" in startup
    assert "MAP_OVERRIDE_COUNT" in startup
    assert "head -1 || true" in startup


def test_map_deployer_pins_the_supplied_canonical_and_runtime_artifacts():
    deploy = (ROOT / "tools" / "deploy_merged_map.sh").read_text(encoding="utf-8")

    assert CANONICAL_SHA256 in deploy
    assert RUNTIME_SHA256 in deploy
    assert "mergedmap.ply" in deploy
    assert RUNTIME_NAME in deploy
    assert "--verify-only" in deploy
    assert "37180425" in deploy
    assert "2696359" in deploy


def test_deployment_installs_the_bringup_script():
    """The bringup lives in $HOME, outside the checkout, so `git pull` does not
    touch it. Left behind, a deployment verifies clean and still brings the
    vehicle up on the previous route and band, without the gates added since."""
    push = (ROOT / "tools" / "push_to_nuc.sh").read_text(encoding="utf-8")

    assert "start_wheelchair_localization.sh" in push
    assert "BRINGUP_DST" in push
    # and it has to be checked, not just copied
    assert "did not install cleanly" in push


def test_the_deploy_script_and_the_bringup_name_the_same_route():
    """Deployment verifies and records a route/band pair by name. When that
    pair drifts from the one the bringup actually launches with, the record
    describes a drive that did not happen - and the old files still being on
    disk is exactly what lets it pass unnoticed."""
    deploy = (ROOT / "tools" / "deploy_merged_map.sh").read_text(encoding="utf-8")
    push = (ROOT / "tools" / "push_to_nuc.sh").read_text(encoding="utf-8")
    startup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8")

    assert 'ROUTE_NAME="%s"' % ROUTE_NAME in deploy
    assert 'BAND_NAME="%s"' % BAND_NAME in deploy
    # push_to_nuc verifies the pair again on the NUC after the pull, so it has
    # to name the same one; it named the superseded pair while deploy did too.
    assert ROUTE_NAME in push
    assert BAND_NAME in push
    for superseded in SUPERSEDED:
        assert superseded not in deploy, superseded
        assert superseded not in push, superseded
    assert shell_default(startup, "ROUTE").endswith("/" + ROUTE_NAME)
    assert shell_default(startup, "BAND").endswith("/" + BAND_NAME)


def test_field_startup_defaults_to_livox_builtin_imu_and_shipped_route():
    startup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8"
    )
    route = json.loads((ROOT / "routes" / ROUTE_NAME).read_text(encoding="utf-8"))
    band = json.loads((ROOT / "routes" / BAND_NAME).read_text(encoding="utf-8"))

    assert shell_default(startup, "VN_IMU") == "0"
    assert shell_default(startup, "ROUTE").endswith("/" + ROUTE_NAME)
    assert shell_default(startup, "BAND").endswith("/" + BAND_NAME)
    assert route["frame"] == band["frame"] == "map"
    assert route["body_frame_profile"] == "builtin"
    assert route["source"].strip(), "the route does not say where it came from"
    # The chair has to be able to pick the route up from where it is parked.
    # The follower locks to the NEAREST waypoint and holds OFF_ROUTE when that
    # is beyond 3.5 m, so what matters is the nearest one, not the first: the
    # 0727 route began at the parking spot; route v5 begins 8.35 m behind it
    # and first passes within 0.20 m at waypoint 45. Both are drivable from a
    # standing start; a route whose closest approach exceeded the geofence
    # would hold OFF_ROUTE instead of pulling away, which is the failure a
    # bookend trim caused once and this pins against.
    origin = (0.0, 0.0)
    first = route["waypoints"][0]
    closest = min(math.hypot(w["x"] - origin[0], w["y"] - origin[1])
                  for w in route["waypoints"])
    assert closest < 3.5, (
        "the route's closest approach to the recorded origin is %.2f m, "
        "beyond the follower geofence" % closest
    )
    assert set(first) == {"x", "y", "z", "yaw_deg"}
    # The point the route is about has to be declared and has to be the chair
    # centre: the follower lays the chair out symmetrically around the pose,
    # which is only true of the centre.
    assert route["reference_point"] == "chair_centre"
    # And the band has to carry which way each edge broke, so a kerb that
    # rises is not read as open pavement.
    assert {"left_kind", "right_kind"} <= set(band["stations"][0])
    assert 'rostopic echo -n1 /livox/imu/header' in startup


def test_field_speed_is_capped_at_point_six_metres_per_second():
    follower = (PACKAGE / "scripts" / "waypoint_follower.py").read_text(
        encoding="utf-8"
    )

    assert re.search(r"^MAX_SPEED\s*=\s*0\.6$", follower, flags=re.MULTILINE)


def test_initializer_is_packaged_and_field_startup_selects_global_only():
    startup = (ROOT / "tools" / "start_wheelchair_localization.sh").read_text(
        encoding="utf-8"
    )
    launch = (PACKAGE / "launch" / "moving_localization.launch").read_text(
        encoding="utf-8"
    )
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")

    assert 'auto_init_route:="$ROUTE"' in startup
    assert 'auto_init_body_frame_profile:="$BODY_FRAME_PROFILE"' in startup
    assert 'auto_init_global_only:=true' in startup
    assert '<param name="route" value="$(arg auto_init_route)"/>' in launch
    assert (
        '<param name="body_frame_profile" '
        'value="$(arg auto_init_body_frame_profile)"/>' in launch
    )
    assert "scripts/auto_initial_pose.py" in cmake
    assert "scripts/initial_pose_candidates.py" in cmake


def test_nuc_push_transfers_the_verified_runtime_map_not_only_the_canonical_ply():
    push = (ROOT / "tools" / "push_to_nuc.sh").read_text(encoding="utf-8")

    assert RUNTIME_NAME in push
    assert 'deploy_merged_map.sh --verify-only "$MAP_SRC"' in push
    assert '"$RUNTIME_PCD"' in push
    assert "built-in Livox IMU" in push


def test_nuc_push_uses_fixed_map_id_and_encoded_remote_arguments():
    push = (ROOT / "tools" / "push_to_nuc.sh").read_text(encoding="utf-8")

    assert 'DEST="$MAP_ID"' in push
    assert 'basename "$MAP_SRC"' not in push
    assert "git check-ref-format --branch" in push
    assert "printf -v" in push and "%q" in push
    assert "--protect-args" not in push
    assert "--inplace" not in push
    assert "REMOTE_STAGE" in push
    assert "mktemp -d" in push
    assert "tar -xf - -C" in push
    assert "EXPECTED_COMMIT" in push
    assert "status --porcelain" in push
    assert "REMOTE_DIRTY" in push
    assert 'DEST_DIR="$MAPS/$DEST"' in push
    assert "EXPECTED_PKG" in push
    assert "readlink -f" in push


def test_map_deployer_rejects_symlink_targets_and_uses_unique_temp_files():
    deploy = (ROOT / "tools" / "deploy_merged_map.sh").read_text(encoding="utf-8")

    assert 'if [ -L "$DEST_DIR" ]' in deploy
    assert '[ "$RUNTIME_PATH" -ef "$runtime_dest" ]' in deploy
    assert "mktemp" in deploy
    assert ".localization-map-manifest.json.tmp" not in deploy
    assert 'canonical_dest="$DEST_DIR/$CANONICAL_NAME"' in deploy
    assert 'cp -f "$CANONICAL_PATH"' in deploy
