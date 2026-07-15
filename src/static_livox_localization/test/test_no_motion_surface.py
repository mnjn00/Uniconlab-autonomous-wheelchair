from pathlib import Path


def test_launch_and_localizer_have_no_motion_or_tf_publication():
    root = Path(__file__).parents[1]
    text = "\n".join(path.read_text(encoding="utf-8") for path in
                     [root / "src" / "static_icp_localizer.cpp", root / "launch" / "static_localization.launch"])
    assert "/cmd_vel" not in text
    assert "TransformBroadcaster" not in text
    assert "move_base" not in text
