from pathlib import Path


ROOT = Path(__file__).parents[1]
PROJECT = ROOT.parents[1]


def test_rviz_has_explicit_red_map_and_green_live_cloud():
    text = (ROOT / "config" / "moving_localization.rviz").read_text(
        encoding="utf-8"
    )
    assert "Color Transformer: FlatColor" in text
    assert "Color: 255; 0; 0" in text
    assert "Color: 0; 255; 0" in text


def test_state_marker_has_manual_and_verifying_colors():
    text = (ROOT / "scripts" / "localization_state_marker.py").read_text(
        encoding="utf-8"
    )
    assert '"MANUAL_ALIGN": (0.0, 1.0, 1.0)' in text
    assert '"VERIFYING": (1.0, 0.0, 1.0)' in text


def test_rviz_has_one_meter_wheelchair_footprint_marker():
    marker = (ROOT / "scripts" / "localization_state_marker.py").read_text(
        encoding="utf-8"
    )
    rviz = (ROOT / "config" / "moving_localization.rviz").read_text(
        encoding="utf-8"
    )

    assert '"/fast_lio_icp/wheelchair_footprint_marker"' in marker
    assert "Marker.CYLINDER" in marker
    assert "footprint.scale.x = footprint.scale.y = 1.0" in marker
    assert "/fast_lio_icp/wheelchair_footprint_marker" in rviz


def test_korean_runbook_documents_explicit_manual_alignment_gate():
    text = (
        PROJECT / "docs" / "runbooks" / "livox-moving-localization-ko.md"
    ).read_text(encoding="utf-8")
    assert "사자상 원형" in text
    assert "MANUAL_ALIGN" in text
    assert "VERIFYING" in text
    assert 'rosservice call /fast_lio_icp/enable_auto_correction "data: true"' in text
    assert 'rosservice call /fast_lio_icp/enable_auto_correction "data: false"' in text
    assert "지도는 움직이지 않는다" in text
