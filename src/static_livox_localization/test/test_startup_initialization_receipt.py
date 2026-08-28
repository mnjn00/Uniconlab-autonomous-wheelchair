from pathlib import Path


def test_startup_accepts_the_receipt_for_the_selected_initialization_mode():
    script = (
        Path(__file__).parents[2] / ".." / "tools" / "start_wheelchair_localization.sh"
    ).resolve().read_text(encoding="utf-8")

    assert 'EXPECTED_AUTO_INITIALIZATION_SOURCE="global_search"' in script
    assert 'EXPECTED_AUTO_INITIALIZATION_SOURCE="known_start_route"' in script
    assert '[ "$AUTO_INITIALIZATION_SOURCE" = "$EXPECTED_AUTO_INITIALIZATION_SOURCE" ]' in script
    assert '[ "$AUTO_INITIALIZATION_SOURCE" = "global_search" ]' not in script
