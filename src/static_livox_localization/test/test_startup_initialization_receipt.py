from pathlib import Path


def test_startup_accepts_global_search_and_opt_in_known_start_receipts():
    script = (
        Path(__file__).parents[2] / ".." / "tools" / "start_wheelchair_localization.sh"
    ).resolve().read_text(encoding="utf-8")

    assert '[ "$AUTO_INITIALIZATION_SOURCE" = "global_search" ]' in script
    assert '[ "$AUTO_INIT_GLOBAL_ONLY" = "false" ]' in script
    assert '[ "$AUTO_INITIALIZATION_SOURCE" = "known_start_route" ]' in script
    assert '[ "$AUTO_INITIALIZATION_SOURCE_ALLOWED" = "true" ]' in script
