"""The bypass step has to fit the corridor it is taken in.

On 2026-08-16 the chair took a 0.60 m step where the band was 1.70 m wide
and the operator stopped it. The step passed bypass_target_ok because that
tests the chair's CENTRE against the band, and an edge with no drop behind
it only insets EDGE_MARGIN - deliberately, since that inset is what buys
room to pass obstacles at all. The chair is 0.70 m wide, so 0.60 m off the
line put its outer edge 0.275 m past the band.

The fix is not a tighter containment test, which would remove the room the
band was drawn to give. It is asking how much room is there before choosing
how far to step.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(
    0, str(ROOT / "src" / "static_livox_localization" / "scripts"))

import cluster_guard as WF

offsets_for = WF.bypass_offsets_for_room


def test_a_wide_corridor_still_gets_the_full_ladder():
    """Nothing changes where there is room; this is the 08-16 fix not
    costing anything on the open stretches."""
    assert offsets_for(3.0, 3.0) == list(WF.BYPASS_OFFSETS)


def test_the_step_is_cut_down_to_the_room_that_is_there():
    """The 08-16 corridor: 0.85 m each side of the line, so 0.25 m kept
    from the edge leaves 0.60 m of room - and the 1.0 m rung cannot be
    taken at its own size."""
    room = 0.85 - WF.BYPASS_EDGE_KEEP_M
    got = offsets_for(room, room)
    assert got == [room, -room], got
    assert max(abs(v) for v in got) <= room + 1e-9


def test_a_side_without_room_is_not_offered():
    """A wall on the right means every candidate is a left one, rather
    than a right step that the centre-point test would have allowed."""
    got = offsets_for(1.2, 0.05)
    assert got, "the open side should still be usable"
    assert all(v > 0 for v in got), got


def test_a_corridor_too_narrow_for_any_step_offers_nothing():
    """Then take_a_way_round holds and says so, which is the honest
    answer: there is no way round this one."""
    assert offsets_for(0.10, 0.10) == []


def test_no_offset_below_the_minimum_survives():
    """A step smaller than this has not gone round anything; it has only
    drifted, and it still pays the full cost of leaving the line."""
    for left in (0.0, 0.1, 0.29, 0.31, 0.6, 2.0):
        for value in offsets_for(left, left):
            assert abs(value) >= WF.BYPASS_OFFSET_MIN_M - 1e-9, (left, value)


def test_nothing_ever_exceeds_the_hard_cap():
    assert all(abs(v) <= WF.BYPASS_OFFSET_MAX_M + 1e-9
               for v in offsets_for(50.0, 50.0))


def test_duplicates_are_collapsed():
    """Both rungs on a side clamp to the same value when the room sits
    between them, and trying it twice only costs a corridor scan."""
    room = 0.8
    got = offsets_for(room, room)
    assert len(got) == len(set(got)), got


def test_the_probe_distances_are_shared_with_the_containment_check():
    """The offset that gets sized and the offset that gets approved have
    to be talking about the same ground, or one of them is measuring a
    corridor the other never looks at."""
    source = (ROOT / "src" / "static_livox_localization" / "scripts"
              / "waypoint_follower.py").read_text(encoding="utf-8")
    assert source.count("BYPASS_PROBE_AHEAD_M") >= 3
    assert "for ahead in (0.5, 1.5, 2.5, 3.5)" not in source
