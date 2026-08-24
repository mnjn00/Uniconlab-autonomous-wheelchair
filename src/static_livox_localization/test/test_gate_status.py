"""The gate says why it stopped, and with what numbers.

Before 2026-08-23 safety_gate published a Twist and nothing else, so a stop
left no record of its own reason. Two stops that day - a 130 s deadlock in
front of a parked motorcycle and two 1.3 s stops on a crest - had to be
reconstructed by borrowing /perception/objects_summary as a window onto
what the gate might have been looking at.

The reason on its own would not have settled either of them. What did was
the range the stopping envelope reached against the range the nearest
return sat at, so these tests are mostly about the numbers travelling with
the reason and about absent measurements staying absent.
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from safety_gate import status_report


def report(evidence=None, reason="", cap=0.8, out_v=0.8, out_w=0.0,
           policies=True):
    return status_report(evidence, reason, cap, out_v, out_w, policies)


def test_a_clear_cycle_says_it_is_clear():
    out = report()
    assert out["reason"] == ""
    assert out["blocked"] is False


def test_a_stop_carries_its_reason():
    out = report(reason="OBSTACLE", out_v=0.0)
    assert out["reason"] == "OBSTACLE"
    assert out["blocked"] is True
    assert out["out_v"] == 0.0


def test_the_numbers_that_settle_a_stop_travel_with_it():
    """envelope_m against zone_nearest_m is the comparison the OBSTACLE
    test makes. A report without both is a report that cannot be read."""
    out = report(evidence={"envelope_m": 1.9, "zone_nearest_m": 1.42,
                           "zone_points": 61, "zone_lateral_m": 0.47},
                 reason="OBSTACLE", out_v=0.0)
    assert out["envelope_m"] == 1.9
    assert out["zone_nearest_m"] == 1.42
    assert out["zone_nearest_m"] < out["envelope_m"], \
        "this is the inequality that stopped the chair"


def test_an_early_exit_leaves_its_fields_absent_not_zero():
    """NO_CLOUD returns before the envelope exists. Reporting 0.0 for it
    would read as an envelope that had been measured and found tiny."""
    out = report(evidence={"cloud_points": 12, "cloud_age_s": 0.04},
                 reason="NO_CLOUD", out_v=0.0)
    assert "envelope_m" not in out
    assert "zone_nearest_m" not in out
    assert out["cloud_points"] == 12


def test_a_measured_absence_is_not_the_same_as_an_unmeasured_one():
    """zone_nearest_m is None when the zone held fewer than the five
    returns the test needs. That is a measurement, and it survives."""
    out = report(evidence={"zone_points": 2, "zone_nearest_m": None},
                 reason="", out_v=0.8)
    assert "zone_nearest_m" in out
    assert out["zone_nearest_m"] is None


def test_the_report_survives_the_wire():
    """It goes out as JSON on a String topic, so anything that cannot be
    serialised is a field that silently never arrives."""
    out = report(evidence={"envelope_m": 1.9, "zone_nearest_m": None,
                           "zone_points": 61, "cloud_points": 48213,
                           "cloud_age_s": 0.041, "sweep_clear_v": 0.42},
                 reason="OBSTACLE_SWEEP", cap=0.35, out_v=0.0)
    assert json.loads(json.dumps(out, sort_keys=True)) == out


def test_the_cap_is_reported_even_when_nothing_is_blocking():
    """The sweep cap is a speed limit, not a stop, so it shapes runs that
    never show a reason at all - and it was invisible until now."""
    out = report(cap=0.44, out_v=0.44)
    assert out["reason"] == ""
    assert out["cap"] == 0.44


def test_policies_off_is_on_the_record():
    """With policies off the gate measures and does not act. A reader who
    cannot tell the two apart will read a quiet run as a safe one."""
    assert report(policies=False)["policies"] is False
    assert report()["policies"] is True


def test_the_caller_cannot_be_corrupted_by_the_report():
    evidence = {"envelope_m": 1.9}
    out = report(evidence=evidence, reason="OBSTACLE")
    assert "reason" not in evidence
    assert out is not evidence


@pytest.mark.parametrize("reason,blocked", [
    ("", False), ("OBSTACLE", True), ("OBSTACLE_SWEEP", True),
    ("NO_CLOUD", True), ("CLOUD_STALE", True), ("INPUT_STALE", True),
    ("INPUT_INVALID", True), ("REVERSE", True)])
def test_every_reason_the_gate_can_give_reads_as_blocked(reason, blocked):
    assert report(reason=reason)["blocked"] is blocked
