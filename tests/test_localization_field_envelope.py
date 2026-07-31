"""The localization gates against what the chair actually measured.

2026-07-31, two complete autonomous runs of the 0727 route - 1446 points end
to end, 154 samples taken while driving - answered the question the whole
staged plan was waiting on: the fix does not drop. Every driving sample in
both runs was TRACKING, no LOCALIZATION hold was published or suppressed,
and the two runs agree to within 0.0006 on both means, which is what makes
it a measurement rather than one lucky lap.

That closes localization. What stays open is the ability to notice it
reopening, and a gate is only a gate while it sits outside the envelope it
is meant to catch excursions from. Tightening max_fitness toward 0.033 or
raising min_inlier_ratio toward 0.97 would put the thresholds inside normal
operation and turn every run into a stop; loosening them past the point
where a real loss registers is the opposite failure. Both are silent edits
to a YAML file, so both are pinned here against the numbers rather than
against someone's memory of them.

The parked-at-the-goal degradation is recorded too. It is not a driving
result and does not belong in the envelope, but it is the one place the fix
was measured near its gate, and a later stage that ends by holding position
there needs to know that was already seen.
"""

from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).parents[1]
CONFIG = (ROOT / "src" / "static_livox_localization" / "config"
          / "moving_localization.yaml")

# Measured while driving, both runs pooled (154 samples).
DRIVING_INLIER_MIN = 0.9666
DRIVING_FITNESS_MAX = 0.0329
# Measured while parked at the goal, 20 min after run 2 (4 samples).
PARKED_INLIER_MIN = 0.124


@pytest.fixture(scope="module")
def config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_the_inlier_gate_sits_below_everything_measured_while_driving(config):
    """It has to be reachable only by a real excursion. At the measured
    worst case there is 4.8x of room."""
    gate = config["min_inlier_ratio"]
    assert gate < DRIVING_INLIER_MIN, (
        "min_inlier_ratio %.3f is inside normal operation (worst driving "
        "sample %.4f) - every run would stop" % (gate, DRIVING_INLIER_MIN))
    assert DRIVING_INLIER_MIN / gate >= 3.0


def test_the_fitness_ceiling_sits_above_everything_measured_while_driving(config):
    gate = config["max_fitness"]
    assert gate > DRIVING_FITNESS_MAX, (
        "max_fitness %.3f is inside normal operation (worst driving sample "
        "%.4f)" % (gate, DRIVING_FITNESS_MAX))
    assert gate / DRIVING_FITNESS_MAX >= 3.0


def test_the_inlier_gate_still_catches_what_was_seen_at_the_goal(config):
    """The only excursion measured all evening. A gate loosened below it
    would have called that healthy, and the one place the fix came near
    failing is the one place it must not be explained away."""
    assert config["min_inlier_ratio"] > PARKED_INLIER_MIN


def test_a_single_bad_correction_is_enough_to_leave_tracking(config):
    """Both runs stayed TRACKING throughout with this at 1, so the strictest
    setting costs nothing on this route and there is no measured reason to
    relax it."""
    assert config["degraded_after_failures"] == 1


def test_the_measured_envelope_is_written_where_the_gates_are(config):
    """Numbers in a test are numbers in a test. Whoever opens the YAML to
    change a threshold has to meet the evidence there."""
    text = CONFIG.read_text(encoding="utf-8")
    assert "2026-07-31" in text
    assert "0.9666" in text and "0.0329" in text
    assert "0.124" in text, "the parked excursion is not recorded beside the gate"
