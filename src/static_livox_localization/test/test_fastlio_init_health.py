"""FAST-LIO's own initialization has to be checked before anything trusts it.

FAST-LIO estimates gravity direction and IMU bias from the first seconds of
data on the assumption the vehicle is still. A mid-route start breaks that
assumption for a mundane reason: the chair was just wheeled into position and
the rider is still settling. The result is a tilted gravity vector, and a
tilted gravity vector leaks a component of g into acceleration, which
integrates into velocity and runs the odometry away in its own frame. No seed
can fix that - the seed only sets map-to-odom, and the odom it is chained to
is the thing that is wrong.

The symptom is visible for free before driving: parked and stationary, a
healthy FAST-LIO holds its pose. These cases pin a stationary verdict that
fails closed, so a startup can restart FAST-LIO instead of driving on an
estimate that is already diverging.
"""

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).parents[1]


def load_health():
    path = ROOT / "scripts" / "fastlio_init_health.py"
    spec = importlib.util.spec_from_file_location("init_health_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


health = load_health()


def samples(count=80, period=0.1, drift_m=0.0, pitch_drift_deg=0.0, jitter_m=0.0):
    """A stationary odometry log, optionally ramping the way a bad init does."""

    rows = []
    for index in range(count):
        fraction = index / float(max(count - 1, 1))
        wobble = jitter_m * (1 if index % 2 else -1)
        rows.append((
            index * period,
            drift_m * fraction + wobble,
            0.0,
            0.0,
            0.0,
            pitch_drift_deg * fraction,
        ))
    return tuple(rows)


def test_a_parked_chair_with_sensor_noise_reads_healthy():
    """Centimetre-level wobble is normal and must not block a drive."""
    verdict = health.stationary_verdict(samples(jitter_m=0.02))

    assert verdict.healthy
    assert verdict.reason == "stationary"
    assert verdict.translation_drift_m <= 0.05


def test_a_ramping_position_is_reported_as_divergence():
    """The signature of a bad gravity or accelerometer-bias init: parked, but
    the estimate keeps moving."""
    verdict = health.stationary_verdict(samples(drift_m=1.2))

    assert not verdict.healthy
    assert verdict.reason == "translation_drift"
    assert verdict.translation_drift_m >= 1.0


def test_a_tilting_attitude_is_reported_even_when_position_holds():
    """A tilted gravity estimate shows up in roll and pitch first; catching it
    there is earlier than waiting for the position to run away."""
    verdict = health.stationary_verdict(samples(pitch_drift_deg=4.0))

    assert not verdict.healthy
    assert verdict.reason == "attitude_drift"
    assert verdict.attitude_drift_deg >= 3.0


def test_too_few_samples_fails_closed_rather_than_passing():
    """Silence is not health. A check that cannot see enough odometry has to
    refuse, or a dead FAST-LIO reads as a clean bill."""
    verdict = health.stationary_verdict(samples(count=4))

    assert not verdict.healthy
    assert verdict.reason == "insufficient_samples"


def test_a_short_window_fails_closed_rather_than_passing():
    """Drift needs time to show. Half a second of perfectly still odometry
    proves nothing about an init that ramps over seconds."""
    verdict = health.stationary_verdict(samples(count=60, period=0.008))

    assert not verdict.healthy
    assert verdict.reason == "window_too_short"


def test_no_samples_at_all_fails_closed():
    verdict = health.stationary_verdict(())

    assert not verdict.healthy
    assert verdict.reason == "insufficient_samples"


def test_the_limits_are_loose_enough_not_to_fail_a_healthy_start():
    """These gate a field startup, so a false alarm costs a drive. The
    thresholds must sit well above stationary sensor noise and well below the
    metres a real divergence produces."""
    assert 0.10 <= health.MAX_STATIONARY_DRIFT_M <= 0.30
    assert 1.0 <= health.MAX_STATIONARY_ATTITUDE_DRIFT_DEG <= 3.0
    assert health.MIN_WINDOW_S >= 4.0


def test_startup_checks_health_before_localization_and_retries_once():
    """The script printed 'keep the wheelchair STILL' and then trusted the
    operator. It has to verify instead, and a failed check is worth one
    restart before giving up - re-initializing FAST-LIO is cheap, driving on
    a diverging estimate is not."""
    script = (
        ROOT.parents[1] / "tools" / "start_wheelchair_localization.sh"
    ).read_text(encoding="utf-8")

    assert "fastlio_init_health.py" in script
    health_check = script.index("fastlio_init_health.py")
    localization = script.index("moving_localization.launch")
    assert health_check < localization
    assert "FASTLIO_HEALTH_RETRIES" in script
