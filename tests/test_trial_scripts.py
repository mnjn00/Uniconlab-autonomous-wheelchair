"""What gets typed at the chair.

Three scripts, and each one's failure mode is different. trial_0727 has to
configure the run it claims to configure - a trial that quietly brings the
guards up measures nothing, because a band refusal and a lost fix are the
same stationary chair. go has to refuse rather than start, since anything it
cannot check is something that gets discovered while the chair is already
moving. stop has to work when everything else has failed, which means
checking nothing at all.
"""

from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
TOOLS = ROOT / "tools"


def script(name):
    return (TOOLS / name).read_text(encoding="utf-8")


def test_the_trial_switches_the_discretionary_guards_off():
    """That is the whole point of it: measure one thing without another
    guard ending the measurement first."""
    assert "export SAFETY_POLICIES=false" in script("trial_0727.sh")


def test_the_trial_leaves_object_avoidance_on():
    """~cluster_avoidance is deliberately not behind ~safety_policies, so the
    trial must not set it - and the bringup must not either."""
    assert "cluster_avoidance" not in script("trial_0727.sh")
    assert "cluster_avoidance" not in script("start_wheelchair_localization.sh")


def test_the_trial_does_not_drive():
    trial = script("trial_0727.sh")
    assert "waypoint_follower/start" not in trial
    assert "mode_cmd" not in trial


def test_go_refuses_when_the_follower_is_not_there():
    go = script("go.sh")
    assert "rosnode ping -c1 /waypoint_follower" in go
    assert "fail " in go


def test_go_refuses_on_a_silent_object_tracker():
    """It is the only guard still watching for people once the policies are
    off, and silence from it reads exactly like clear road."""
    go = script("go.sh")
    assert "/perception/objects_summary" in go


def test_go_refuses_unless_localization_is_tracking():
    go = script("go.sh")
    assert "localization_diagnostics" in go
    assert 'TRACKING' in go


def test_go_checks_everything_before_it_commands_anything():
    """A check after the first command is a check performed while the chair
    is already moving."""
    go = script("go.sh")
    last_check = max(go.index("/perception/objects_summary"),
                     go.index("localization_diagnostics"),
                     go.index("rosnode ping"))
    assert last_check < go.index("rostopic pub -1 /mode_cmd")
    assert last_check < go.index("waypoint_follower/start")


def test_stop_checks_nothing_and_tolerates_failure():
    """A stop that refuses because a precondition failed is not a stop."""
    stop = script("stop.sh")
    assert "set -e" not in stop
    assert stop.count("|| true") >= 2
    assert 'data: false' in stop


@pytest.mark.parametrize("name", ["trial_0727.sh", "go.sh", "stop.sh"])
def test_each_script_is_deployed_and_checksum_verified(name):
    """They live in $HOME, outside the checkout, so a pull never touches
    them - the same way the bringup was found still running the previous
    week's route."""
    push = script("push_to_nuc.sh")
    assert name in push
    assert "did not install cleanly" in push


@pytest.mark.parametrize("name", ["trial_0727.sh", "go.sh", "stop.sh"])
def test_each_script_is_executable(name):
    assert (TOOLS / name).stat().st_mode & 0o111, "%s is not executable" % name
