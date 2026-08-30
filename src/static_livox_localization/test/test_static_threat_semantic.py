import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load_supervisor():
    class Stamp:
        def __init__(self, seconds=0.0):
            self.seconds = float(seconds)

        def to_sec(self):
            return self.seconds

        def __sub__(self, other):
            return Stamp(self.seconds - other.seconds)

    class Clock:
        now_s = 10.1

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class String:
        def __init__(self, data=""):
            self.data = data

    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda _name, default=None: default
    rospy.loginfo = lambda *_args, **_kwargs: None
    rospy.logwarn_throttle = lambda *_args, **_kwargs: None
    rospy.ROSInitException = RuntimeError
    rospy.ROSInterruptException = RuntimeError
    rospy.Time = type(
        "Time", (), {"now": staticmethod(lambda: Stamp(Clock.now_s))})

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs_msg.Int16MultiArray = type("Int16MultiArray", (), {})
    std_msgs.msg = std_msgs_msg

    replacements = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    saved = {name: sys.modules.get(name) for name in replacements}
    cached_scripts = {
        name: value for name, value in sys.modules.items()
        if Path(getattr(value, "__file__", None) or "/").parent == SCRIPTS
    }
    sys.modules.update(replacements)
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            "static_threat_semantic_test_subject",
            SCRIPTS / "person_bypass_semantic_supervisor.py",
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        from cluster_guard import parse_summary
        from person_bypass_policy import (
            BypassPermit,
            STATIC_THREAT_BYPASS,
            STATIC_THREAT_DROPOUT_GRACE,
        )
        from semantic_safety_policy import PersonStopLatch, ThreatView
        module.BypassPermit = BypassPermit
        module.STATIC_THREAT_BYPASS = STATIC_THREAT_BYPASS
        module.STATIC_THREAT_DROPOUT_GRACE = STATIC_THREAT_DROPOUT_GRACE
        module.PersonStopLatch = PersonStopLatch
        module.ThreatView = ThreatView
        module.parse_summary = parse_summary
        module.TestClock = Clock
        module.TestStamp = Stamp
        module.TestTwist = Twist
        return module
    finally:
        sys.path.remove(str(SCRIPTS))
        for name, value in list(sys.modules.items()):
            if Path(getattr(value, "__file__", None) or "/").parent == SCRIPTS:
                sys.modules.pop(name, None)
        sys.modules.update(cached_scripts)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def summary(module, objects, *, status="OK"):
    return module.parse_summary(json.dumps({
        "stamp": 10.0,
        "status": status,
        "objects": objects,
    }))


def tracked(track_id, label, motion="static", x_m=2.0, y_m=0.0):
    return {
        "id": track_id,
        "class": label,
        "motion": motion,
        "source": "geometric",
        "directly_observed": True,
        "geometry_valid": True,
        "x": x_m,
        "y": y_m,
        "size": [0.6, 0.6, 1.0],
    }


def active_permit(module, *, track_id=7, label="cart", x_m=2.0,
                  y_m=0.0, reason=None):
    return module.BypassPermit(
        capable=True,
        active=True,
        stamp_s=10.0,
        expires_s=10.45,
        track_id=track_id,
        target_x_m=x_m,
        target_y_m=y_m,
        threat_label=label,
        static_for_s=2.0,
        max_speed_mps=0.35,
        min_clearance_m=0.35,
        reason=reason or module.STATIC_THREAT_BYPASS,
    )


def make_subject(module, objects, permit):
    subject = module.PersonBypassSemanticSupervisor.__new__(
        module.PersonBypassSemanticSupervisor)
    subject.summary = summary(module, objects)
    subject.bypass_permit = permit
    subject.maximum_permit_age_s = 0.45
    subject.maximum_target_error_m = 0.45
    subject.bypass_maximum_forward_m = 8.0
    subject.bypass_maximum_lateral_m = 1.0
    subject.corridor_half_width_m = 0.50
    subject.person_half_width_m = 0.65
    subject.expected_summary_frame = "chair_centre"
    subject.summary_frame = "chair_centre"
    subject.command = module.TestTwist()
    subject.command.linear.x = 0.35
    subject.command.angular.x = 11.0
    subject.command.angular.z = 0.2
    subject.command_stamp = module.TestStamp(10.0)
    subject.measured_speed = 0.0
    subject.person_memory = None
    subject.person_memory_s = 1.0
    subject.person_latch = module.PersonStopLatch(0.30)
    subject.maximum_summary_age_s = 1.5
    subject.maximum_command_age_s = 0.6
    subject.accumulation_s = 0.6
    subject.pipeline_s = 0.2
    subject.minimum_deceleration_mps2 = 0.5
    subject.geometry_margin_m = 0.9
    subject.pub = types.SimpleNamespace(publish=lambda value: published.append(value))
    subject.status_pub = types.SimpleNamespace(
        publish=lambda value: statuses.append(json.loads(value.data)))
    subject.test_published = published = []
    subject.test_statuses = statuses = []
    return subject


def test_permitted_target_becoming_unknown_stops_immediately():
    # Given: the previously permitted target is now unknown motion.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, "cart", motion="unknown")],
        active_permit(module))

    # When: the current frame is assessed.
    validated, _observation, fault, dynamic = subject.validated_bypass(10.1)

    # Then: the old permit is rejected and the target remains a stop threat.
    assert validated is None
    assert fault == "STATIC_THREAT_PERMIT_MISMATCH"
    assert dynamic.track_id == 7


def test_malformed_object_fails_closed_even_with_matching_permit():
    # Given: one permitted static target and a non-object summary entry.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, "cart"), "malformed"],
        active_permit(module))

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: malformed input emits zero instead of crashing or being skipped.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_published[-1].angular.x == 0.0
    assert subject.test_statuses[-1]["reason"] == "MOVING_OBJECT"


def test_dropout_grace_uses_permit_identity_only_without_dynamic_conflict():
    # Given: a fresh bounded dropout permit and a healthy empty frame.
    module = load_supervisor()
    permit = active_permit(
        module, label="person", reason=module.STATIC_THREAT_DROPOUT_GRACE)
    subject = make_subject(module, [], permit)

    # When: the one-frame producer dropout is assessed.
    validated, observation, fault, dynamic = subject.validated_bypass(10.1)

    # Then: grace remains bound to the permit identity, not a fabricated box.
    assert validated == permit
    assert observation is None
    assert fault == ""
    assert dynamic is None


def test_dropout_grace_rejects_a_replacement_static_identity():
    # Given: a dropout permit but the healthy frame contains another track.
    module = load_supervisor()
    permit = active_permit(
        module, label="person", reason=module.STATIC_THREAT_DROPOUT_GRACE)
    subject = make_subject(module, [tracked(8, "person")], permit)

    # When: the grace permit is validated against current identities.
    validated, observation, fault, _dynamic = subject.validated_bypass(10.1)

    # Then: a replacement track is an identity mismatch, not a dropout.
    assert validated is None
    assert observation is None
    assert fault == "STATIC_THREAT_PERMIT_MISMATCH"


@pytest.mark.parametrize("label", ("cart", "person"))
def test_step_passes_matching_static_threat_with_generic_status(label):
    # Given: a fresh exact threat permit and a healthy command/summary frame.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, label)], active_permit(module, label=label))

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: motion passes under the permit cap and status is generic v2 state.
    assert subject.test_published[-1].linear.x == 0.35
    assert subject.test_published[-1].angular.x == 11.0
    report = subject.test_statuses[-1]
    assert report["static_threat_bypass_active"] is True
    assert report["static_threat_bypass_label"] == label
    assert "person_bypass_active" not in report


def test_step_stops_on_permit_mismatch_even_for_static_object():
    # Given: a fresh active permit whose position does not match the cart.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, "cart")], active_permit(module, x_m=4.0))

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: it emits zero rather than falling through to the planner command.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_statuses[-1]["reason"] == \
        "STATIC_THREAT_PERMIT_MISMATCH"


def test_step_stops_for_second_dynamic_threat_anywhere_in_corridor():
    # Given: a valid static permit plus a moving person farther down-corridor.
    module = load_supervisor()
    subject = make_subject(module, [
        tracked(7, "cart"),
        tracked(8, "person", motion="moving", x_m=6.0),
    ], active_permit(module))

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: the second dynamic threat vetoes the narrow target exception.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_statuses[-1]["reason"] == "DYNAMIC_THREAT_CONFLICT"


def test_step_keeps_health_stops_above_a_matching_permit():
    # Given: a matching permit but a stale command and wrong summary frame.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, "cart")], active_permit(module))
    subject.command_stamp = module.TestStamp(8.0)
    subject.summary_frame = "wrong_frame"

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: perception health fails closed before the permit can authorize.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_statuses[-1]["reason"] == "PERCEPTION_UNUSABLE"


def test_step_keeps_unrelated_static_person_stop_with_object_permit():
    # Given: an exact cart permit and a different static person in the envelope.
    module = load_supervisor()
    subject = make_subject(module, [
        tracked(7, "cart"),
        tracked(3, "person", x_m=0.8),
    ], active_permit(module))

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: only the cart is exempt and the unrelated person remains latched.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_statuses[-1]["reason"] == "PERSON"
    assert subject.person_latch.track_id == 3


def test_step_stale_command_stops_with_healthy_matching_summary():
    # Given: a matching permit and healthy summary but a stale planned command.
    module = load_supervisor()
    subject = make_subject(
        module, [tracked(7, "cart")], active_permit(module))
    subject.command_stamp = module.TestStamp(8.0)

    # When: the executable runs one semantic control cycle.
    subject.step()

    # Then: input freshness remains an independent stop authority.
    assert subject.test_published[-1].linear.x == 0.0
    assert subject.test_statuses[-1]["reason"] == "INPUT_STALE"
