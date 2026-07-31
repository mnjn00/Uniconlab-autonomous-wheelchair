"""Which stops survive when the safety policies are switched off.

The field question for the first autonomous run of the 0727 line is narrow:
does localization stay attached for 383 m? Every discretionary stop between
the plan and the wheels answers a different question, and each one that fires
ends the measurement early - a band refusal at station 96 and a lost fix at
station 96 look identical from the outside once the chair is stationary.

So the policies can be switched off for that one measurement, leaving the
operator's joystick as the failsafe. What cannot be switched off is anything
the joystick override itself rests on:

  MANUAL_MODE  is the override.
  BASE_STALE   is whether the channel that reports it is still alive. Drop
               this and the chair drives on with no way to observe that the
               operator has taken control, which is not a reduced failsafe
               but no failsafe at all.
  NO_POSE      is not a judgement, it is the absence of the input every
               later test reads.
  PAUSED/DONE  are state, not policy.
  INPUT_STALE  one layer down, is how the gate notices the planner died.
  INPUT_INVALID  likewise: a NaN command is a fault, not a hazard.
  REVERSE      the chair has no rear sensing at any setting, and nothing
               upstream ever commands it, so this can only fire on a fault.

Everything else - band containment, the obstacle envelope, the swept
footprint, the localization-health hold, the motion-estimate gate - is a
judgement about the world, and judgements are what this switches off.

A switched-off policy is still evaluated and still reported. Driving with
them off is how the run finds out where each one would have fired, and
throwing that away would waste the only run that can measure it.
"""

POLICY = "policy"
OVERRIDE = "override"


def evaluate_holds(candidates, policies_enabled):
    """Resolve ordered hold candidates into (binding, suppressed).

    `candidates` yields (reason, kind) highest priority first; it is consumed
    lazily because the order is also the order in which the tests are safe to
    run - NO_POSE is what guarantees the position tests below it have a
    position to read.

    `binding` stops the chair. `suppressed` is the highest-priority policy
    that applied while switched off, so the caller can publish where the
    guard would have intervened. With the policies enabled every candidate
    binds, the first one wins, and `suppressed` is always None - i.e. this
    reduces exactly to "the first reason that applies".
    """
    suppressed = None
    for reason, kind in candidates:
        if kind != POLICY or policies_enabled:
            return reason, suppressed
        if suppressed is None:
            suppressed = reason
    return None, suppressed


def announce(policies_enabled, node, switched_off, still_watching=()):
    """The line a node logs at startup about its own guards.

    Silence here is how a diagnostic build gets driven by someone who thinks
    it is the normal one, so the disabled case names every guard it dropped
    rather than saying "diagnostic mode". It names what is left too, and
    says so in as many words when that is nothing - an operator deciding how
    closely to hold the joystick needs the answer, not the question.
    """
    if policies_enabled:
        return "%s: safety policies ENABLED" % node
    return ("%s: SAFETY POLICIES OFF - not checking %s. Still watching: %s. "
            "The joystick is the failsafe." % (
                node, ", ".join(switched_off),
                ", ".join(still_watching) if still_watching else "NOTHING"))
