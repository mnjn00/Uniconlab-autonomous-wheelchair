"""Pure policy for selecting objects excluded from map registration.

The hybrid collision source keeps mapped walls and fixtures, so its complete
box stream must never be sent to the localizer. Registration exclusions are
limited to directly person-like or measured-moving evidence, plus learned
person/vehicle/two-wheeler boxes. Uncertain geometric-only walls are retained
for localization until motion evidence says otherwise.
"""

DYNAMIC_CLASSES = frozenset(("person", "vehicle", "two_wheeler"))


def normalize(value):
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def should_exclude(item):
    if not isinstance(item, dict):
        return False
    label = normalize(item.get("class"))
    motion = normalize(item.get("motion"))
    source = normalize(item.get("source", "geometric"))
    learned_label = normalize(item.get("learned_class"))

    # A measured moving object is absent from the immutable map whatever its
    # classifier called it.
    if motion == "moving":
        return True
    # Person geometry is deliberately conservative even before the tracker
    # has enough history to decide motion.
    if label == "person":
        return True
    # Learned dynamic-class geometry may be excluded while its motion is
    # unknown. This does not apply to geometric-only vehicle heuristics: a
    # long mapped wall can look vehicle-sized and must remain registration
    # evidence.
    if source in ("learned_only", "geometric+learned"):
        if label in DYNAMIC_CLASSES or learned_label in DYNAMIC_CLASSES:
            return True
    return False
