"""Pure policy for selecting objects excluded from map registration.

The hybrid collision source keeps mapped walls and fixtures, so its complete
box stream must never be sent to the localizer. Registration exclusions are
limited to directly person-like geometry and measured moving evidence.

An unknown-motion learned ``vehicle`` is not enough. A false vehicle label on
a mapped wall would remove the strongest registration structure exactly when
the pose needs it. Vehicle/two-wheeler boxes become exclusions only after the
geometric tracker measures them moving.
"""


def normalize(value):
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def should_exclude(item):
    if not isinstance(item, dict):
        return False
    label = normalize(item.get("class"))
    motion = normalize(item.get("motion"))

    # A measured moving object is absent from the immutable map whatever its
    # classifier called it.
    if motion == "moving":
        return True
    # Person geometry is deliberately conservative even before the tracker
    # has enough history to decide motion. Learning may promote a geometric
    # box to person, but cannot make a non-person static wall disappear merely
    # by calling it a vehicle.
    return label == "person"
