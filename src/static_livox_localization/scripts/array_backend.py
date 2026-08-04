"""Which array library the heavy search runs on, and how it is allowed to fail.

The NUC11PHKi7C carries an RTX 2060 that nothing in this stack has ever
used, while the thing that most needs the compute - the seedless global
initial-pose search - is bounded by what a CPU can afford rather than by
what would actually find the chair. This module is the seam between them.

CuPy rather than hand-written CUDA, deliberately. The scoring code is
array arithmetic; expressed against a backend module it is the SAME source
on both devices, so the CPU path is not a fallback that drifts out of date
- it is the reference the GPU path is tested against, line for line. Hand
written kernels would have been faster and would have had no such thing.

THE RULE HERE: the GPU may make an answer arrive sooner. It may never make
it different, and it may never make it absent. Anything that fails on the
device falls back to NumPy and says so out loud; a search that would have
succeeded on the CPU must not fail because an accelerator was present.
That is why `resolve` probes with a real allocation instead of trusting
`import cupy` - a machine with the library and no working driver is a
configuration this stack has to survive, not diagnose in the field.
"""

import os

import numpy as np

_CACHED = None


class Backend(object):
    """An array module plus the two conversions the callers need."""

    def __init__(self, xp, name, reason=""):
        self.xp = xp
        self.name = name
        self.reason = reason

    @property
    def on_gpu(self):
        return self.name == "cupy"

    def asarray(self, array, dtype=np.float32):
        return self.xp.asarray(array, dtype=dtype)

    def tohost(self, array):
        """Back to NumPy, whichever device it was on."""
        if self.name == "cupy":
            return self.xp.asnumpy(array)
        return np.asarray(array)


def _probe(cupy):
    """A real allocation and a real reduction.

    Importing cupy succeeds on a machine whose driver is missing, wedged, or
    a version behind the runtime; the failure then lands on the first kernel
    launch, which here would be in the middle of localising a chair. Spend a
    few milliseconds finding out at startup instead.
    """
    probe = cupy.arange(1024, dtype=cupy.float32)
    value = float((probe * 2.0).sum())
    if not np.isclose(value, 1023.0 * 1024.0):
        raise RuntimeError("device arithmetic returned %r" % value)
    cupy.cuda.Device().synchronize()


def resolve(prefer_gpu=True, log=None):
    """Pick a backend once and remember it.

    prefer_gpu=False forces NumPy - the switch an operator gets when the
    GPU is suspected, and the one the equivalence tests use to run both
    paths over the same input.
    """
    global _CACHED
    if _CACHED is not None:
        return _CACHED
    say = log or (lambda message: None)
    if not prefer_gpu:
        _CACHED = Backend(np, "numpy", "not requested")
    elif os.environ.get("WHEELCHAIR_DISABLE_GPU", "") == "1":
        _CACHED = Backend(np, "numpy", "WHEELCHAIR_DISABLE_GPU=1")
    else:
        try:
            import cupy
            _probe(cupy)
            _CACHED = Backend(cupy, "cupy")
        except Exception as error:                      # noqa: BLE001
            _CACHED = Backend(np, "numpy", "%s: %s"
                              % (type(error).__name__, error))
    if _CACHED.on_gpu:
        say("array backend: cupy (GPU)")
    else:
        say("array backend: numpy (CPU) - %s" % _CACHED.reason)
    return _CACHED


def reset():
    """Forget the cached choice. Tests only."""
    global _CACHED
    _CACHED = None
