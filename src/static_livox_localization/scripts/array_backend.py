"""Which array library the heavy search runs on, and how it is allowed to fail.

The NUC11PHKi7C carries an RTX 2060. CuPy is the common array seam used by
GPU-capable search code, while NumPy remains the deterministic reference.
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
        if self.name == "cupy":
            return self.xp.asnumpy(array)
        return np.asarray(array)


def _probe(cupy):
    """Require a real allocation, reduction and synchronization."""
    probe = cupy.arange(1024, dtype=cupy.float32)
    value = float((probe * 2.0).sum())
    if not np.isclose(value, 1023.0 * 1024.0):
        raise RuntimeError("device arithmetic returned %r" % value)
    cupy.cuda.Device().synchronize()


def resolve(prefer_gpu=True, log=None):
    """Return the requested backend.

    ``prefer_gpu=False`` is an explicit operator/test request and must always
    force NumPy, even after a GPU backend was cached earlier in the process.
    The cache is used only for repeated GPU-preferred probes.
    """
    global _CACHED
    say = log or (lambda message: None)

    if not prefer_gpu:
        backend = Backend(np, "numpy", "not requested")
        say("array backend: numpy (CPU) - not requested")
        return backend

    if _CACHED is not None:
        return _CACHED

    if os.environ.get("WHEELCHAIR_DISABLE_GPU", "") == "1":
        _CACHED = Backend(np, "numpy", "WHEELCHAIR_DISABLE_GPU=1")
    else:
        try:
            import cupy
            _probe(cupy)
            _CACHED = Backend(cupy, "cupy")
        except Exception as error:                      # noqa: BLE001
            _CACHED = Backend(
                np, "numpy", "%s: %s" % (type(error).__name__, error))

    if _CACHED.on_gpu:
        say("array backend: cupy (GPU)")
    else:
        say("array backend: numpy (CPU) - %s" % _CACHED.reason)
    return _CACHED


def reset():
    """Forget the cached GPU-preferred choice. Tests only."""
    global _CACHED
    _CACHED = None
