"""Compatibility wrapper for ``scipy.spatial.cKDTree.query``.

Ubuntu 20.04 / ROS Noetic ships a SciPy release whose parallel keyword is
``n_jobs`` (or has no parallel keyword), while newer releases use ``workers``.
The field planner was written against the newer spelling and therefore raised
``TypeError`` before scoring a single DWA trajectory on a stock Noetic image.

``install()`` replaces only the public constructor in ``scipy.spatial``. New
trees return a transparent proxy that retries the query with the spelling the
installed SciPy accepts. All other attributes and methods are delegated to the
real Cython tree.
"""

from __future__ import annotations

import threading


_LOCK = threading.Lock()
_INSTALLED = False
_ORIGINAL = None


class CompatibleKDTree(object):
    """Delegate to the real tree while normalising its query keyword."""

    def __init__(self, *args, **kwargs):
        if _ORIGINAL is None:
            raise RuntimeError("cKDTree compatibility wrapper is not installed")
        self._tree = _ORIGINAL(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._tree, name)

    def query(self, *args, **kwargs):
        if "workers" not in kwargs:
            return self._tree.query(*args, **kwargs)
        workers = kwargs.pop("workers")
        try:
            return self._tree.query(*args, workers=workers, **kwargs)
        except TypeError as workers_error:
            try:
                return self._tree.query(*args, n_jobs=workers, **kwargs)
            except TypeError:
                try:
                    return self._tree.query(*args, **kwargs)
                except TypeError:
                    raise workers_error


def install():
    """Install once and return the compatible constructor."""
    global _INSTALLED, _ORIGINAL
    with _LOCK:
        if _INSTALLED:
            return CompatibleKDTree
        import scipy.spatial
        current = scipy.spatial.cKDTree
        if current is CompatibleKDTree:
            _INSTALLED = True
            return CompatibleKDTree
        _ORIGINAL = current
        scipy.spatial.cKDTree = CompatibleKDTree
        _INSTALLED = True
        return CompatibleKDTree
