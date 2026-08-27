#!/usr/bin/env python3
"""DWA follower with RTX/CuPy nearest-neighbour acceleration enabled."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Stock ROS Noetic SciPy predates the ``workers`` cKDTree keyword used by the
# planner. Install the compatibility constructor before either dwa_core or the
# GPU backend imports cKDTree.
from scipy_ckdtree_compat import install as install_ckdtree_compat
install_ckdtree_compat()

import rospy

import dwa_core
from gpu_dwa_backend import GpuRequiredError, install_gpu_planner

# ``dwa_follower.DwaFollower`` looks up ``dwa_core.DwaPlanner`` during its
# constructor. Install the accelerated subclass before importing/constructing
# the follower; the rest of the field-tested follower remains unchanged.
install_gpu_planner(dwa_core)
from dwa_follower import DwaFollower  # noqa: E402


if __name__ == "__main__":
    try:
        DwaFollower().spin()
    except GpuRequiredError as error:
        rospy.logfatal("required RTX DWA backend failed: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
