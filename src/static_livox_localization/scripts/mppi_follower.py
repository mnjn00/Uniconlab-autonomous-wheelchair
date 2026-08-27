#!/usr/bin/env python3
"""MPPI control profile behind the field-tested DWA follower safety shell.

This file intentionally subclasses ``DwaFollower`` instead of copying its
``step()`` method. The semantic WAIT/GO_ROUND decision, measured 0.55 s
actuation lead, speed policy, command ramp, joystick override and every
inherited fail-closed guard therefore stay byte-for-byte on the field-tested
path. Only ``self.planner`` is replaced.

The branch is for bench/bag replay and controlled wheel-off-ground testing
before any field promotion. The existing DWA profile remains the rollback.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ROS Noetic's SciPy compatibility shim must be installed before planner
# modules import cKDTree, exactly as in gpu_dwa_follower.py.
from scipy_ckdtree_compat import install as install_ckdtree_compat
install_ckdtree_compat()

import rospy

from dwa_follower import DwaFollower
from gpu_dwa_backend import GpuRequiredError
import mppi_core


class MppiFollower(DwaFollower):
    CONTROL_LAW = "mppi"

    def __init__(self):
        # Builds the proven follower shell first. DwaFollower creates its old
        # planner during construction; it is replaced immediately below and
        # never receives a motion cycle.
        DwaFollower.__init__(self)
        route_mask = self.planner.route_mask

        prefer_default = os.environ.get("WHEELCHAIR_MPPI_GPU", "1") != "0"
        require_default = os.environ.get("WHEELCHAIR_REQUIRE_GPU", "0") == "1"
        prefer_gpu = bool(rospy.get_param("~prefer_gpu", prefer_default))
        require_gpu = bool(rospy.get_param("~require_gpu", require_default))

        self.planner = mppi_core.MppiPlanner(
            self.band,
            self.waypoints,
            route_mask=route_mask,
            horizon_steps=int(rospy.get_param(
                "~horizon_steps", mppi_core.HORIZON_STEPS)),
            model_dt=float(rospy.get_param("~model_dt", mppi_core.MODEL_DT)),
            batch_size=int(rospy.get_param(
                "~batch_size", mppi_core.BATCH_SIZE)),
            temperature=float(rospy.get_param(
                "~temperature", mppi_core.TEMPERATURE)),
            noise_v=float(rospy.get_param("~noise_v", mppi_core.NOISE_V)),
            noise_w=float(rospy.get_param("~noise_w", mppi_core.NOISE_W)),
            seed=int(rospy.get_param("~seed", mppi_core.SEED)),
            grace=float(rospy.get_param("~band_grace", 0.0)),
            prefer_gpu=prefer_gpu,
            require_gpu=require_gpu,
            log=lambda message: rospy.loginfo(message),
        )

        rospy.set_param("~distance_backend", self.planner.backend_name)
        rospy.set_param("~gpu_active", bool(self.planner.gpu_active))
        rospy.set_param("~mppi_batch_size", int(self.planner.batch_size))
        rospy.set_param("~mppi_horizon_steps", int(self.planner.steps))
        rospy.set_param("~mppi_model_dt", float(self.planner.dt))
        rospy.loginfo(
            "MPPI profile: %d samples x %d steps @ %.3f s, backend=%s, "
            "DWA safety shell retained",
            self.planner.batch_size, self.planner.steps, self.planner.dt,
            self.planner.backend_name)

    def publish_state(self, text, state=None):
        # DwaFollower.step is deliberately reused without copying safety code.
        # Translate its diagnostic vocabulary at the final publication seam.
        text = str(text).replace("DWA", "MPPI")
        state = None if state is None else str(state).replace("DWA", "MPPI")
        return DwaFollower.publish_state(self, text, state)


if __name__ == "__main__":
    try:
        MppiFollower().run()
    except GpuRequiredError as error:
        rospy.logfatal("required MPPI GPU backend failed: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
