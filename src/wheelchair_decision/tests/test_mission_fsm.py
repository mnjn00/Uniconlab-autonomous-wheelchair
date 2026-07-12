#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


CORE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "mission_core.py"
SPEC = importlib.util.spec_from_file_location("mission_core", str(CORE))
mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mission)

ROUTE = {
    "valid": True, "graph_valid": True, "map_valid": True, "hash_valid": True,
    "route_id": "route-a", "map_id": "map-a", "route_hash": "hash-a",
    "waypoint_count": 2,
}


class MissionLifecycleTest(unittest.TestCase):
    def ready(self, fsm):
        fsm.arm(ROUTE, 0.0)
        for kind, value in (("LOCALIZATION", True), ("GEOFENCE", True),
                            ("COLLISION", "clear"), ("SLOPE", "safe"),
                            ("MODE", True), ("DRIVER", True)):
            output = fsm.update(mission.MissionEvent(kind, value), 0.0)
        return output

    def test_pending_goal_cannot_activate_after_readiness_revocation(self):
        fsm = mission.MissionFSM(mission.MissionConfig(), lambda: 0.0)
        pending = self.ready(fsm)
        self.assertEqual("pending", pending.goal_state)
        revoked = fsm.update(mission.MissionEvent("GEOFENCE", False), 0.01)
        self.assertEqual(mission.MissionState.FAULT, revoked.state)
        self.assertTrue(revoked.cancel_goal)
        stale = fsm.update(mission.MissionEvent(
            "MOVE_BASE_ACTIVE", {"generation": pending.goal_generation}), 0.02)
        self.assertEqual(mission.MissionState.FAULT, stale.state)
        self.assertEqual(mission.MotionIntent.HOLD, stale.intent)

    def test_stale_generation_and_signal_ttl_fail_closed(self):
        fsm = mission.MissionFSM(mission.MissionConfig(), lambda: 0.0)
        pending = self.ready(fsm)
        active = fsm.update(mission.MissionEvent(
            "MOVE_BASE_ACTIVE", {"generation": pending.goal_generation}), 0.01)
        self.assertEqual(mission.MissionState.NAVIGATING, active.state)
        stale = fsm.update(mission.MissionEvent(
            "MOVE_BASE_SUCCEEDED", {"generation": pending.goal_generation - 1}), 0.02)
        self.assertEqual(mission.MissionState.NAVIGATING, stale.state)
        expired = fsm.tick(0.11)
        self.assertEqual(mission.MissionState.FAULT, expired.state)
        self.assertEqual(mission.MotionIntent.HOLD, expired.intent)


if __name__ == "__main__":
    unittest.main()
