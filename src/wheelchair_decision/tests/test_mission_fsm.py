#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest
from types import SimpleNamespace


CORE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "mission_core.py"
SPEC = importlib.util.spec_from_file_location("mission_core", str(CORE))
mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mission)

NODE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "mission_node.py"
NODE_SPEC = importlib.util.spec_from_file_location("mission_node", str(NODE))
node = importlib.util.module_from_spec(NODE_SPEC)
sys.modules[NODE_SPEC.name] = node
NODE_SPEC.loader.exec_module(node)

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


    def test_paused_prerequisite_loss_faults_without_resume(self):
        config = mission.MissionConfig(
            localization_ttl_s=2.0, geofence_ttl_s=2.0, collision_ttl_s=2.0,
            slope_ttl_s=2.0, mode_ttl_s=2.0, driver_ttl_s=2.0,
            obstacle_stop_entry_s=0.1)
        fsm = mission.MissionFSM(config, lambda: 0.0)
        pending = self.ready(fsm)
        fsm.update(mission.MissionEvent(
            "MOVE_BASE_ACTIVE", {"generation": pending.goal_generation}), 0.01)
        fsm.update(mission.MissionEvent("COLLISION", "blocked"), 0.11)
        paused = fsm.tick(0.21)
        self.assertEqual(mission.MissionState.PAUSED_OBSTACLE, paused.state)
        fault = fsm.update(mission.MissionEvent("GEOFENCE", False), 0.22)
        self.assertEqual(mission.MissionState.FAULT, fault.state)
        self.assertEqual("geofence_lost", fault.reason)
        self.assertEqual("canceling", fault.goal_state)

    def test_cancel_acknowledgement_is_generation_scoped_and_blocks_rearm(self):
        fsm = mission.MissionFSM(mission.MissionConfig(), lambda: 0.0)
        pending = self.ready(fsm)
        disarmed = fsm.disarm(0.01)
        self.assertEqual(mission.MissionState.DISARMED, disarmed.state)
        self.assertEqual("canceling", disarmed.goal_state)
        blocked = fsm.arm(ROUTE, 0.02)
        self.assertEqual(mission.MissionState.DISARMED, blocked.state)
        self.assertEqual("cancel_ack_pending", blocked.reason)
        stale = fsm.update(mission.MissionEvent(
            "MOVE_BASE_CANCELED", {"generation": pending.goal_generation - 1}), 0.03)
        self.assertEqual("canceling", stale.goal_state)
        acknowledged = fsm.update(mission.MissionEvent(
            "MOVE_BASE_CANCELED", {"generation": pending.goal_generation}), 0.04)
        self.assertEqual("canceled", acknowledged.goal_state)
        reset = fsm.reset(0.05)
        self.assertEqual(mission.MissionState.DISARMED, reset.state)
        armed = fsm.arm(ROUTE, 0.06)
        self.assertEqual(mission.MissionState.LOCALIZING, armed.state)


class RouteProgressAndZoneRegressionTest(unittest.TestCase):
    def setUp(self):
        self.binding = node.RouteBinding(
            "mission-a", "route-a", "outbound",
            "map-a",
            "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278",
            "route-hash",
            "a3c51baf020eb79e1550ba0d1a7fb40dddfff7e50ff2d142f1ebc3479bf732dc",
            None,
        )
        self.receipt = node.RouteProgressReceipt()

    def message(self, sequence, stamp, state=1, mission_id="mission-a",
                route_id="route-a", map_id="map-a"):
        return SimpleNamespace(
            sequence=sequence, state=state, mission_id=mission_id,
            route_id=route_id, map_id=map_id,
            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)),
        )

    def test_progress_rejects_regression_replay_identity_and_uncorrelated_complete(self):
        active = (1, 2)
        self.assertTrue(self.receipt.accept(self.message(10, 10.0), self.binding, 10.1, active, 3, False))
        self.assertFalse(self.receipt.accept(self.message(9, 10.2), self.binding, 10.25, active, 3, False))
        self.assertFalse(self.receipt.accept(self.message(11, 10.3), self.binding, 10.35, active, 3, False))
        self.assertFalse(self.receipt.accept(self.message(11, 10.4), self.binding, 10.45, active, 3, False))
        self.receipt.reset()
        self.assertTrue(self.receipt.accept(self.message(1, 20.0), self.binding, 20.1, active, 3, False))
        self.assertFalse(self.receipt.accept(
            self.message(2, 20.2, mission_id="other"), self.binding, 20.25, active, 3, False))
        self.assertFalse(self.receipt.accept(self.message(2, 20.3, 3), self.binding, 20.35, active, 3, False))

    def test_exact_hash_bound_simulation_zone_tuple(self):
        self.assertEqual(
            "simulation_unsurveyed",
            node.classify_speed_zone(
                ["zone-simulation-candidate", "candidate-unsurveyed"],
                self.binding.safety_manifest_sha256, self.binding.map_sha256),
        )
        for zones in (
                ["candidate-unsurveyed"],
                ["zone-simulation-candidate"],
                ["zone-simulation-candidate", "candidate-unsurveyed", "unknown"],
                ["zone-simulation-candidate", "candidate-unsurveyed "],
        ):
            with self.assertRaises(ValueError):
                node.classify_speed_zone(
                    zones, self.binding.safety_manifest_sha256, self.binding.map_sha256)

if __name__ == "__main__":
    unittest.main()
