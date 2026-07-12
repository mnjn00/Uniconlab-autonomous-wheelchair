#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


CORE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "mission_core.py"
SPEC = importlib.util.spec_from_file_location("mission_core", str(CORE))
mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mission)


ROUTE = {
    "valid": True,
    "graph_valid": True,
    "map_valid": True,
    "hash_valid": True,
    "route_id": "route-a",
    "map_id": "map-a",
    "route_hash": "sha256:abc",
    "waypoint_count": 2,
}


class MissionCoreTest(unittest.TestCase):
    def make_fsm(self, **overrides):
        values = {
            "evidence_timeout_s": 10.0,
            "progress_timeout_s": 10.0,
            "action_timeout_s": 2.0,
            "obstacle_stop_entry_s": 0.2,
            "obstacle_clear_s": 0.3,
        }
        values.update(overrides)
        return mission.MissionFSM(mission.MissionConfig(**values), lambda: 0.0)

    def make_ready(self, fsm, now=0.0):
        fsm.arm(ROUTE, now)
        updates = (
            (mission.EventType.LOCALIZATION, True),
            (mission.EventType.GEOFENCE, True),
            (mission.EventType.COLLISION, "clear"),
            (mission.EventType.SLOPE, "safe"),
            (mission.EventType.MODE, True),
            (mission.EventType.DRIVER, True),
        )
        output = None
        for kind, value in updates:
            output = fsm.update(mission.MissionEvent(kind, value), now)
        self.assertEqual(mission.MissionState.READY, output.state)
        self.assertEqual(0, output.send_waypoint_index)
        return output

    def make_navigating(self, fsm, now=0.0):
        self.make_ready(fsm, now)
        output = fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_ACTIVE), now)
        self.assertEqual(mission.MissionState.NAVIGATING, output.state)
        return output

    def test_transition_table_is_exhaustive_and_dispatches_every_pair(self):
        expected = {
            (state, event)
            for state in mission.MissionState
            for event in mission.EventType
        }
        self.assertEqual(expected, set(mission.MissionFSM.TRANSITION_TABLE))
        self.assertTrue(mission.MissionFSM.validate_transition_table())

        values = {
            mission.EventType.ARM: ROUTE,
            mission.EventType.ROUTE_STATUS: {
                "route_valid": True, "map_valid": True,
                "hash_valid": True, "graph_valid": True,
            },
            mission.EventType.LOCALIZATION: True,
            mission.EventType.GEOFENCE: True,
            mission.EventType.COLLISION: "clear",
            mission.EventType.SLOPE: "safe",
            mission.EventType.MODE: True,
            mission.EventType.DRIVER: True,
            mission.EventType.PROGRESS: 0,
        }
        for state in mission.MissionState:
            for event in mission.EventType:
                fsm = self.make_fsm()
                fsm.state = state
                if state == mission.MissionState.NAVIGATING:
                    fsm._route = dict(ROUTE)
                    fsm._waypoint_count = 2
                    fsm._goal_active = True
                    fsm._action_updated = 0.0
                    fsm._progress_updated = 0.0
                    for key in fsm._EVIDENCE:
                        fsm._updated[key] = 0.0
                    fsm._values.update({
                        "localization": True, "geofence": True,
                        "collision": "clear", "slope": "safe",
                        "mode": True, "driver": True,
                    })
                output = fsm.update(
                    mission.MissionEvent(event, values.get(event)), 0.0
                )
                self.assertIsInstance(output, mission.MissionOutput)


    def test_localizing_route_progress_seeds_first_navigation_waypoint(self):
        fsm = self.make_fsm()
        fsm.arm(ROUTE, 0.0)
        seeded = fsm.update(
            mission.MissionEvent(mission.EventType.PROGRESS, 1), 0.01)
        self.assertEqual(mission.MissionState.LOCALIZING, seeded.state)
        self.assertEqual(1, seeded.progress)

        output = seeded
        for kind, value in (
            (mission.EventType.LOCALIZATION, True),
            (mission.EventType.GEOFENCE, True),
            (mission.EventType.COLLISION, "clear"),
            (mission.EventType.SLOPE, "safe"),
            (mission.EventType.MODE, True),
            (mission.EventType.DRIVER, True),
        ):
            output = fsm.update(mission.MissionEvent(kind, value), 0.02)
        self.assertEqual(mission.MissionState.READY, output.state)
        self.assertEqual(1, output.send_waypoint_index)

        invalid = self.make_fsm()
        invalid.arm(ROUTE, 0.0)
        rejected = invalid.update(
            mission.MissionEvent(mission.EventType.PROGRESS, 2), 0.01)
        self.assertEqual(mission.MissionState.FAULT, rejected.state)
        self.assertEqual("invalid_initial_progress", rejected.reason)

    def test_only_navigating_can_emit_motion_and_slow_only_reduces_caps(self):
        fsm = self.make_fsm()
        for state in mission.MissionState:
            fsm.state = state
            output = fsm.output()
            if state != mission.MissionState.NAVIGATING:
                self.assertEqual(mission.MotionIntent.HOLD, output.intent)
                self.assertEqual((0.0, 0.0), (output.max_linear_mps, output.max_angular_rps))
        fsm = self.make_fsm()

        self.make_navigating(fsm)
        normal = fsm.output()
        slow = fsm.update(mission.MissionEvent(mission.EventType.SLOPE, "slow"), 0.1)
        self.assertEqual(mission.MotionIntent.PROCEED, normal.intent)
        self.assertEqual(mission.MotionIntent.SLOW, slow.intent)
        self.assertLess(slow.max_linear_mps, normal.max_linear_mps)
        self.assertLess(slow.max_angular_rps, normal.max_angular_rps)

    def test_critical_fault_cancels_latches_and_requires_reset_then_rearm(self):
        fsm = self.make_fsm()
        self.make_navigating(fsm)
        fault = fsm.update(mission.MissionEvent(mission.EventType.LOCALIZATION, False), 0.1)
        self.assertEqual(mission.MissionState.FAULT, fault.state)
        self.assertTrue(fault.cancel_goal)
        self.assertEqual(mission.MotionIntent.HOLD, fault.intent)
        self.assertEqual(mission.MissionState.FAULT, fsm.disarm(0.15).state)

        fsm.update(mission.MissionEvent(mission.EventType.LOCALIZATION, True), 0.2)
        fsm.update(mission.MissionEvent(mission.EventType.DRIVER, True), 0.2)
        self.assertEqual(mission.MissionState.FAULT, fsm.arm(ROUTE, 0.2).state)
        self.assertEqual(mission.MissionState.DISARMED, fsm.reset(0.3).state)
        self.assertEqual(mission.MissionState.LOCALIZING, fsm.arm(ROUTE, 0.3).state)

    def test_estop_driver_and_graph_faults_do_not_recover_implicitly(self):
        cases = (
            mission.MissionEvent(mission.EventType.COLLISION, "estop"),
            mission.MissionEvent(mission.EventType.DRIVER, False),
            mission.MissionEvent(mission.EventType.ROUTE_STATUS, {
                "route_valid": True, "map_valid": True,
                "hash_valid": False, "graph_valid": True,
            }),
        )
        for event in cases:
            fsm = self.make_fsm()
            self.make_navigating(fsm)
            output = fsm.update(event, 0.1)
            self.assertEqual(mission.MissionState.FAULT, output.state)
            self.assertTrue(output.cancel_goal)
            self.assertEqual(mission.MissionState.FAULT, fsm.tick(0.2).state)

    def test_obstacle_has_stop_and_clear_hysteresis_and_explicit_resume(self):
        fsm = self.make_fsm()
        self.make_navigating(fsm)
        blocked = fsm.update(mission.MissionEvent(mission.EventType.COLLISION, "blocked"), 0.0)
        self.assertEqual(mission.MissionState.NAVIGATING, blocked.state)
        self.assertEqual(mission.MissionState.NAVIGATING, fsm.tick(0.19).state)
        paused = fsm.tick(0.21)
        self.assertEqual(mission.MissionState.PAUSED_OBSTACLE, paused.state)
        self.assertTrue(paused.cancel_goal)

        fsm.update(mission.MissionEvent(mission.EventType.COLLISION, "clear"), 0.22)
        self.assertEqual(mission.MissionState.PAUSED_OBSTACLE, fsm.tick(0.60).state)
        resumed = fsm.resume(0.61)
        self.assertEqual(mission.MissionState.READY, resumed.state)
        self.assertEqual(0, resumed.send_waypoint_index)
        self.assertEqual(mission.MotionIntent.HOLD, resumed.intent)

    def test_action_and_progress_loss_are_fail_closed(self):
        action_fsm = self.make_fsm(action_timeout_s=0.5)
        self.make_navigating(action_fsm)
        action_fault = action_fsm.tick(0.51)
        self.assertEqual(mission.MissionState.FAULT, action_fault.state)
        self.assertEqual("stale_move_base", action_fault.reason)

        progress_fsm = self.make_fsm(action_timeout_s=5.0, progress_timeout_s=0.5)
        self.make_navigating(progress_fsm)
        progress_fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_ACTIVE), 0.4)
        progress_fault = progress_fsm.tick(0.51)
        self.assertEqual(mission.MissionState.FAULT, progress_fault.state)
        self.assertEqual("stale_progress", progress_fault.reason)

        process_fsm = self.make_fsm()
        self.make_ready(process_fsm)
        process_fault = process_fsm.update(
            mission.MissionEvent(mission.EventType.PROCESS_LOST, "route_manager"), 0.1
        )
        self.assertEqual(mission.MissionState.FAULT, process_fault.state)
        self.assertTrue(process_fault.cancel_goal)

    def test_stale_safety_evidence_faults_while_navigating(self):
        fsm = self.make_fsm(evidence_timeout_s=0.5, action_timeout_s=2.0)
        self.make_navigating(fsm)
        output = fsm.tick(0.51)
        self.assertEqual(mission.MissionState.FAULT, output.state)
        self.assertTrue(output.reason.startswith("stale_"))
        self.assertTrue(output.cancel_goal)

    def test_terminal_goal_and_abort_are_stopped_and_need_reset(self):
        fsm = self.make_fsm()
        self.make_navigating(fsm)
        first = fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_SUCCEEDED), 0.1)
        self.assertEqual(mission.MissionState.READY, first.state)
        self.assertEqual(1, first.send_waypoint_index)
        fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_ACTIVE), 0.2)
        done = fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_SUCCEEDED), 0.3)
        self.assertEqual(mission.MissionState.GOAL_REACHED, done.state)
        self.assertEqual("SUCCEEDED", done.terminal_status)
        self.assertEqual(mission.MotionIntent.HOLD, done.intent)
        self.assertEqual(mission.MissionState.GOAL_REACHED, fsm.tick(5.0).state)
        self.assertEqual(mission.MissionState.DISARMED, fsm.reset(5.1).state)

        aborted_fsm = self.make_fsm()
        self.make_navigating(aborted_fsm)
        aborted = aborted_fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_ABORTED), 0.1)
        self.assertEqual(mission.MissionState.ABORTED, aborted.state)
        self.assertTrue(aborted.cancel_goal)
        self.assertEqual(mission.MotionIntent.HOLD, aborted.intent)
        self.assertEqual(mission.MissionState.ABORTED, aborted_fsm.tick(1.0).state)

    def test_impossible_action_order_faults_armed_mission(self):
        fsm = self.make_fsm()
        self.make_ready(fsm)
        output = fsm.update(mission.MissionEvent(mission.EventType.MOVE_BASE_SUCCEEDED), 0.1)
        self.assertEqual(mission.MissionState.FAULT, output.state)
        self.assertEqual("move_base_success_out_of_order", output.reason)

    def test_invalid_route_fails_closed(self):
        bad = dict(ROUTE)
        bad["hash_valid"] = False
        output = self.make_fsm().arm(bad, 0.0)
        self.assertEqual(mission.MissionState.FAULT, output.state)
        self.assertEqual(mission.MotionIntent.HOLD, output.intent)

    def test_motion_caps_must_be_finite_positive_and_strictly_lower_for_slow(self):
        invalid = (
            {"max_linear_mps": float("nan")},
            {"slow_linear_mps": 0.35},
            {"slow_angular_rps": 0.65},
            {"max_angular_rps": 0.0},
        )
        for values in invalid:
            with self.assertRaises(ValueError):
                mission.MissionConfig(**values)


if __name__ == "__main__":
    unittest.main()
