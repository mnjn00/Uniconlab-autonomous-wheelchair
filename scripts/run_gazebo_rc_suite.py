#!/usr/bin/env python3
"""Deterministic Gazebo RC scenario orchestration and evidence reporting.

The fake backend is deliberately an import-only test seam.  The command-line runner always
uses a live ROS1/Gazebo process graph and fails closed when that graph or its evidence probe
is unavailable.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

CLAIM_TAG = "SIMULATION_ONLY"
REPORT_TYPE = "algorithm-adversarial-test-report"
REQUIRED_TOPICS = [
    "/clock", "/gazebo/model_states", "/simulation/contacts",
    "/route/progress", "/localization/status", "/safety/collision_status",
    "/route_safety/geofence_status", "/safety/slope_status", "/safety/state",
    "/cmd_vel_nav", "/cmd_vel_safe",
]
FAULT_EVENT_TOPIC = "/simulation/fault_event"
ACTUATOR_COMMAND_TOPIC = "/wheelchair_base_controller/cmd_vel"
FAULT_REQUIRED_TOPICS = (FAULT_EVENT_TOPIC, ACTUATOR_COMMAND_TOPIC)
APPROVED_STOP_BUDGET_S = 0.15
SAFETY_REASON_BITS = {
    1: "ESTOP",
    2: "STALE_CMD",
    4: "MODE",
    8: "GEOFENCE",
    16: "COLLISION",
    32: "LOCALIZATION",
    64: "DRIVER",
    128: "INVALID_CMD",
    256: "CLOCK",
    512: "STALE_INTENT",
    1024: "INTERNAL_FAULT",
    2048: "STARTUP",
    4096: "SENSOR_STALE",
    8192: "COLLISION_BLIND",
    16384: "COLLISION_TTC",
    32768: "COLLISION_DISTANCE",
    65536: "SLOPE",
    131072: "IMU_UNCALIBRATED",
    262144: "ROUTE_MANIFEST",
    524288: "GRAPH_TOPOLOGY",
    1048576: "TF",
    2097152: "BACKPRESSURE",
    4194304: "DEADLINE_MISS",
    8388608: "MANUAL_OVERRIDE",
    16777216: "HARDWARE_UNVERIFIED",
    33554432: "MAP_MISMATCH",
    67108864: "COLLISION_OCCLUDED",
    134217728: "LOCALIZATION_INCONSISTENT",
    268435456: "RESOURCE",
    536870912: "CORRUPT_DATA",
    1073741824: "RESET_REJECTED",
    2147483648: "INPUT_UNKNOWN",
    4294967296: "ROUTE_STATE",
    8589934592: "ODOM_STALE",
    17179869184: "IMU_STALE",
    34359738368: "LIDAR_STALE",
    68719476736: "POLICY_MISMATCH",
}
DEFINED_SAFETY_REASON_MASK = sum(SAFETY_REASON_BITS)
MINIMUM_TERMINAL_SETTLE_S = 0.60
REQUIRED_WORLD_IDS = {
    "empty",
    "road_free_space",
    "sidewalk_obstacles",
    "static_dynamic_obstacles",
}
DYNAMIC_OBSTACLE_WORLD = "static_dynamic_obstacles"
APPROVED_ROBUSTNESS_SEEDS = tuple(range(30))
APPROVED_CEILINGS = {
    "cross_track_mean_m": 0.20,
    "cross_track_p95_m": 0.35,
    "cross_track_max_m": 0.45,
    "goal_error_m": 0.30,
    "goal_error_yaw_deg": 10.0,
    "linear_cap_mps": 0.55,
    "angular_cap_rps": 0.85,
}
REQUIRED_FAULTS = {
    "lidar_loss": "sensor_loss",
    "imu_loss": "sensor_loss",
    "odom_loss": "sensor_loss",
    "tf_loss": "transform_loss",
    "localizer_loss": "process_loss",
    "decision_loss": "process_loss",
    "safety_loss": "process_loss",
    "driver_loss": "process_loss",
    "generic_process_loss": "process_loss",
    "stale_lidar": "timestamp",
    "future_imu": "timestamp",
    "out_of_order_odom": "timestamp",
    "nan_command": "malformed",
    "duplicate_cmd_publisher": "graph",
    "duplicate_tf_authority": "graph",
    "clock_reset": "clock",
    "cpu_pressure": "resource",
    "queue_pressure": "resource",
    "estop_asserted": "estop",
    "reset_while_asserted": "reset_misuse",
    "reset_while_moving": "reset_misuse",
    "reset_in_auto": "reset_misuse",
    "graph_bypass": "graph",
}
REQUIRED_FAULT_REASONS = {
    "lidar_loss": ("SENSOR_STALE",),
    "imu_loss": ("SENSOR_STALE",),
    "odom_loss": ("ODOM_STALE",),
    "tf_loss": ("TF",),
    "localizer_loss": ("LOCALIZATION",),
    "decision_loss": ("STALE_INTENT",),
    "safety_loss": ("INTERNAL_FAULT",),
    "driver_loss": ("DRIVER",),
    "generic_process_loss": ("ROUTE_STATE",),
    "stale_lidar": ("SENSOR_STALE",),
    "future_imu": ("CLOCK",),
    "out_of_order_odom": ("CLOCK",),
    "nan_command": ("INVALID_CMD",),
    "duplicate_cmd_publisher": ("GRAPH_TOPOLOGY",),
    "duplicate_tf_authority": ("GRAPH_TOPOLOGY", "TF"),
    "clock_reset": ("CLOCK",),
    "cpu_pressure": ("DEADLINE_MISS",),
    "queue_pressure": ("BACKPRESSURE",),
    "estop_asserted": ("ESTOP",),
    "reset_while_asserted": ("ESTOP", "RESET_REJECTED"),
    "reset_while_moving": ("RESET_REJECTED",),
    "reset_in_auto": ("RESET_REJECTED",),
    "graph_bypass": ("GRAPH_TOPOLOGY",),
}
REQUIRED_LATCH_FAULTS = {
    "nan_command",
    "duplicate_cmd_publisher",
    "duplicate_tf_authority",
    "clock_reset",
    "estop_asserted",
    "reset_while_asserted",
    "reset_while_moving",
    "reset_in_auto",
    "graph_bypass",
}
BINDING_SOURCES = {
    "a13": (
        "contracts/wp0/A13-simulator-fidelity.yaml",
        "f7984b85d8ccf7d0f481f7daa73135109440209ca8c62847c57095da47604123",
    ),
    "route_truth": (
        "src/wheelchair_gazebo/config/route_truth_outbound.yaml",
        "627d3a2e69d2601f4e69045f9be9e95179e42dde612dc91a22bb151247df947e",
    ),
}
FROZEN_BINDING_OPTIONS = {
    "--scenario-sha256": "{scenario_sha256}",
    "--a13-sha256": "{a13_sha256}",
    "--claim-tag": CLAIM_TAG,
    "--route-truth": "{route_truth}",
    "--route-truth-sha256": "{route_truth_sha256}",
    "--scenario": "{scenario}",
}


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _contained_regular_path(root, reference):
    if not isinstance(reference, str) or Path(reference).is_absolute():
        raise ValueError("binding source path must be a relative repository path")
    candidate = root / reference
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError("binding source escapes repository root") from exc
    current = root
    for part in candidate.relative_to(root).parts:
        current /= part
        if current.is_symlink():
            raise ValueError("binding source must not be symlinked")
    if not candidate.is_file():
        raise ValueError("binding source is missing or not a regular file")
    return candidate


def _validate_binding_contract(config):
    bindings = config.get("binding_sources")
    if not isinstance(bindings, dict) or set(bindings) != set(BINDING_SOURCES):
        raise ValueError("binding_sources must define exactly the immutable repository sources")
    for name, (expected_path, expected_sha256) in BINDING_SOURCES.items():
        value = bindings.get(name)
        if not isinstance(value, dict):
            raise ValueError("binding source {} is missing".format(name))
        if value.get("path") != expected_path or value.get("sha256") != expected_sha256:
            raise ValueError("binding source {} does not match the immutable contract".format(name))
    return bindings


def _validate_binding_sources(config, config_path):
    repository = Path(__file__).resolve().parents[1]
    config_path = Path(config_path)
    try:
        config_path.relative_to(repository)
    except ValueError as exc:
        raise ValueError("scenario config must be contained by repository root") from exc
    if config_path.is_symlink() or not config_path.is_file():
        raise ValueError("scenario config must be a non-symlink regular file")

    bindings = _validate_binding_contract(config)
    resolved = {}
    for name, (_, expected_sha256) in BINDING_SOURCES.items():
        value = bindings[name]
        source = _contained_regular_path(repository, value["path"])
        actual_sha256 = _sha256(source)
        if actual_sha256 != expected_sha256:
            raise ValueError("binding source {} hash mismatch".format(name))
        resolved[name] = source
    return {
        "config_path": str(config_path),
        "scenario_sha256": _sha256(config_path),
        "a13_sha256": bindings["a13"]["sha256"],
        "route_truth": str(resolved["route_truth"]),
        "route_truth_sha256": bindings["route_truth"]["sha256"],
    }


def _validate_collector_binding_options(config):
    for option, expected in FROZEN_BINDING_OPTIONS.items():
        if _collector_value(config, option) != expected:
            raise ValueError("{} must be fixed to {}".format(option, expected))


def _runtime_binding(config):
    binding = config.get("_collector_binding")
    if not isinstance(binding, dict):
        raise ValueError("collector binding was not loaded from an immutable config")
    _validate_collector_binding_options(config)
    current = _validate_binding_sources(config, binding.get("config_path"))
    if current != binding:
        raise ValueError("binding sources changed after config load")
    return binding


class PlatformUnavailable(RuntimeError):
    """Raised when a real Noetic/Gazebo surface cannot be exercised."""


class EvidenceError(RuntimeError):
    """Raised when a live process did not produce trustworthy scenario evidence."""

def _collector_value(config, option):
    command = config.get("collector_command", [])
    if not isinstance(command, list) or command.count(option) != 1:
        raise ValueError("collector command must provide {} exactly once".format(option))
    index = command.index(option)
    if index + 1 >= len(command):
        raise ValueError("collector command must provide a value for {}".format(option))
    return command[index + 1]


def _collector_option(config, option):
    return _finite_number(_collector_value(config, option), option)


def _finite_number(value, description):
    if isinstance(value, bool):
        raise ValueError(description + " must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(description + " must be a finite number")
    if not math.isfinite(number):
        raise ValueError(description + " must be a finite number")
    return number


def _validate_ac5_matrix(data):
    worlds = data.get("worlds")
    if not isinstance(worlds, list) or len(worlds) != len(REQUIRED_WORLD_IDS):
        raise ValueError("AC5 requires exactly four worlds")
    if not all(isinstance(world, dict) for world in worlds):
        raise ValueError("every AC5 world must be an object")
    world_ids = [world.get("id") for world in worlds]
    if (not all(isinstance(world_id, str) for world_id in world_ids)
            or len(set(world_ids)) != len(world_ids)
            or set(world_ids) != REQUIRED_WORLD_IDS):
        raise ValueError("AC5 world IDs do not match the approved four-world matrix")
    if any(not isinstance(world.get("file"), str) or not world["file"] for world in worlds):
        raise ValueError("every AC5 world must provide a non-empty file")

    repetitions = data.get("deterministic_repetitions")
    if isinstance(repetitions, bool) or not isinstance(repetitions, int) or repetitions < 10:
        raise ValueError("AC5 requires at least ten deterministic repetitions per world")

    seeds = data.get("robustness_seeds")
    if (not isinstance(seeds, list)
            or any(isinstance(seed, bool) or not isinstance(seed, int) for seed in seeds)
            or len(seeds) != 30
            or len(set(seeds)) != 30
            or tuple(seeds) != APPROVED_ROBUSTNESS_SEEDS):
        raise ValueError("AC5 requires the thirty approved unique deterministic robustness seeds")

    minimum = data.get("minimum_robustness_acceptable")
    if (isinstance(minimum, bool) or not isinstance(minimum, int)
            or minimum < 29 or minimum > len(seeds)):
        raise ValueError("robustness acceptance must be at least 29 and no greater than the seed count")

    for field in ("robustness_world", "fault_world"):
        if data.get(field) != DYNAMIC_OBSTACLE_WORLD:
            raise ValueError(field + " must be fixed to the dynamic obstacle world")

    thresholds = data.get("thresholds")
    if not isinstance(thresholds, dict):
        raise ValueError("AC5 thresholds must be an object")
    for field, ceiling in APPROVED_CEILINGS.items():
        value = _finite_number(thresholds.get(field), field)
        if value <= 0.0 or value > ceiling:
            raise ValueError("{} must be positive and no greater than {}".format(field, ceiling))

    collector_linear = _collector_option(data, "--linear-cap-mps")
    collector_angular = _collector_option(data, "--angular-cap-rps")
    if (not math.isfinite(collector_linear)
            or not math.isfinite(collector_angular)
            or collector_linear <= 0.0
            or collector_linear > _finite_number(
                thresholds["linear_cap_mps"], "linear_cap_mps")
            or collector_angular <= 0.0
            or collector_angular > _finite_number(
                thresholds["angular_cap_rps"], "angular_cap_rps")):
        raise ValueError("collector software caps must not exceed validated threshold caps")

    frozen_options = {
        "--fault-event-topic": FAULT_EVENT_TOPIC,
        "--actuator-command-topic": ACTUATOR_COMMAND_TOPIC,
    }
    for option, expected in frozen_options.items():
        if _collector_value(data, option) != expected:
            raise ValueError("{} must be fixed to {}".format(option, expected))
    if _collector_option(data, "--stop-budget-s") != APPROVED_STOP_BUDGET_S:
        raise ValueError("collector stop budget must be fixed to {:.2f}s".format(
            APPROVED_STOP_BUDGET_S))

    faults = data.get("fault_matrix")
    if not isinstance(faults, list) or not faults:
        raise ValueError("AC5 requires a non-empty complete fault matrix")
    if not all(isinstance(fault, dict) for fault in faults):
        raise ValueError("every fault entry must be an object")
    fault_ids = [fault.get("id") for fault in faults]
    if (not all(isinstance(fault_id, str) for fault_id in fault_ids)
            or len(set(fault_ids)) != len(fault_ids)):
        raise ValueError("fault matrix IDs must be unique strings")
    actual_faults = {fault.get("id"): fault.get("category") for fault in faults}
    if actual_faults != REQUIRED_FAULTS:
        raise ValueError("fault matrix must contain every approved fault ID and category")
    for fault in faults:
        fault_id = fault["id"]
        if tuple(fault.get("expected_reasons", ())) != REQUIRED_FAULT_REASONS[fault_id]:
            raise ValueError("fault expected reasons do not match the approved matrix")
        requires_latch = fault_id in REQUIRED_LATCH_FAULTS
        if "requires_latch" not in fault or fault["requires_latch"] is not requires_latch:
            raise ValueError("fault latch requirement does not match the approved matrix")

    if (data.get("claim_tag") != CLAIM_TAG
            or data.get("simulation_only") is not True
            or data.get("hardware_motion_authorized") is not False
            or data.get("passenger_operation_authorized") is not False):
        raise ValueError("qualification config must claim simulation-only authority")
    _validate_collector_binding_options(data)
    _validate_binding_contract(data)

def validate_config(data):
    if not isinstance(data, dict):
        raise ValueError("scenario config must be an object")
    _validate_ac5_matrix(data)
    launch = data.get("launch", {})
    if launch.get("package") != "wheelchair_bringup" or launch.get("file") != "sim_bringup.launch":
        raise ValueError("qualification requires wheelchair_bringup/sim_bringup.launch")
    direction = data.get("scenario_direction")
    lengths = data.get("route_lengths_m", {})
    if direction not in ("outbound", "return") or direction not in lengths:
        raise ValueError("selected scenario direction has no route length")
    route_length = float(lengths[direction])
    linear_cap = _finite_number(data["thresholds"]["linear_cap_mps"], "linear cap")
    margin = _finite_number(data.get("route_timeout_margin_s", 0.0), "route timeout margin")
    collector_timeout = _collector_option(data, "--timeout")
    settle_time = _collector_option(data, "--settle-time")
    run_timeout = _finite_number(data.get("run_timeout_s", 0.0), "run timeout")
    values = (route_length, linear_cap, margin, collector_timeout, settle_time, run_timeout)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("route and timeout configuration must be finite")
    if route_length <= 0.0 or linear_cap <= 0.0 or margin < 0.0:
        raise ValueError("route length/cap/margin configuration is invalid")
    minimum_timeout = route_length / linear_cap + margin
    if collector_timeout < minimum_timeout:
        raise ValueError(
            "collector timeout {:.3f}s cannot finish selected {:.3f}m route "
            "(minimum {:.3f}s at configured cap plus margin)".format(
                collector_timeout, route_length, minimum_timeout))
    if settle_time < MINIMUM_TERMINAL_SETTLE_S:
        raise ValueError("terminal settle time must be at least {:.2f}s".format(
            MINIMUM_TERMINAL_SETTLE_S))
    if run_timeout <= collector_timeout:
        raise ValueError("run timeout must exceed collector timeout")
    return data



def load_config(path):
    config_path = Path(path).absolute()
    text = config_path.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as exc:
            raise ValueError("scenario config is not JSON and PyYAML is unavailable") from exc
        data = yaml.safe_load(text)
    data = validate_config(data)
    data["_collector_binding"] = _validate_binding_sources(data, config_path)
    return data


def percentile(values, percentage):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise EvidenceError("cross-track sample list is empty")
    rank = (len(ordered) - 1) * percentage / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    return ordered[low] if low == high else ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def validate_result(raw, thresholds, require_completion):
    """Validate one run, returning normalized evidence plus every blocking reason."""
    result = dict(raw)
    failures = []
    if result.get("live_evidence") is not True:
        failures.append("missing live ROS/Gazebo evidence")
    if result.get("passed") is False:
        failures.append("collector reported blocking failures")
    if "missing_topics" in result and result["missing_topics"] != []:
        failures.append("collector is missing required stream evidence")
    verdicts = result.get("verdicts")
    if isinstance(verdicts, dict):
        for name in ("topics_complete", "samples_finite", "clock_monotonic",
                     "terminal_evidence", "terminal_inputs_fresh",
                     "command_limits", "zero_after_stop", "stopping_envelope"):
            if name in verdicts and verdicts[name] is not True:
                failures.append("collector safety verdict failed: " + name)

    outcome = result.get("route_outcome")
    result["route_outcome"] = outcome
    if outcome not in ("completed", "safe_abort"):
        failures.append("route outcome is neither completed nor safe_abort")
    if require_completion and outcome != "completed":
        failures.append("deterministic route did not complete")

    for field in ("footprint_collisions", "geofence_exits"):
        if result.get(field) != 0:
            failures.append(field + " is nonzero")

    command = result.get("command", {})
    if command.get("finite") is not True:
        failures.append("command stream contains a non-finite value")
    if command.get("caps_respected") is not True:
        failures.append("command cap was exceeded")
    if command.get("shape_respected") is False:
        failures.append("command shape was violated")
    if command.get("nonzero_after_fault", 0) != 0:
        failures.append("nonzero command observed after fault")

    stop = result.get("stop", {})
    if stop.get("envelope_respected") is not True:
        failures.append("stopping envelope was violated")
    ttc = stop.get("minimum_ttc_s")
    if not isinstance(ttc, (int, float)) or not math.isfinite(ttc):
        failures.append("minimum TTC is missing or non-finite")

    hysteresis = result.get("hysteresis", {})
    if hysteresis.get("stop_observed") is not True:
        failures.append("required stop was not observed")
    events = hysteresis.get("reason_events")
    if not isinstance(events, list) or not events:
        failures.append("structured stop reason events are missing")

    if outcome == "completed":
        samples = result.get("cross_track_samples_m")
        if (not isinstance(samples, list) or not samples or
                not all(isinstance(x, (int, float)) and math.isfinite(x)
                        for x in samples)):
            failures.append("cross-track samples are missing or non-finite")
        else:
            absolute = [abs(float(x)) for x in samples]
            result["cross_track_m"] = {
                "mean": sum(absolute) / len(absolute),
                "p95": percentile(absolute, 95.0),
                "max": max(absolute),
            }
            for name in ("mean", "p95", "max"):
                limit = float(thresholds["cross_track_" + name + "_m"])
                if result["cross_track_m"][name] > limit:
                    failures.append("cross-track {} exceeds {:.3f} m".format(name, limit))
        for field, limit_key in (("goal_error_m", "goal_error_m"),
                                 ("goal_error_yaw_deg", "goal_error_yaw_deg")):
            value = result.get(field)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                failures.append(field + " is missing or non-finite")
            elif float(value) > float(thresholds[limit_key]):
                failures.append(field + " exceeds limit")
        if hysteresis.get("resume_after_clear") is not True:
            failures.append("clear hysteresis/resume was not observed")
    elif outcome == "safe_abort":
        has_stop_reason = any(
            isinstance(event, dict) and event.get("event") == "stop" and
            isinstance(event.get("source"), str) and event["source"]
            for event in events
        )
        if not has_stop_reason:
            failures.append("safe abort lacks a structured stop reason")
        for field in ("trigger_stamp_s", "zero_stamp_s", "latency_s", "overshoot_m"):
            value = stop.get(field)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                failures.append("safe abort lacks stop boundary evidence: " + field)

    result["failures"] = list(dict.fromkeys(failures))
    result["passed"] = not result["failures"]
    return result


class FakeBackend:
    """Deterministic backend used only by unit tests; never exposed by the CLI."""

    def __init__(self, results):
        self.results = results
        self.calls = []

    def run_scenario(self, world, seed, robustness=False, fault=None):
        self.calls.append((world["id"], seed, robustness, fault))
        value = self.results[(world["id"], seed, robustness, fault)]
        return dict(value)


class RosGazeboBackend:
    """Noetic process backend using roslaunch plus a live metrics collector node."""

    def __init__(self, config, startup_timeout=30.0, run_timeout=None):
        self.config = config
        self.startup_timeout = float(startup_timeout)
        self.run_timeout = float(run_timeout or config.get("run_timeout_s", 180.0))

    def preflight(self):
        commands = (
            "roslaunch", "rosnode", "rostopic", "rosservice", "rospack", "rosrun",
        )
        missing = [name for name in commands if shutil.which(name) is None]
        if missing:
            raise PlatformUnavailable("missing Noetic commands: " + ", ".join(missing))
        if os.environ.get("ROS_DISTRO") != "noetic":
            raise PlatformUnavailable("ROS_DISTRO must be noetic")
        for package in self.config.get("required_packages", ()):
            check = subprocess.run(
                ["rospack", "find", package], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, timeout=10)
            if check.returncode:
                raise PlatformUnavailable("ROS package unavailable: " + package)

    @staticmethod
    def _stop_process(process):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3)

    @staticmethod
    def _names(command, timeout):
        completed = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=timeout)
        if completed.returncode:
            raise EvidenceError(
                "{} failed: {}".format(" ".join(command), completed.stderr.strip()))
        return set(completed.stdout.split())

    def _wait_live_graph(self, launch, launch_log, fault=False):
        deadline = time.monotonic() + self.startup_timeout
        last_error = "ROS graph not yet available"
        action_topics = {
            action + suffix
            for action in self.config.get("required_actions", ())
            for suffix in ("/goal", "/cancel", "/status")
        }
        required_topics = set(self.config["canonical_topics"].values()) | action_topics
        if fault:
            required_topics.update(FAULT_REQUIRED_TOPICS)
        required_nodes = set(self.config.get("required_nodes", ()))
        required_services = set(self.config.get("required_services", ()))
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise EvidenceError(
                    "roslaunch exited before the full graph became live; "
                    "roslaunch log: {}".format(launch_log))
            probe_timeout = min(5.0, max(0.1, deadline - time.monotonic()))
            try:
                nodes = self._names(["rosnode", "list"], probe_timeout)
                topics = self._names(["rostopic", "list"], probe_timeout)
                services = self._names(["rosservice", "list"], probe_timeout)
                missing_nodes = sorted(required_nodes - nodes)
                missing_topics = sorted(required_topics - topics)
                missing_services = sorted(required_services - services)
                if missing_nodes or missing_topics or missing_services:
                    last_error = "missing nodes [{}], topics [{}], services [{}]".format(
                        ", ".join(missing_nodes), ", ".join(missing_topics),
                        ", ".join(missing_services))
                else:
                    clock = subprocess.run(
                        ["rostopic", "echo", "-n", "1", "/clock"],
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                        timeout=probe_timeout)
                    if clock.returncode == 0 and clock.stdout.strip():
                        return
                    last_error = "no live /clock sample: " + clock.stderr.strip()
            except (subprocess.TimeoutExpired, EvidenceError) as exc:
                last_error = str(exc)
            time.sleep(0.25)
        raise EvidenceError(
            "full ROS/Gazebo graph readiness timed out: {}; roslaunch log: {}".format(
                last_error, launch_log))

    def run_scenario(self, world, seed, robustness=False, fault=None):
        binding = _runtime_binding(self.config)
        self.preflight()
        launch_spec = self.config["launch"]
        world_file = world["file"]
        prefix = "$(find wheelchair_gazebo)/"
        if world_file.startswith(prefix):
            package_path = subprocess.run(
                ["rospack", "find", "wheelchair_gazebo"], check=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                timeout=10).stdout.strip()
            world_file = str(Path(package_path) / world_file[len(prefix):])
        scenario = fault or ("robustness" if robustness else world["id"])
        fault_id = fault or "normal"
        direction = self.config["scenario_direction"]
        launch_cmd = [
            "roslaunch", launch_spec["package"], launch_spec["file"],
            "gui:=false", "headless:=true", "paused:=false", "auto_start:=true",
            "seed:={}".format(seed), "scenario:={}".format(scenario),
            "fault_id:={}".format(fault_id),
            "scenario_direction:={}".format(direction),
            "world:={}".format(world_file),
        ]
        descriptor, launch_log = tempfile.mkstemp(
            prefix="gazebo-rc-launch-", suffix=".log")
        os.close(descriptor)
        succeeded = False
        with tempfile.TemporaryDirectory(prefix="gazebo-rc-") as temporary:
            evidence_path = Path(temporary) / "evidence.json"
            with open(launch_log, "w", encoding="utf-8") as launch_output:
                launch = subprocess.Popen(
                    launch_cmd, stdout=launch_output, stderr=subprocess.STDOUT,
                    text=True, start_new_session=True)
                try:
                    self._wait_live_graph(launch, launch_log, fault=bool(fault))
                    template = self.config["collector_command"]
                    collector = [
                        str(item).format(
                            output=str(evidence_path), world=world["id"], seed=seed,
                            robustness=str(bool(robustness)).lower(),
                            fault=fault or "none", scenario=scenario,
                            scenario_sha256=binding["scenario_sha256"],
                            a13_sha256=binding["a13_sha256"],
                            route_truth=binding["route_truth"],
                            route_truth_sha256=binding["route_truth_sha256"])
                        for item in template
                    ]
                    completed = subprocess.run(
                        collector, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                        text=True, timeout=self.run_timeout)
                    if completed.returncode:
                        raise EvidenceError(
                            "live metrics collector failed: {}; roslaunch log: {}".format(
                                completed.stderr.strip(), launch_log))
                    if not evidence_path.is_file():
                        raise EvidenceError(
                            "live metrics collector produced no evidence artifact; "
                            "roslaunch log: " + launch_log)
                    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
                    observed = evidence.get("source_topics", [])
                    required_topics = list(REQUIRED_TOPICS)
                    if fault:
                        required_topics.extend(FAULT_REQUIRED_TOPICS)
                    missing = [topic for topic in required_topics if topic not in observed]
                    authority_ok = (
                        evidence.get("simulation_only") is True
                        and evidence.get("hardware_motion_authorized") is False
                        and evidence.get("passenger_operation_authorized") is False
                    )
                    if (evidence.get("live_evidence") is not True or missing
                            or not authority_ok):
                        raise EvidenceError(
                            "collector did not prove live simulation-only canonical "
                            "topics/authority (missing: {}); roslaunch log: {}".format(
                                ", ".join(missing), launch_log))
                    succeeded = True
                    return evidence
                finally:
                    self._stop_process(launch)
                    if succeeded:
                        os.unlink(launch_log)


def execute_suite(config, backend):
    thresholds = config["thresholds"]
    runs = []
    repetitions = int(config.get("deterministic_repetitions", 10))
    for world in config["worlds"]:
        for repetition in range(repetitions):
            raw = backend.run_scenario(world, repetition, robustness=False)
            item = validate_result(raw, thresholds, require_completion=True)
            item.update({"world": world["id"], "seed": repetition, "kind": "deterministic"})
            runs.append(item)
    robustness_world = next(world for world in config["worlds"] if world["id"] == config["robustness_world"])
    for seed in config.get("robustness_seeds", list(range(30))):
        raw = backend.run_scenario(robustness_world, int(seed), robustness=True)
        item = validate_result(raw, thresholds, require_completion=False)
        item.update({"world": robustness_world["id"], "seed": int(seed), "kind": "robustness"})
        runs.append(item)
    robustness = [item for item in runs if item["kind"] == "robustness"]
    acceptable = sum(item["route_outcome"] in ("completed", "safe_abort") and item["passed"] for item in robustness)
    failures = [item for item in runs if not item["passed"]]
    if acceptable < int(config.get("minimum_robustness_acceptable", 29)):
        failures.append({"kind": "aggregate", "failures": ["fewer than 29 robustness runs completed or safely aborted"]})
    return {"passed": not failures, "runs": runs, "summary": {"total": len(runs), "failed": len(failures), "robustness_acceptable": acceptable}, "failures": failures}


def provenance(config_path):
    path = Path(config_path)
    revision = "unknown"
    try:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(path.resolve().parents[2]), stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True, timeout=5).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        pass
    return {"host": socket.gethostname(), "platform": platform.platform(), "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "git_revision": revision, "config_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def make_report(config_path, invocation, status, result=None, error=None):
    return {"artifactType": REPORT_TYPE, "schemaVersion": 1, "claimTag": CLAIM_TAG,
            "hardwareMotionAuthorized": False, "passengerOperationAuthorized": False,
            "invocation": invocation, "provenance": provenance(config_path), "status": status,
            "generatedAtUnixNs": time.time_ns(), "result": result,
            "platformUnavailable": error if status == "PLATFORM_UNAVAILABLE" else None,
            "failure": error if status == "FAIL" else None}


def main(argv=None):
    parser = argparse.ArgumentParser()
    default_config = Path(__file__).resolve().parents[1] / "src/wheelchair_gazebo/config/scenarios.yaml"
    parser.add_argument("--config", default=str(default_config))
    parser.add_argument("--output", required=True)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    invocation = [sys.executable, str(Path(__file__).resolve())] + list(argv if argv is not None else sys.argv[1:])
    try:
        config = load_config(args.config)
        result = execute_suite(config, RosGazeboBackend(config, args.startup_timeout))
        report = make_report(args.config, invocation, "PASS" if result["passed"] else "FAIL", result=result)
        exit_code = 0 if result["passed"] else 1
    except PlatformUnavailable as exc:
        report = make_report(args.config, invocation, "PLATFORM_UNAVAILABLE", error=str(exc))
        exit_code = 2
    except (EvidenceError, subprocess.SubprocessError, OSError, ValueError, json.JSONDecodeError) as exc:
        report = make_report(args.config, invocation, "FAIL", error=str(exc))
        exit_code = 1
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(str(output))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
