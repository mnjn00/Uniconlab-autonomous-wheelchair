import json
import math
from dataclasses import dataclass

# noqa: SIZE_OK — proposal schema and its actuator rollout are one wire contract

SCHEMA, ACTUATOR_SCHEMA = "wheelchair.trajectory_proposal/v1", "wheelchair.actuator_state/v1"
MAX_ACCEL_MPS2, MAX_DECEL_MPS2 = 0.18, 0.6
MAX_JERK_MPS3, MAX_SPEED_MPS = 0.8, 0.8
YAW_SLEW_RPS2 = 1.5


class ProposalValidationError(ValueError):
    pass


def _finite(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProposalValidationError("%s must be numeric" % field)
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ProposalValidationError("%s must be finite" % field)
    return parsed


def _identity(value, field):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ProposalValidationError("%s must be a non-negative integer" % field)
    return int(value)


def normalize_side(value):
    side = "NONE" if value is None else str(value).upper()
    if side not in ("NONE", "LEFT", "RIGHT"):
        raise ProposalValidationError("committed_side is invalid")
    return side


def normalize_minimum_turn(value):
    turn = _finite(value, "minimum_turn_rps")
    if turn < 0.0:
        raise ProposalValidationError(
            "minimum_turn_rps must be non-negative")
    return turn


def yaw_matches_side(side, yaw_rate_rps):
    return (side == "NONE" or side == "LEFT" and yaw_rate_rps > 0.0
            or side == "RIGHT" and yaw_rate_rps < 0.0)


@dataclass(frozen=True, slots=True)
class ActuatorState:
    speed_mps: float
    yaw_rate_rps: float
    acceleration_mps2: float
    control_step_s: float = 0.1

    def __post_init__(self):
        speed = _finite(self.speed_mps, "speed_mps")
        if speed < 0.0:
            raise ProposalValidationError("speed_mps must be non-negative")
        object.__setattr__(self, "speed_mps", speed)
        object.__setattr__(
            self, "yaw_rate_rps", _finite(self.yaw_rate_rps, "yaw_rate_rps"))
        object.__setattr__(
            self, "acceleration_mps2",
            _finite(self.acceleration_mps2, "acceleration_mps2"))
        control_step = _finite(self.control_step_s, "control_step_s")
        if control_step <= 0.0:
            raise ProposalValidationError("control_step_s must be positive")
        object.__setattr__(self, "control_step_s", control_step)

    def to_json(self):
        return json.dumps({
            "schema": ACTUATOR_SCHEMA,
            "speed_mps": self.speed_mps,
            "yaw_rate_rps": self.yaw_rate_rps,
            "acceleration_mps2": self.acceleration_mps2,
            "control_step_s": self.control_step_s,
        }, allow_nan=False, separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, text):
        try:
            payload = json.loads(text)
        except (json.JSONDecodeError, TypeError) as error:
            raise ProposalValidationError("actuator state is not valid JSON") from error
        expected = {"schema", "speed_mps", "yaw_rate_rps",
                    "acceleration_mps2", "control_step_s"}
        if (not isinstance(payload, dict) or set(payload) != expected
                or payload.get("schema") != ACTUATOR_SCHEMA):
            raise ProposalValidationError("actuator state fields are invalid")
        return cls(payload["speed_mps"], payload["yaw_rate_rps"],
                   payload["acceleration_mps2"], payload["control_step_s"])


@dataclass(frozen=True, slots=True)
class ProposalMetadata:
    proposal_seq: int
    stamp_s: float
    permit_track_id: int
    committed_side: str = "NONE"

    def __post_init__(self):
        object.__setattr__(
            self, "proposal_seq", _identity(self.proposal_seq, "proposal_seq"))
        stamp = _finite(self.stamp_s, "stamp_s")
        if stamp < 0.0:
            raise ProposalValidationError("stamp_s must be non-negative")
        object.__setattr__(self, "stamp_s", stamp)
        object.__setattr__(
            self, "permit_track_id",
            _identity(self.permit_track_id, "permit_track_id"))
        object.__setattr__(
            self, "committed_side", normalize_side(self.committed_side))


@dataclass(frozen=True, slots=True)
class RolloutSpec:
    pose: tuple
    target_speed_mps: float
    target_yaw_rate_rps: float
    actuator_state: ActuatorState
    distance_m: float

    def __post_init__(self):
        if not isinstance(self.pose, (tuple, list)) or len(self.pose) < 3:
            raise ProposalValidationError("pose must contain x, y, and yaw")
        pose = tuple(_finite(value, "pose") for value in self.pose[:3])
        object.__setattr__(self, "pose", pose)
        target_speed = _finite(self.target_speed_mps, "target_speed_mps")
        if target_speed <= 0.0:
            raise ProposalValidationError("target_speed_mps must be positive")
        object.__setattr__(self, "target_speed_mps", target_speed)
        object.__setattr__(
            self, "target_yaw_rate_rps",
            _finite(self.target_yaw_rate_rps, "target_yaw_rate_rps"))
        distance = _finite(self.distance_m, "distance_m")
        if distance <= 0.0:
            raise ProposalValidationError("distance_m must be positive")
        object.__setattr__(self, "distance_m", distance)


@dataclass(frozen=True, slots=True)
class TrajectoryProposal:
    proposal_seq: int
    stamp_s: float
    permit_track_id: int
    committed_side: str
    frame_id: str
    horizon_s: float
    actuator_state: ActuatorState
    target_speed_mps: float
    target_yaw_rate_rps: float
    poses: tuple
    speeds_mps: tuple
    yaw_rates_rps: tuple

    def __post_init__(self):
        metadata = ProposalMetadata(
            self.proposal_seq, self.stamp_s, self.permit_track_id,
            self.committed_side)
        object.__setattr__(self, "proposal_seq", metadata.proposal_seq)
        object.__setattr__(self, "stamp_s", metadata.stamp_s)
        object.__setattr__(self, "permit_track_id", metadata.permit_track_id)
        object.__setattr__(self, "committed_side", metadata.committed_side)
        if self.frame_id != "body":
            raise ProposalValidationError("frame_id must be body")
        horizon = _finite(self.horizon_s, "horizon_s")
        if horizon <= 0.0:
            raise ProposalValidationError("horizon_s must be positive")
        object.__setattr__(self, "horizon_s", horizon)
        if not isinstance(self.actuator_state, ActuatorState):
            raise ProposalValidationError("actuator_state is invalid")
        target_speed = _finite(self.target_speed_mps, "target_speed_mps")
        if target_speed <= 0.0:
            raise ProposalValidationError("target_speed_mps must be positive")
        object.__setattr__(self, "target_speed_mps", target_speed)
        object.__setattr__(self, "target_yaw_rate_rps", _finite(
            self.target_yaw_rate_rps, "target_yaw_rate_rps"))
        if not isinstance(self.poses, (tuple, list)):
            raise ProposalValidationError("poses must be a series")
        if not isinstance(self.speeds_mps, (tuple, list)):
            raise ProposalValidationError("speeds_mps must be a series")
        if not isinstance(self.yaw_rates_rps, (tuple, list)):
            raise ProposalValidationError("yaw_rates_rps must be a series")
        if any(not isinstance(pose, (tuple, list)) or len(pose) != 3
               for pose in self.poses):
            raise ProposalValidationError("poses must contain xyz triples")
        poses = tuple(tuple(_finite(value, "poses") for value in pose)
                      for pose in self.poses)
        speeds = tuple(_finite(value, "speeds_mps") for value in self.speeds_mps)
        yaw_rates = tuple(
            _finite(value, "yaw_rates_rps") for value in self.yaw_rates_rps)
        if not poses:
            raise ProposalValidationError("poses must contain xyz triples")
        if len(poses) != len(speeds) or len(poses) != len(yaw_rates):
            raise ProposalValidationError("proposal series lengths must match")
        expected_horizon = len(poses) * self.actuator_state.control_step_s
        if not math.isclose(horizon, expected_horizon, rel_tol=0.0, abs_tol=1e-9):
            raise ProposalValidationError("horizon_s does not match series")
        if any(speed < 0.0 for speed in speeds):
            raise ProposalValidationError("speeds_mps must be non-negative")
        object.__setattr__(self, "poses", poses)
        object.__setattr__(self, "speeds_mps", speeds)
        object.__setattr__(self, "yaw_rates_rps", yaw_rates)

    @property
    def first_applied_speed_mps(self):
        return self.speeds_mps[0]

    @property
    def first_applied_yaw_rate_rps(self):
        return self.yaw_rates_rps[0]

    def to_json(self):
        return json.dumps({
            "schema": SCHEMA,
            "proposal_seq": self.proposal_seq,
            "stamp_s": self.stamp_s,
            "permit_track_id": self.permit_track_id,
            "committed_side": self.committed_side,
            "frame_id": self.frame_id,
            "horizon_s": self.horizon_s,
            "actuator_state": json.loads(self.actuator_state.to_json()),
            "target_speed_mps": self.target_speed_mps,
            "target_yaw_rate_rps": self.target_yaw_rate_rps,
            "first_applied_speed_mps": self.first_applied_speed_mps,
            "first_applied_yaw_rate_rps": self.first_applied_yaw_rate_rps,
            "poses": self.poses,
            "speeds_mps": self.speeds_mps,
            "yaw_rates_rps": self.yaw_rates_rps,
        }, allow_nan=False, separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, text):
        try:
            payload = json.loads(text)
        except (json.JSONDecodeError, TypeError) as error:
            raise ProposalValidationError("proposal is not valid JSON") from error
        if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
            raise ProposalValidationError("proposal schema is invalid")
        expected = {
            "schema", "proposal_seq", "stamp_s", "permit_track_id",
            "committed_side", "frame_id", "horizon_s", "actuator_state",
            "target_speed_mps", "target_yaw_rate_rps",
            "first_applied_speed_mps", "first_applied_yaw_rate_rps",
            "poses", "speeds_mps", "yaw_rates_rps",
        }
        if set(payload) != expected:
            raise ProposalValidationError("proposal fields are invalid")
        if not isinstance(payload["actuator_state"], dict):
            raise ProposalValidationError("actuator_state is invalid")
        try:
            actuator_json = json.dumps(
                payload["actuator_state"], allow_nan=False)
        except (TypeError, ValueError) as error:
            raise ProposalValidationError("actuator_state is invalid") from error
        actuator_state = ActuatorState.from_json(actuator_json)
        proposal = cls(
            proposal_seq=payload["proposal_seq"],
            stamp_s=payload["stamp_s"],
            permit_track_id=payload["permit_track_id"],
            committed_side=payload["committed_side"],
            frame_id=payload["frame_id"],
            horizon_s=payload["horizon_s"],
            actuator_state=actuator_state,
            target_speed_mps=payload["target_speed_mps"],
            target_yaw_rate_rps=payload["target_yaw_rate_rps"],
            poses=payload["poses"],
            speeds_mps=payload["speeds_mps"],
            yaw_rates_rps=payload["yaw_rates_rps"],
        )
        first_speed = _finite(
            payload["first_applied_speed_mps"],
            "first_applied_speed_mps")
        first_yaw = _finite(
            payload["first_applied_yaw_rate_rps"],
            "first_applied_yaw_rate_rps")
        if (first_speed != proposal.first_applied_speed_mps
                or first_yaw != proposal.first_applied_yaw_rate_rps):
            raise ProposalValidationError(
                "first applied command does not match proposal series")
        return proposal


def rollout_actuation(spec):
    dt = spec.actuator_state.control_step_s
    x, y, yaw = spec.pose
    speed = spec.actuator_state.speed_mps
    yaw_rate = spec.actuator_state.yaw_rate_rps
    acceleration = spec.actuator_state.acceleration_mps2
    poses, speeds, yaw_rates = [], [], []
    travelled = 0.0
    while travelled < spec.distance_m:
        desired = min(max((spec.target_speed_mps - speed) / dt,
                          -MAX_DECEL_MPS2), MAX_ACCEL_MPS2)
        jerk_room = MAX_JERK_MPS3 * dt
        acceleration = min(max(desired, acceleration - jerk_room),
                           acceleration + jerk_room)
        speed = min(max(speed + acceleration * dt, 0.0), MAX_SPEED_MPS)
        yaw_delta = min(max(spec.target_yaw_rate_rps - yaw_rate,
                            -YAW_SLEW_RPS2 * dt),
                        YAW_SLEW_RPS2 * dt)
        yaw_rate += yaw_delta
        applied_yaw_rate = yaw_rate if speed > 0.02 else 0.0
        yaw += applied_yaw_rate * dt
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        travelled += speed * dt
        poses.append((x, y, yaw))
        speeds.append(speed)
        yaw_rates.append(applied_yaw_rate)
        if len(poses) >= 10000 and travelled < spec.distance_m:
            raise ProposalValidationError("actuator rollout did not reach span")
    return tuple(poses), tuple(speeds), tuple(yaw_rates)
