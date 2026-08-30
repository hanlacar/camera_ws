"""ROS-independent safety state machine for camera Pure Pursuit control."""

from dataclasses import dataclass
import math

from .pure_pursuit import dynamic_lookahead, select_lookahead_point


@dataclass(frozen=True)
class FollowerConfig:
    commanded_speed: float = 2.0
    wheelbase_m: float = 0.73
    minimum_lookahead_m: float = 3.0
    lookahead_gain_s: float = 0.0
    maximum_lookahead_m: float = 3.0
    maximum_steering_deg: float = 27.0
    steering_sign: float = 1.0
    lateral_to_right_sign: float = -1.0
    minimum_confidence: float = 0.45
    path_timeout_sec: float = 0.15
    source_stamp_timeout_sec: float = 0.15
    controller_timeout_sec: float = 0.10
    steering_rate_limit_deg_s: float = 100.0
    saturation_timeout_sec: float = 0.50


@dataclass(frozen=True)
class MetricHealth:
    stamp_ns: int
    path_valid: bool
    confidence: float
    calibration_valid: bool


@dataclass(frozen=True)
class PathSample:
    stamp_ns: int
    points: tuple
    received_at: float
    finite: bool


@dataclass(frozen=True)
class ControlCommand:
    drive: float
    wheel: int
    wheel_float_deg: float
    raw_steering_deg: float
    reason: str
    safe: bool
    lookahead_m: float
    target_point: object


class StabilizedPathFollower:
    """Latest-only controller with exact-stamp health and fail-safe output."""

    def __init__(self, config=FollowerConfig()):
        self.config = config
        self.path = None
        self.health = None
        self.last_seen_path_stamp = None
        self.last_control_time = None
        self.previous_steering = 0.0
        self.saturation_started_at = None
        self.duplicate_count = 0
        self.stale_count = 0

    def ingest_path(self, stamp_ns, points, received_at):
        stamp_ns = int(stamp_ns)
        if self.last_seen_path_stamp is not None:
            if stamp_ns == self.last_seen_path_stamp:
                self.duplicate_count += 1
                return False
            if stamp_ns < self.last_seen_path_stamp:
                self.stale_count += 1
                return False
        converted = []
        finite = True
        for point in points:
            if len(point) < 2:
                finite = False
                continue
            forward = float(point[0])
            lateral_right = self.config.lateral_to_right_sign * float(point[1])
            if not (math.isfinite(forward) and math.isfinite(lateral_right)):
                finite = False
            converted.append((forward, lateral_right))
        self.last_seen_path_stamp = stamp_ns
        self.path = PathSample(stamp_ns, tuple(converted), float(received_at), finite)
        return True

    def ingest_health(self, health):
        if not isinstance(health, MetricHealth):
            raise TypeError("health must be MetricHealth")
        if self.health is None or health.stamp_ns >= self.health.stamp_ns:
            self.health = health

    @staticmethod
    def _raw_steering(point, wheelbase_m):
        if point is None:
            return float("nan")
        forward, lateral = point
        distance_sq = forward * forward + lateral * lateral
        if not math.isfinite(distance_sq) or distance_sq <= 1.0e-9:
            return float("nan")
        curvature = 2.0 * lateral / distance_sq
        return math.degrees(math.atan(wheelbase_m * curvature))

    def _slew(self, requested, dt):
        maximum_delta = self.config.steering_rate_limit_deg_s * max(0.0, dt)
        delta = max(-maximum_delta, min(maximum_delta, requested - self.previous_steering))
        return self.previous_steering + delta

    def step(self, now, ros_now_ns, measured_speed=None):
        now = float(now)
        control_gap = (
            None if self.last_control_time is None else now - self.last_control_time)
        self.last_control_time = now
        dt = control_gap if control_gap is not None else 0.0
        reason = "ok"
        target = None
        raw_steering = 0.0

        if control_gap is not None and control_gap > self.config.controller_timeout_sec:
            reason = "controller_timeout"
        elif self.health is None or not self.health.calibration_valid:
            reason = "calibration_invalid"
        elif self.path is None:
            reason = "path_missing"
        elif self.health.stamp_ns != self.path.stamp_ns:
            reason = "path_health_stamp_mismatch"
        elif not self.health.path_valid:
            reason = "path_invalid"
        elif not math.isfinite(self.health.confidence):
            reason = "nonfinite_health"
        elif self.health.confidence < self.config.minimum_confidence:
            reason = "path_low_confidence"
        elif now - self.path.received_at > self.config.path_timeout_sec:
            reason = "path_stale"
        elif (self.path.stamp_ns > 0 and
              (int(ros_now_ns) - self.path.stamp_ns) * 1.0e-9 >
              self.config.source_stamp_timeout_sec):
            reason = "source_stamp_stale"
        elif not self.path.finite:
            reason = "nonfinite_path"
        elif len(self.path.points) < 2:
            reason = "path_too_short"
        else:
            reference_speed = (
                float(measured_speed) if measured_speed is not None and
                math.isfinite(float(measured_speed)) else self.config.commanded_speed)
            lookahead = dynamic_lookahead(
                reference_speed, self.config.minimum_lookahead_m,
                self.config.lookahead_gain_s, self.config.maximum_lookahead_m)
            target = select_lookahead_point(self.path.points, lookahead)
            raw_steering = self._raw_steering(target, self.config.wheelbase_m)
            raw_steering *= self.config.steering_sign
            if not math.isfinite(raw_steering):
                reason = "nonfinite_control"

        lookahead = dynamic_lookahead(
            self.config.commanded_speed, self.config.minimum_lookahead_m,
            self.config.lookahead_gain_s, self.config.maximum_lookahead_m)
        saturated = reason == "ok" and abs(raw_steering) >= self.config.maximum_steering_deg
        if saturated:
            if self.saturation_started_at is None:
                self.saturation_started_at = now
            elif now - self.saturation_started_at >= self.config.saturation_timeout_sec:
                reason = "steering_saturation_timeout"
        else:
            self.saturation_started_at = None

        safe = reason == "ok"
        requested = max(
            -self.config.maximum_steering_deg,
            min(self.config.maximum_steering_deg, raw_steering if safe else 0.0))
        steering = self._slew(requested, dt)
        if not math.isfinite(steering):
            steering = 0.0
            safe = False
            reason = "nonfinite_control"
        steering = max(
            -self.config.maximum_steering_deg,
            min(self.config.maximum_steering_deg, steering))
        self.previous_steering = steering
        wheel = int(round(steering))
        drive = self.config.commanded_speed if safe else 0.0
        if not math.isfinite(drive):
            drive, wheel, steering, safe, reason = 0.0, 0, 0.0, False, "nonfinite_control"
            self.previous_steering = 0.0
        return ControlCommand(
            drive, wheel, steering, raw_steering, reason, safe,
            lookahead, target)
