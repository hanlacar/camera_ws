#!/usr/bin/env python3
"""50 Hz camera path follower publishing the canonical MCU relay inputs."""

from collections import deque
import json
import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

from .stabilized_path_follower import (
    FollowerConfig, MetricHealth, StabilizedPathFollower)


class RollingRate:
    def __init__(self, window_sec=10.0):
        self.window_sec = float(window_sec)
        self.samples = deque()

    def mark(self, now):
        self.samples.append(float(now))
        self._trim(float(now))

    def _trim(self, now):
        while self.samples and now - self.samples[0] > self.window_sec:
            self.samples.popleft()

    def hz(self, now):
        self._trim(float(now))
        if len(self.samples) < 2:
            return 0.0
        elapsed = self.samples[-1] - self.samples[0]
        return (len(self.samples) - 1) / elapsed if elapsed > 0.0 else 0.0


class CameraPathFollowerNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "camera_path_follower", parameter_overrides=parameter_overrides or [])
        defaults = {
            "path_topic": "/camera/path",
            "metric_status_topic": "/camera/metric_path_status",
            "drive_topic": "/cmd_drive",
            "wheel_topic": "/cmd_wheel",
            "diagnostics_topic": "/control/camera_path_follower_diagnostics",
            "commanded_speed": 2.0,
            "wheelbase_m": 0.73,
            "minimum_lookahead_m": 3.0,
            "lookahead_gain_s": 0.0,
            "maximum_lookahead_m": 3.0,
            "maximum_steering_deg": 27.0,
            "steering_sign": 1.0,
            "lateral_to_right_sign": -1.0,
            "minimum_confidence": 0.45,
            "path_timeout_sec": 0.15,
            "source_stamp_timeout_sec": 0.15,
            "controller_timeout_sec": 0.10,
            "steering_rate_limit_deg_s": 100.0,
            "saturation_timeout_sec": 0.50,
            "control_rate_hz": 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        cfg = FollowerConfig(**{
            name: self.get_parameter(name).value for name in FollowerConfig.__dataclass_fields__
        })
        if not math.isfinite(cfg.commanded_speed) or not 0.0 <= cfg.commanded_speed <= 3.0:
            raise ValueError("commanded_speed must be finite and in [0.0, 3.0]")
        if cfg.maximum_steering_deg <= 0.0 or cfg.maximum_steering_deg > 27.0:
            raise ValueError("maximum_steering_deg must be in (0, 27]")
        rate = float(self.get_parameter("control_rate_hz").value)
        if not math.isfinite(rate) or rate < 45.0:
            raise ValueError("control_rate_hz must be finite and >= 45 Hz")
        self.follower = StabilizedPathFollower(cfg)
        self.input_rate = RollingRate()
        self.output_rate = RollingRate()
        self.path_received = 0
        self.path_accepted = 0
        self.output_count = 0
        self.last_command = None
        self.drive_pub = self.create_publisher(
            Float32, str(self.get_parameter("drive_topic").value), 1)
        self.wheel_pub = self.create_publisher(
            Int32, str(self.get_parameter("wheel_topic").value), 1)
        self.diag_pub = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 10)
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self.on_path, 1)
        self.create_subscription(
            String, str(self.get_parameter("metric_status_topic").value),
            self.on_metric_status, 1)
        self.create_timer(1.0 / rate, self.control)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f"camera path follower ready: {rate:.1f} Hz, drive={cfg.commanded_speed:.2f}, "
            "exact-stamp metric health required; no mission inputs")

    @staticmethod
    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def on_path(self, msg):
        now = time.monotonic()
        self.path_received += 1
        self.input_rate.mark(now)
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.follower.ingest_path(self.stamp_ns(msg.header), points, now):
            self.path_accepted += 1

    def on_metric_status(self, msg):
        try:
            payload = json.loads(msg.data)
            health = MetricHealth(
                stamp_ns=int(payload["stamp_ns"]),
                path_valid=bool(payload["path_valid"]),
                confidence=float(payload["confidence"]),
                calibration_valid=bool(payload["calibration_valid"]),
            )
            self.follower.ingest_health(health)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"rejected metric path status: {exc}")

    def control(self):
        now = time.monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        command = self.follower.step(now, ros_now_ns)
        self.drive_pub.publish(Float32(data=float(command.drive)))
        self.wheel_pub.publish(Int32(data=int(command.wheel)))
        self.output_count += 1
        self.output_rate.mark(now)
        if self.last_command is None or command.reason != self.last_command.reason:
            message = (
                f"control state={command.reason} drive={command.drive:.2f} "
                f"wheel={command.wheel}")
            if command.safe:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
        self.last_command = command

    def publish_diagnostics(self):
        now = time.monotonic()
        command = self.last_command
        payload = {
            "path_received_fps": self.input_rate.hz(now),
            "control_output_fps": self.output_rate.hz(now),
            "path_received": self.path_received,
            "path_accepted": self.path_accepted,
            "control_output_count": self.output_count,
            "duplicate": self.follower.duplicate_count,
            "stale": self.follower.stale_count,
            "backlog": 0,
            "latest_only": True,
            "safe": bool(command.safe) if command else False,
            "reason": command.reason if command else "controller_starting",
            "cmd_drive": command.drive if command else 0.0,
            "cmd_wheel": command.wheel if command else 0,
            "raw_steering_deg": command.raw_steering_deg if command else 0.0,
        }
        self.diag_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def destroy_node(self):
        # Best-effort final stop while the ROS context is still valid.
        if rclpy.ok():
            self.drive_pub.publish(Float32(data=0.0))
            self.wheel_pub.publish(Int32(data=0))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
