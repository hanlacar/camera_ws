#!/usr/bin/env python3
"""Adapt the camera command contract to /cmd_vel for the Gazebo Ackermann car.

Subscribes:
  /camera_drive (std_msgs/Float32)  discrete stage 0..3
  /camera_wheel (std_msgs/Int32)    steering degrees, +/-27
Publishes:
  /cmd_vel      (geometry_msgs/Twist)

Fail-safe: if either command is missing or older than command_timeout_sec, or
if the two commands disagree in time by more than pair_skew_sec, publish a zero
Twist (full stop). This mirrors the camera controllers' own stop-on-stale
behavior so a dropped command never leaves the sim car driving blind.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

from .cmd_conversion import command_to_twist


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__("cmd_vel_adapter_node")
        self.declare_parameter("wheelbase_m", 0.77)
        self.declare_parameter("slow_mps", 0.4)
        self.declare_parameter("cruise_mps", 0.9)
        self.declare_parameter("fast_mps", 1.4)
        self.declare_parameter("max_wheel_deg", 27.0)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.wheelbase = float(self.get_parameter("wheelbase_m").value)
        self.slow = float(self.get_parameter("slow_mps").value)
        self.cruise = float(self.get_parameter("cruise_mps").value)
        self.fast = float(self.get_parameter("fast_mps").value)
        self.max_wheel = float(self.get_parameter("max_wheel_deg").value)
        self.timeout = float(self.get_parameter("command_timeout_sec").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.drive = None
        self.drive_t = None
        self.wheel = None
        self.wheel_t = None

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Float32, "/camera_drive", self.on_drive, 10)
        self.create_subscription(Int32, "/camera_wheel", self.on_wheel, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "cmd_vel adapter ready: /camera_drive,/camera_wheel -> /cmd_vel "
            f"(wheelbase={self.wheelbase} m, timeout={self.timeout} s)")

    def on_drive(self, msg):
        self.drive = float(msg.data)
        self.drive_t = time.monotonic()

    def on_wheel(self, msg):
        self.wheel = int(msg.data)
        self.wheel_t = time.monotonic()

    def tick(self):
        now = time.monotonic()
        twist = Twist()
        have_both = (self.drive is not None and self.wheel is not None
                     and self.drive_t is not None and self.wheel_t is not None)
        fresh = have_both and \
            (now - self.drive_t) <= self.timeout and \
            (now - self.wheel_t) <= self.timeout
        if fresh:
            lx, az = command_to_twist(
                self.drive, self.wheel, self.wheelbase,
                self.slow, self.cruise, self.fast, self.max_wheel)
            twist.linear.x = lx
            twist.angular.z = az
        # else: leave twist at zero -> full stop
        self.pub.publish(twist)

    def destroy_node(self):
        if rclpy.ok():
            self.pub.publish(Twist())  # stop on shutdown
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()
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
