#!/usr/bin/env python3
"""Convert mission speed stages and steering degrees into Gazebo Ackermann Twist."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class GazeboCommandAdapter(Node):
    def __init__(self):
        super().__init__("gazebo_command_adapter")
        defaults = {"stage_1_mps": 0.35, "stage_2_mps": 0.60, "stage_3_mps": 0.90,
                    "maximum_steering_deg": 30.0, "command_timeout_sec": 0.5,
                    "publish_rate_hz": 20.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.stage = 0
        self.steering_deg = 0.0
        self.drive_time = self.steer_time = -1e9
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Int32, "/cmd_driving", self.on_drive, 10)
        self.create_subscription(Float32, "/cmd_steer", self.on_steer, 10)
        self.create_timer(1.0 / float(self.p("publish_rate_hz")), self.tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_drive(self, msg):
        self.stage = max(-3, min(3, int(msg.data)))
        self.drive_time = time.monotonic()

    def on_steer(self, msg):
        if math.isfinite(msg.data):
            self.steering_deg = float(msg.data)
            self.steer_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = now-self.drive_time <= self.p("command_timeout_sec") and now-self.steer_time <= self.p("command_timeout_sec")
        command = Twist()
        if fresh:
            sign = -1.0 if self.stage < 0 else 1.0
            command.linear.x = sign * float(self.p(f"stage_{abs(self.stage)}_mps")) if self.stage else 0.0
            limit = float(self.p("maximum_steering_deg"))
            command.angular.z = math.radians(max(-limit, min(limit, self.steering_deg)))
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args); node = GazeboCommandAdapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
