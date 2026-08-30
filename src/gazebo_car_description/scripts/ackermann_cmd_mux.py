#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class AckermannCmdMux(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_cmd_adapter')

        self.declare_parameter('desired_speed_topic', '/desired_speed_mps')
        self.declare_parameter('desired_steer_topic', '/desired_steer_rad')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('steering_to_angular_mode', 'direct_angular')
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('steering_limit_rad', 0.60)
        self.declare_parameter('speed_limit_mps', 2.0)
        self.declare_parameter('low_speed_epsilon_mps', 0.05)
        self.declare_parameter('legacy_input_enabled', True)
        self.declare_parameter('legacy_speed_topic', '/cmd_drive')
        self.declare_parameter('legacy_steer_topic', '/cmd_steer')

        self._desired_speed_topic = str(self.get_parameter('desired_speed_topic').value)
        self._desired_steer_topic = str(self.get_parameter('desired_steer_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._mode = str(self.get_parameter('steering_to_angular_mode').value)
        self._wheel_base = max(1.0e-3, float(self.get_parameter('wheel_base').value))
        self._steering_limit = abs(float(self.get_parameter('steering_limit_rad').value))
        self._speed_limit = abs(float(self.get_parameter('speed_limit_mps').value))
        self._low_speed_epsilon = max(
            1.0e-3, abs(float(self.get_parameter('low_speed_epsilon_mps').value))
        )
        self._legacy_input_enabled = bool(self.get_parameter('legacy_input_enabled').value)
        self._legacy_speed_topic = str(self.get_parameter('legacy_speed_topic').value)
        self._legacy_steer_topic = str(self.get_parameter('legacy_steer_topic').value)

        self._speed_mps = 0.0
        self._steer_rad = 0.0

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Float32, self._desired_speed_topic, self._speed_callback, 10)
        self.create_subscription(Float32, self._desired_steer_topic, self._steer_callback, 10)

        if self._legacy_input_enabled:
            self.create_subscription(Int32, self._legacy_speed_topic, self._legacy_drive_callback, 10)
            self.create_subscription(Float32, self._legacy_steer_topic, self._steer_callback, 10)

        self.create_timer(1.0 / self._publish_rate_hz, self._publish_cmd_vel)
        self.get_logger().info(
            'vehicle_cmd_adapter started | '
            f'speed_topic={self._desired_speed_topic} '
            f'steer_topic={self._desired_steer_topic} '
            f'cmd_vel_topic={self._cmd_vel_topic} '
            f'mode={self._mode} '
            f'legacy_input_enabled={self._legacy_input_enabled}'
        )
        self.get_logger().info(
            'direct_angular mode maps desired_steer_rad directly into Twist.angular.z for Gazebo '
            'AckermannSteering end-to-end testing. '
            'ackermann_kinematic mode instead writes body yaw-rate omega=v/L*tan(delta).'
        )

    def _speed_callback(self, msg: Float32) -> None:
        self._speed_mps = float(msg.data)

    def _legacy_drive_callback(self, msg: Int32) -> None:
        self._speed_mps = float(msg.data) / 3.6

    def _steer_callback(self, msg: Float32) -> None:
        self._steer_rad = float(msg.data)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _compute_angular_command(self, speed_mps: float, steer_rad: float) -> float:
        if self._mode == 'ackermann_kinematic':
            if abs(speed_mps) < self._low_speed_epsilon:
                return 0.0
            omega = speed_mps / self._wheel_base * math.tan(steer_rad)
            omega_limit = self._speed_limit / self._wheel_base * math.tan(self._steering_limit)
            return self._clamp(omega, abs(omega_limit))

        return steer_rad

    def _publish_cmd_vel(self) -> None:
        speed_mps = self._clamp(self._speed_mps, self._speed_limit)
        steer_rad = self._clamp(self._steer_rad, self._steering_limit)

        twist = Twist()
        twist.linear.x = speed_mps
        twist.angular.z = self._compute_angular_command(speed_mps, steer_rad)
        self._cmd_vel_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = AckermannCmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
