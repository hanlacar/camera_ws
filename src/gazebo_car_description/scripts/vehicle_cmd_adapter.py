#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class VehicleCmdAdapter(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_cmd_adapter')

        self.declare_parameter('speed_topic', '/cmd/speed_mps')
        self.declare_parameter('steer_topic', '/cmd/steer_deg')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode', 'ackermann_kinematic')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('steering_limit_rad', 0.60)
        self.declare_parameter('speed_limit_mps', 2.0)
        self.declare_parameter('low_speed_epsilon_mps', 0.05)
        self.declare_parameter('input_timeout_sec', 0.75)
        self.declare_parameter('stale_speed_mps', 0.0)
        self.declare_parameter('stale_steer_deg', 0)
        self.declare_parameter('log_saturation', False)
        self.declare_parameter('debug_log', False)

        self.speed_topic = str(self.get_parameter('speed_topic').value)
        self.steer_topic = str(self.get_parameter('steer_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.mode = str(self.get_parameter('mode').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.wheel_base = max(1.0e-3, float(self.get_parameter('wheel_base').value))
        self.steering_limit_rad = abs(float(self.get_parameter('steering_limit_rad').value))
        self.speed_limit_mps = abs(float(self.get_parameter('speed_limit_mps').value))
        self.low_speed_epsilon_mps = max(
            1.0e-3, abs(float(self.get_parameter('low_speed_epsilon_mps').value))
        )
        self.input_timeout_sec = max(
            0.0, float(self.get_parameter('input_timeout_sec').value)
        )
        self.stale_speed_mps = float(self.get_parameter('stale_speed_mps').value)
        self.stale_steer_deg = int(self.get_parameter('stale_steer_deg').value)
        self.log_saturation = bool(self.get_parameter('log_saturation').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.speed_mps = 0.0
        self.steer_deg = 0
        self.last_debug_signature = None
        self.last_stale_state = None
        self.last_speed_stamp = self.get_clock().now()
        self.last_steer_stamp = self.get_clock().now()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Float32, self.speed_topic, self._speed_callback, 10)
        self.create_subscription(Int32, self.steer_topic, self._steer_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_cmd_vel)

        self.get_logger().info(
            'vehicle_cmd_adapter started | '
            f'speed_topic={self.speed_topic} '
            f'steer_topic={self.steer_topic} '
            f'cmd_vel_topic={self.cmd_vel_topic} '
            f'mode={self.mode}'
        )
        self.get_logger().info(
            'adapter expects integer steering degrees and converts them to radians before '
            'Ackermann cmd_vel generation.'
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _speed_callback(self, msg: Float32) -> None:
        self.speed_mps = float(msg.data)
        self.last_speed_stamp = self.get_clock().now()

    def _steer_callback(self, msg: Int32) -> None:
        self.steer_deg = int(msg.data)
        self.last_steer_stamp = self.get_clock().now()

    def _stale_input_active(self) -> bool:
        if self.input_timeout_sec <= 0.0:
            return False
        now = self.get_clock().now()
        speed_age = (now - self.last_speed_stamp).nanoseconds / 1.0e9
        steer_age = (now - self.last_steer_stamp).nanoseconds / 1.0e9
        return speed_age > self.input_timeout_sec or steer_age > self.input_timeout_sec

    def _compute_angular_z(self, speed_mps: float, steer_rad: float) -> float:
        if self.mode == 'direct_angular':
            return steer_rad

        if abs(speed_mps) < self.low_speed_epsilon_mps:
            return 0.0

        omega = speed_mps / self.wheel_base * math.tan(steer_rad)
        omega_limit = self.speed_limit_mps / self.wheel_base * math.tan(self.steering_limit_rad)
        return self._clamp(omega, abs(omega_limit))

    def _publish_cmd_vel(self) -> None:
        max_steer_deg = math.degrees(self.steering_limit_rad)
        stale_active = self._stale_input_active()
        if stale_active:
            speed_mps = self._clamp(self.stale_speed_mps, self.speed_limit_mps)
            steer_deg = int(round(self._clamp(float(self.stale_steer_deg), max_steer_deg)))
        else:
            speed_mps = self._clamp(self.speed_mps, self.speed_limit_mps)
            steer_deg = int(round(self._clamp(float(self.steer_deg), max_steer_deg)))

        steer_rad = math.radians(steer_deg)

        if self.log_saturation and not stale_active:
            if abs(self.speed_mps) > self.speed_limit_mps or abs(self.steer_deg) > max_steer_deg:
                self.get_logger().warn(
                    'adapter input saturated | '
                    f'speed_mps={self.speed_mps:.3f} steer_deg={self.steer_deg}'
                )

        twist = Twist()
        twist.linear.x = speed_mps
        twist.angular.z = self._compute_angular_z(speed_mps, steer_rad)
        self.cmd_vel_pub.publish(twist)

        if stale_active != self.last_stale_state:
            self.last_stale_state = stale_active
            if stale_active:
                self.get_logger().warn(
                    'adapter input timeout | '
                    f'using stale fallback speed={speed_mps:.3f} steer_deg={steer_deg}'
                )
            else:
                self.get_logger().info('adapter input stream recovered')

        if self.debug_log:
            signature = (
                round(speed_mps, 4),
                steer_deg,
                round(steer_rad, 4),
                round(twist.angular.z, 4),
                stale_active,
            )
            if signature != self.last_debug_signature:
                self.last_debug_signature = signature
                self.get_logger().info(
                    'cmd_vel updated | '
                    f'linear.x={twist.linear.x:.3f} '
                    f'steer_deg={steer_deg} '
                    f'steer_rad={steer_rad:.3f} '
                    f'angular.z={twist.angular.z:.3f} '
                    f'mode={self.mode} '
                    f'stale={stale_active}'
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleCmdAdapter()
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
