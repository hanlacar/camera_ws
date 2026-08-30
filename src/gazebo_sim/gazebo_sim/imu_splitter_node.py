#!/usr/bin/env python3
"""Republish one Gazebo IMU as the two sample topics the BEV node expects.

camera_metric_path_node (the BEV pipeline) subscribes to:
  /camera/camera/accel/sample (sensor_msgs/Imu)  -- uses linear_acceleration
  /camera/camera/gyro/sample  (sensor_msgs/Imu)  -- uses angular_velocity

A Gazebo IMU sensor publishes a single sensor_msgs/Imu carrying BOTH fields on
one topic (here /camera/imu). This node simply forwards each incoming Imu to
both sample topics unchanged, so the metric node's accel/gyro handlers each get
the data they read. Only needed for the BEV pipeline; the pixel pipeline
ignores the IMU entirely.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSplitter(Node):
    def __init__(self):
        super().__init__("imu_splitter_node")
        self.declare_parameter("input_topic", "/camera/imu")
        self.declare_parameter("accel_topic", "/camera/camera/accel/sample")
        self.declare_parameter("gyro_topic", "/camera/camera/gyro/sample")
        src = str(self.get_parameter("input_topic").value)
        self.accel_pub = self.create_publisher(
            Imu, str(self.get_parameter("accel_topic").value), 10)
        self.gyro_pub = self.create_publisher(
            Imu, str(self.get_parameter("gyro_topic").value), 10)
        self.create_subscription(Imu, src, self.on_imu, 20)
        self.get_logger().info(
            f"imu splitter ready: {src} -> accel+gyro sample topics")

    def on_imu(self, msg):
        # Forward the same message to both topics; each downstream handler
        # reads only the field it needs (accel or gyro).
        self.accel_pub.publish(msg)
        self.gyro_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSplitter()
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
