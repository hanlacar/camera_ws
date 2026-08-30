#!/usr/bin/env python3
"""Synthetic input publisher for offline BEV (metric) stack testing.

This node exists ONLY to let the BEV camera_ws stack run against a recorded
mp4 file, on a machine with no D456 hardware attached. It is a standalone
tool outside the colcon workspace's package tree -- it does not modify, wrap,
or import any existing camera_ws node, and camera_metric_path_node /
camera_image_path_node / camera_yolo_inference_node are run completely
unmodified.

Why three synthetic topics are needed
--------------------------------------
The BEV pipeline is:
    /camera/image_raw (+/camera/camera_info)
      -> camera_yolo_inference_node -> /perception/semantic_path_frame
      -> camera_image_path_node     -> /camera/image_path_typed
      -> camera_metric_path_node (+ IMU accel/gyro) -> /camera/path

camera_metric_path_node additionally requires a one-time, boot-window
stationary IMU attitude lock (see ground_plane_calibration.py's
check_stationary/gravity_roll_pitch_deg) before it will ever leave
NOT_CONFIGURED; there is no code path that bypasses this. The recorded mp4
carries none of CameraInfo, accel, or gyro, and this workspace does not ship
a per-device calibration file for the test rig either. So all three are
synthesized here:

  1. /camera/camera_info -- D456 640x480 NOMINAL (datasheet-typical)
     intrinsics, NOT this specific unit's calibration. Any BEV meter-space
     path this produces is only an approximation; it is good enough to
     exercise the pipeline end-to-end, not to validate metric accuracy.
  2. /camera/camera/accel/sample, /camera/camera/gyro/sample -- a
     synthetic, perfectly level, stationary IMU. This means the boot-time
     attitude lock always resolves to "no correction needed", so
     camera_metric_path_node applies camera_mount.yaml's
     reference_pitch_deg/reference_roll_deg AS-IS with zero delta. This
     test therefore validates "does this video's road get detected and
     turned into a BEV metric path", NOT "is the camera_mount.yaml pitch
     actually correct for how the camera was held/mounted while this video
     was recorded". Angle/metric accuracy needs a real, physically level,
     stationary IMU capture (or a measured ground-truth marker), not this.
  3. /camera/image_raw -- the mp4's frames, resized to 640x480 bgr8 if
     necessary.

The accel vector: why (0.0, -9.81, 0.0)
----------------------------------------
camera_metric_path_node consumes accel in the camera OPTICAL frame (ROS
convention: x=right, y=down, z=forward) and converts it with
    OPTICAL_TO_MECHANICAL = [[0,0,1],[-1,0,0],[0,-1,0]]
into the mechanical/base-aligned frame (x=forward, y=left, z=up) before
computing roll/pitch from gravity
    (ground_plane_calibration.gravity_roll_pitch_deg / check_stationary).
A stationary sensor measures the REACTION to gravity, i.e. +9.81 m/s^2 along
the mechanical "up" axis. In the optical frame, "up" is -y (optical y points
down), so the stationary reading is optical (0, -9.81, 0). Converting:
    OPTICAL_TO_MECHANICAL @ [0, -9.81, 0] = [0, 0, 9.81]
which gives roll=atan2(-y,z)=atan2(0,9.81)=0 deg, pitch=atan2(x,hypot(y,z))
=atan2(0,9.81)=0 deg, and norm=9.81 m/s^2 -- squarely inside the default
stationary gate (init_gravity_norm_min/max_mps2 = 8.5..11.0). This has been
verified numerically (see the task report), not just derived on paper.

Do NOT publish optical (0, 0, 9.81) instead: converting that vector gives
mechanical (9.81, 0, 0), i.e. measured_pitch = atan2(9.81, 0) = 90 degrees
-- the stationary gate would likely still pass (norm is still 9.81) but the
computed attitude would be wildly wrong, corrupting every downstream BEV
projection. This has already been checked and rejected; the (0,-9.81,0)
vector above is the only one that reproduces a level, zero-correction boot.
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

TARGET_WIDTH = 640
TARGET_HEIGHT = 480
# D456 640x480 NOMINAL (datasheet-typical) color intrinsics -- NOT a
# per-unit factory/manual calibration. See module docstring.
NOMINAL_FX = 386.0
NOMINAL_FY = 386.0
NOMINAL_CX = 320.0
NOMINAL_CY = 240.0
# Stationary-IMU reaction-to-gravity vector in the camera OPTICAL frame.
# See module docstring "The accel vector: why (0.0, -9.81, 0.0)".
STATIONARY_ACCEL_OPTICAL = (0.0, -9.7727, -0.8550)
ZERO_GYRO_OPTICAL = (0.0, 0.0, 0.0)


class BevTestPublisher(Node):
    def __init__(self):
        super().__init__("bev_test_pub")
        self.declare_parameter("video_path", "")
        self.declare_parameter("loop", True)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        video_path = str(self.get_parameter("video_path").value)
        if not video_path:
            raise ValueError("video_path parameter is required")
        self.loop = bool(self.get_parameter("loop").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        if not (rate_hz == rate_hz) or rate_hz <= 0.0:  # NaN-safe check
            raise ValueError("rate_hz must be finite and positive")
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.capture = cv2.VideoCapture(video_path)
        if not self.capture.isOpened():
            raise RuntimeError(f"could not open video: {video_path}")
        self.get_logger().info(
            f"bev_test_pub: reading {video_path} at {rate_hz:.1f} Hz "
            f"(loop={self.loop}); publishing synthetic CameraInfo + "
            "stationary-level accel/gyro alongside each frame")

        self.image_pub = self.create_publisher(
            Image, "/camera/image_raw", qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.accel_pub = self.create_publisher(
            Imu, "/camera/camera/accel/sample", qos_profile_sensor_data)
        self.gyro_pub = self.create_publisher(
            Imu, "/camera/camera/gyro/sample", qos_profile_sensor_data)

        self._camera_info_msg = self._build_camera_info()
        self._timer = self.create_timer(1.0/rate_hz, self._tick)

    def _build_camera_info(self):
        msg = CameraInfo()
        msg.width = TARGET_WIDTH
        msg.height = TARGET_HEIGHT
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [NOMINAL_FX, 0.0, NOMINAL_CX,
                0.0, NOMINAL_FY, NOMINAL_CY,
                0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        msg.p = [NOMINAL_FX, 0.0, NOMINAL_CX, 0.0,
                0.0, NOMINAL_FY, NOMINAL_CY, 0.0,
                0.0, 0.0, 1.0, 0.0]
        return msg

    def _next_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            if not self.loop:
                return None
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
            if not ok:
                return None
        if frame.shape[1] != TARGET_WIDTH or frame.shape[0] != TARGET_HEIGHT:
            frame = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT),
                               interpolation=cv2.INTER_LINEAR)
        return frame

    def _tick(self):
        frame = self._next_frame()
        if frame is None:
            self.get_logger().info("bev_test_pub: end of video, loop=false -- stopping timer")
            self._timer.cancel()
            return
        stamp = self.get_clock().now().to_msg()

        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        image_msg.height = TARGET_HEIGHT
        image_msg.width = TARGET_WIDTH
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = 0
        image_msg.step = TARGET_WIDTH*3
        image_msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        self.image_pub.publish(image_msg)

        info_msg = self._camera_info_msg
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        self.camera_info_pub.publish(info_msg)

        accel_msg = Imu()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.frame_id
        (accel_msg.linear_acceleration.x, accel_msg.linear_acceleration.y,
         accel_msg.linear_acceleration.z) = STATIONARY_ACCEL_OPTICAL
        self.accel_pub.publish(accel_msg)

        gyro_msg = Imu()
        gyro_msg.header.stamp = stamp
        gyro_msg.header.frame_id = self.frame_id
        (gyro_msg.angular_velocity.x, gyro_msg.angular_velocity.y,
         gyro_msg.angular_velocity.z) = ZERO_GYRO_OPTICAL
        self.gyro_pub.publish(gyro_msg)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BevTestPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
