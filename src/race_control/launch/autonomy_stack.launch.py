import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    perception = get_package_share_directory("race_perception")
    control = get_package_share_directory("race_control")
    return LaunchDescription([
        LogInfo(msg="Observation/control validation only; no serial bridge or vehicle actuation is launched"),
        Node(
            package="race_perception", executable="yolo_camera", name="yolo_camera",
            output="screen", parameters=[os.path.join(perception, "config", "yolo_camera.yaml")],
        ),
        Node(
            package="race_control", executable="pure_pursuit", name="pure_pursuit",
            output="screen", parameters=[os.path.join(control, "config", "pure_pursuit.yaml")],
        ),
    ])
