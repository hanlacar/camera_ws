"""Non-BEV pipeline in sim: sim_base + perception + pixel controller.

Pipeline:
  Gazebo camera -> YOLO -> image_path_node -> /camera/image_path_typed
    -> camera_pixel_controller_node -> /camera_drive,/camera_wheel
    -> cmd_vel_adapter (in sim_base) -> /cmd_vel -> Ackermann plugin

No camera extrinsics, no IMU needed. Best pipeline to try first while the
camera pitch is still being adjusted.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("gazebo_sim")
    nav_pkg = get_package_share_directory("camera_navigation")

    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(sim_pkg, "launch", "perception.launch.py")))

    pixel_controller = Node(
        package="camera_navigation", executable="camera_pixel_controller_node",
        name="camera_pixel_controller_node", output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "camera_pixel_controller.yaml"),
            {"use_sim_time": True},
        ])

    return LaunchDescription([perception, pixel_controller])
