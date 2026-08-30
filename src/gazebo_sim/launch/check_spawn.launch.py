"""Minimal verification launch: sim_base + rqt_image_view on the camera.

Use this FIRST to confirm the basics in order, before adding YOLO/control:
  1. Gazebo opens with the S-curve world.
  2. The turtle_car spawns at x=3.25 (a few seconds after Gazebo starts).
  3. The camera publishes /camera/image_raw, shown live in rqt_image_view.

If the car spawns and you see road in the rqt window, the sim harness is good
and you can move on to pixel_control.launch.py / bev_control.launch.py.

Checks while this runs:
  ros2 topic list | grep camera          # /camera/image_raw, /camera/camera_info
  ros2 topic hz /camera/image_raw        # should be ~30 Hz
  ros2 topic echo /odom --once           # confirms the vehicle exists in sim
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_sim")

    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg, "launch", "sim_base.launch.py")))

    # rqt_image_view pointed at the camera. Delayed so the topic exists first.
    rqt = TimerAction(period=8.0, actions=[
        Node(package="rqt_image_view", executable="rqt_image_view",
             name="rqt_image_view", output="screen",
             arguments=["/camera/image_raw"])])

    return LaunchDescription([base, rqt])
