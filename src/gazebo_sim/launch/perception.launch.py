"""Perception common to both pipelines: YOLO + pixel path planner.

Runs the REAL camera_yolo_inference node and the REAL
camera_navigation/camera_image_path_node against the simulated camera. YOLO
subscribes to /camera/image_raw + /camera/camera_info (both provided by
sim_base). Its /perception/semantic_path_frame feeds the image path planner,
which publishes /camera/image_path_typed (pixel path).

This is where the YOLO-vs-Gazebo recognition risk lives: if YOLO does not
recognize the rendered road/lanes, /perception/semantic_path_frame will be
empty and no path is produced. Watch /camera/perception_overlay_image and
/camera/navigation_mask_available to see whether YOLO is finding the road.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("gazebo_sim")
    yolo_pkg = get_package_share_directory("camera_yolo_inference")
    nav_pkg = get_package_share_directory("camera_navigation")

    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(sim_pkg, "launch", "sim_base.launch.py")))

    # Real YOLO inference node. Uses its own config; sim time on.
    yolo = Node(
        package="camera_yolo_inference",
        executable="camera_yolo_inference_node",
        name="camera_yolo_inference_node", output="screen",
        parameters=[
            os.path.join(yolo_pkg, "config", "yolo_inference.yaml"),
            {"use_sim_time": True},
        ])

    # Real pixel path planner -> /camera/image_path_typed.
    image_path = Node(
        package="camera_navigation", executable="camera_image_path_node",
        name="camera_image_path_node", output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "image_path.yaml"),
            {"use_sim_time": True},
        ])

    return LaunchDescription([base, yolo, image_path])
