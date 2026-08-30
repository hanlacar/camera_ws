"""Calibration-time integrated launch: camera -> perception -> image path
(pixel-space, control-free) -> optional two-view RQT. Single command replaces
starting each component launch in its own terminal during D456 ground-plane
calibration work. No metric path, no controller -- see full_autonomy.launch.py
for the post-calibration stack.

Purely a composition of existing, unmodified component launches via
IncludeLaunchDescription; no perception/planner/control code or config here.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_share = Path(get_package_share_directory("camera_bringup"))
    yolo_share = Path(get_package_share_directory("camera_yolo_inference"))
    nav_share = Path(get_package_share_directory("camera_navigation"))

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "d456_bringup.launch.py")),
        launch_arguments={
            "serial_no": LaunchConfiguration("serial_no"),
            "color_fps": LaunchConfiguration("color_fps"),
        }.items())
    inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(yolo_share / "launch" / "yolo_inference.launch.py")))
    image_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_share / "launch" / "image_path.launch.py")),
        launch_arguments={"require_control_mode": "false"}.items())
    rqt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "two_view_rqt.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_rqt")))

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("color_fps", default_value="60"),
        DeclareLaunchArgument("launch_rqt", default_value="false"),
        camera, inference, image_path, rqt,
    ])
