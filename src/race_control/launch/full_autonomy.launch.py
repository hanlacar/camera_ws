"""Post-calibration integrated launch: camera -> perception -> image path
(pixel-space) -> metric path -> controller, with optional two-view RQT.
Single command replaces starting each component launch in its own terminal.

Built directly on d456_bringup.launch.py (not d456_production.launch.py, whose
structure/args differ) so this stays consistent with perception_view.launch.py.
While camera_mount.yaml calibration is INVALID, camera_metric_path_node
publishes no metric path and camera_path_follower fail-safes to
drive=0.0/wheel=0 -- that is expected, unchanged behavior and is left alone
here.

Purely a composition of existing, unmodified component launches/nodes via
IncludeLaunchDescription/Node; no perception/planner/control code or config
here beyond the two runtime arguments (serial_no, color_fps, commanded_speed,
launch_rqt) already exposed by those components.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory("camera_bringup"))
    yolo_share = Path(get_package_share_directory("camera_yolo_inference"))
    nav_share = Path(get_package_share_directory("camera_navigation"))
    control_share = Path(get_package_share_directory("race_control"))

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
    metric_path = Node(
        package="camera_navigation",
        executable="camera_metric_path_node",
        name="camera_metric_path_node",
        output="screen",
        parameters=[str(camera_share / "config" / "camera_mount.yaml")])
    controller = Node(
        package="race_control",
        executable="camera_path_follower",
        name="camera_path_follower",
        output="screen",
        parameters=[
            str(control_share / "config" / "camera_path_follower.yaml"),
            {"commanded_speed": LaunchConfiguration("commanded_speed")},
        ])
    rqt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "two_view_rqt.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_rqt")))

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("color_fps", default_value="60"),
        DeclareLaunchArgument("commanded_speed", default_value="2.0"),
        DeclareLaunchArgument("launch_rqt", default_value="false"),
        camera, inference, image_path, metric_path, controller, rqt,
    ])
