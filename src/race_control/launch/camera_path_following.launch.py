"""Mission-free D456 -> semantic -> metric path -> command production stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory("camera_bringup"))
    control_share = Path(get_package_share_directory("race_control"))

    production = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(camera_share / "launch" / "d456_production.launch.py")),
        launch_arguments={
            "launch_rqt": "false",
            "visualization_only_path": "true",
            "serial_no": LaunchConfiguration("serial_no"),
            "color_fps": "60",
            "rmw_implementation": "rmw_cyclonedds_cpp",
        }.items())
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
    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("commanded_speed", default_value="2.0"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        production,
        metric_path,
        controller,
    ])
