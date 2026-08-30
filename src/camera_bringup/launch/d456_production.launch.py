"""One-command production stack: D456 color bring-up + TensorRT YOLO-Seg
inference + image-space path generation.

Decision/control (Pure Pursuit, steering, stop-line, traffic-light, traffic20)
is intentionally NOT included here: that logic is out of scope for this
launch and is not yet finished. Only real, installed executables are
referenced (camera bring-up, camera_yolo_inference_node, and
camera_image_path_node), matching the same /camera/image_raw contract used
by the test_video-based video_perception_test launch.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = Path(get_package_share_directory("camera_bringup"))
    yolo_share = Path(get_package_share_directory("camera_yolo_inference"))
    nav_share = Path(get_package_share_directory("camera_navigation"))

    declarations = [
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("launch_path", default_value="true"),
        DeclareLaunchArgument("launch_rqt", default_value="false"),
        DeclareLaunchArgument("visualization_only_path", default_value="true"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("color_fps", default_value="60"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("require_cuda", default_value="true"),
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value="rmw_cyclonedds_cpp",
            description=(
                "RMW used by every process in this launch. CycloneDDS is the "
                "validated default for sustained 640x480 RGB8 at 60 FPS."
            ),
        ),
        DeclareLaunchArgument("perception_overlay_max_fps", default_value="45.0"),
        DeclareLaunchArgument("path_overlay_max_fps", default_value="45.0"),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "d456_bringup.launch.py")),
        launch_arguments={
            "serial_no": LaunchConfiguration("serial_no"),
            "color_fps": LaunchConfiguration("color_fps"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_camera")))

    inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(yolo_share / "launch" / "yolo_inference.launch.py")),
        launch_arguments={
            "device": LaunchConfiguration("device"),
            "require_cuda": LaunchConfiguration("require_cuda"),
            "enable_depth_assist": "false",
            "perception_overlay_max_fps": LaunchConfiguration("perception_overlay_max_fps"),
        }.items())

    path_config = str(nav_share / "config" / "image_path.yaml")
    path_node = Node(
        package="camera_navigation",
        executable="camera_image_path_node",
        name="camera_image_path_node",
        output="screen",
        parameters=[path_config, {
            "visualization_only": LaunchConfiguration("visualization_only_path"),
            "path_overlay_max_fps": LaunchConfiguration("path_overlay_max_fps"),
        }],
        condition=IfCondition(LaunchConfiguration("launch_path")))

    rqt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(camera_share / "launch" / "two_view_rqt.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_rqt")))

    middleware = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
    )
    return LaunchDescription(
        declarations + [middleware, camera, inference, path_node, rqt]
    )
