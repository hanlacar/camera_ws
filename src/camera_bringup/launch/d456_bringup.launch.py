"""Start one RealSense D456 node with production RGB, aligned depth, and motion.

The production parameter file enables 640x480x60 RGB, 200 Hz gyro, and
100 Hz accel. Depth is aligned to color for stop-line ranging; infrared,
pointcloud, and RGBD remain disabled."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _positive_integer(name, value):
    try:
        parsed = int(value)
    except ValueError as error:
        raise RuntimeError(f"{name} must be a positive integer, got {value!r}") from error
    if parsed <= 0:
        raise RuntimeError(f"{name} must be a positive integer, got {value!r}")
    return parsed


def _launch_setup(context):
    serial_no = LaunchConfiguration("serial_no").perform(context).strip()

    width = _positive_integer("color_width", LaunchConfiguration("color_width").perform(context))
    height = _positive_integer("color_height", LaunchConfiguration("color_height").perform(context))
    fps = _positive_integer("color_fps", LaunchConfiguration("color_fps").perform(context))
    profile = f"{width}x{height}x{fps}"

    config_file = Path(get_package_share_directory("camera_bringup")) / "config" / "d456.yaml"
    if not config_file.is_file():
        raise RuntimeError(f"RealSense parameter file not found: {config_file}")

    runtime_overrides = {"rgb_camera.color_profile": profile}
    if serial_no:
        runtime_overrides["serial_no"] = serial_no

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        emulate_tty=True,
        parameters=[
            str(config_file),
            runtime_overrides,
        ],
        remappings=[
            ("/camera/camera/color/image_raw", "/camera/image_raw"),
            ("/camera/camera/color/camera_info", "/camera/camera_info"),
            ("/camera/camera/aligned_depth_to_color/image_raw",
             "/camera/aligned_depth_to_color/image_raw"),
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return [
        LogInfo(msg=(f"D456 serial selection: {serial_no}" if serial_no else
                     "D456 serial selection: automatic discovery")),
        LogInfo(msg=f"RGB profile: {profile}, format: RGB8, auto exposure/gain: enabled"),
        LogInfo(msg=("Production mode: RGB 640x480 plus color-aligned depth, "
                     "gyro=200 Hz and accel=100 Hz; "
                     "infra/pointcloud/rgbd disabled. Outputs include "
                     "/camera/image_raw and /camera/aligned_depth_to_color/image_raw")),
        LogInfo(msg=f"Parameters: {config_file}"),
        camera,
        RegisterEventHandler(
            OnProcessExit(
                target_action=camera,
                on_exit=[Shutdown(reason="RealSense camera node exited")],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description=("Optional D456 serial number; empty selects the "
                         "connected RealSense automatically"),
        ),
        DeclareLaunchArgument(
            "color_width",
            default_value="640",
            description="RGB image width in pixels",
        ),
        DeclareLaunchArgument(
            "color_height",
            default_value="480",
            description="RGB image height in pixels",
        ),
        DeclareLaunchArgument(
            "color_fps",
            default_value="60",
            description="RGB frames per second (D456 supports 60 Hz at 640x480)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
