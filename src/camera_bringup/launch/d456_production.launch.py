"""Production D456 stack with one camera command selector."""
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
    rgb_share = Path(get_package_share_directory("camera_rgb_traffic_light"))
    imu_share = Path(get_package_share_directory("imu_manager"))

    declarations = [
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("launch_path", default_value="true"),
        DeclareLaunchArgument("launch_rqt", default_value="false"),
        DeclareLaunchArgument("visualization_only_path", default_value="false"),
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

    imu = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(imu_share / "launch" / "imu_manager.launch.py")))

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

    metric_path = Node(
        package="camera_navigation", executable="camera_metric_path_node",
        name="camera_metric_path_node", output="screen",
        parameters=[str(camera_share / "config" / "camera_mount.yaml")])
    controller = Node(
        package="camera_navigation", executable="camera_path_controller_node",
        name="camera_path_controller_node", output="screen",
        parameters=[str(nav_share / "config" / "camera_path_controller.yaml")])
    mission_perception = Node(
        package="camera_navigation", executable="camera_mission_perception_node",
        name="camera_mission_perception_node", output="screen",
        parameters=[str(nav_share / "config" / "mission_perception.yaml")])
    rgb = Node(
        package="camera_rgb_traffic_light", executable="rgb_traffic_light_node",
        name="rgb_traffic_light_node", output="screen",
        parameters=[str(rgb_share / "config" / "rgb_traffic_light.yaml")])
    fusion = Node(
        package="camera_navigation", executable="traffic_light_fusion_node",
        name="traffic_light_fusion_node", output="screen",
        parameters=[str(nav_share / "config" / "traffic_light_fusion.yaml")])
    mission_decision = Node(
        package="camera_navigation", executable="camera_mission_decision_node",
        name="camera_mission_decision_node", output="screen",
        parameters=[str(nav_share / "config" / "mission_decision.yaml")])
    selector = Node(
        package="camera_navigation", executable="camera_command_selector_node",
        name="camera_command_selector_node", output="screen",
        parameters=[str(nav_share / "config" / "camera_command_selector.yaml")])
    reference_adapter = Node(
        package="camera_navigation", executable="camera_reference_path_adapter_node",
        name="camera_reference_path_adapter_node", output="screen",
        parameters=[str(nav_share / "config" / "camera_reference_path_adapter.yaml")])

    rqt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(camera_share / "launch" / "two_view_rqt.launch.py")),
        condition=IfCondition(LaunchConfiguration("launch_rqt")))

    middleware = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
    )
    domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "12")
    return LaunchDescription(
        declarations + [middleware, domain, camera, imu, inference, path_node,
                        metric_path, controller, mission_perception, rgb, fusion,
                        mission_decision, selector, reference_adapter, rqt]
    )
