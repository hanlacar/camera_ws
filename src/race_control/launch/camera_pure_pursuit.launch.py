import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = get_package_share_directory("camera_bringup")
    imu_share = get_package_share_directory("imu_manager")
    yolo_share = get_package_share_directory("camera_yolo_inference")
    navigation_share = get_package_share_directory("camera_navigation")
    control_share = get_package_share_directory("race_control")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, "launch", "d456_bringup.launch.py")
        )
    )
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, "launch", "yolo_inference.launch.py")
        )
    )
    # UPDATED (D456 ground-plane calibration pass): camera_navigation now ships
    # camera_metric_path_node, which publishes nav_msgs/Path on /camera/path
    # ONLY while CALIBRATION_VALID (real base-mount config + boot-time IMU
    # lock; see camera_bringup/config/camera_mount.yaml). Until an operator
    # sets camera_mount.configured: true with measured values, the node stays
    # in NOT_CONFIGURED/INVALID and this leg fail-safes to "no metric path" --
    # pixel-space /camera/image_path* remains unaffected either way.
    image_path = Node(
        package="camera_navigation",
        executable="camera_image_path_node",
        name="camera_image_path_node",
        output="screen",
        parameters=[os.path.join(navigation_share, "config", "image_path.yaml")],
    )
    planner = Node(
        package="camera_navigation",
        executable="camera_metric_path_node",
        name="camera_metric_path_node",
        output="screen",
        parameters=[os.path.join(camera_share, "config", "camera_mount.yaml")],
    )
    imu_pitch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_share, "launch", "imu_manager.launch.py")
        )
    )
    pure_pursuit = Node(
        package="race_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "pure_pursuit.yaml"),
            {
                "path_input_type": "nav_path",
                "path_topic": "/camera/path",
                "nav_path_valid_topic": "/camera/path_valid",
                "nav_path_confidence_topic": "/camera/path_confidence",
                "nav_path_lateral_to_right_sign": -1.0,
                "commanded_speed_mps": LaunchConfiguration("commanded_speed_mps"),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("commanded_speed_mps",default_value="0.0",description="Pure Pursuit target speed; default is propulsion locked"),
        LogInfo(msg="Camera + IMU + YOLO + metric path + Pure Pursuit; vehicle actuation is not launched"),
        camera,
        imu_pitch,
        yolo,
        image_path,
        planner,
        pure_pursuit,
    ])
