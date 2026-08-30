"""BEV pipeline in sim: sim_base + perception + IMU split + metric + BEV ctrl.

Pipeline:
  Gazebo camera -> YOLO -> image_path_node -> /camera/image_path_typed
                                                     |
                     Gazebo IMU -> imu_splitter ------+--> camera_metric_path_node
                                                          -> /camera/path (metric)
                                                          -> camera_path_controller_node
                                                          -> /camera_drive,/camera_wheel
                                                          -> cmd_vel_adapter -> /cmd_vel

The metric node needs a boot-time stationary IMU attitude lock, so keep the car
still for the first ~1-2 s after launch. Because the sim camera pose is exact
(from the URDF), BEV projection should be accurate here even while the real
camera's pitch is still being tuned. camera_mount.yaml's configured:true and
the sim IMU together drive the calibration gate.
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

    # Split the single sim IMU into the accel/gyro sample topics.
    imu_splitter = Node(
        package="gazebo_sim", executable="imu_splitter_node.py",
        name="imu_splitter_node", output="screen",
        parameters=[{"use_sim_time": True}])

    # Metric (BEV) path node. Uses the SIM mount config (base_link at vehicle
    # center -> position_x_m = -0.075), NOT the real vehicle's camera_mount.yaml.
    metric = Node(
        package="camera_navigation", executable="camera_metric_path_node",
        name="camera_metric_path_node", output="screen",
        parameters=[
            os.path.join(sim_pkg, "config", "camera_mount_sim.yaml"),
            {"use_sim_time": True},
        ])

    bev_controller = Node(
        package="camera_navigation", executable="camera_path_controller_node",
        name="camera_path_controller_node", output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "camera_path_controller.yaml"),
            {"use_sim_time": True},
        ])

    return LaunchDescription([perception, imu_splitter, metric, bev_controller])
