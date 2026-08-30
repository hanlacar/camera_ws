"""Base simulation: Gazebo (gz-sim8) + turtle_car with camera + bridges.

Brings up everything common to both control pipelines:
  - the S-curve world in gz-sim
  - the turtle_car spawned at the world's start pose (x=3.25, y=0, yaw=0)
  - ros_gz bridges for cmd_vel, odom, tf, camera_info, imu, lidar
  - ros_gz_image bridge for /camera/image_raw
  - robot_state_publisher for TF from the URDF
  - the cmd_vel adapter (/camera_drive+/camera_wheel -> /cmd_vel)

Spawn robustness: the xacro is expanded to a temporary .urdf on disk at launch
time and the vehicle is spawned from that FILE (-file), not from a topic. This
removes the race where `create` runs before robot_state_publisher has published
/robot_description. The spawn is also delayed a few seconds (TimerAction) so the
Gazebo server is fully up first -- spawning into a not-yet-ready server is the
most common cause of "gazebo window opens but no vehicle appears".
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_sim")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg, "worlds", "s_curve_avoidance.sdf")
    xacro_path = os.path.join(pkg, "urdf", "turtle_car_with_camera.urdf.xacro")

    # Expand xacro -> robot_description string, and also write it to a temp
    # .urdf file so `create` can spawn from the file directly.
    robot_desc = xacro.process_file(xacro_path).toxml()
    urdf_file = os.path.join(tempfile.gettempdir(), "turtle_car_sim.urdf")
    with open(urdf_file, "w") as fh:
        fh.write(robot_desc)

    # Launch gz-sim with the world.
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v3 {world_path}"}.items())

    # Publish robot_description + TF.
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}])

    # Spawn the vehicle FROM FILE at the world's start pose, delayed so the
    # Gazebo server is ready. z clears the 0.18 m wheel radius so wheels settle.
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=[
            "-name", "turtle_car",
            "-file", urdf_file,
            "-x", "3.25", "-y", "0.0", "-z", "0.20", "-Y", "0.0",
        ])
    delayed_spawn = TimerAction(period=4.0, actions=[spawn])

    # Parameter/sensor bridge (cmd_vel, odom, tf, camera_info, imu, lidar).
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="gz_bridge", output="screen",
        parameters=[{
            "config_file": os.path.join(pkg, "config", "gz_bridge.yaml"),
            "use_sim_time": True,
        }])

    # Image bridge (dedicated image_bridge for /camera/image_raw).
    image_bridge = Node(
        package="ros_gz_image", executable="image_bridge",
        name="camera_image_bridge", output="screen",
        arguments=["/camera/image_raw"],
        parameters=[{"use_sim_time": True}])

    # Discrete command -> cmd_vel.
    adapter = Node(
        package="gazebo_sim", executable="cmd_vel_adapter_node.py",
        name="cmd_vel_adapter_node", output="screen",
        parameters=[
            os.path.join(pkg, "config", "cmd_vel_adapter.yaml"),
            {"use_sim_time": True},
        ])

    return LaunchDescription([
        gz, rsp, delayed_spawn, bridge, image_bridge, adapter])
