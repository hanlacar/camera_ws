"""Manual (keyboard) driving with live YOLO perception + path check.

Purpose: you drive the car with the keyboard while watching whether YOLO
recognizes the road/lanes and whether a path is generated. NO controller and
NO cmd_vel adapter run here -- the keyboard owns /cmd_vel directly, so there is
no conflict on that topic.

This launch is SELF-CONTAINED (it does not include sim_base) specifically so
the cmd_vel adapter is absent. Everything else (Gazebo, vehicle, camera,
bridges) is the same as sim_base.

What this launches:
  - Gazebo + turtle_car (camera+IMU) at the start pose
  - ros_gz bridges + image bridge
  - YOLO inference node (real) on the sim camera
  - image_path_node (real), with camera mode activated once at startup so its
    generated path is reported as CAMERA_PATH_OWNER
  - no GUI visualization processes by default; subscribe explicitly when needed

Keyboard teleop is NOT started here (it needs its own terminal stdin).
--------------------------------------------------------------------
STEP 1 (this launch), terminal A:
  # Headless performance benchmark:
  ros2 launch gazebo_sim manual_perception_check.launch.py gui:=false

  # Gazebo GUI + camera navigation / CAMERA_PATH_OWNER validation:
  ros2 launch gazebo_sim manual_perception_check.launch.py \
    enable_navigation:=true gui:=true

STEP 2, terminal B -- keyboard driving (publishes /cmd_vel directly):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  (install once if missing: sudo apt install ros-jazzy-teleop-twist-keyboard)
  IMPORTANT: keep the default (do NOT set stamped:=true). The gz bridge and
  Ackermann plugin expect a plain geometry_msgs/Twist; teleop's default is
  already unstamped, which is correct here.
  Keys: i = forward, , = back, j/l = turn left/right, k = stop. Go slow.

WHAT TO WATCH (open explicitly when needed):
  rqt_image_view /perception/detections_image
  rqt_image_view /camera/path_debug_image
  - /camera/perception_overlay_image : YOLO mask over the camera image.
      Road/lanes lighting up as you drive => YOLO recognizes the sim road.
  - /camera/path_debug_image         : generated path drawn on the image.
      A path line that tracks the road => path generation works.

Numeric confirmation (terminal C, optional):
  ros2 topic echo /camera/navigation_mask_available   # true when road found
  ros2 topic echo /camera/image_path_valid            # true when path made
  ros2 topic echo /camera/image_path_confidence       # 0..1
--------------------------------------------------------------------
"""
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_sim")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    yolo_pkg = Path(get_package_share_directory("camera_yolo_inference"))
    nav_pkg = get_package_share_directory("camera_navigation")

    yolo_model = yolo_pkg / "models" / "hanla_yolo11n_seg_best.engine"
    if not yolo_model.is_file():
        raise RuntimeError(
            "YOLO TensorRT model must exist and be a regular file: "
            f"{yolo_model}"
        )
    yolo_manifest = yolo_pkg / "config" / "class_manifest.yaml"

    world_path = os.path.join(pkg, "worlds", "s_curve_avoidance.sdf")
    xacro_path = os.path.join(pkg, "urdf", "turtle_car_with_camera.urdf.xacro")
    robot_desc = xacro.process_file(xacro_path).toxml()
    urdf_file = os.path.join(tempfile.gettempdir(), "turtle_car_sim.urdf")
    with open(urdf_file, "w") as fh:
        fh.write(robot_desc)

    gz_args = PythonExpression([
        "'-r -v3 ", world_path, "' if '", LaunchConfiguration("gui"),
        "' == 'true' else '-s -r -v3 ", world_path, "'",
    ])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        # gui=false keeps the server-only performance path; gui=true removes
        # only -s, so this single include starts Gazebo server + GUI together.
        launch_arguments={"gz_args": gz_args}.items())

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}])

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=[
            "-name", "turtle_car", "-file", urdf_file,
            "-x", "3.25", "-y", "0.0", "-z", "0.20", "-Y", "0.0",
        ])
    delayed_spawn = TimerAction(period=4.0, actions=[spawn])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="gz_bridge", output="screen",
        parameters=[{
            "config_file": os.path.join(pkg, "config", "gz_bridge.yaml"),
            "use_sim_time": True,
        }])

    image_bridge = Node(
        package="ros_gz_image", executable="image_bridge",
        name="camera_image_bridge", output="screen",
        arguments=["/camera/image_raw"],
        parameters=[{"use_sim_time": True}])

    # NOTE: no cmd_vel_adapter here -- teleop owns /cmd_vel.

    yolo = Node(
        package="camera_yolo_inference",
        executable="camera_yolo_inference_node",
        name="camera_yolo_inference_node", output="screen",
        parameters=[
            str(yolo_pkg / "config" / "yolo_inference.yaml"),
            {
                "segmentation_model_path": str(yolo_model),
                "class_manifest_path": str(yolo_manifest),
                "input_depth": 1,
                # Higher than inference FPS so bursty simulation callbacks are
                # not quantized down; the worker's single pending slot is the
                # actual backlog bound.
                "detections_image_max_fps": 1000.0,
                "use_sim_time": True,
            },
        ],
        additional_env={
            "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
        })

    image_path = Node(
        package="camera_navigation", executable="camera_image_path_node",
        name="camera_image_path_node", output="screen",
        parameters=[
            os.path.join(nav_pkg, "config", "image_path.yaml"),
            {"use_sim_time": True, "visualization_only": True},
        ],
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
        additional_env={
            "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
        })

    activate_camera_owner = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once",
                "/mission/control_mode", "std_msgs/msg/Int8", "{data: 1}",
            ],
            name="activate_camera_path_owner", output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_navigation", default_value="false",
            description="Also run camera_image_path_node; disabled for pure YOLO FPS tests"),
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="Start Gazebo GUI; false keeps the server-only benchmark mode"),
        # This laptop is hybrid-GPU.  Force Ogre2's off-screen camera render
        # onto NVIDIA instead of the default Intel/software EGL path.
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        SetEnvironmentVariable("__VK_LAYER_NV_optimus", "NVIDIA_only"),
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
        gz, rsp, delayed_spawn, bridge, image_bridge,
        LogInfo(msg=f"YOLO TensorRT model: {yolo_model}"),
        yolo, image_path, activate_camera_owner])
