from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_car_description')

    bridge_config = PathJoinSubstitution(
        [pkg_share, 'config', 'race_B_cmd_vel_bridge.yaml']
    )
    urdf_file = PathJoinSubstitution(
        [pkg_share, 'urdf', 'm2wr_gz.urdf']
    )
    gui_config = PathJoinSubstitution(
        [pkg_share, 'config', 'gz_gui.config']
    )
    obstacle_sdf = PathJoinSubstitution(
        [pkg_share, 'models', 'race_B_obstacle_dynamic', 'model.sdf']
    )

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    headless = LaunchConfiguration('headless')

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                )
            ]
        ),
        launch_arguments={
            'gz_args': [world, ' -r --gui-config ', gui_config],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                )
            ]
        ),
        launch_arguments={
            'gz_args': [world, ' -r -s'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    spawn_robot_once = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_m2wr',
        output='screen',
        arguments=[
            '-world', world_name,
            '-file', urdf_file,
            '-name', 'm2wr',
            '-allow_renaming', 'false',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    teleop_sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='race_B_teleop_sensor_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
            }
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[
            {
                'use_sim_time': False,
            }
        ],
    )

    odom_to_chassis_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_chassis_static_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'chassis',
        ],
    )

    blinking_obstacle = Node(
        package='gazebo_car_description',
        executable='race_b_blinking_obstacle',
        name='race_b_blinking_obstacle',
        output='screen',
        parameters=[
            {
                'world_name': ParameterValue(world_name, value_type=str),
                'model_name': 'race_B_obstacle_dynamic',
                'model_sdf': ParameterValue(obstacle_sdf, value_type=str),
                'x': -6.0,
                'y': 0.0,
                'z': 0.195,
                'yaw': 1.5707963267948966,
                'hover_height': 1.0,
                'hover_duration_sec': 5.0,
                'drop_duration_sec': 1.0,
                'rise_duration_sec': 1.0,
                'ground_duration_sec': 5.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'worlds', 'race_B.world']
                ),
                description='Race B world file.',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='default',
                description='World name read from race_B.world.',
            ),
            DeclareLaunchArgument(
                'x',
                default_value='3.0',
                description='Spawn x position.',
            ),
            DeclareLaunchArgument(
                'y',
                default_value='-3.0',
                description='Spawn y position.',
            ),
            DeclareLaunchArgument(
                'z',
                default_value='0.25',
                description='Spawn z position.',
            ),
            DeclareLaunchArgument(
                'yaw',
                default_value='3.141592653589793',
                description='Spawn yaw angle in radians.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run Gazebo server-only with no GUI.',
            ),

            robot_state_publisher,
            joint_state_publisher,
            odom_to_chassis_tf,

            gz_sim_gui,
            gz_sim_headless,

            TimerAction(period=4.0, actions=[spawn_robot_once]),
            TimerAction(period=5.0, actions=[teleop_sensor_bridge]),
            TimerAction(period=6.0, actions=[blinking_obstacle]),
        ]
    )
