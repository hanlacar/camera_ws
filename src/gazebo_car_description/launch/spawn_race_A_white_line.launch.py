from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_car_description')
    bridge_config = PathJoinSubstitution([pkg_share, 'config', 'race_A_cmd_vel_bridge.yaml'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'm2wr_gz.urdf'])
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    headless = LaunchConfiguration('headless')

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'spawn_m2wr.launch.py'])]
        ),
        launch_arguments={
            'world': world,
            'world_name': world_name,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'robot_name': robot_name,
            'headless': headless,
        }.items(),
    )

    teleop_sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='race_A_teleop_sensor_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
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
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            '0',
            '--pitch',
            '0',
            '--yaw',
            '0',
            '--frame-id',
            'odom',
            '--child-frame-id',
            'chassis',
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution([pkg_share, 'worlds', 'race_A.world']),
                description='Race A world file.',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='default',
                description='World name read from race_A.world.',
            ),
            # Lower outer white line, away from the upper red obstacle and the x=0 ramp.
            DeclareLaunchArgument('x', default_value='3.0', description='Spawn x position.'),
            DeclareLaunchArgument('y', default_value='-3.225', description='Spawn y position.'),
            DeclareLaunchArgument('z', default_value='0.25', description='Spawn z position.'),
            # The lower straight is driven along -x in the default clockwise lap.
            DeclareLaunchArgument(
                'yaw',
                default_value='3.141592653589793',
                description='Spawn yaw angle in radians.',
            ),
            DeclareLaunchArgument(
                'robot_name',
                default_value='m2wr',
                description='Spawned entity name.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run Gazebo server-only with no GUI.',
            ),
            robot_state_publisher,
            joint_state_publisher,
            odom_to_chassis_tf,
            spawn_launch,
            teleop_sensor_bridge,
        ]
    )
