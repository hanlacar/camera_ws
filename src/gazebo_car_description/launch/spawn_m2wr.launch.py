from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    headless = LaunchConfiguration('headless')

    pkg_share = FindPackageShare('gazebo_car_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'm2wr_gz.urdf'])
    gui_config = PathJoinSubstitution([pkg_share, 'config', 'gz_gui.config'])

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={
            'gz_args': [world, ' -r --gui-config ', gui_config],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={
            'gz_args': [world, ' -r -s'],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[
            {
                'world': world_name,
                'file': urdf_file,
                'name': robot_name,
                'allow_renaming': False,
                'x': x,
                'y': y,
                'z': z,
                'Y': yaw,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution([pkg_share, 'worlds', 'lane_track.world']),
                description='Absolute path to the provided lane track world file.',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='default',
                description='World name read from lane_track.world.',
            ),
            DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position.'),
            DeclareLaunchArgument('y', default_value='2.78', description='Spawn y position.'),
            DeclareLaunchArgument('z', default_value='0.25', description='Spawn z position.'),
            DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw angle in radians.'),
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
            gz_sim_gui,
            gz_sim_headless,
            TimerAction(period=3.0, actions=[spawn_robot]),
        ]
    )
