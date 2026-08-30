from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_car_description')
    control_config = PathJoinSubstitution([pkg_share, 'config', 'lane_control.yaml'])
    adapter_config = PathJoinSubstitution([pkg_share, 'config', 'vehicle_cmd_adapter.yaml'])
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    lane_control = Node(
        package='gazebo_car_description',
        executable='lane_control',
        name='lane_control',
        output='screen',
        parameters=[control_config],
    )

    vehicle_cmd_adapter = Node(
        package='gazebo_car_description',
        executable='vehicle_cmd_adapter',
        name='vehicle_cmd_adapter',
        output='screen',
        parameters=[
            adapter_config,
            {
                'cmd_vel_topic': cmd_vel_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='/cmd_vel',
                description='Final autonomy Twist topic bridged to Gazebo.',
            ),
            lane_control,
            vehicle_cmd_adapter,
        ]
    )
