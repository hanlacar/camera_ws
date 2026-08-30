from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_car_description')
    camera_topic = LaunchConfiguration('camera_topic')
    perception_config = PathJoinSubstitution([pkg_share, 'config', 'lane_perception.yaml'])

    lane_perception = Node(
        package='gazebo_car_description',
        executable='lane_perception',
        name='lane_perception',
        output='screen',
        parameters=[
            perception_config,
            {
                'image_topic': camera_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_topic',
                default_value='/image_rect',
                description='ROS image topic for lane perception.',
            ),
            lane_perception,
        ]
    )
