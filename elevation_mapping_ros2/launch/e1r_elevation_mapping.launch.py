from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('elevation_mapping_ros2'),
        'config',
        'e1r_elevation_mapping.yaml'
    ])

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to ROS2 parameter file for elevation mapping node',
        ),
        Node(
            package='elevation_mapping_ros2',
            executable='elevation_mapping_node',
            name='elevation_mapping_ros2',
            output='screen',
            parameters=[params_file],
        )
    ])
