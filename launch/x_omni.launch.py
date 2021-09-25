import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    serial_path_launch_arg = DeclareLaunchArgument(
        'serial_port', default_value=TextSubstitution(text='/dev/ttyXOmni')
    )

    return LaunchDescription([
        serial_path_launch_arg,
        Node(
            package='ros2_x_omni',
            executable='x_omni_node',
            parameters=[
                    {'serial_port': LaunchConfiguration('serial_port')},
            ],
        ),
        Node(
            package='ros2_x_omni',
            executable='x_odom_node',
        )
    ])
