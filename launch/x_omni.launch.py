from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ros2_x_omni',
            executable='x_omni_node',
            parameters=[
                    {'serial_port': '/dev/ttyXOmni'},
            ],
        ),
        Node(
            package='ros2_x_omni',
            executable='x_odom_node',
        )
    ])
