import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_node'),
        'config',
        'control.yaml'
    )

    return LaunchDescription([
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[config],
        )
    ])
