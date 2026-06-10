import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('control_node')
    config = os.path.join(pkg, 'config', 'control.yaml')

    # caminho do .mot: por padrao procura em share/control_node/data/,
    # mas pode ser sobrescrito na linha de comando:
    #   ros2 launch control_node control.launch.py mot_file:=/caminho/arquivo.mot
    default_mot = os.path.join(pkg, 'data', 'subject01_walk1.mot')

    mot_arg = DeclareLaunchArgument(
        'mot_file',
        default_value=default_mot,
        description='Caminho do arquivo .mot do OpenSim'
    )

    return LaunchDescription([
        mot_arg,

        # No de controle (PID + CAN)
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[config],
        ),

        # No de trajetoria (le o .mot e publica a referencia)
        Node(
            package='control_node',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[{
                'mot_file': LaunchConfiguration('mot_file'),
                'column': 'knee_angle_r',
                'update_rate_ms': 1,
                'filter_hz': 6.0,
                'loop': True,
            }],
        ),
    ])
