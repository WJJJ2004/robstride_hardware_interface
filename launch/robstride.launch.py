import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    packages_name = 'robstride_hardware_interface'

    config_file_path = os.path.join(
        get_package_share_directory(packages_name),
        'config',
        'motor_setting.yaml'
    )

    hardware_interface_node = Node(
        package=packages_name,
        executable='hardware_interface_node',
        name='hardware_interface_node',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([hardware_interface_node])
