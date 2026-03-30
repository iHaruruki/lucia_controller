import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lucia_controller_pkg = get_package_share_directory('lucia_controller')
    twist_mux_config = os.path.join(lucia_controller_pkg, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    ld = LaunchDescription()
    ld.add_action(twist_mux_node)

    return ld
