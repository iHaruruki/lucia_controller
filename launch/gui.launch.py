from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lucia_controller',
            executable='mode_display_gui_node',
            name='mode_display_gui_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
