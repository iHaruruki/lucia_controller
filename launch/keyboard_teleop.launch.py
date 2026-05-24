from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # teleop_twist_keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='lucia_teleop_twist_keyboard',
            prefix='gnome-terminal --',
            output='screen',
            parameters=[{
                'stamped': False,
                'speed': 0.10,
                'turn': 0.30,
            }],
            remappings=[
                ('/cmd_vel', '/key_vel')
            ],
        ),
    ])