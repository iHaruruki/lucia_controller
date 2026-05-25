from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='lucia_remote_joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 0.0,
            }],
            remappings=[
                ('/joy', '/joy_remote')
            ],
        ),
        # joy_to_cmd_vel_node 
        Node(
            package='lucia_controller',
            executable='joy_to_cmdvel_node',
            name='lucia_remote_joy_to_cmdvel_node',
            parameters=[{
                'linear_x_base': 0.1,
                'linear_y_base': 0.1,
                'linear_z_base': 0.1,
                'angular_x_base': 0.3,
                'angular_y_base': 0.3,
                'angular_z_base': 0.3,
            }],
            remappings=[
                ('/cmd_vel', '/joy_vel_remote')
            ],
            output='screen',
        ),
    ])