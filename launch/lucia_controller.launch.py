from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # Lucia Controller
        Node(
            package='lucia_controller',
            executable='lucia_controller_node',
            name='lucia_controller_node',
            output='screen',
        ),

        # Velocity smoother
        Node(
            package='lucia_controller',
            executable='lucia_velocity_smoother_node',
            name='lucia_velocity_smoother_node',
            parameters=[{
                'max_linear_vel_x': 0.3,
                'max_linear_vel_y': 0.3,
                'max_angular_vel': 0.8,
                'tau_linear_x': 0.7,
                'tau_linear_y': 0.37,
                'tau_angular': 0.2,
                'update_frequency': 30,
            }],
            remappings=[
                ('/cmd_vel', '/collision_monitor/cmd_vel')
            ],
            output='screen',
        ),
        
        # lucia_trajectory_drawer_node
        Node(
            package='lucia_controller',
            executable='lucia_trajectory_drawer_node',
            name='lucia_trajectory_drawer_node',
            parameters=[
                {
                    'odom_topic': 'odom',
                }
            ],
            output='screen',
        ),

        # lucia_collision_monitor_node
        Node(
            package='lucia_controller',
            executable='lucia_collision_monitor_node',
            name='lucia_collision_monitor_node',
            remappings=[
                ('/cmd_vel', '/twist_mux/cmd_vel')
            ],
            # ros_arguments=['--log-level', 'debug'],
            output='screen',
        ),
    ])