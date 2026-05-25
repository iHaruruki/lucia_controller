from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    
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
    ])