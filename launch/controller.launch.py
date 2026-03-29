from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    
    return LaunchDescription([
        # Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='nav2_velocity_smoother',
            name='lucia_velocity_smoother',
            parameters=[
                {
                    'smoothing_frequency': 100.0,
                    'scale_max_vel': 1.0,
                    'scale_max_angular_vel': 1.0,
                    'max_velocity': [1.0, 1.0],
                    'max_angular_velocity': 2.0,
                    'deadband_velocity': [0.0, 0.0],
                    'deadband_angular_velocity': 0.0,
                    'max_accel': [1.0, 1.0],
                    'max_decel': [1.0, 1.0],
                    'max_angular_accel': 2.0,
                    'max_angular_decel': 2.0,
                }
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
            output='screen',
        ),
        
        # Lucia Controller
        Node(
            package='lucia_controller',
            executable='lucia_controller',
            name='lucia_controller',
            output='screen',
        ),
    ])