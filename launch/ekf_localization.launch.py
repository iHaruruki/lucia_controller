from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # imu_complementary_filter
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_node',
            parameters=[{
                'publish_tf': True,
                'use_mag': False,
                'bias_alpha': 0.01,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
            }],
            remappings=[
                ('/imu/data_raw', '/camera/gyro_accel/sample')
            ],
            output='screen',
        ),

        # robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[{
                'publish_tf': True,
                'use_mag': False,
                'bias_alpha': 0.01,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
            }],
            remappings=[
                ('/imu/data_raw', '/camera/gyro_accel/sample')
            ],
            output='screen',
        ),
    ])