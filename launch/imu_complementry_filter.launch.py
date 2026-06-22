import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define the complementary filter node
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter_node',
        output='screen',
        parameters=[{
            'use_mag': False,           # Set to True if you have a magnetometer (9-DoF)
            # 'do_bias_estimation': True, # Over time estimates gyro bias 
            # 'do_adaptive_gain': True,   # Adapts gain based on acceleration magnitude
            # 'gain_acc': 0.01,           # Accelerometer gain (trust factor)
            # 'gain_mag': 0.01,           # Magnetometer gain (if use_mag is True)
            'publish_tf': False,         # Broadcast transform from fixed frame to IMU frame
            # 'fixed_frame': 'camera_gyro_optical_frame',      # The parent frame for orientation
            'bias_alpha' : 0.01,
        }],
        remappings=[
            # Map 'imu/data_raw' to your actual raw hardware IMU topic
            ('imu/data_raw', '/camera/gyro_accel/sample')
        ]
    )

    # imu_odom_node = Node(
    #     package='lucia_controller',
    #     executable='complementary_filter_node',
    #     name='imu_complementary_filter_node',
    #     output='screen',
    #     parameters=[{
    #         'use_mag': False,           # Set to True if you have a magnetometer (9-DoF)
    #         # 'do_bias_estimation': True, # Over time estimates gyro bias 
    #         # 'do_adaptive_gain': True,   # Adapts gain based on acceleration magnitude
    #         # 'gain_acc': 0.01,           # Accelerometer gain (trust factor)
    #         # 'gain_mag': 0.01,           # Magnetometer gain (if use_mag is True)
    #         'publish_tf': True,         # Broadcast transform from fixed frame to IMU frame
    #         'fixed_frame': 'camera_link',      # The parent frame for orientation
    #         'bias_alpha' : 0.01,
    #     }],
    #     remappings=[
    #         # Map 'imu/data_raw' to your actual raw hardware IMU topic
    #         ('imu/data_raw', '/camera/gyro_accel/sample')
    #     ]
    # )

    return LaunchDescription([
        imu_filter_node,
        # imu_odom_node
    ])