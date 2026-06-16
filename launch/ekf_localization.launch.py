from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for complementary_filter_node
    complementary_args = [
        DeclareLaunchArgument("cf_publish_tf", default_value="true"),
        DeclareLaunchArgument("cf_use_mag", default_value="false"),
        DeclareLaunchArgument("cf_bias_alpha", default_value="0.01"),
        DeclareLaunchArgument("cf_gain_acc", default_value="0.01"),
        DeclareLaunchArgument("cf_gain_mag", default_value="0.01"),
    ]

    # Declare launch arguments for ekf_node
    ekf_args = [
        DeclareLaunchArgument("ekf_publish_tf", default_value="true"),
        DeclareLaunchArgument("ekf_use_mag", default_value="false"),
        DeclareLaunchArgument("ekf_bias_alpha", default_value="0.01"),
        DeclareLaunchArgument("ekf_gain_acc", default_value="0.01"),
        DeclareLaunchArgument("ekf_gain_mag", default_value="0.01"),
    ]

    # Build parameters for each node
    cf_parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in complementary_args]
    ekf_parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in ekf_args]

    complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        parameters=cf_parameters,
        remappings=[
            ('/imu/data_raw', '/camera/gyro_accel/sample')
        ],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=ekf_parameters,
        remappings=[
            ('/imu/data_raw', '/camera/gyro_accel/sample')
        ],
        output='screen',
    )

    return LaunchDescription(
        complementary_args + ekf_args + [
            complementary_filter_node,
            ekf_node,
        ]
    )