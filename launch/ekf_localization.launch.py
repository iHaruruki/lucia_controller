import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # complementary filter args
    complementary_args = [
        DeclareLaunchArgument("cf_publish_tf", default_value="true"),
        DeclareLaunchArgument("cf_use_mag", default_value="false"),
        DeclareLaunchArgument("cf_bias_alpha", default_value="0.01"),
        DeclareLaunchArgument("cf_gain_acc", default_value="0.01"),
        DeclareLaunchArgument("cf_gain_mag", default_value="0.01"),
    ]

    ekf_yaml_arg = DeclareLaunchArgument(
        "ef_ekf_yaml",
        default_value=PathJoinSubstitution([
            FindPackageShare('lucia_controller'),
            'config',
            'ekf.yaml'
        ]),
        description="Path to EKF parameter file"
    )

    # Optional topic override args
    ekf_topic_args = [
        DeclareLaunchArgument("ef_odom0", default_value="/odom"),
        DeclareLaunchArgument("ef_imu0", default_value="/imu/data"),  # CF出力に合わせる
    ]

    cf_parameters = [{
        "publish_tf": LaunchConfiguration("cf_publish_tf"),
        "use_mag": LaunchConfiguration("cf_use_mag"),
        "bias_alpha": LaunchConfiguration("cf_bias_alpha"),
        "gain_acc": LaunchConfiguration("cf_gain_acc"),
        "gain_mag": LaunchConfiguration("cf_gain_mag"),
    }]

    complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        parameters=cf_parameters,
        remappings=[
            ('/imu/data_raw', '/camera/gyro_accel/sample'),
        ],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[
            LaunchConfiguration("ef_ekf_yaml"),
            {
                "odom0": LaunchConfiguration("ef_odom0"),
                "imu0": LaunchConfiguration("ef_imu0"),
            }
        ],
        output='screen',
    )

    return LaunchDescription(
        complementary_args + [ekf_yaml_arg] + ekf_topic_args + [
            complementary_filter_node,
            ekf_node,
        ]
    )