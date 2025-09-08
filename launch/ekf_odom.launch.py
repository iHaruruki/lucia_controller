from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_controller')
    ekf_path = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_path]
        ),
        Node(
            package='lucia_controller',
            executable='lucia_controller_node',   # 実際のターゲット名に合わせる
            name='lucia_controller',
            parameters=[
                {"publish_tf": False},
                {"raw_odom_topic": "wheel_odom"},
                {"raw_odom_frame": "wheel_odom"},
                {"base_link_frame": "base_link"}
            ],
            output='screen'
        )
    ])