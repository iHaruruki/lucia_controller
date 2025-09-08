from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_controller')  # 例: パッケージ名
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
            executable='lucia_controller_node',
            name='lucia_controller',
            parameters=[
                {"publish_tf": False},
                {"raw_odom_topic": "wheel_odom"},
                {"raw_odom_frame": "wheel_odom"},
                {"base_link_frame": "base_link"},
                {"use_smoothing": True},
                {"use_ramp": True}
            ],
            output='screen'
        ),
        # base_link と base_footprint を分けたい場合:
        # Node(
        #   package="tf2_ros",
        #   executable="static_transform_publisher",
        #   arguments=["0","0","0","0","0","0","base_link","base_footprint"]
        # )
    ])