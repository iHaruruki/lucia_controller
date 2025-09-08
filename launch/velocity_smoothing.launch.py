from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_controller')
    smoother_yaml = os.path.join(pkg_share, 'config', 'velocity_smoother.yaml')

    if not os.path.exists(smoother_yaml):
        raise FileNotFoundError(f"Velocity smoother config not found: {smoother_yaml}")

    lucia_controller = Node(
        package='lucia_controller',
        executable='lucia_controller_node',
        name='lucia_controller',
        output='screen',
        # parameters=[... 追加したい場合ここに辞書 or yaml]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[smoother_yaml]
    )

    return LaunchDescription([
        velocity_smoother,
        lucia_controller,
    ])