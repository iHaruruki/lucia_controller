from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory
from yaml import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lucia_controller')
    smoother_yaml = os.path.join(pkg_share, 'config', 'velocity_smoother.yaml')

    lucia_controller = LaunchNode(
        package='lucia_controller',
        executable='lucia_controller_node',
        name='lucia_controller',
        output='screen',
    )

    velocity_smoother = LaunchNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[smoother_yaml]
    )

    return LaunchDescription([
        lucia_controller,
        velocity_smoother
    ])
