from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from yaml import Node
import os


def generate_launch_description():
    pkg = get_package_share_directory('lucia_controller')
    smoother_yaml = os.path.join(pkg, 'config', 'velocity_smoother.yaml')

    # lucia_controller = Node(
    #     package='lucia_controller',
    #     executable='lucia_controller',
    #     name='lucia_controller',
    #     output='screen',
    # )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[smoother_yaml]
    )

    return LaunchDescription([
        velocity_smoother
    ])
