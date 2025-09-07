from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    lucia_controller_node = ComposableNodeContainer(
        name='lucia_controller',
        namespace='',
        package='lucia_controller',
        executable='lucia_controller_node',
        output='screen',
    )

    ld.add_action(lucia_controller_node)

    return ld