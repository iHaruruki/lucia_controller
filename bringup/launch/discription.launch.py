from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # URDFファイルのパスを取得
    urdf_file = os.path.join(
        get_package_share_directory('lucia_controller'),
        'urdf',
        'lucia.urdf'
    )

    # robot_state_publisher ノードの設定
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
