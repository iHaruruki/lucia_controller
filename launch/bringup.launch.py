import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージのshareディレクトリの取得
    lucia_description_pkg = get_package_share_directory('lucia_description')
    lucia_controller_pkg = get_package_share_directory('lucia_controller')
    urg_node2_pkg = get_package_share_directory('urg_node2')
    dual_laser_merger_pkg = get_package_share_directory('dual_laser_merger')

    # launchファイルへのフルパス
    description = os.path.join(lucia_description_pkg, 'launch', 'robot.launch.py')
    controller = os.path.join(lucia_controller_pkg, 'launch', 'controller.launch.py')
    urg_node2 = os.path.join(urg_node2_pkg, 'launch', 'urg_node2_2lidar.launch.py')
    dual_laser_merger = os.path.join(dual_laser_merger_pkg, 'launch', 'urg_node2_merger.launch.py')

    # IncludeLaunchDescription
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description)
    )

    ld = LaunchDescription()
    ld.add_action(description)
    ld.add_action(controller)
    ld.add_action(urg_node2)
    ld.add_action(dual_laser_merger)

    return ld
