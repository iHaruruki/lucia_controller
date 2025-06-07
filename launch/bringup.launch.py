import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # パッケージのshareディレクトリの取得
    lucia_description_pkg = get_package_share_directory('lucia_description')
    lucia_controller_pkg = get_package_share_directory('lucia_controller')
    urg_node2_pkg = get_package_share_directory('urg_node2')
    dual_laser_merger_pkg = get_package_share_directory('dual_laser_merger')

    # launchファイルへのフルパス
    description_path = os.path.join(lucia_description_pkg, 'launch', 'robot.launch.py')
    controller_path = os.path.join(lucia_controller_pkg, 'launch', 'controller.launch.py')
    urg_node2_path = os.path.join(urg_node2_pkg, 'launch', 'urg_node2_2lidar.launch.py')
    dual_laser_merger_path = os.path.join(dual_laser_merger_pkg, 'launch', 'urg_node2_merger.launch.py')

    # IncludeLaunchDescription オブジェクトを作成
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_path),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_path)
    )

    include_urg_node2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_node2_path),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    include_dual_laser_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_laser_merger_path),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    ld = LaunchDescription()
    ld.add_action(include_description)
    ld.add_action(include_controller)
    ld.add_action(include_urg_node2)
    ld.add_action(include_dual_laser_merger)

    return ld
