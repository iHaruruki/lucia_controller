import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    pkg_path = get_package_share_directory("lucia_controller")
    nav2_bringup_path = get_package_share_directory("nav2_bringup")

    rviz_config_file = PathJoinSubstitution(
        [pkg_path, "rviz", "nav.rviz"]
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, "launch"), "/robot.launch.py"]),
        launch_arguments={"config_file": rviz_config_file}.items()
    )

    nav = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel",
                     dst="/robot_base_controller/cmd_vel_unstamped"),
            SetRemap(src="/odom",
                     dst="/robot_base_controller/odom"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_path, "launch",
                                 "bringup_launch.py"),
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "map": os.path.join(pkg_path, "map", "map.yaml"),
                    "params_file": os.path.join(pkg_path, "config", "nav2_params.yaml"),
                }.items(),
            )
        ]
    )

    return LaunchDescription([robot, nav])
