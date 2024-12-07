import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("lucia_controller")
    rplidar_path = get_package_share_directory("urg_node2")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [pkg_path, "rviz", "robot.rviz"]),
            description="RViz2 config file.",
        )
    ]

    gui = LaunchConfiguration("gui")
    rviz_config_file = LaunchConfiguration("config_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_path, "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            pkg_path,
            "config",
            "robot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="lucia_controller",
        executable="lucia_controller_node",
        #parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    #imu_broadcaster_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["imu_sensor_broadcaster",
    #               "--controller-manager", "/controller_manager"],
    #)

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_base_controller",
                   "--controller-manager", "/controller_manager"],
    )

    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(rplidar_path, "launch"),
                                       "/urg_node2.launch.py"]),
        launch_arguments={
            "frame_id": "laser_link"
        }.items()
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        laser_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        #imu_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)