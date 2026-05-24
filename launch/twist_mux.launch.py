from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    twist_mux_config_path = PathJoinSubstitution([
        FindPackageShare('lucia_controller'),
        'config',
        'twist_mux.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_twist_mux',
            default_value=twist_mux_config_path,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'),
        
        # Twist Mux node
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='lucia_twist_mux',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_twist_mux'),
            ],
            remappings=[
                ('/cmd_vel_out', '/merged_cmd_vel')
            ],
        ),
    ])