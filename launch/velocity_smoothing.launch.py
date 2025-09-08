from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel_raw')  # Nav2 の出力を raw 化
        ],
        parameters=[]  # 既存の Nav2 パラメータ
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=['config/velocity_smoother.yaml']  # raw_cmd_vel_topic: /cmd_vel_raw
    )

    robot_driver = Node(
        package='lucia_controller',
        executable='lucia_controller_node',
        name='lucia_controller',
        output='screen'
    )

    return LaunchDescription([
        controller_server,
        velocity_smoother,
        robot_driver
    ])