from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # robot_state_publisher ノードの起動
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', FindPackageShare('my_robot_description'), '/urdf/my_robot.urdf'])}]
        ),

        # Nav2 の Controller サーバーの起動
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['config/nav2_params.yaml']
        ),

        # 他の Nav2 コンポーネントのノードもここに追加
    ])
