from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    default_model_path = PathJoinSubstitution([
        FindPackageShare('lucia_description'),
        'urdf',
        'lucia_main.urdf',
    ])

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='URDF/Xacro ファイルへのパス'
    )
    declare_use_gui = DeclareLaunchArgument('use_gui', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # xacroを実行し、その出力を「文字列として」robot_descriptionへ
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }]
    )

    jsp = Node(
        condition=UnlessCondition(use_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    jsp_gui = Node(
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        declare_model,
        declare_use_gui,
        declare_use_sim_time,
        jsp,
        jsp_gui,
        rsp
    ])