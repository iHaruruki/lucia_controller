import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    EmitEvent,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    # =====================================================================
    # DECLARE ALL LAUNCH ARGUMENTS
    # =====================================================================
    
    # Robot description arguments
    ld.add_action(DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Use Joint State Publisher GUI'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    ))
    
    default_model_path = PathJoinSubstitution([
        FindPackageShare('lucia_description'),
        'urdf',
        'lucia_spina_unitree.urdf',
    ])
    ld.add_action(DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='URDF/Xacro file path'
    ))

    # LiDAR arguments
    ld.add_action(DeclareLaunchArgument(
        'auto_start', default_value='true',
        description='Auto-start LiDAR nodes to Active state'
    ))
    ld.add_action(DeclareLaunchArgument(
        'node_name_1st', default_value='urg_node2_1st',
        description='First LiDAR node name'
    ))
    ld.add_action(DeclareLaunchArgument(
        'node_name_2nd', default_value='urg_node2_2nd',
        description='Second LiDAR node name'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_topic_name_1st', default_value='scan_1st',
        description='First LiDAR scan topic'
    ))
    ld.add_action(DeclareLaunchArgument(
        'scan_topic_name_2nd', default_value='scan_2nd',
        description='Second LiDAR scan topic'
    ))

    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    auto_start = LaunchConfiguration('auto_start')
    node_name_1st = LaunchConfiguration('node_name_1st')
    node_name_2nd = LaunchConfiguration('node_name_2nd')
    scan_topic_name_1st = LaunchConfiguration('scan_topic_name_1st')
    scan_topic_name_2nd = LaunchConfiguration('scan_topic_name_2nd')

    # =====================================================================
    # SECTION 1: ROBOT STATE PUBLISHER & JOINT STATE PUBLISHER
    # =====================================================================

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model]),
        value_type=str
    )

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }]
    ))

    ld.add_action(Node(
        condition=UnlessCondition(use_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    ))

    ld.add_action(Node(
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    ))

    # =====================================================================
    # SECTION 2: LUCIA CONTROLLER NODES
    # =====================================================================

    ld.add_action(Node(
        package='lucia_controller',
        executable='lucia_controller_node',
        name='lucia_controller_node',
        output='screen',
    ))

    ld.add_action(Node(
        package='lucia_controller',
        executable='lucia_trajectory_drawer_node',
        name='lucia_trajectory_drawer_node',
        parameters=[{
            'odom_topic': 'odom',
        }],
        output='screen',
    ))

    # =====================================================================
    # SECTION 3: LIDAR (urg_node2) WITH LIFECYCLE MANAGEMENT
    # =====================================================================

    # Load parameter files for both LiDARs
    config_file_path_1st = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether.yaml'
    )
    config_file_path_2nd = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether_2nd.yaml'
    )

    with open(config_file_path_1st, 'r') as file:
        config_params_1st = yaml.safe_load(file)['urg_node2']['ros__parameters']

    with open(config_file_path_2nd, 'r') as file:
        config_params_2nd = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # First LiDAR - Lifecycle Node
    lifecycle_node_1st = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=node_name_1st,
        remappings=[('scan', scan_topic_name_1st)],
        parameters=[config_params_1st],
        namespace='laser1',
        output='screen',
    )
    ld.add_action(lifecycle_node_1st)

    # Second LiDAR - Lifecycle Node
    lifecycle_node_2nd = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=node_name_2nd,
        remappings=[('scan', scan_topic_name_2nd)],
        parameters=[config_params_2nd],
        namespace='laser2',
        output='screen',
    )
    ld.add_action(lifecycle_node_2nd)

    # Event handlers for first LiDAR lifecycle transitions
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_1st,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_1st,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_1st),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    ))

    # Event handlers for second LiDAR lifecycle transitions
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node_2nd,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node_2nd,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node_2nd),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    ))

    # =====================================================================
    # SECTION 4: DUAL LASER MERGER (COMPOSABLE NODE CONTAINER)
    # =====================================================================

    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    ld.add_action(ComposableNodeContainer(
        name='urg_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/laser1/first'},
                    {'laser_2_topic': '/laser2/first'},
                    {'merged_scan_topic': 'scan'},
                    {'target_frame': 'lsc_mount'},
                    {'laser_1_x_offset': -0.05},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': 0.05},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    {'tolerance': 0.01},
                    {'queue_size': 5},
                    {'angle_increment': 0.001},
                    {'scan_time': 0.067},
                    {'range_min': 0.01},
                    {'range_max': 30.0},
                    {'min_height': -1.0},
                    {'max_height': 1.0},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': True},
                    {'allowed_radius': 0.45},
                    {'enable_shadow_filter': True},
                    {'enable_average_filter': True},
                ],
            )
        ],
        output='screen',
    ))

    return ld