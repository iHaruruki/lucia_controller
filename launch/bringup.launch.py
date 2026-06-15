from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    package_share = get_package_share_directory('lucia_controller')
    launch_dir = os.path.join(package_share, 'launch')

    # Robot description
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lucia_state_publisher.launch.py')
        ),
    ))

    # Lucia controller
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lucia_controller.launch.py')
        )
    ))

    # twist_mux
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'twist_mux.launch.py')
        )
    ))
    
    # LiDAR(urg_node2)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'urg_node2_2lidar.launch.py')
        )
    ))
    
    # Dual laser merger
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'urg_node2_merger_multi_echo.launch.py')
        )
    ))
    
    return ld