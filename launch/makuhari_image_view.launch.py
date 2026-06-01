from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='makuhari_image_view',
            arguments=[
                '/lucia_astra_camera/color/image_raw',
                '--ros-args',
                '-p', 'image_transport:=compressed'
            ],
            output='screen'
        )
    ])
