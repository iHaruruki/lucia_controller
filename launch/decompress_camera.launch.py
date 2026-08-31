from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ==========================================
        # 1. FRONT CAMERA
        # ==========================================
        # Color (compressed -> raw)
        Node(
            package='image_transport',
            executable='republish',
            name='front_color_decompressor',
            parameters=[
                {'in_transport': 'compressed'},
                {'out_transport': 'raw'}
            ],
            remappings=[
                ('in/compressed', '/camera/color/image_raw/compressed'),
                ('out', '/camera/color/image_raw')
            ]
        ),
        # Depth (compressedDepth -> raw)
        Node(
            package='image_transport',
            executable='republish',
            name='front_depth_decompressor',
            parameters=[
                {'in_transport': 'compressedDepth'},
                {'out_transport': 'raw'}
            ],
            remappings=[
                ('in/compressedDepth', '/camera/depth/image_raw/compressedDepth'),
                ('out', '/camera/depth/image_raw')
            ]
        ),
    ])