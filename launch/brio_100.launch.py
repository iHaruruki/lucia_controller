from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Twist Mux node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='brio_100_node',
            parameters=[{
                'auto_white_balance': True,
                'autoexposure': True,
                'autofocus': True, # default: False
                'av_device_format': 'YUV422P',
                'brightness': -1,
                # 'camera_info_url':
                'camera_name': 'brio_100',
                'contrast': -1,
                'exposure': 100,
                'focus': -1,
                'frame_id': 'brio_100_link',
                'framerate': 30.0,
                'gain': -1,
                'io_method': 'mmap',
                'pixel_format': 'yuyv2rgb',
                'saturation': -1,
                'sharpness': -1,
                'use_sim_time': False,
                'video_device': '/dev/video0',
                'white_balance': 4000,
            }],
            remappings=[
                ('/camera_info', '/brio_100/camera_info'),
                ('/image_raw', '/brio_100/image_raw'),
                ('/image_raw/compressed', '/brio_100/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/brio_100/image_raw/compressedDepth'),
                ('/image_raw/theora', '/brio_100/image_raw/theora'),
            ],
        ),
    ])

# param list

# /usb_cam:
#   auto_white_balance
#   autoexposure
#   autofocus
#   av_device_format
#   brightness
#   camera_info_url
#   camera_name
#   contrast
#   exposure
#   focus
#   frame_id
#   framerate
#   gain
#   image_height
#   image_raw.enable_pub_plugins
#   image_raw.format
#   image_raw.jpeg_quality
#   image_raw.png_level
#   image_raw.tiff.res_unit
#   image_raw.tiff.xdpi
#   image_raw.tiff.ydpi
#   image_width
#   io_method
#   pixel_format
#   qos_overrides./parameter_events.publisher.depth
#   qos_overrides./parameter_events.publisher.durability
#   qos_overrides./parameter_events.publisher.history
#   qos_overrides./parameter_events.publisher.reliability
#   saturation
#   sharpness
#   use_sim_time
#   video_device
#   white_balance
