from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 引数の宣言
    declare_video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )

    declare_image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='image_height'
    )

    declare_image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='image_width'
    )
    
    declare_framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )
    
    declare_exposure_arg = DeclareLaunchArgument(
        'exposure',
        default_value='100',
        description='Camera exposure'
    )
    
    declare_brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='-1',
        description='Camera brightness'
    )
    
    declare_white_balance_arg = DeclareLaunchArgument(
        'white_balance',
        default_value='4000',
        description='Camera white balance'
    )
    
    # LaunchConfiguration で引数を参照
    video_device = LaunchConfiguration('video_device')
    framerate = LaunchConfiguration('framerate')
    exposure = LaunchConfiguration('exposure')
    brightness = LaunchConfiguration('brightness')
    white_balance = LaunchConfiguration('white_balance')
    image_height = LaunchConfiguration('image_height')
    image_width = LaunchConfiguration('image_width')
    
    return LaunchDescription([
        declare_video_device_arg,
        declare_framerate_arg,
        declare_exposure_arg,
        declare_brightness_arg,
        declare_white_balance_arg,
        declare_image_height_arg,
        declare_image_width_arg,
        
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='brio_100_node',
            parameters=[{
                'auto_white_balance': True,
                'autoexposure': True,
                'autofocus': True,
                'av_device_format': 'YUV422P',
                'brightness': brightness,
                'camera_info_url': 'file://' + get_package_share_directory('lucia_controller') + '/config/brio_100.yaml',
                'camera_name': 'brio_100',
                'contrast': -1,
                'exposure': exposure,
                'focus': -1,
                'frame_id': 'brio_100_link',
                'framerate': framerate,
                'gain': -1,
                'io_method': 'mmap',
                'pixel_format': 'yuyv2rgb',
                'saturation': -1,
                'sharpness': -1,
                'use_sim_time': False,
                'video_device': video_device,
                'white_balance': white_balance,
                'image_raw.ffmpeg.encoder.ffmpeg.encoder': 'h264',
                'image_height': image_height,
                'image_width': image_width,
            }],
            remappings=[
                ('/camera_info', '/brio_100/camera_info'),
                ('/image_raw', '/brio_100/image_raw'),
                ('/image_raw/compressed', '/brio_100/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/brio_100/image_raw/compressedDepth'),
                ('/image_raw/theora', '/brio_100/image_raw/theora'),
                ('/image_raw/ffmpeg', '/brio_100/image_raw/ffmpeg'),
            ],
        ),
    ])