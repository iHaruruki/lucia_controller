from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Launch arguments declarations
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='lucia_astra_pro',
        description='Unique camera name'
    )
    declare_depth_registration = DeclareLaunchArgument(
        'depth_registration',
        default_value='true',
        description='Hardware depth registration'
    )
    declare_serial_number = DeclareLaunchArgument(
        'serial_number',
        default_value='',
        description='Serial number'
    )
    declare_device_num = DeclareLaunchArgument(
        'device_num',
        default_value='1',
        description='Device number'
    )
    declare_vendor_id = DeclareLaunchArgument(
        'vendor_id',
        default_value='0'
    )
    declare_product_id = DeclareLaunchArgument(
        'product_id',
        default_value='0'
    )
    declare_enable_point_cloud = DeclareLaunchArgument(
        'enable_point_cloud',
        default_value='true'
    )
    declare_enable_colored_point_cloud = DeclareLaunchArgument(
        'enable_colored_point_cloud',
        default_value='true'
    )
    declare_point_cloud_qos = DeclareLaunchArgument(
        'point_cloud_qos',
        default_value='default'
    )
    declare_connection_delay = DeclareLaunchArgument(
        'connection_delay',
        default_value='100'
    )
    declare_color_width = DeclareLaunchArgument(
        'color_width',
        default_value='640'
    )
    declare_color_height = DeclareLaunchArgument(
        'color_height',
        default_value='480'
    )
    declare_color_fps = DeclareLaunchArgument(
        'color_fps',
        default_value='30'
    )
    declare_enable_color = DeclareLaunchArgument(
        'enable_color',
        default_value='true'
    )
    declare_flip_color = DeclareLaunchArgument(
        'flip_color',
        default_value='false'
    )
    declare_color_qos = DeclareLaunchArgument(
        'color_qos',
        default_value='default'
    )
    declare_color_camera_info_qos = DeclareLaunchArgument(
        'color_camera_info_qos',
        default_value='default'
    )
    declare_depth_width = DeclareLaunchArgument(
        'depth_width',
        default_value='640'
    )
    declare_depth_height = DeclareLaunchArgument(
        'depth_height',
        default_value='480'
    )
    declare_depth_fps = DeclareLaunchArgument(
        'depth_fps',
        default_value='30'
    )
    declare_enable_depth = DeclareLaunchArgument(
        'enable_depth',
        default_value='true'
    )
    declare_flip_depth = DeclareLaunchArgument(
        'flip_depth',
        default_value='false'
    )
    declare_depth_qos = DeclareLaunchArgument(
        'depth_qos',
        default_value='default'
    )
    declare_depth_camera_info_qos = DeclareLaunchArgument(
        'depth_camera_info_qos',
        default_value='default'
    )
    declare_ir_width = DeclareLaunchArgument(
        'ir_width',
        default_value='640'
    )
    declare_ir_height = DeclareLaunchArgument(
        'ir_height',
        default_value='480'
    )
    declare_ir_fps = DeclareLaunchArgument(
        'ir_fps',
        default_value='30'
    )
    declare_enable_ir = DeclareLaunchArgument(
        'enable_ir',
        default_value='true'
    )
    declare_flip_ir = DeclareLaunchArgument(
        'flip_ir',
        default_value='false'
    )
    declare_ir_qos = DeclareLaunchArgument(
        'ir_qos',
        default_value='default'
    )
    declare_ir_camera_info_qos = DeclareLaunchArgument(
        'ir_camera_info_qos',
        default_value='default'
    )
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true'
    )
    declare_tf_publish_rate = DeclareLaunchArgument(
        'tf_publish_rate',
        default_value='10.0'
    )
    declare_ir_info_url = DeclareLaunchArgument(
        'ir_info_url',
        default_value=''
    )
    declare_color_info_url = DeclareLaunchArgument(
        'color_info_url',
        default_value=''
    )
    declare_color_roi_x = DeclareLaunchArgument(
        'color_roi_x',
        default_value='-1'
    )
    declare_color_roi_y = DeclareLaunchArgument(
        'color_roi_y',
        default_value='-1'
    )
    declare_color_roi_width = DeclareLaunchArgument(
        'color_roi_width',
        default_value='-1'
    )
    declare_color_roi_height = DeclareLaunchArgument(
        'color_roi_height',
        default_value='-1'
    )
    declare_depth_roi_x = DeclareLaunchArgument(
        'depth_roi_x',
        default_value='-1'
    )
    declare_depth_roi_y = DeclareLaunchArgument(
        'depth_roi_y',
        default_value='-1'
    )
    declare_depth_roi_width = DeclareLaunchArgument(
        'depth_roi_width',
        default_value='-1'
    )
    declare_depth_roi_height = DeclareLaunchArgument(
        'depth_roi_height',
        default_value='-1'
    )
    declare_depth_scale = DeclareLaunchArgument(
        'depth_scale',
        default_value='1'
    )
    declare_color_depth_synchronization = DeclareLaunchArgument(
        'color_depth_synchronization',
        default_value='true'
    )
    declare_use_uvc_camera = DeclareLaunchArgument(
        'use_uvc_camera',
        default_value='true'
    )
    declare_uvc_vendor_id = DeclareLaunchArgument(
        'uvc_vendor_id',
        default_value='0x2bc5'
    )
    declare_uvc_product_id = DeclareLaunchArgument(
        'uvc_product_id',
        default_value='0x0501'
    )
    declare_uvc_retry_count = DeclareLaunchArgument(
        'uvc_retry_count',
        default_value='100'
    )
    declare_uvc_camera_format = DeclareLaunchArgument(
        'uvc_camera_format',
        default_value='mjpeg'
    )
    declare_uvc_flip = DeclareLaunchArgument(
        'uvc_flip',
        default_value='false'
    )
    declare_oni_log_level = DeclareLaunchArgument(
        'oni_log_level',
        default_value='verbose'
    )
    declare_oni_log_to_console = DeclareLaunchArgument(
        'oni_log_to_console',
        default_value='false'
    )
    declare_oni_log_to_file = DeclareLaunchArgument(
        'oni_log_to_file',
        default_value='false'
    )
    declare_enable_d2c_viewer = DeclareLaunchArgument(
        'enable_d2c_viewer',
        default_value='false'
    )
    declare_enable_publish_extrinsic = DeclareLaunchArgument(
        'enable_publish_extrinsic',
        default_value='false'
    )

    # Get launch configurations
    camera_name = LaunchConfiguration('camera_name')
    depth_registration = LaunchConfiguration('depth_registration')
    serial_number = LaunchConfiguration('serial_number')
    device_num = LaunchConfiguration('device_num')
    vendor_id = LaunchConfiguration('vendor_id')
    product_id = LaunchConfiguration('product_id')
    enable_point_cloud = LaunchConfiguration('enable_point_cloud')
    enable_colored_point_cloud = LaunchConfiguration('enable_colored_point_cloud')
    point_cloud_qos = LaunchConfiguration('point_cloud_qos')
    connection_delay = LaunchConfiguration('connection_delay')
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    color_fps = LaunchConfiguration('color_fps')
    enable_color = LaunchConfiguration('enable_color')
    flip_color = LaunchConfiguration('flip_color')
    color_qos = LaunchConfiguration('color_qos')
    color_camera_info_qos = LaunchConfiguration('color_camera_info_qos')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    depth_fps = LaunchConfiguration('depth_fps')
    enable_depth = LaunchConfiguration('enable_depth')
    flip_depth = LaunchConfiguration('flip_depth')
    depth_qos = LaunchConfiguration('depth_qos')
    depth_camera_info_qos = LaunchConfiguration('depth_camera_info_qos')
    ir_width = LaunchConfiguration('ir_width')
    ir_height = LaunchConfiguration('ir_height')
    ir_fps = LaunchConfiguration('ir_fps')
    enable_ir = LaunchConfiguration('enable_ir')
    flip_ir = LaunchConfiguration('flip_ir')
    ir_qos = LaunchConfiguration('ir_qos')
    ir_camera_info_qos = LaunchConfiguration('ir_camera_info_qos')
    publish_tf = LaunchConfiguration('publish_tf')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    ir_info_url = LaunchConfiguration('ir_info_url')
    color_info_url = LaunchConfiguration('color_info_url')
    color_roi_x = LaunchConfiguration('color_roi_x')
    color_roi_y = LaunchConfiguration('color_roi_y')
    color_roi_width = LaunchConfiguration('color_roi_width')
    color_roi_height = LaunchConfiguration('color_roi_height')
    depth_roi_x = LaunchConfiguration('depth_roi_x')
    depth_roi_y = LaunchConfiguration('depth_roi_y')
    depth_roi_width = LaunchConfiguration('depth_roi_width')
    depth_roi_height = LaunchConfiguration('depth_roi_height')
    depth_scale = LaunchConfiguration('depth_scale')
    color_depth_synchronization = LaunchConfiguration('color_depth_synchronization')
    use_uvc_camera = LaunchConfiguration('use_uvc_camera')
    uvc_vendor_id = LaunchConfiguration('uvc_vendor_id')
    uvc_product_id = LaunchConfiguration('uvc_product_id')
    uvc_retry_count = LaunchConfiguration('uvc_retry_count')
    uvc_camera_format = LaunchConfiguration('uvc_camera_format')
    uvc_flip = LaunchConfiguration('uvc_flip')
    oni_log_level = LaunchConfiguration('oni_log_level')
    oni_log_to_console = LaunchConfiguration('oni_log_to_console')
    oni_log_to_file = LaunchConfiguration('oni_log_to_file')
    enable_d2c_viewer = LaunchConfiguration('enable_d2c_viewer')
    enable_publish_extrinsic = LaunchConfiguration('enable_publish_extrinsic')

    return LaunchDescription([
        # Declare all arguments
        declare_camera_name,
        declare_depth_registration,
        declare_serial_number,
        declare_device_num,
        declare_vendor_id,
        declare_product_id,
        declare_enable_point_cloud,
        declare_enable_colored_point_cloud,
        declare_point_cloud_qos,
        declare_connection_delay,
        declare_color_width,
        declare_color_height,
        declare_color_fps,
        declare_enable_color,
        declare_flip_color,
        declare_color_qos,
        declare_color_camera_info_qos,
        declare_depth_width,
        declare_depth_height,
        declare_depth_fps,
        declare_enable_depth,
        declare_flip_depth,
        declare_depth_qos,
        declare_depth_camera_info_qos,
        declare_ir_width,
        declare_ir_height,
        declare_ir_fps,
        declare_enable_ir,
        declare_flip_ir,
        declare_ir_qos,
        declare_ir_camera_info_qos,
        declare_publish_tf,
        declare_tf_publish_rate,
        declare_ir_info_url,
        declare_color_info_url,
        declare_color_roi_x,
        declare_color_roi_y,
        declare_color_roi_width,
        declare_color_roi_height,
        declare_depth_roi_x,
        declare_depth_roi_y,
        declare_depth_roi_width,
        declare_depth_roi_height,
        declare_depth_scale,
        declare_color_depth_synchronization,
        declare_use_uvc_camera,
        declare_uvc_vendor_id,
        declare_uvc_product_id,
        declare_uvc_retry_count,
        declare_uvc_camera_format,
        declare_uvc_flip,
        declare_oni_log_level,
        declare_oni_log_to_console,
        declare_oni_log_to_file,
        declare_enable_d2c_viewer,
        declare_enable_publish_extrinsic,

        # Group with namespace
        GroupAction([
            PushRosNamespace(camera_name),
            Node(
                package='astra_camera',
                executable='astra_camera_node',
                name='camera',
                output='screen',
                parameters=[
                    {'camera_name': camera_name},
                    {'depth_registration': depth_registration},
                    {'serial_number': serial_number},
                    {'device_num': device_num},
                    {'vendor_id': vendor_id},
                    {'product_id': product_id},
                    {'enable_point_cloud': enable_point_cloud},
                    {'enable_colored_point_cloud': enable_colored_point_cloud},
                    {'point_cloud_qos': point_cloud_qos},
                    {'connection_delay': connection_delay},
                    {'color_width': color_width},
                    {'color_height': color_height},
                    {'color_fps': color_fps},
                    {'enable_color': enable_color},
                    {'flip_color': flip_color},
                    {'color_qos': color_qos},
                    {'color_camera_info_qos': color_camera_info_qos},
                    {'depth_width': depth_width},
                    {'depth_height': depth_height},
                    {'depth_fps': depth_fps},
                    {'enable_depth': enable_depth},
                    {'flip_depth': flip_depth},
                    {'depth_qos': depth_qos},
                    {'depth_camera_info_qos': depth_camera_info_qos},
                    {'ir_width': ir_width},
                    {'ir_height': ir_height},
                    {'ir_fps': ir_fps},
                    {'enable_ir': enable_ir},
                    {'flip_ir': flip_ir},
                    {'ir_qos': ir_qos},
                    {'ir_camera_info_qos': ir_camera_info_qos},
                    {'publish_tf': publish_tf},
                    {'tf_publish_rate': tf_publish_rate},
                    {'ir_info_url': ir_info_url},
                    {'color_info_url': color_info_url},
                    {'color_roi_x': color_roi_x},
                    {'color_roi_y': color_roi_y},
                    {'color_roi_width': color_roi_width},
                    {'color_roi_height': color_roi_height},
                    {'depth_roi_x': depth_roi_x},
                    {'depth_roi_y': depth_roi_y},
                    {'depth_roi_width': depth_roi_width},
                    {'depth_roi_height': depth_roi_height},
                    {'depth_scale': depth_scale},
                    {'color_depth_synchronization': color_depth_synchronization},
                    {'use_uvc_camera': use_uvc_camera},
                    {'uvc_vendor_id': uvc_vendor_id},
                    {'uvc_product_id': uvc_product_id},
                    {'uvc_retry_count': uvc_retry_count},
                    {'uvc_camera_format': uvc_camera_format},
                    {'uvc_flip': uvc_flip},
                    {'oni_log_level': oni_log_level},
                    {'oni_log_to_console': oni_log_to_console},
                    {'oni_log_to_file': oni_log_to_file},
                    {'enable_d2c_viewer': enable_d2c_viewer},
                    {'enable_publish_extrinsic': enable_publish_extrinsic},
                ],
                remappings=[
                    ('/camera/depth/color/points', '/camera/depth_registered/points')
                ]
            ),
        ]),

        # Image transport republish nodes (outside of namespace group)
        Node(
            package='image_transport',
            executable='republish',
            name='lucia_color_republish',
            output='screen',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/lucia_astra_pro/color/image_raw'),
                ('out/compressed', '/lucia_astra_pro/color/image_raw/compressed'),
            ]
        ),

        Node(
            package='image_transport',
            executable='republish',
            name='lucia_depth_republish',
            output='screen',
            arguments=['raw', 'compressedDepth'],
            remappings=[
                ('in', '/lucia_astra_pro/depth/image_raw'),
                ('out/compressedDepth', '/lucia_astra_pro/depth/image_raw/compressed'),
            ]
        ),
    ])