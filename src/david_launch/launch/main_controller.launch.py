import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_pkg = get_package_share_directory('david_controller')
    controller_param = os.path.join(controller_pkg, 'config', 'controller_sim.yaml')

    controller_node = Node(
        package='david_controller',
        executable='controller',
        name='controller',
        parameters=[controller_param],
        output='screen',
    )

    # Start both RealSense camera publishers alongside the McQueen controller.
    # Serial numbers can be overridden from the ros2 launch command.
    rgbd_launch_path = os.path.join(
        get_package_share_directory('david_launch'),
        'launch',
        'dual_rgbd.launch.py',
    )
    rgbd_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rgbd_launch_path),
        condition=IfCondition(LaunchConfiguration('enable_cameras')),
        launch_arguments={
            'left_serial': LaunchConfiguration('left_camera_serial'),
            'right_serial': LaunchConfiguration('right_camera_serial'),
            'color_profile': LaunchConfiguration('color_profile'),
            'depth_profile': LaunchConfiguration('depth_profile'),
            'enable_grayscale': LaunchConfiguration('enable_grayscale'),
            'video_encoder': LaunchConfiguration('video_encoder'),
        }.items(),
    )

    # Independent raw RGB-D Ethernet bridge. Do not change the controller DDS.
    ethernet_sender = GroupAction(
        scoped=True,
        actions=[
            SetEnvironmentVariable(
                name='FASTDDS_BUILTIN_TRANSPORTS', value='LARGE_DATA'
            ),
            Node(
                package='david_launch',
                executable='ethernet_rgbd_sender',
                name='david_ethernet_rgbd_sender',
                parameters=[{
                    'laptop_ip': LaunchConfiguration('ethernet_laptop_ip'),
                    'onboard_ip': LaunchConfiguration('ethernet_onboard_ip'),
                }],
                output='screen',
                respawn=True,
                respawn_delay=3.0,
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_ethernet_rgbd')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_cameras',
            default_value='true',
            description='Start the two RealSense RGB-D publishers with McQueen.',
        ),
        DeclareLaunchArgument(
            'left_camera_serial',
            default_value='261222075899',
            description='RealSense serial number on the left wrist.',
        ),
        DeclareLaunchArgument(
            'right_camera_serial',
            default_value='250122074198',
            description='RealSense serial number on the right wrist.',
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='640x480x30',
            description='RGB width x height x FPS for both RealSense cameras.',
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value='640x480x30',
            description='Depth width x height x FPS for both RealSense cameras.',
        ),
        DeclareLaunchArgument(
            'enable_grayscale',
            default_value='false',
            description='Also publish optional grayscale PNG streams (uses extra CPU).',
        ),
        DeclareLaunchArgument(
            'video_encoder',
            default_value='auto',
            description='auto: probe VA-API HEVC, fallback to software H.264; also hevc_vaapi, libx264, off.',
        ),
        DeclareLaunchArgument(
            'enable_ethernet_rgbd', default_value='true',
        ),
        DeclareLaunchArgument(
            'ethernet_laptop_ip', default_value='169.254.89.190',
        ),
        DeclareLaunchArgument(
            'ethernet_onboard_ip', default_value='169.254.40.222',
        ),
        controller_node,
        rgbd_cameras,
        ethernet_sender,
    ])
