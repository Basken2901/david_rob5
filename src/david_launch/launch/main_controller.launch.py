import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        }.items(),
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
        controller_node,
        rgbd_cameras,
    ])
