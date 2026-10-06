import os
import yaml
import xacro
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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





    return LaunchDescription([
        controller_node,
        
    ])