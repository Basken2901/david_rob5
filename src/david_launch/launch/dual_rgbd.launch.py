"""Start both raw RealSense cameras and their dedicated Ethernet sender."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Keep camera DDS strictly onboard and independent of the control graph (domain 1).
# The two camera nodes and the Ethernet sender share this private local domain.
CAMERA_ENV = {
    "ROS_DOMAIN_ID": "78",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "ROS_STATIC_PEERS": "",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "FASTDDS_BUILTIN_TRANSPORTS": "LARGE_DATA",
}


def _camera_environment():
    return [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in CAMERA_ENV.items()
    ]


def _start_cameras(context):
    serials = {}
    for arm in ("left", "right"):
        serial = LaunchConfiguration(arm + "_serial").perform(context).strip().lstrip("_")
        if not serial.isdecimal():
            raise RuntimeError(f"{arm}_serial must be a numeric RealSense serial number")
        serials[arm] = serial
    if serials["left"] == serials["right"]:
        raise RuntimeError("Camera serial numbers must be different")

    driver_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
    ])
    color_profile = LaunchConfiguration("color_profile").perform(context)
    depth_profile = LaunchConfiguration("depth_profile").perform(context)
    actions = [LogInfo(msg="DAVID cameras: raw RGB + aligned depth; local DDS domain 78; Ethernet TCP only")]

    for arm in ("left", "right"):
        raw_config = PathJoinSubstitution([
            FindPackageShare("david_launch"), "config", f"cam_{arm}_raw.yaml"
        ])
        actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                *_camera_environment(),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(driver_launch),
                    launch_arguments={
                        "camera_namespace": "david",
                        "camera_name": "cam_" + arm,
                        "serial_no": "_" + serials[arm],
                        "config_file": raw_config,
                        "enable_color": "true",
                        "enable_depth": "true",
                        "enable_sync": "true",
                        "align_depth.enable": "true",
                        "enable_rgbd": "false",
                        "pointcloud.enable": "false",
                        "rgb_camera.color_profile": color_profile,
                        "depth_module.depth_profile": depth_profile,
                    }.items(),
                ),
            ],
        ))

    actions.append(GroupAction(
        scoped=True,
        actions=[
            *_camera_environment(),
            Node(
                package="david_launch",
                executable="ethernet_rgbd_sender",
                name="david_ethernet_rgbd_sender",
                parameters=[{
                    "laptop_ip": LaunchConfiguration("ethernet_laptop_ip").perform(context),
                    "onboard_ip": LaunchConfiguration("ethernet_onboard_ip").perform(context),
                }],
                output="screen",
                respawn=True,
                respawn_delay=3.0,
            ),
        ],
        condition=IfCondition(LaunchConfiguration("enable_ethernet_rgbd")),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("left_serial", default_value=""),
        DeclareLaunchArgument("right_serial", default_value=""),
        DeclareLaunchArgument("color_profile", default_value="1280x720x30"),
        DeclareLaunchArgument("depth_profile", default_value="1280x720x30"),
        DeclareLaunchArgument("enable_ethernet_rgbd", default_value="true"),
        DeclareLaunchArgument("ethernet_laptop_ip", default_value="169.254.89.190"),
        DeclareLaunchArgument("ethernet_onboard_ip", default_value="169.254.40.222"),
        OpaqueFunction(function=_start_cameras),
    ])
