"""Launch DAVID's two physical RealSense RGB-D cameras on the onboard PC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _start_cameras(context):
    serials = {}
    for arm in ("left", "right"):
        serial = LaunchConfiguration(arm + "_serial").perform(context).strip().lstrip("_")
        if not serial.isdecimal():
            raise RuntimeError(f"{arm}_serial must be the numeric RealSense serial number")
        serials[arm] = serial

    if serials["left"] == serials["right"]:
        raise RuntimeError("Camera serial numbers must be different")

    driver_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
    ])
    color_profile = LaunchConfiguration("color_profile").perform(context)
    depth_profile = LaunchConfiguration("depth_profile").perform(context)

    enable_grayscale = LaunchConfiguration("enable_grayscale").perform(context).lower() in ("true", "1", "yes")
    actions = []
    for arm in ("left", "right"):
        # RealSense's rs_launch.py supports a per-node flat parameter dictionary
        # through config_file. Do not use SetParameter here: it injects
        # global_params into the launch context and crashes these camera nodes.
        codec_config = PathJoinSubstitution([
            FindPackageShare("david_launch"),
            "config",
            f"cam_{arm}_lossless.yaml",
        ])
        actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(driver_launch),
                    launch_arguments={
                        "camera_namespace": "david",
                        "camera_name": "cam_" + arm,
                        "serial_no": "_" + serials[arm],
                        "config_file": codec_config,
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
        if enable_grayscale:
            actions.append(Node(
                package="david_launch",
                executable="grayscale_camera",
                name=f"cam_{arm}_grayscale",
                namespace="david",
                parameters=[{"camera_name": f"cam_{arm}"}],
                output="screen",
            ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("left_serial", default_value=""),
        DeclareLaunchArgument("right_serial", default_value=""),
        DeclareLaunchArgument("color_profile", default_value="640x480x30"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x30"),
        DeclareLaunchArgument("enable_grayscale", default_value="true"),
        OpaqueFunction(function=_start_cameras),
    ])
