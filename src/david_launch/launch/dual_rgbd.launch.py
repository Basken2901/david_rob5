"""Launch DAVID's two physical RealSense RGB-D cameras on the onboard PC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    actions = []
    for arm in ("left", "right"):
        # RealSense's underscore prefix preserves numeric serials as strings in YAML.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                "camera_namespace": "david",
                "camera_name": "cam_" + arm,
                "serial_no": "_" + serials[arm],
                "enable_color": "true",
                "enable_depth": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "enable_rgbd": "true",
                "pointcloud.enable": "true",
                "pointcloud.stream_filter": "2",
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("left_serial", default_value=""),
        DeclareLaunchArgument("right_serial", default_value=""),
        DeclareLaunchArgument("color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x15"),
        OpaqueFunction(function=_start_cameras),
    ])
