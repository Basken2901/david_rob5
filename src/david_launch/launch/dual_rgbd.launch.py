"""Launch dual RealSense RGB-D, preserving accurate RVL depth and adding HEVC video."""

import glob
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _vaapi_hevc_works():
    """Actually encode test frames through the GPU *inside this Docker container*."""
    if not shutil.which("ffmpeg"):
        return False

    for device in sorted(glob.glob("/dev/dri/renderD*")):
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
            "-vaapi_device", device,
            "-f", "lavfi", "-i", "color=c=black:s=640x480:r=30",
            "-frames:v", "2",
            "-vf", "format=nv12,hwupload",
            "-c:v", "hevc_vaapi", "-b:v", "2M",
            "-f", "null", "-",
        ]
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=8, check=False
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if result.returncode == 0:
            return True
    return False


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
    requested = LaunchConfiguration("video_encoder").perform(context).strip()
    if requested not in ("auto", "hevc_vaapi", "libx264", "off"):
        raise RuntimeError("video_encoder must be auto, hevc_vaapi, libx264, or off")
    if requested == "auto":
        encoder = "hevc_vaapi" if _vaapi_hevc_works() else "libx264"
        status = (
            "Radeon VA-API HEVC encoder passed test; using hardware H.265"
            if encoder == "hevc_vaapi" else
            "Hardware HEVC unavailable in Docker; using software H.264 (libx264)"
        )
    else:
        encoder = requested
        status = f"Video encoder selected: {encoder}"
    actions = [LogInfo(msg=f"DAVID cameras: {status}; depth stays lossless RVL")]
    for arm in ("left", "right"):
        # RealSense's rs_launch.py supports a per-node flat parameter dictionary
        # through config_file. Do not use SetParameter here: it injects
        # global_params into the launch context and crashes these camera nodes.
        config_name = {
            "hevc_vaapi": "hevc",
            "libx264": "h264",
            "off": "lossless",
        }[encoder]
        codec_config = PathJoinSubstitution([
            FindPackageShare("david_launch"),
            "config",
            f"cam_{arm}_{config_name}.yaml",
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
        DeclareLaunchArgument("enable_grayscale", default_value="false"),
        DeclareLaunchArgument("video_encoder", default_value="auto"),
        OpaqueFunction(function=_start_cameras),
    ])
