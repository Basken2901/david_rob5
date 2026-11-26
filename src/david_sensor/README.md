# DAVID dual RealSense RGB-D over Ethernet

The normal onboard Docker workflow is unchanged: start the UR driver first,
then build/source the workspace and run:

```bash
ros2 launch david_launch main_controller.launch.py
```

The controller remains in its existing ROS domain (1). The two RealSense
D435i cameras and their sender run on a local-only ROS domain (78) using
Fast DDS LARGE_DATA. No remote ROS image subscriber is required or intended.
The camera nodes publish RGB8, aligned 16-bit depth and CameraInfo locally at
the existing 640x480x30 profile. The sender connects from 169.254.40.222 to
the laptop 169.254.89.190 on TCP 5510 (left) and 5511 (right).
Both streams use the existing DRB2 protocol: calibration JSON, raw RGB and raw
aligned depth. Isaac receives directly and uses its USD camera poses.
No separate camera service or compression pipeline is used.

Optional launch arguments: ethernet_laptop_ip, ethernet_onboard_ip,
color_profile, depth_profile, enable_cameras, enable_ethernet_rgbd.
The last switch disables the TCP sender for local diagnostics.

To inspect the local camera topics in a separate onboard Docker shell:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 topic list --no-daemon
```

Check the sender logs for approximately 30 bundles/s per camera once Isaac
is playing. Actual performance must be measured on the robot. A freshly
rebuilt Docker image no longer explicitly installs the old codec plugins.
The per-camera raw allowlists also restrict image_transport publishers on
existing images supporting that parameter.

If a legacy host camera service was previously installed, stop/disable it
before starting this launch to avoid competing for the same USB devices:
`systemctl --user disable --now david-rgbd-cameras.service`.
