# DAVID dual RGB-D cameras

The physical left and right RealSense cameras are attached to **DAVID's onboard PC**.
The camera launch file is `david_launch/launch/dual_rgbd.launch.py`. It uses the standard ROS 2 RealSense wrapper. Point-cloud generation is disabled; RGB and aligned depth are bundled into the native `realsense2_camera_msgs/msg/RGBD` message (including camera intrinsics). No compression or downsampling is applied.

## One-time onboard PC setup

On the onboard PC, install ROS 2 Jazzy and `realsense2_camera`, connect both
cameras, and check `rs-enumerate-devices` for their serial numbers. Keep a copy
of the `david_rob5` workspace on the onboard PC.

From that onboard workspace:
```bash
colcon build --packages-select david_launch
bash setup/onboard/install_camera_service.sh
```

Then edit `~/.config/david/cameras.env` with the left and right serial numbers.
The new user service is installed **without being enabled at boot**.
Check `systemctl --user` works in a noninteractive SSH session. The Docker
host needs passwordless SSH access to that onboard user.

## Normal McQueen startup (from Docker host)

First click the **existing real-DAVID desktop shortcut** (starts Docker and
the UR drivers, unchanged). Replace the manual McQueen command inside Docker
with this **host-side** wrapper:

```bash
export DAVID_ONBOARD_SSH=onboard-user@onboard-ip
bash setup/start_mcqueen_with_cameras.sh <the McQueen command you already use>
```

The command/arguments after the wrapper are executed in the real-DAVID
container with the ROS 2 workspace sourced. The wrapper starts the camera
service over SSH before launching McQueen, and stops the camera service when
McQueen exits. McQueen's actual launch command is not in this GitHub repo,
so it is intentionally **not hard-coded**. If you continue entering Docker
and starting McQueen directly, this wrapper will NOT be invoked.

## Topics to use remotely (Isaac)

- `/david/cam_left/rgbd`
- `/david/cam_right/rgbd`

The standard RealSense wrapper still advertises separate RGB, depth, camera-info and metadata topics. They do not normally carry camera frames over Wi-Fi without a matching remote subscriber. Have Isaac subscribe to ONLY the two combined `/rgbd` topics and close other remote image subscribers. Changing the camera publisher alone does not update the old Isaac script's subscriptions.

The physical-camera data is in the camera's optical frame. A future Isaac
subscriber/reconstruction stage must apply calibrated camera-on-wrist TFs and
time-aligned robot poses. Merely receiving the topics does not put their points
in DAVID's world frame.

At 640x480/60fps, the two uncompressed RGB-D topics together contain about 1.47 Gbit/s of pixel data before DDS/network overhead. Combining colour and depth in one ROS message reduces the topic/subscription count, NOT the number of image bytes. Start with a single remote subscriber and measure the receiving rate before opening both streams. The publishers are isolated from DAVID's control system.

Current `docker_real-compose.yml` sets ROS_DOMAIN_ID=1/Fast DDS. Make sure
the onboard ROS 2 environment and Isaac subscriber use compatible DDS settings.
This does not implement the Isaac scene renderer or TSDF persistence.
