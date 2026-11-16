# DAVID dual RGB-D cameras

The physical left and right RealSense cameras are attached to **DAVID's onboard PC**.
The camera launch file is in `david_launch/launch/dual_rgbd.launch.py` and uses the existing ROS 2 RealSense wrapper, which can publish colour,
aligned depth, CameraInfo, RGB-D and colour-textured PointCloud2 topics. There is
no reason to write a separate point-cloud conversion publisher for the first pass.

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

## Expected ROS 2 topics

- `/david/cam_left/color/image_raw`
- `/david/cam_left/aligned_depth_to_color/image_raw`
- `/david/cam_left/depth/color/points`
- `/david/cam_left/rgbd`
- Matching `/david/cam_right/...` topics.

The physical-camera data is in the camera's optical frame. A future Isaac
subscriber/reconstruction stage must apply calibrated camera-on-wrist TFs and
time-aligned robot poses. Merely receiving the topics does not put their points
in DAVID's world frame.

Two uncompressed full-resolution clouds plus RGB-D over Wi-Fi may be expensive.
Start at 640x480/15fps, measure bandwidth, and then decide whether to reduce
resolution/frames, filter points, or compress the transfer. The publishers
are isolated from DAVID's control system.

Current `docker_real-compose.yml` sets ROS_DOMAIN_ID=1/Fast DDS. Make sure
the onboard ROS 2 environment and Isaac subscriber use compatible DDS settings.
This does not implement the Isaac scene renderer or TSDF persistence.
