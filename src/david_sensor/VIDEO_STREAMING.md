# DAVID physical RGB-D: low-bandwidth video + accurate depth

DAVID's onboard Ryzen 9 PRO 8945HS can potentially encode H.265 using
Radeon 780M VA-API, but hardware access is *not yet confirmed*. The host
`vainfo --display drm --device /dev/dri/renderD128` initially failed.

## What ships in this branch

**Camera DDS default: `LARGE_DATA`.** The two RealSense camera processes now run
with `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` scoped to each camera launch group.
McQueen retains its original Fast DDS environment and transport. This is a
transport-only change: the cameras remain at 640x480x30, H.265 RGB, and RVL
depth. No extra Docker packages are needed for Large Data mode.

This mode uses UDP for initial discovery and TCP/SHM for application data.
**Remote camera receivers must use compatible TCP/SHM transport.** The
`camera_link_monitor` now sets `LARGE_DATA` for its own process automatically;
other laptop ROS nodes and Isaac Sim do not change automatically. In the
future, the Isaac camera receiver must use compatible transport; do not set
`LARGE_DATA` for the whole Isaac process without checking its other ROS bridge
participants and joint-state subscriptions.

This is an experiment, not a guarantee of 30 FPS: TCP retransmissions can
increase age/latency, and the network still has finite bandwidth. Make sure
both cameras are connected, the robot is stationary, and no remote robot
control depends on the camera link before restarting `main_controller`.

The existing **one-command launch** remains:

```bash
ros2 launch david_launch main_controller.launch.py
```

It starts the McQueen controller, two RealSense D435i cameras, and advertises
FFmpeg video transports. In `video_encoder:=auto` mode the launch tests
a two-frame `hevc_vaapi` encode via every visible `/dev/dri/renderD*`
device **inside Docker**. If it passes, video uses hardware HEVC; if not,
the launch explicitly logs that it uses **software H.264/libx264**, to keep
colour images available while diagnosing VA-API. The actual image-transport
encoder is initialized only on demand when a video subscriber connects.
You may also pass `video_encoder:=hevc_vaapi`, `video_encoder:=libx264`,
or `video_encoder:=off` (FFmpeg plugin still advertises if installed but
not explicitly configured). `enable_grayscale:=false` by default; old
grayscale test can be enabled when comparing codecs.

Cameras continue capturing RGB8 and Z16 at **640×480 @ 30 FPS**.
RGB is *lossy* after H.265/H.264. Aligned **16UC1 depth remains lossless RVL**
with `depth_max: 65.535` m. No lossy depth conversion, pointcloud, or
combined RGB-D topics. The actual decoder retains original depth units; valid
data are limited by the RealSense sensor's range, calibration, and accuracy.

Select ONLY these streams on the laptop:

- `/david/cam_left/color/image_raw/ffmpeg`
- `/david/cam_right/color/image_raw/ffmpeg`
- `/david/cam_left/aligned_depth_to_color/image_raw/compressedDepth`
- `/david/cam_right/aligned_depth_to_color/image_raw/compressedDepth`

Do not subscribe remotely to uncompressed `image_raw`, previous grayscale
PNG, or additional compressed transports while benchmarking. ROS may
advertise those transports without streaming any data.

**Important:** FFmpeg `/ffmpeg` topics use the custom
`ffmpeg_image_transport_msgs/msg/FFMPEGPacket` message; they are *not*
`CompressedImage`. To visualize, use an image_transport-aware viewer with
FFmpeg plugin on the laptop (rqt_image_view with `ffmpeg` transport). RVL
depth still uses compressedDepth. Codec parameters in the RealSense config
may produce 'unsupported parameter' *warnings* from its launch wrapper;
these do not by themselves mean that the plugin ignored the settings.
Verify the running encoder parameter and actual packet encoding.

## On DAVID: update and restart (requires image rebuild)

Stop the existing main launch **safely** before replacing Docker: it also
contains the active robot controller. Then on the onboard PC:

```bash
cd ~/ROB5/david_rob5
git pull --ff-only origin feat/david-rgbd-mixed-reality
cd docker
docker build --network=host -t docker-ros2 .
docker compose -f docker_real-compose.yml up -d --force-recreate --no-build
docker exec -it david_docker_real bash
cd /ros2_ws
colcon build --packages-select david_launch
source install/setup.bash
ros2 launch david_launch main_controller.launch.py
```

The `--network=host` build is intentional: this machine previously had
DNS failures resolving `packages.ros.org` during the default Docker build.

The Docker image now contains FFmpeg, `ffmpeg_image_transport`,
`mesa-va-drivers`, and `vainfo`. Compose allows DRM devices
(character major 226), with `/dev` already mounted. Host `vainfo` failure
could mean no render node, permission problem, or VA-API driver failure.
The launch checks actual encoding success and logs the chosen codec.
If hardware encoding is unavailable, it does not claim that HEVC is active.

## On laptop: measure the *delivered* video and depth

```bash
sudo apt install ros-jazzy-ffmpeg-image-transport
cd ~/rob5/david_rob5
git pull --ff-only origin feat/david-rgbd-mixed-reality
colcon build --packages-select david_launch
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run david_launch camera_link_monitor
```

The monitor subscribes to all four compressed topics simultaneously with
sensor-data QoS (BEST_EFFORT, VOLATILE, shallow queue) **over Large Data TCP**,
reporting delivery
rate, bandwidth and worst interarrival gap **every 10 seconds**. PASS
requires >=27 messages/sec and <=0.20 s worst gap for each topic.
A video frame usually produces one FFmpeg packet, so packet rate is an
approximation to frame rate; it is not pixel-decoded frame verification.
Do not mistake local `ros2 topic hz` for laptop delivery.
For manual camera measurements on the laptop use
`FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA ros2 topic hz <camera-topic>`;
ordinary `ros2 topic hz` processes keep default UDP/SHM and may discover
camera publishers without receiving their TCP-only data. If packets do not
arrive, also check TCP connectivity/firewall rules between machines.
Restart the camera launch after pulling/rebuilding `david_launch`; a source
change cannot alter DDS transport in already-running camera processes.

Check hardware detection/codec on DAVID *after a video subscriber connects*:

```bash
ls -l /dev/dri/
vainfo --display drm --device /dev/dri/renderD128
ros2 param list /david/cam_left | grep ffmpeg
ros2 param get /david/cam_left cam_left.color.image_raw.ffmpeg.encoder
```

On laptop (after normal ROS sourcing), read the actual FFmpeg packet codec:

```bash
ros2 topic echo /david/cam_left/color/image_raw/ffmpeg --once --field encoding
```

If encoder setup fails, its log is in the `main_controller.launch.py` terminal.
If `vainfo` or `hevc_vaapi` fails, diagnose `/dev/dri` on **host and
Docker** before forcing hardware selection. Do not assume `renderD128` is
the correct device, and do not restart/move a physical robot casually.

## Reality check

Video is only one part of the network traffic. Two 30 FPS RVL depth streams
were ~153 KB per frame each (roughly **74 Mbps total**), and an HEVC target
of 2 Mbps/camera adds ~4 Mbps before UDP/DDS/Wi-Fi overhead. Under congestion,
both colour and depth can still drop; video compression cannot guarantee
30 FPS depth across the Wi-Fi. Depth image values are exact *for frames
that arrive*; 'accurate' is not synonymous with every frame delivered.
