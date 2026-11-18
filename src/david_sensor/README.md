# DAVID dual RealSense RGB-D over Wi-Fi (lossless)

Both RealSense D435i cameras attach to DAVID's onboard PC. With `david_docker_real`
running, starting the **existing** `ros2 launch david_launch main_controller.launch.py`
launches McQueen and both cameras at **640×480, 30 FPS** for color and depth.
The RealSense driver still aligns depth to the RGB image.

## Transport to Isaac (subscribe ONLY to these image streams)

- `/david/cam_left/color/image_raw/compressed` — `sensor_msgs/msg/CompressedImage`, **PNG** (lossless RGB).
- `/david/cam_left/aligned_depth_to_color/image_raw/compressedDepth` — `sensor_msgs/msg/CompressedImage`, **RVL** (lossless 16-bit depth).
- Same two topics under `/david/cam_right/`.
- Small calibration topics: `/david/cam_left/color/camera_info` and `/david/cam_right/color/camera_info`.

No JPEG, grayscale, colour reduction, depth downsampling, or point-cloud generation.
The combined `/rgbd` publisher is **disabled**. Native raw image topics may still
be listed by ROS 2; they are **not transmitted across Wi-Fi** without subscribers.
Close other camera viewers or `ros2 topic echo/hz` instances on raw topics
while testing compressed transport.

Compressed image topics use the RealSense wrapper's existing `image_transport`
publishers and ROS 2 Jazzy's `image_transport_plugins`, installed in
`docker/Dockerfile`. The launch file sets per-camera plugin parameters in
scoped groups:
- PNG codec on colour `image_raw/compressed`.
- RVL codec on aligned `image_raw/compressedDepth`.
- `compressedDepth.depth_max = 65.535` metres so **raw 16UC1 millimetre
  depth** isn't clipped at the plugin's default 10-metre threshold.

The exact wire format can be verified by reading the `format` field. PNG must
say `png`, **not `jpeg`**; depth must say `compressedDepth rvl`.
Do not assume it is lossless until these values are verified in the running node.

## Deploy on onboard PC

After pulling the `feat/david-rgbd-mixed-reality` branch, stop the active
real-DAVID launch safely. Rebuild the **real** Docker image to install the
compression plugins:

```bash
cd ~/ROB5/david_rob5/docker
docker compose -f docker_real-compose.yml up -d --build --force-recreate
docker exec -it david_docker_real bash
cd /ros2_ws
colcon build --packages-select david_launch
source install/setup.bash
ros2 launch david_launch main_controller.launch.py
```

Verify inside Docker (use a different terminal so the launch remains running):

```bash
ros2 topic list | grep -E 'cam_(left|right)/.*(compressedDepth|compressed)'
ros2 param list /david/cam_left | grep -E 'compressed.*(format|depth_max)'
ros2 topic echo /david/cam_left/color/image_raw/compressed --once --field format
ros2 topic echo /david/cam_left/aligned_depth_to_color/image_raw/compressedDepth --once --field format
```

The `--once --field format` commands require a subscriber and may briefly
trigger encoding of one frame; this is intended. In the normal remote system,
subscribe only to the four compressed image topics plus two CameraInfo topics,
with **best-effort, keep-last depth 1** subscriber QoS.

## On laptop / Isaac PC

Use ROS_DOMAIN_ID=1 and working DDS discovery. Example:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic hz /david/cam_left/color/image_raw/compressed
```

The older Isaac Script Node subscribes to raw images or `/rgbd` and **will not
receive compressed data as-is**. Replace its subscribers with the compressed
transport version, retaining camera-info and image timestamps. For correct
persistent world-anchored reconstruction, the Isaac model-prims' optical-frame
calibration and synchronized robot/camera timestamps are still required.

**Performance note:** Even lossless compression can exceed available Wi-Fi
throughput on textured/noisy scenes. Evaluate delivered FPS and dropped frames
for **all four** streams concurrently; 30 FPS is a requested sensor profile,
not a guaranteed remote delivery rate.
