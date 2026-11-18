# Grayscale PNG test (DAVID physical cameras)

`main_controller.launch.py` now starts McQueen, both RealSense cameras, and
**two onboard grayscale converters** by default. The converters receive raw
RGB locally, convert to 8-bit grayscale (`mono8`) and publish lossless PNG.
The cameras still capture RGB8 at 640×480×30 and depth Z16 at 640×480×30.
**This test reduces Wi-Fi RGB payload, not camera USB bandwidth or depth traffic.**

**Laptop: subscribe to these new topics (not original RGB):**

- `/david/cam_left/color/image_mono/compressed`
- `/david/cam_right/color/image_mono/compressed`

Aligned depth is unchanged:

- `/david/cam_left/aligned_depth_to_color/image_raw/compressedDepth`
- `/david/cam_right/aligned_depth_to_color/image_raw/compressedDepth`

Original RGB topics remain for A/B comparison. **Do not subscribe to raw
`/color/image_raw` from the laptop while measuring grayscale bandwidth.**
No combined RGB-D or pointcloud streams are enabled.

The grayscale node also advertises a `/david/cam_*/color/image_mono` raw
Image topic to make `rqt_image_view` transport selection possible, but
publishes that raw topic only when subscribed. In rqt select
`/david/cam_left/color/image_mono` and `compressed` transport.
PNG image format should be `mono8; png compressed mono8`; RGB is **not
recoverable** from grayscale, although the PNG encoding is lossless.

## Deploy / test

On DAVID's onboard host, after stopping the current launch safely:

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

Note: Build the Docker image using host networking to avoid the known DNS
issue with `packages.ros.org`. This Dockerfile now explicitly installs
`python3-opencv` and `python3-numpy` for the converters.

On the laptop, after sourcing ROS 2 Jazzy and setting the matching DDS
environment, measure the **new** grayscale stream:

```bash
ros2 topic echo /david/cam_left/color/image_mono/compressed --once --field format
ros2 topic hz /david/cam_left/color/image_mono/compressed
ros2 topic bw /david/cam_left/color/image_mono/compressed
```

Test both new grayscale topics plus both RVL depth topics concurrently before
claiming the full camera system supports 30 FPS over Wi-Fi.

Set `enable_grayscale:=false` on the main launch to disable both converters.
