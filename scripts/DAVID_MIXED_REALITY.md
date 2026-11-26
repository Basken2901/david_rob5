# DAVID mixed reality

The normal main controller launch defaults to both raw RGB-D cameras at 1280x720x30, with left serial 250122074198 and right serial 261222075899. Verify physical placement with robot motion inhibited. The camera graph remains on local domain 78; control remains on domain 1. TCP ports 5510 and 5511 are unchanged.

On the onboard host, pull the branch, then inside the existing container build with `colcon build --packages-select david_launch`, source `install/setup.bash`, and launch as normal. No image rebuild is required for these changes. To fall back to 640x480, pass `color_profile:=640x480x30 depth_profile:=640x480x30`.

On the laptop, pull the same branch. With the DAVID USD open and playback stopped, run this in Isaac's Script Editor (adjust the repo path in the installer if needed):

```python
exec(compile(open("/home/bjarke/rob5/david_rob5/scripts/install_david_rgbd_inline.py").read(), "install_david_rgbd_inline.py", "exec"))
```

Click Reload Script on the RGBD Script Node and save the USD. Ensure On Playback Tick is connected to Exec In. The source is embedded; no external receiver or runtime script file is required. The mesh target is up to 30 Hz with stride 10; actual update rate depends on frame timing and GPU/USD load. If rendering or VR input deteriorates, use a larger stride or lower update rate and reinstall.

For the existing VR publisher, ensure its private ROS context uses domain 1 (or reads ROS_DOMAIN_ID with default 1) rather than hardcoding 0. The installer does not modify the VR graph or safety/controller logic.

720p raw RGB8 plus aligned 16-bit depth for two cameras at 30 FPS uses about 2.21 Gbit/s before overhead. Verify camera profiles, link utilization, RX rate and VR input rate before enabling physical motion. This configuration is not yet hardware validated.
