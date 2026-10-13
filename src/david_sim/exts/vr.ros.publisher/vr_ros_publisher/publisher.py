import math
import time

import omni.ext
import omni.kit.app
import omni.usd
from omni.kit.xr.core import XRCore
from pxr import UsdGeom
from david_interfaces.msg import (
    VRInput,
    VRRightController,
    VRLeftController,
    VRHead,
)



class VRROSPublisherExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        import rclpy
        from rclpy.context import Context
        from geometry_msgs.msg import Pose
        from david_interfaces.msg import (
            VRInput,
            VRRightController,
            VRLeftController,
            VRHead,
        )

        self._Pose = Pose
        self._VRInput = VRInput
        self._VRRightController = VRRightController
        self._VRLeftController = VRLeftController
        self._VRHead = VRHead

        self._context = Context()
        self._context.init(args=[])
        self._node = rclpy.create_node(
            "david_vr_publisher",
            context=self._context,
            start_parameter_services=False,
        )

        # One publisher containing all devices.
        self._publisher = self._node.create_publisher(
            VRInput, "/vr/input", 1
        )

        self._last_error = {}
        self._reported = set()

        self._subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._update, name="David combined VR publisher"
            )
        )

        print("[VR ROS] Publishing combined input on /vr/input", flush=True)

    def _read_pose(self, device, pose_name, scale):
        if not device.has_pose(pose_name):
            raise RuntimeError("Pose unavailable")

        matrix = device.get_virtual_world_pose(pose_name)
        position = matrix.ExtractTranslation()
        quaternion = matrix.ExtractRotationQuat()

        x, y, z = (float(v) for v in position)
        qx, qy, qz = (float(v) for v in quaternion.GetImaginary())
        qw = float(quaternion.GetReal())

        if not all(
            math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)
        ):
            raise RuntimeError("Non-finite pose")

        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-8:
            raise RuntimeError("Invalid quaternion")

        pose = self._Pose()
        pose.position.x = x * scale
        pose.position.y = y * scale
        pose.position.z = z * scale
        pose.orientation.x = qx / norm
        pose.orientation.y = qy / norm
        pose.orientation.z = qz / norm
        pose.orientation.w = qw / norm
        return pose

    def _read_click(self, device, input_name):
        # Missing buttons remain false.
        if not device.has_input_gesture(input_name, "click"):
            return False

        value = float(
            device.get_input_gesture_value(input_name, "click")
        )
        if not math.isfinite(value):
            raise RuntimeError(f"Invalid {input_name} button value")
        return value > 0.5

    def _read_controller(self, xr, hand, scale):
        device = xr.get_input_device(f"/user/hand/{hand}")
        if device is None:
            raise RuntimeError("Controller unavailable")

        result = (
            self._VRRightController()
            if hand == "right"
            else self._VRLeftController()
        )
        result.pose = self._read_pose(device, "grip", scale)

        squeeze = float(
            device.get_input_gesture_value("squeeze", "value")
        )
        if not math.isfinite(squeeze):
            raise RuntimeError("Invalid squeeze value")
        result.dms = max(0.0, min(1.0, squeeze))

        result.trigger = self._read_click(device, "trigger")
        result.thumbstick_click = self._read_click(device, "thumbstick")

        if hand == "right":
            result.a = self._read_click(device, "a")
            result.b = self._read_click(device, "b")
        else:
            result.x = self._read_click(device, "x")
            result.y = self._read_click(device, "y")
            result.menu = self._read_click(device, "menu")

        result.pose_available = True
        return result

    def _read_head(self, xr, scale):
        device = xr.get_input_device("/user/head")
        if device is None:
            raise RuntimeError("Headset unavailable")

        result = self._VRHead()
        result.pose = self._read_pose(device, "", scale)
        result.pose_available = True
        return result

    def _report_error(self, name, error):
        now = time.monotonic()
        if now - self._last_error.get(name, float("-inf")) > 3.0:
            print(f"[VR ROS] Waiting for {name}: {error}", flush=True)
            self._last_error[name] = now

    def _update(self, event):
        # New message every update: failed device reads leave buttons
        # false, DMS zero, and pose_available false.
        message = self._VRInput()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "isaac_world"

        stage = omni.usd.get_context().get_stage()

        try:
            if stage is None:
                raise RuntimeError("No USD stage open")

            xr = XRCore.get_singleton()
            scale = UsdGeom.GetStageMetersPerUnit(stage)

            for name in ("right", "left", "head"):
                try:
                    if name == "head":
                        result = self._read_head(xr, scale)
                    else:
                        result = self._read_controller(xr, name, scale)

                    setattr(message, name, result)

                    if name not in self._reported:
                        print(f"[VR ROS] Reading {name}", flush=True)
                        self._reported.add(name)

                except Exception as error:
                    self._report_error(name, error)

        except Exception as error:
            self._report_error("XR", error)

        self._publisher.publish(message)

    def on_shutdown(self):
        self._subscription = None

        if getattr(self, "_node", None) is not None:
            self._node.destroy_node()
            self._node = None

        if getattr(self, "_context", None) is not None:
            self._context.try_shutdown()
            self._context = None

        print("[VR ROS] Publisher stopped", flush=True)