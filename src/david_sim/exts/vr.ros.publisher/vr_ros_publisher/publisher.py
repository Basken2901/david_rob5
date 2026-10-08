import math
import time

import omni.ext
import omni.kit.app
import omni.usd
from omni.kit.xr.core import XRCore
from pxr import UsdGeom


class VRROSPublisherExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        import rclpy
        from rclpy.context import Context
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, Float32

        self._PoseStamped = PoseStamped
        self._Bool = Bool
        self._Float32 = Float32

        self._context = Context()
        self._context.init(args=[])
        self._node = rclpy.create_node(
            "david_vr_publisher",
            context=self._context,
            start_parameter_services=False,
        )

        self._devices = {
            "left": "/user/hand/left",
            "right": "/user/hand/right",
            "head": "/user/head",
        }

        self._pose_pubs = {
            name: self._node.create_publisher(
                PoseStamped, f"/vr/{name}/pose", 1
            )
            for name in self._devices
        }

        self._clutch_pubs = {}
        self._trigger_pubs = {}

        for hand in ("left", "right"):
            self._clutch_pubs[hand] = self._node.create_publisher(
                Bool, f"/vr/{hand}/clutch", 1
            )
            self._trigger_pubs[hand] = self._node.create_publisher(
                Float32, f"/vr/{hand}/gripper", 1
            )

        self._last_error = {
            name: float("-inf") for name in self._devices
        }
        self._reported = set()

        self._subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._update, name="David VR ROS publisher"
            )
        )

        print(
            "[VR ROS] Publisher ready: left, right, head",
            flush=True,
        )

    def _publish_buttons(self, hand, squeeze, trigger):
        clutch = self._Bool()
        clutch.data = squeeze > 0.6
        self._clutch_pubs[hand].publish(clutch)

        gripper = self._Float32()
        gripper.data = max(0.0, min(1.0, trigger))
        self._trigger_pubs[hand].publish(gripper)

    def _make_pose(self, pose, scale, stamp):
        position = pose.ExtractTranslation()
        quaternion = pose.ExtractRotationQuat()
        imaginary = quaternion.GetImaginary()

        x, y, z = (float(v) for v in position)
        qx, qy, qz = (float(v) for v in imaginary)
        qw = float(quaternion.GetReal())

        if not all(
            math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)
        ):
            raise RuntimeError("Non-finite pose")

        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-8:
            raise RuntimeError("Invalid quaternion")

        message = self._PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = "isaac_world"

        message.pose.position.x = x * scale
        message.pose.position.y = y * scale
        message.pose.position.z = z * scale

        message.pose.orientation.x = qx / norm
        message.pose.orientation.y = qy / norm
        message.pose.orientation.z = qz / norm
        message.pose.orientation.w = qw / norm

        return message

    def _update(self, event):
        stage = omni.usd.get_context().get_stage()
        xr = XRCore.get_singleton()
        stamp = self._node.get_clock().now().to_msg()

        # Handle devices independently: one missing controller must
        # not prevent the other controller or headset publishing.
        for name, path in self._devices.items():
            try:
                if stage is None:
                    raise RuntimeError("No USD stage open")

                device = xr.get_input_device(path)

                if device is None:
                    raise RuntimeError("Device unavailable")

                if name == "head":
                    if not device.has_pose(""):
                        raise RuntimeError("Head pose unavailable")
                    pose = device.get_virtual_world_pose("")
                else:
                    if not device.has_pose("grip"):
                        raise RuntimeError("Grip pose unavailable")
                    pose = device.get_virtual_world_pose("grip")

                message = self._make_pose(
                    pose,
                    UsdGeom.GetStageMetersPerUnit(stage),
                    stamp,
                )

                if name != "head":
                    squeeze = float(
                        device.get_input_gesture_value(
                            "squeeze", "value"
                        )
                    )
                    trigger = float(
                        device.get_input_gesture_value(
                            "trigger", "value"
                        )
                    )
                    if not all(
                        math.isfinite(v) for v in (squeeze, trigger)
                    ):
                        raise RuntimeError("Non-finite trigger data")

                self._pose_pubs[name].publish(message)

                if name != "head":
                    self._publish_buttons(name, squeeze, trigger)

                if name not in self._reported:
                    print(f"[VR ROS] Publishing {name}", flush=True)
                    self._reported.add(name)

            except Exception as error:
                if name != "head":
                    self._publish_buttons(name, 0.0, 0.0)

                now = time.monotonic()
                if now - self._last_error[name] > 3.0:
                    print(
                        f"[VR ROS] Waiting for {name}: {error}",
                        flush=True,
                    )
                    self._last_error[name] = now

    def on_shutdown(self):
        self._subscription = None

        if getattr(self, "_node", None) is not None:
            self._node.destroy_node()
            self._node = None

        if getattr(self, "_context", None) is not None:
            self._context.try_shutdown()
            self._context = None

        print("[VR ROS] Publisher stopped", flush=True)