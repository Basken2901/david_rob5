"""Forward DAVID's existing raw RealSense RGB-D topics over dedicated Ethernet.

One TCP connection per camera, one self-contained frame bundle per message.
The sender never opens camera devices and never publishes robot control topics.
Isaac Sim can receive the raw bytes without a ROS/DDS subscriber.

Wire format (big-endian):
  !4sIII: b'DRB2', metadata JSON length, RGB byte length, depth byte length
  UTF-8 JSON metadata (calibration, dimensions, encoding, timestamps, side)
  raw sensor_msgs/Image RGB data
  raw sensor_msgs/Image aligned-depth data
"""

import json
import socket
import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

MAGIC = b"DRB2"
HEADER = struct.Struct("!4sIII")
SIDES = ("left", "right")
PORTS = {"left": 5510, "right": 5511}
MAX_SKEW_NS = 70_000_000  # Test pipeline: up to 70 ms between image timestamps.


def timestamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class EthernetRgbdSender(Node):
    def __init__(self):
        super().__init__("david_ethernet_rgbd_sender")
        self.declare_parameter("laptop_ip", "169.254.89.190")
        self.declare_parameter("onboard_ip", "169.254.40.222")
        self.laptop_ip = self.get_parameter("laptop_ip").value
        self.onboard_ip = self.get_parameter("onboard_ip").value
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.pending = {side: threading.Event() for side in SIDES}
        self.frames = {
            side: {"rgb": deque(maxlen=6), "depth": deque(maxlen=6), "info": None}
            for side in SIDES
        }
        self.rx = {side: {"rgb": 0, "depth": 0, "info": 0} for side in SIDES}
        self.tx = {side: 0 for side in SIDES}
        self.subscriptions = []  # retain ROS subscription lifetimes
        self.workers = []

        for side in SIDES:
            prefix = f"/david/cam_{side}"
            topics = (
                ("rgb", Image, f"{prefix}/color/image_raw"),
                ("depth", Image, f"{prefix}/aligned_depth_to_color/image_raw"),
                ("info", CameraInfo, f"{prefix}/aligned_depth_to_color/camera_info"),
            )
            for key, msg_type, topic in topics:
                self.subscriptions.append(self.create_subscription(
                    msg_type, topic,
                    lambda msg, s=side, k=key: self.on_message(s, k, msg),
                    qos_profile_sensor_data,
                ))
                self.get_logger().info(f"Input {topic}")
            worker = threading.Thread(target=self.transmit, args=(side,), daemon=True)
            self.workers.append(worker)
            worker.start()

        self.create_timer(5.0, self.report)
        self.previous_rx = {side: dict(self.rx[side]) for side in SIDES}
        self.previous_tx = dict(self.tx)

    def on_message(self, side, kind, msg):
        with self.lock:
            if kind == "info":
                self.frames[side]["info"] = msg
            else:
                self.frames[side][kind].append(msg)
            self.rx[side][kind] += 1
        self.pending[side].set()

    def newest_pair(self, side, last_depth_stamp):
        with self.lock:
            data = self.frames[side]
            info = data["info"]
            if info is None or not data["rgb"] or not data["depth"]:
                return None
            # Find the newest depth frame with a close RGB neighbour.
            # Latest-only queues ensure a slow network never builds backlog.
            for depth in reversed(data["depth"]):
                t = timestamp_ns(depth)
                if t == last_depth_stamp:
                    continue
                rgb = min(data["rgb"], key=lambda m: abs(timestamp_ns(m) - t))
                if abs(timestamp_ns(rgb) - t) > MAX_SKEW_NS:
                    continue
                if (rgb.width != depth.width or rgb.height != depth.height
                        or info.width != depth.width or info.height != depth.height):
                    continue
                if depth.encoding.lower() not in ("16uc1", "mono16"):
                    continue
                if rgb.encoding.lower() not in ("rgb8", "bgr8", "rgba8", "bgra8"):
                    continue
                return rgb, depth, info, t
        return None

    @staticmethod
    def make_packet(side, rgb, depth, info):
        # Image.data is already raw pixels from the RealSense ROS driver.
        rgb_bytes = bytes(rgb.data)
        depth_bytes = bytes(depth.data)
        meta = {
            "side": side,
            "timestamp_ns": timestamp_ns(depth),
            "rgb_timestamp_ns": timestamp_ns(rgb),
            "frame_id": depth.header.frame_id,
            "rgb": {
                "width": rgb.width, "height": rgb.height, "step": rgb.step,
                "encoding": rgb.encoding, "bigendian": bool(rgb.is_bigendian),
            },
            "depth": {
                "width": depth.width, "height": depth.height, "step": depth.step,
                "encoding": depth.encoding, "bigendian": bool(depth.is_bigendian),
                "unit_m": 0.001,  # ROS RealSense 16UC1 depth values in millimetres.
            },
            "camera_info": {
                "width": info.width, "height": info.height, "k": list(info.k),
                "d": list(info.d), "r": list(info.r), "p": list(info.p),
                "distortion_model": info.distortion_model,
            },
        }
        metadata = json.dumps(meta, separators=(",", ":")).encode("utf-8")
        header = HEADER.pack(MAGIC, len(metadata), len(rgb_bytes), len(depth_bytes))
        return header, metadata, rgb_bytes, depth_bytes

    def transmit(self, side):
        port = PORTS[side]
        warned_at = 0.0
        while not self.stop.is_set() and rclpy.ok():
            try:
                with socket.create_connection(
                    (self.laptop_ip, port), timeout=4,
                    source_address=(self.onboard_ip, 0),
                ) as sock:
                    sock.settimeout(5.0)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info(
                        f"{side}: Ethernet connected: {self.onboard_ip} -> "
                        f"{self.laptop_ip}:{port}"
                    )
                    last_depth_stamp = None
                    while not self.stop.is_set() and rclpy.ok():
                        self.pending[side].wait(0.1)
                        self.pending[side].clear()
                        pair = self.newest_pair(side, last_depth_stamp)
                        if pair is None:
                            continue
                        rgb, depth, info, t = pair
                        header, metadata, color, z16 = self.make_packet(side, rgb, depth, info)
                        sock.sendall(header + metadata)
                        sock.sendall(color)
                        sock.sendall(z16)
                        last_depth_stamp = t
                        with self.lock:
                            self.tx[side] += 1
            except OSError as error:
                now = time.monotonic()
                if now - warned_at >= 15.0 and not self.stop.is_set():
                    self.get_logger().warning(
                        f"{side}: Ethernet receiver unavailable ({error}); retrying"
                    )
                    warned_at = now
                self.stop.wait(2.0)

    def report(self):
        with self.lock:
            rx = {side: dict(self.rx[side]) for side in SIDES}
            tx = dict(self.tx)
        for side in SIDES:
            self.get_logger().info(
                f"{side}: ROS RGB {(rx[side]['rgb']-self.previous_rx[side]['rgb'])/5:.1f} Hz, "
                f"depth {(rx[side]['depth']-self.previous_rx[side]['depth'])/5:.1f} Hz, "
                f"info {(rx[side]['info']-self.previous_rx[side]['info'])/5:.1f} Hz; "
                f"Ethernet {(tx[side]-self.previous_tx[side])/5:.1f} bundles/s"
            )
        self.previous_rx, self.previous_tx = rx, tx

    def close(self):
        self.stop.set()
        for event in self.pending.values():
            event.set()
        for worker in self.workers:
            worker.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = EthernetRgbdSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
