"""Measure four compressed DAVID streams on the laptop with sensor-data QoS.

Outputs rolling delivered rates and largest arrival gap. Does not decode video;
packet rate is a proxy for FPS for one-packet-per-frame HEVC/H.264 encoders.
"""

import time
from dataclasses import dataclass, field

import rclpy
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


@dataclass
class Sample:
    count: int = 0
    bytes: int = 0
    previous: float = None
    largest_gap: float = 0.0


class CameraLinkMonitor(Node):
    def __init__(self):
        super().__init__("david_camera_link_monitor")
        self.period = 10.0
        self.started = time.monotonic()
        self.stats = {}
        self.subscriptions = []

        for side in ("left", "right"):
            name = f"cam_{side}"
            prefix = f"/david/{name}"
            self._add(
                f"{name} depth (RVL)",
                f"{prefix}/aligned_depth_to_color/image_raw/compressedDepth",
                CompressedImage,
            )
            self._add(
                f"{name} color (video)",
                f"{prefix}/color/image_raw/ffmpeg",
                FFMPEGPacket,
            )
        self.timer = self.create_timer(self.period, self.report)
        self.get_logger().info(
            "Monitoring four streams; BEST_EFFORT, VOLATILE, KEEP_LAST=5. "
            "Do not run competing camera viewers during baseline tests."
        )

    def _add(self, name, topic, kind):
        self.stats[name] = Sample()
        self.subscriptions.append(self.create_subscription(
            kind, topic, lambda msg, label=name: self.on_frame(label, msg),
            qos_profile_sensor_data,
        ))

    def on_frame(self, label, msg):
        now = time.monotonic()
        sample = self.stats[label]
        if sample.previous is not None:
            sample.largest_gap = max(sample.largest_gap, now - sample.previous)
        sample.previous = now
        sample.count += 1
        sample.bytes += len(msg.data)

    def report(self):
        now = time.monotonic()
        elapsed = now - self.started
        self.get_logger().info(f"--- delivered over last {elapsed:.1f} seconds ---")
        for name, sample in self.stats.items():
            fps = sample.count / elapsed
            mbps = sample.bytes * 8 / elapsed / 1_000_000
            gap = sample.largest_gap
            if not sample.count:
                state = "NO DATA"
            elif fps >= 27.0 and gap <= 0.20:
                state = "PASS"
            else:
                state = "BELOW TARGET"
            self.get_logger().info(
                f"{name:25s} {fps:5.1f} msg/s  {mbps:5.1f} Mbps  "
                f"max gap {gap:5.2f}s  {state}"
            )
        self.stats = {key: Sample() for key in self.stats}
        self.started = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraLinkMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
