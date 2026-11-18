"""Lossless monochrome PNG bridge for DAVID's two RealSense RGB streams.

Converts RGB8/BGR8 to mono8 on the onboard PC; leaves RealSense RGB and
aligned-depth publishers untouched. No data goes over Wi-Fi without a subscriber.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class GrayscaleCamera(Node):
    def __init__(self):
        super().__init__("grayscale_camera")
        self.declare_parameter("camera_name", "cam_left")
        camera_name = self.get_parameter("camera_name").value
        if camera_name not in ("cam_left", "cam_right"):
            raise ValueError("camera_name must be cam_left or cam_right")

        prefix = f"/david/{camera_name}/color"
        self.source_topic = f"{prefix}/image_raw"
        self.mono_topic = f"{prefix}/image_mono"

        # Volatile and depth=1: this is a live video stream, not a history buffer.
        # RELIABLE keeps compatibility with generic ROS image subscribers.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.raw_pub = self.create_publisher(Image, self.mono_topic, output_qos)
        self.png_pub = self.create_publisher(
            CompressedImage, self.mono_topic + "/compressed", output_qos
        )
        self.sub = self.create_subscription(
            Image, self.source_topic, self.on_image, qos_profile_sensor_data
        )
        cv2.setNumThreads(1)
        self.get_logger().info(
            f"Grayscale PNG: {self.source_topic} -> {self.mono_topic}/compressed"
        )

    def on_image(self, msg):
        raw_subscribers = self.raw_pub.get_subscription_count()
        png_subscribers = self.png_pub.get_subscription_count()
        if not (raw_subscribers or png_subscribers):
            return

        if msg.encoding == "rgb8":
            conversion = cv2.COLOR_RGB2GRAY
        elif msg.encoding == "bgr8":
            conversion = cv2.COLOR_BGR2GRAY
        else:
            self.get_logger().error(
                f"Unsupported RGB encoding: {msg.encoding}", throttle_duration_sec=10.0
            )
            return

        channels = 3
        if msg.step < msg.width * channels:
            self.get_logger().error("Invalid RGB image step", throttle_duration_sec=10.0)
            return

        try:
            # Respect row padding, if any, while avoiding copying RGB input data.
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.step)
            )
            rgb = rows[:, : msg.width * channels].reshape(
                (msg.height, msg.width, channels)
            )
            mono = cv2.cvtColor(rgb, conversion)

            if raw_subscribers:
                gray_msg = Image()
                gray_msg.header = msg.header
                gray_msg.height = msg.height
                gray_msg.width = msg.width
                gray_msg.encoding = "mono8"
                gray_msg.is_bigendian = 0
                gray_msg.step = msg.width
                gray_msg.data = mono.tobytes()
                self.raw_pub.publish(gray_msg)

            if png_subscribers:
                ok, encoded = cv2.imencode(
                    ".png", mono, [cv2.IMWRITE_PNG_COMPRESSION, 3]
                )
                if not ok:
                    self.get_logger().error(
                        "Failed to encode grayscale PNG", throttle_duration_sec=10.0
                    )
                    return
                png_msg = CompressedImage()
                png_msg.header = msg.header
                png_msg.format = "mono8; png compressed mono8"
                png_msg.data = encoded.tobytes()
                self.png_pub.publish(png_msg)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(
                f"Grayscale conversion failed: {exc}", throttle_duration_sec=10.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GrayscaleCamera()
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
