import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class CameraTF(Node):
    def __init__(self):
        super().__init__('david_camera_tf')
        self.frames = [None, None]
        self.poses = None
        self.lock = threading.Lock()
        self.broadcaster = TransformBroadcaster(self)
        for index in range(2):
            self.create_subscription(
                PointCloud2, f'/david/cam{index + 1}/depth/color/points',
                lambda msg, i=index: self.record_frame(i, msg), 10)
        self.create_timer(1 / 30, self.publish)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 15151))
        self.sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()

    def record_frame(self, index, msg):
        self.frames[index] = msg.header.frame_id

    def receive(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(8192)
                parsed = json.loads(data)
                if len(parsed.get('poses', [])) == 2:
                    with self.lock:
                        self.poses = parsed['poses']
            except socket.timeout:
                continue
            except (ValueError, KeyError, TypeError):
                continue

    def publish(self):
        with self.lock:
            poses = self.poses
        if poses is None:
            return
        now = self.get_clock().now().to_msg()
        for index, pose in enumerate(poses):
            frame = self.frames[index]
            if not frame:
                continue
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = 'david_world'
            transform.child_frame_id = frame
            p = pose['position']
            q = pose['quaternion']
            transform.transform.translation.x = p[0]
            transform.transform.translation.y = p[1]
            transform.transform.translation.z = p[2]
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(transform)

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
