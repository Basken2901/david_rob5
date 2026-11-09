import math, rclpy
from rclpy.node import Node
from interface.msg import VRInput

class FakeVR(Node):
    def __init__(self):
        super().__init__('fake_vr')
        self.pub = self.create_publisher(VRInput, '/vr/input', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)          # 50 Hz

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        m = VRInput()
        x = 0.05 * math.sin(2 * math.pi * t / 8)    # +/- 5 cm hand motion
        for side in (m.left, m.right):
            side.pose_available = True
            side.pose.orientation.w = 1.0           # must be set, or the quaternion is all zeros
            side.pose.position.x = x
            side.dms = 1.0 if (t % 12) < 8 else 0.0
        self.pub.publish(m)

rclpy.init()
rclpy.spin(FakeVR())