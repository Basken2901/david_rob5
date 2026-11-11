"""Publish equivalent, bounded joint angles for Isaac Sim's articulation drives."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacJointStates(Node):
    def __init__(self):
        super().__init__('isaac_joint_states')
        self.publisher = self.create_publisher(JointState, '/isaac/joint_states', 10)
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.forward, 10
        )

    def forward(self, message):
        if len(message.name) != len(message.position):
            self.get_logger().error('Ignoring JointState with mismatched names and positions')
            return
        if any(not math.isfinite(angle) for angle in message.position):
            self.get_logger().error('Ignoring JointState with a non-finite position')
            return

        output = JointState()
        output.header = message.header
        output.name = message.name
        output.position = [math.remainder(angle, math.tau) for angle in message.position]
        output.velocity = message.velocity
        output.effort = message.effort
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = IsaacJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
