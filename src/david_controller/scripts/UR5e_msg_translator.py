#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


UR_JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


LEFT_MAPPING = {
    'shoulder_pan_joint':  'revolute_1',
    'shoulder_lift_joint': 'revolute_2',
    'elbow_joint':         'revolute_3',
    'wrist_1_joint':       'revolute_4',
    'wrist_2_joint':       'revolute_5',
    'wrist_3_joint':       'revolute_6',
}


RIGHT_MAPPING = {
    'shoulder_pan_joint':  'revolute_7',
    'shoulder_lift_joint': 'revolute_8',
    'elbow_joint':         'revolute_9',
    'wrist_1_joint':       'revolute_10',
    'wrist_2_joint':       'revolute_11',
    'wrist_3_joint':       'revolute_12',
}


class URJointMapper(Node):

    def __init__(self):
        super().__init__('ur_joint_mapper')

        self.declare_parameter('arm', 'left')
        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter(
            'output_topic',
            '/isaac/left/joint_states'
        )

        self.arm = self.get_parameter(
            'arm'
        ).get_parameter_value().string_value

        self.input_topic = self.get_parameter(
            'input_topic'
        ).get_parameter_value().string_value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value

        if self.arm == 'left':
            self.mapping = LEFT_MAPPING

        elif self.arm == 'right':
            self.mapping = RIGHT_MAPPING

        else:
            raise ValueError(
                f"Invalid arm '{self.arm}'. Use 'left' or 'right'."
            )

        self.publisher = self.create_publisher(
            JointState,
            self.output_topic,
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            self.input_topic,
            self.joint_state_callback,
            10
        )

        self.get_logger().info(
            f'UR -> Isaac mapper started'
        )
        self.get_logger().info(
            f'Arm:    {self.arm}'
        )
        self.get_logger().info(
            f'Input:  {self.input_topic}'
        )
        self.get_logger().info(
            f'Output: {self.output_topic}'
        )

        for ur_name in UR_JOINT_ORDER:
            self.get_logger().info(
                f'{ur_name} -> {self.mapping[ur_name]}'
            )

    def joint_state_callback(self, msg: JointState):

        positions = dict(zip(msg.name, msg.position))

        velocities = {}
        if len(msg.velocity) == len(msg.name):
            velocities = dict(zip(msg.name, msg.velocity))

        efforts = {}
        if len(msg.effort) == len(msg.name):
            efforts = dict(zip(msg.name, msg.effort))

        missing = [
            name
            for name in UR_JOINT_ORDER
            if name not in positions
        ]

        if missing:
            self.get_logger().warning(
                f'Missing joints: {missing}'
            )
            return

        output = JointState()

        output.header = msg.header

        output.name = [
            self.mapping[name]
            for name in UR_JOINT_ORDER
        ]

        output.position = [
            positions[name]
            for name in UR_JOINT_ORDER
        ]

        if velocities:
            output.velocity = [
                velocities.get(name, 0.0)
                for name in UR_JOINT_ORDER
            ]

        if efforts:
            output.effort = [
                efforts.get(name, 0.0)
                for name in UR_JOINT_ORDER
            ]

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = URJointMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

