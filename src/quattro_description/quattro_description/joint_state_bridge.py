#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quattro_msgs.msg import JointAngles


JOINT_NAMES = [
    'motor_front_left_hip',
    'motor_front_left_upper_leg',
    'motor_front_left_lower_leg',
    'motor_front_right_hip',
    'motor_front_right_upper_leg',
    'motor_front_right_lower_leg',
    'motor_back_left_hip',
    'motor_back_left_upper_leg',
    'motor_back_left_lower_leg',
    'motor_back_right_hip',
    'motor_back_right_upper_leg',
    'motor_back_right_lower_leg',
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('quattro_joint_state_bridge')

        self.declare_parameter('source_topic', '/quattro/joints_raw')
        self.declare_parameter('degrees', True)

        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        self.degrees = self.get_parameter('degrees').get_parameter_value().bool_value

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            JointAngles,
            source_topic,
            self.joint_angles_callback,
            10,
        )

        unit = 'deg' if self.degrees else 'rad'
        self.get_logger().info(
            f'Publishing /joint_states from {source_topic} ({unit}) for quattro_description'
        )

    def joint_angles_callback(self, msg):
        positions = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]

        if self.degrees:
            positions = [math.radians(float(position)) for position in positions]
        else:
            positions = [float(position) for position in positions]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = JOINT_NAMES
        joint_state.position = positions

        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
