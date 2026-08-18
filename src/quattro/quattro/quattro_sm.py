#!/usr/bin/env python3
"""
Quattro high-level command state machine.

Input:
  /quattro/cmd_raw  (quattro_msgs/QuattroCmd)

Output:
  /quattro/cmd      (quattro_msgs/QuattroCmd)

Purpose:
- Keep keyboard/joystick input separate from the command consumed by commander.
- Provide a single safety gate.
- Detect lost input heartbeat.
- On timeout, publish a safe Stop command continuously.
- Preserve pose/movement mode fields from the most recent valid command.

Input nodes should publish /quattro/cmd_raw continuously (20 Hz recommended).
"""

import copy
import time

import rclpy
from rclpy.node import Node

from quattro_msgs.msg import QuattroCmd


class QuattroStateMachine(Node):
    def __init__(self):
        super().__init__('quattro_sm')

        self.declare_parameter('input_topic', '/quattro/cmd_raw')
        self.declare_parameter('output_topic', '/quattro/cmd')
        self.declare_parameter('publish_hz', 100.0)
        self.declare_parameter('timeout_sec', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.pub = self.create_publisher(QuattroCmd, output_topic, 20)
        self.sub = self.create_subscription(
            QuattroCmd, input_topic, self._cmd_raw_cb, 20
        )

        self.last_rx_monotonic = None
        self.have_input = False
        self.timeout_active = False

        self.latest = self._make_default_command()

        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f'STARTING NODE: quattro State Machine (Python) | '
            f'{input_topic} -> {output_topic} | timeout={self.timeout_sec:.2f}s'
        )

    @staticmethod
    def _make_default_command():
        msg = QuattroCmd()
        msg.x_velocity = 0.0
        msg.y_velocity = 0.0
        msg.rate = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.z = 0.0
        msg.motion = 'Stop'
        msg.movement = 'Stepping'
        msg.pose_cmd = 'Normal'
        msg.imu_auto_pose = False
        return msg

    def _cmd_raw_cb(self, msg):
        self.latest = copy.deepcopy(msg)
        self.last_rx_monotonic = time.monotonic()
        self.have_input = True

        if self.timeout_active:
            self.timeout_active = False
            self.get_logger().info('Command heartbeat restored; leaving E-STOP timeout state.')

    def _safe_timeout_command(self):
        # Preserve posture/mode selection, but remove commanded locomotion.
        msg = copy.deepcopy(self.latest)
        msg.x_velocity = 0.0
        msg.y_velocity = 0.0
        msg.rate = 0.0
        msg.motion = 'Stop'
        return msg

    def _tick(self):
        now = time.monotonic()

        stale = (
            not self.have_input
            or self.last_rx_monotonic is None
            or (now - self.last_rx_monotonic) > self.timeout_sec
        )

        if stale:
            if not self.timeout_active:
                self.timeout_active = True
                self.get_logger().error('TIMEOUT...ENGAGING E-STOP!')
            self.pub.publish(self._safe_timeout_command())
            return

        self.pub.publish(self.latest)


def main(args=None):
    rclpy.init(args=args)
    node = QuattroStateMachine()
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
