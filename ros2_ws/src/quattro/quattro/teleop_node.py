#!/usr/bin/env python3
"""
Joystick teleoperation for Quattro.

sensor_msgs/Joy
   -> /quattro/cmd_raw (QuattroCmd, continuous 20 Hz heartbeat)
   -> /joybuttons      (JoyButtons, event-style auxiliary controls)

Default Xbox/typical Linux joystick mapping can be changed with ROS parameters.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from quattro_msgs.msg import JoyButtons, QuattroCmd


def axis(msg, index):
    return float(msg.axes[index]) if 0 <= index < len(msg.axes) else 0.0


def button(msg, index):
    return int(msg.buttons[index]) if 0 <= index < len(msg.buttons) else 0


class QuattroTeleop(Node):
    def __init__(self):
        super().__init__('quattro_teleop')

        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('yaw_scale', 0.5)
        self.declare_parameter('deadzone', 0.08)

        # Default mappings; override from launch/yaml for your controller.
        self.declare_parameter('axis_forward', 1)
        self.declare_parameter('axis_lateral', 0)
        self.declare_parameter('axis_yaw', 3)

        self.declare_parameter('button_estop', 6)
        self.declare_parameter('button_motion', 0)
        self.declare_parameter('button_mode', 3)
        self.declare_parameter('button_clearance_up', 4)
        self.declare_parameter('button_clearance_down', 5)

        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.yaw_scale = float(self.get_parameter('yaw_scale').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        self.axis_forward = int(self.get_parameter('axis_forward').value)
        self.axis_lateral = int(self.get_parameter('axis_lateral').value)
        self.axis_yaw = int(self.get_parameter('axis_yaw').value)

        self.button_estop = int(self.get_parameter('button_estop').value)
        self.button_motion = int(self.get_parameter('button_motion').value)
        self.button_mode = int(self.get_parameter('button_mode').value)
        self.button_clearance_up = int(self.get_parameter('button_clearance_up').value)
        self.button_clearance_down = int(self.get_parameter('button_clearance_down').value)

        self.cmd_pub = self.create_publisher(QuattroCmd, '/quattro/cmd_raw', 20)
        self.jb_pub = self.create_publisher(JoyButtons, '/joybuttons', 20)
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 20)

        self.cmd = QuattroCmd()
        self.cmd.x_velocity = 0.0
        self.cmd.y_velocity = 0.0
        self.cmd.rate = 0.0
        self.cmd.roll = 0.0
        self.cmd.pitch = 0.0
        self.cmd.yaw = 0.0
        self.cmd.z = 0.0
        self.cmd.motion = 'Stop'
        self.cmd.movement = 'Stepping'
        self.cmd.pose_cmd = 'Normal'
        self.cmd.imu_auto_pose = False

        self.estop = False
        self.last_buttons = []

        publish_hz = float(self.get_parameter('publish_hz').value)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_heartbeat)

        self.get_logger().info(
            'STARTING NODE: Teleoperation (ROS2) | '
            '/joy -> /quattro/cmd_raw + /joybuttons'
        )

    def _dz(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def _rising(self, msg, idx):
        cur = button(msg, idx)
        prev = self.last_buttons[idx] if 0 <= idx < len(self.last_buttons) else 0
        return cur == 1 and prev == 0

    def _joy_cb(self, msg):
        # Toggle controls on rising edge.
        if self._rising(msg, self.button_estop):
            self.estop = not self.estop
            self.get_logger().warn(
                'Joystick E-STOP ON' if self.estop else 'Joystick E-STOP OFF'
            )

        if self._rising(msg, self.button_mode) and not self.estop:
            self.cmd.movement = (
                'Viewing' if self.cmd.movement == 'Stepping' else 'Stepping'
            )

        if self._rising(msg, self.button_motion) and not self.estop:
            self.cmd.motion = 'Go' if self.cmd.motion != 'Go' else 'Stop'

        if self.estop:
            self.cmd.x_velocity = 0.0
            self.cmd.y_velocity = 0.0
            self.cmd.rate = 0.0
            self.cmd.motion = 'Stop'
        else:
            self.cmd.x_velocity = (
                self._dz(axis(msg, self.axis_forward)) * self.linear_scale
            )
            self.cmd.y_velocity = (
                self._dz(axis(msg, self.axis_lateral)) * self.linear_scale
            )
            self.cmd.rate = self._dz(axis(msg, self.axis_yaw)) * self.yaw_scale

            moving = (
                abs(self.cmd.x_velocity) > 0.0
                or abs(self.cmd.y_velocity) > 0.0
                or abs(self.cmd.rate) > 0.0
            )
            if moving:
                self.cmd.motion = 'Go'

        jb = JoyButtons()
        # Preserve the known up/down auxiliary command semantics.
        if button(msg, self.button_clearance_up):
            jb.updown = 1
        elif button(msg, self.button_clearance_down):
            jb.updown = -1
        else:
            jb.updown = 0
        self.jb_pub.publish(jb)

        self.last_buttons = list(msg.buttons)

    def _publish_heartbeat(self):
        # Continuous heartbeat lets quattro_sm distinguish "no key movement"
        # from "input process disappeared".
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = QuattroTeleop()
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
