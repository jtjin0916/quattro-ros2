#!/usr/bin/env python3
"""ROS2 <-> Teensy serial bridge for Quattro.

Host -> Teensy command frame
  SOF(AA 55) | type=01 | seq(u16 LE) | count(u8) |
  repeated: motor_id(u8), q_des(f32), qdot_des(f32), kp(f32), kd(f32), tau_ff(f32) |
  crc16-ccitt(u16 LE)

Teensy -> Host feedback frame
  SOF(AA 55) | type=02 | seq(u16 LE) | count(u8) |
  repeated: motor_id(u8), valid(u8), q(f32), qdot(f32), torque(f32) |
  crc16-ccitt(u16 LE)

Only feedback marked valid/fresh by Teensy is used.  The bridge keeps the latest
valid state per joint and publishes /joint_states once all 12 joints have been
observed at least once.  This prevents a disconnected CAN bus from appearing as
12 zero-angle motors in RViz.
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quattro_msgs.msg import JointCommand

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

SOF = b'\xAA\x55'
TYPE_COMMAND = 0x01
TYPE_FEEDBACK = 0x02
CMD_ITEM = struct.Struct('<Bfffff')
FB_ITEM = struct.Struct('<BBfff')
HEADER = struct.Struct('<2sBHB')
CRC = struct.Struct('<H')

JOINT_NAMES = [
    'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
    'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
    'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
    'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
]
NAME_TO_ID = {name: i for i, name in enumerate(JOINT_NAMES)}


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_command_frame(msg: JointCommand, seq: int) -> bytes:
    count = len(msg.name)
    fields = [msg.position, msg.velocity, msg.kp, msg.kd, msg.torque_ff]
    if count == 0 or count > 12 or any(len(v) != count for v in fields):
        raise ValueError('JointCommand arrays must have the same non-zero length (max 12).')

    body = bytearray(HEADER.pack(SOF, TYPE_COMMAND, seq & 0xFFFF, count))
    seen = set()
    for i, name in enumerate(msg.name):
        if name not in NAME_TO_ID:
            raise ValueError(f'Unknown joint name: {name}')
        motor_id = NAME_TO_ID[name]
        if motor_id in seen:
            raise ValueError(f'Duplicate joint name: {name}')
        seen.add(motor_id)
        body += CMD_ITEM.pack(
            motor_id,
            float(msg.position[i]), float(msg.velocity[i]),
            float(msg.kp[i]), float(msg.kd[i]), float(msg.torque_ff[i]),
        )
    body += CRC.pack(crc16_ccitt(body))
    return bytes(body)


class QuattroSerialBridge(Node):
    def __init__(self):
        super().__init__('quattro_serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('serial_timeout_sec', 0.01)
        self.declare_parameter('feedback_stale_sec', 0.1)

        if serial is None:
            raise RuntimeError('pyserial is required: sudo apt install python3-serial')

        port = str(self.get_parameter('port').value)
        baudrate = int(self.get_parameter('baudrate').value)
        timeout = float(self.get_parameter('serial_timeout_sec').value)
        self.feedback_stale_sec = float(self.get_parameter('feedback_stale_sec').value)

        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.seq = 0
        self.rx_buffer = bytearray()
        self.lock = threading.Lock()
        self.running = True

        self.pos = [0.0] * 12
        self.vel = [0.0] * 12
        self.effort = [0.0] * 12
        self.ever_valid = [False] * 12
        self.last_valid = [0.0] * 12
        self.last_feedback_time = 0.0

        self.cmd_sub = self.create_subscription(
            JointCommand, '/quattro/joint_command', self._on_command, 1
        )
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_timer = self.create_timer(1.0, self._status_tick)

        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'Serial bridge ready: {port} @ {baudrate} baud')

    def _on_command(self, msg: JointCommand):
        try:
            frame = build_command_frame(msg, self.seq)
            self.seq = (self.seq + 1) & 0xFFFF
            with self.lock:
                self.ser.write(frame)
        except Exception as exc:
            self.get_logger().error(f'Command serial TX failed: {exc}', throttle_duration_sec=1.0)

    def _reader_loop(self):
        while self.running:
            try:
                data = self.ser.read(max(1, self.ser.in_waiting))
                if data:
                    self.rx_buffer.extend(data)
                    self._drain_frames()
            except Exception as exc:
                if self.running:
                    self.get_logger().error(f'Serial RX failed: {exc}', throttle_duration_sec=1.0)
                time.sleep(0.02)

    def _drain_frames(self):
        while True:
            start = self.rx_buffer.find(SOF)
            if start < 0:
                if len(self.rx_buffer) > 1:
                    del self.rx_buffer[:-1]
                return
            if start:
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < HEADER.size:
                return

            _, frame_type, _, count = HEADER.unpack_from(self.rx_buffer)
            if frame_type != TYPE_FEEDBACK or count > 12:
                del self.rx_buffer[0]
                continue

            frame_len = HEADER.size + count * FB_ITEM.size + CRC.size
            if len(self.rx_buffer) < frame_len:
                return
            frame = bytes(self.rx_buffer[:frame_len])
            del self.rx_buffer[:frame_len]

            expected = CRC.unpack_from(frame, frame_len - CRC.size)[0]
            if crc16_ccitt(frame[:-CRC.size]) != expected:
                self.get_logger().warn('Serial feedback CRC mismatch', throttle_duration_sec=1.0)
                continue
            self._consume_feedback(frame, count)

    def _consume_feedback(self, frame: bytes, count: int):
        now = time.monotonic()
        offset = HEADER.size
        any_valid = False
        for _ in range(count):
            motor_id, valid, q, qd, tau = FB_ITEM.unpack_from(frame, offset)
            offset += FB_ITEM.size
            if motor_id >= 12 or not valid:
                continue
            self.pos[motor_id] = float(q)
            self.vel[motor_id] = float(qd)
            self.effort[motor_id] = float(tau)
            self.ever_valid[motor_id] = True
            self.last_valid[motor_id] = now
            any_valid = True

        if any_valid:
            self.last_feedback_time = now

        # Wait until every physical/virtual motor has been seen at least once.
        if not all(self.ever_valid):
            return

        # Do not publish a misleading complete robot state when either CAN bus is stale.
        if any(now - stamp > self.feedback_stale_sec for stamp in self.last_valid):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.pos)
        msg.velocity = list(self.vel)
        msg.effort = list(self.effort)
        self.joint_state_pub.publish(msg)

    def _status_tick(self):
        missing = [JOINT_NAMES[i] for i, ok in enumerate(self.ever_valid) if not ok]
        if missing:
            self.get_logger().warn(
                f'Waiting for valid motor feedback ({len(missing)} missing).',
                throttle_duration_sec=2.0,
            )
            return
        now = time.monotonic()
        stale = [JOINT_NAMES[i] for i, stamp in enumerate(self.last_valid)
                 if now - stamp > self.feedback_stale_sec]
        if stale:
            self.get_logger().warn(
                f'Motor feedback stale: {len(stale)} joint(s).',
                throttle_duration_sec=1.0,
            )

    def destroy_node(self):
        self.running = False
        try:
            if hasattr(self, 'reader'):
                self.reader.join(timeout=0.2)
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuattroSerialBridge()
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
