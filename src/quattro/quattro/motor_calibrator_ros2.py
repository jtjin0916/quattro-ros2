#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""단독 실행 모터 캘리브레이터.

mit_publisher / pwm_publisher 없이 하드웨어를 직접 제어한다.

  gim  ->  MITBackend  : MIT CAN (can0) 직접 제어
  quattro -> PWMBackend  : PCA9685 PWM (I2C) 직접 제어

YAML 선택:
  1) 시작 메뉴에서 [1]/[2] 선택
  2) --ros-args -p calib_yaml:=gim_servo_calib.yaml 으로 직접 지정

키 조작:
  숫자 + Enter : 모터 ID 선택 (0-11)
  i / k        : +1 / -1 도
  l            : YAML 값과 비교해 새 캘리브레이션 값 출력
  x            : 모터 선택 해제
  q / ESC      : 종료
"""

import atexit
import math
import os
import re
import select
import signal
import struct
import sys
import termios
import time
import tty

from ament_index_python.packages import get_package_share_directory
import can
import rclpy
from rclpy.node import Node
import yaml


# ── CAN / MIT 프로토콜 상수 ──────────────────────────────────────────────────
CMD_SET_AXIS_STATE = 0x007
CMD_MIT_CONTROL = 0x008
CMD_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_LIMITS = 0x00F
CMD_CLEAR_ERRORS = 0x018

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_POSITION = 3
INPUT_MODE_MIT = 9

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -65.0, 65.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -50.0, 50.0

# ── 기타 상수 ────────────────────────────────────────────────────────────────
MOTOR_NAMES = [
    'FL-S', 'FL-E', 'FL-W',
    'FR-S', 'FR-E', 'FR-W',
    'BL-S', 'BL-E', 'BL-W',
    'BR-S', 'BR-E', 'BR-W',
]

CALIB_YAMLS = {
    'quattro': 'quattro_servo_calib.yaml',
    'gim':  'gim_servo_calib.yaml',
}

DEFAULT_DIRS = [-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]


# ── 터미널 입력 ──────────────────────────────────────────────────────────────
class RawTTY:
    """원시 터미널 입력 핸들러."""

    def __init__(self, path='/dev/tty'):
        self.use_stdin = False
        try:
            self.f = open(path, 'rb', buffering=0)
            self.fd = self.f.fileno()
        except Exception:
            if sys.stdin.isatty():
                self.f = sys.stdin.buffer
                self.fd = sys.stdin.fileno()
                self.use_stdin = True
            else:
                raise RuntimeError('No TTY available. Use a real terminal or "ssh -t".')
        self.orig = termios.tcgetattr(self.fd)
        self.active = False

    def enter(self):
        if not self.active:
            tty.setraw(self.fd)
            self.active = True

    def restore(self):
        if self.active:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.orig)
            self.active = False
        if not self.use_stdin:
            try:
                self.f.close()
            except Exception:
                pass

    def ready(self, timeout=0):
        return self.f in select.select([self.f], [], [], timeout)[0]

    def getch(self):
        return self.f.read(1).decode(errors='ignore')


# ── YAML 로더 ─────────────────────────────────────────────────────────────────
def load_calib_from_yaml(yaml_filename, pkg_name='quattro'):
    """지정한 YAML 에서 캘리브레이션 파라미터를 모두 읽어온다."""
    directions = list(DEFAULT_DIRS)
    offsets = [0.0] * 12
    neutral_us = [1500] * 12
    us_per_rad = 636.0

    try:
        pkg_share = get_package_share_directory(pkg_name)
        yaml_path = os.path.join(pkg_share, 'config', yaml_filename)

        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}

            params = cfg.get('/**', {}).get('ros__parameters', {}).get('calibration', cfg)
            directions = params.get('direction', directions)
            offsets = params.get('offset_deg', offsets)
            neutral_us = params.get('neutral_us', neutral_us)
            us_per_rad = float(params.get('us_per_rad', us_per_rad))
            print(f'YAML 로드: {yaml_path}')
        else:
            print(f'[WARN] YAML 없음: {yaml_path}  ->  기본값 사용')
    except Exception as e:
        print(f'[WARN] YAML 로드 실패: {e}  ->  기본값 사용')

    return directions, offsets, neutral_us, us_per_rad


def select_yaml_interactively():
    """시작 시 로봇/YAML 을 선택하는 메뉴. (robot_key, yaml_filename) 반환."""
    entries = list(CALIB_YAMLS.items())
    print('\n캘리브레이션 YAML 선택:')
    for idx, (key, fname) in enumerate(entries, 1):
        tag = '  [기본값]' if idx == 1 else ''
        print(f'  [{idx}] {key:<6} ->  {fname}{tag}')
    print()

    while True:
        try:
            raw = input(f'선택 [1-{len(entries)}, Enter=1]: ').strip()
        except EOFError:
            raw = ''

        if raw == '':
            return entries[0]

        if raw.isdigit() and 1 <= int(raw) <= len(entries):
            return entries[int(raw) - 1]

        print(f'  1 ~ {len(entries)} 사이의 숫자를 입력하세요.')


# ── 하드웨어 백엔드 ──────────────────────────────────────────────────────────
class MITBackend:
    """GIM 로봇용 MIT CAN 백엔드."""

    def __init__(self, directions, offsets, interface0='can0', interface1='can1',
                 bitrate=500000, kp=60.0, kd=0.8, current_lim=20.0):
        self.dir = directions
        self.offset_deg = offsets
        self.interface0 = interface0
        self.interface1 = interface1
        self.bitrate = bitrate
        self.kp = kp
        self.kd = kd
        self.current_lim = current_lim
        self.bus0 = None
        self.bus1 = None

    def _bus_for(self, node_id):
        return self.bus0 if node_id < 6 else self.bus1

    @staticmethod
    def _f2u(x, x_min, x_max, bits):
        x = max(x_min, min(x_max, x))
        return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

    def _send_can(self, node_id, cmd_id, data=b''):
        bus = self._bus_for(node_id)
        if bus is None:
            return
        msg = can.Message(
            arbitration_id=(node_id << 5) | cmd_id,
            data=data,
            is_extended_id=False,
        )
        bus.send(msg)

    def _send_mit(self, node_id, p_rad, v_rad=0.0, kp=None, kd=None, t_ff=0.0):
        kp = kp if kp is not None else self.kp
        kd = kd if kd is not None else self.kd
        p_int = self._f2u(p_rad, P_MIN, P_MAX, 16)
        v_int = self._f2u(v_rad, V_MIN, V_MAX, 12)
        kp_int = self._f2u(kp, KP_MIN, KP_MAX, 12)
        kd_int = self._f2u(kd, KD_MIN, KD_MAX, 12)
        t_int = self._f2u(t_ff, T_MIN, T_MAX, 12)
        data = bytearray(8)
        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF
        data[2] = (v_int >> 4) & 0xFF
        data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
        data[4] = kp_int & 0xFF
        data[5] = (kd_int >> 4) & 0xFF
        data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
        data[7] = t_int & 0xFF
        self._send_can(node_id, CMD_MIT_CONTROL, data)

    def _open_bus(self, interface):
        try:
            return can.interface.Bus(
                channel=interface, bustype='socketcan', bitrate=self.bitrate,
            )
        except Exception as e:
            raise RuntimeError(f'CAN 버스 열기 실패 ({interface}): {e}') from e

    def setup(self):
        print(f'CAN 버스 초기화 중 ({self.interface0}, {self.interface1})...')
        self.bus0 = self._open_bus(self.interface0)
        self.bus1 = self._open_bus(self.interface1)

        for ch in range(12):
            self._send_can(ch, CMD_CLEAR_ERRORS)
            time.sleep(0.02)
            self._send_can(ch, CMD_SET_LIMITS,
                           struct.pack('<ff', 50.0, self.current_lim))
            time.sleep(0.02)
            self._send_can(ch, CMD_SET_CONTROLLER_MODE,
                           struct.pack('<II', CONTROL_MODE_POSITION, INPUT_MODE_MIT))
            time.sleep(0.02)
            self._send_can(ch, CMD_SET_AXIS_STATE,
                           struct.pack('<I', AXIS_STATE_CLOSED_LOOP))
            time.sleep(0.02)
        print('모터 초기화 완료.')

    def _recv_encoder_angles(self, angles, duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            for bus in (self.bus0, self.bus1):
                msg = bus.recv(timeout=0.005)
                if msg is None:
                    continue
                node_id = msg.arbitration_id >> 5
                cmd_id = msg.arbitration_id & 0x01F
                if cmd_id == CMD_ENCODER_ESTIMATES and len(msg.data) == 8:
                    if 0 <= node_id < 12:
                        pos_turns, _ = struct.unpack('<ff', msg.data)
                        pos_rad = pos_turns * 2.0 * math.pi
                        raw_deg = math.degrees(pos_rad) / 8.0 / self.dir[node_id]
                        angles[node_id] = round(raw_deg - self.offset_deg[node_id], 1)

    def sync(self):
        """엔코더를 읽어 캘리브레이션 공간 각도(deg)를 반환한다."""
        print('엔코더 동기화 중 (1초)...')
        for node_id in range(12):
            req = can.Message(
                arbitration_id=(node_id << 5) | CMD_ENCODER_ESTIMATES,
                is_extended_id=False,
                is_remote_frame=True,
            )
            try:
                self._bus_for(node_id).send(req)
            except Exception:
                pass

        angles = [0.0] * 12
        self._recv_encoder_angles(angles, duration=1.0)
        return angles

    def move(self, motor_id, angle_deg):
        v_deg = self.dir[motor_id] * (angle_deg + self.offset_deg[motor_id])
        self._send_mit(motor_id, math.radians(v_deg))

    def calibration_values(self, motor_angles):
        """새 offset_deg 제안값과 현재 YAML 값을 반환한다."""
        new_vals = [round(motor_angles[i] + self.offset_deg[i], 1) for i in range(12)]
        return 'offset_deg', new_vals, list(self.offset_deg)

    def shutdown(self):
        for ch in range(12):
            try:
                self._send_can(ch, CMD_SET_AXIS_STATE,
                               struct.pack('<I', AXIS_STATE_IDLE))
            except Exception:
                pass
        for bus in (self.bus0, self.bus1):
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception:
                    pass
        self.bus0 = None
        self.bus1 = None


class PWMBackend:
    """QUATTRO 로봇용 PCA9685 PWM 백엔드."""

    PULSE_MIN = 500
    PULSE_MAX = 2500
    FREQ = 50

    def __init__(self, directions, offsets, neutral_us, us_per_rad):
        self.dir = directions
        self.offset_deg = offsets
        self.neutral_us = neutral_us
        self.us_per_rad = us_per_rad
        self.motor_map = None

    def setup(self):
        print('PCA9685 초기화 중 (I2C 0x40/0x41)...')
        try:
            import board    # noqa: F401 (RPi 전용 lazy import)
            import busio
            from adafruit_pca9685 import PCA9685
        except ImportError as e:
            raise RuntimeError(f'PCA9685 라이브러리 없음: {e}') from e

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            pca0 = PCA9685(i2c, address=0x40)
            pca0.frequency = self.FREQ
            pca1 = PCA9685(i2c, address=0x41)
            pca1.frequency = self.FREQ
        except Exception as e:
            raise RuntimeError(f'PCA9685 초기화 실패: {e}') from e

        self.motor_map = {
            0: (pca0, 0),  1: (pca0, 1),  2:  (pca0, 2),
            3: (pca0, 3),  4: (pca0, 4),  5:  (pca0, 5),
            6: (pca1, 0),  7: (pca1, 1),  8:  (pca1, 2),
            9: (pca1, 3), 10: (pca1, 4), 11: (pca1, 5),
        }
        print('PCA9685 초기화 완료.')

    def sync(self):
        """PWM 서보에는 엔코더가 없으므로 0 도로 초기화한다."""
        return [0.0] * 12

    def move(self, motor_id, angle_deg):
        rad = math.radians(angle_deg)
        pulse_us = int(round(
            self.neutral_us[motor_id] + self.dir[motor_id] * self.us_per_rad * rad
        ))
        pulse_us = max(self.PULSE_MIN, min(self.PULSE_MAX, pulse_us))
        period_us = 1e6 / self.FREQ
        duty = max(0, min(65535, int(65535 * pulse_us / period_us)))
        pca, ch = self.motor_map[motor_id]
        pca.channels[ch].duty_cycle = duty

    def calibration_values(self, motor_angles):
        """새 neutral_us 제안값과 현재 YAML 값을 반환한다."""
        new_vals = [
            int(round(
                self.neutral_us[i]
                + self.dir[i] * self.us_per_rad * math.radians(motor_angles[i])
            ))
            for i in range(12)
        ]
        return 'neutral_us', new_vals, list(self.neutral_us)

    def shutdown(self):
        pass  # PCA9685 는 마지막 위치 유지, 별도 종료 불필요


# ── 상태 표시 ─────────────────────────────────────────────────────────────────
def print_status(current_id, motor_angles, input_buffer=''):
    sys.stdout.write('\x1b[2K\r')
    if current_id is None:
        sys.stdout.write(
            f'Motor ID (0-11) 입력 후 Enter  '
            f'[l]=캘리브 확인  [q]=종료  : {input_buffer}'
        )
    else:
        angle = motor_angles[current_id]
        sys.stdout.write(
            f'Motor {current_id:>2} [{MOTOR_NAMES[current_id]}]  '
            f'각도: {angle:>8.1f} deg   '
            f'[i/k]=+-1deg  [x]=모터변경  [l]=캘리브  [q]=종료'
        )
    sys.stdout.flush()


# ── ROS2 노드 ─────────────────────────────────────────────────────────────────
class ManualMotorCalibrator(Node):
    """단독 실행 인터랙티브 모터 캘리브레이터."""

    def __init__(self, robot_key, yaml_filename):
        super().__init__('manual_motor_calibrator')

        self.declare_parameter('calib_pkg', 'quattro')
        self.declare_parameter('can_interface_0', 'can0')
        self.declare_parameter('can_interface_1', 'can1')
        self.declare_parameter('bitrate', 500000)
        self.declare_parameter('calib_yaml', yaml_filename)

        calib_pkg = self.get_parameter('calib_pkg').value
        can_interface_0 = self.get_parameter('can_interface_0').value
        can_interface_1 = self.get_parameter('can_interface_1').value
        bitrate = int(self.get_parameter('bitrate').value)

        self.active_yaml = yaml_filename
        dirs, offsets, neutral_us, us_per_rad = load_calib_from_yaml(yaml_filename, calib_pkg)

        if robot_key == 'gim':
            self.backend = MITBackend(
                dirs, offsets,
                interface0=can_interface_0, interface1=can_interface_1,
                bitrate=bitrate,
            )
        else:
            self.backend = PWMBackend(dirs, offsets, neutral_us, us_per_rad)

        self.motor_angles = [0.0] * 12
        self.current_motor_id = None
        self.input_buffer = ''
        self.running = True

        self.ttydev = RawTTY()
        atexit.register(self.ttydev.restore)
        signal.signal(signal.SIGINT, lambda _s, _f: setattr(self, 'running', False))
        signal.signal(signal.SIGTERM, lambda _s, _f: setattr(self, 'running', False))

    def _show_calibration(self):
        """엔코더를 재동기화하고 캘리브레이션 제안값을 출력한다. [s]로 YAML 자동 저장."""
        # MITBackend: 현재 물리 위치를 live 엔코더로 재동기화
        if isinstance(self.backend, MITBackend) and self.backend.bus0 is not None:
            sys.stdout.write('\r\n\033[93m엔코더 재동기화 중...\033[0m\r\n')
            sys.stdout.flush()
            for node_id in range(12):
                req = can.Message(
                    arbitration_id=(node_id << 5) | CMD_ENCODER_ESTIMATES,
                    is_extended_id=False,
                    is_remote_frame=True,
                )
                try:
                    self.backend._bus_for(node_id).send(req)
                except Exception:
                    pass
            live_angles = list(self.motor_angles)
            self.backend._recv_encoder_angles(live_angles, duration=0.5)
            self.motor_angles = live_angles

        field, new_vals, old_vals = self.backend.calibration_values(self.motor_angles)
        use_float = isinstance(new_vals[0], float)

        sys.stdout.write('\r\n\033[96m======== 캘리브레이션 값 비교 ========\033[0m\r\n')
        sys.stdout.write(
            f'  {"ID":<3} {"이름":<7} {"현재제어각":>10}  '
            f'{"현재 YAML":>12}  {"-> 제안값":>12}\r\n'
        )
        for i in range(12):
            old_v, new_v = old_vals[i], new_vals[i]
            marker = '  <- 변경' if abs(new_v - old_v) > 0.5 else ''
            if use_float:
                old_str, new_str = f'{old_v:>12.1f}', f'{new_v:>12.1f}'
            else:
                old_str, new_str = f'{old_v:>12d}', f'{new_v:>12d}'
            sys.stdout.write(
                f'  {i:<3} {MOTOR_NAMES[i]:<7} {self.motor_angles[i]:>10.1f} deg  '
                f'{old_str}  {new_str}{marker}\r\n'
            )

        if use_float:
            vals_str = '[' + ', '.join(f'{v:.1f}' for v in new_vals) + ']'
        else:
            vals_str = '[' + ', '.join(str(v) for v in new_vals) + ']'

        sys.stdout.write(f'\r\n\033[92m  {field}: {vals_str}\033[0m\r\n')
        sys.stdout.write(f'\r\n  [s]=YAML 저장  [다른키]=취소  : ')
        sys.stdout.flush()

        while rclpy.ok() and self.running:
            if self.ttydev.ready(0.05):
                key = self.ttydev.getch()
                if key in ('s', 'S'):
                    sys.stdout.write('\r\n')
                    self._save_calib(field, new_vals)
                break

        print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

    def _save_calib(self, field, new_vals):
        """새 캘리브레이션 값을 소스/인스톨 YAML에 기록하고 현재 세션 offset을 갱신한다."""
        use_float = isinstance(new_vals[0], float)
        if use_float:
            vals_str = '[' + ', '.join(f'{v:.1f}' for v in new_vals) + ']'
        else:
            vals_str = '[' + ', '.join(str(v) for v in new_vals) + ']'

        pattern = re.compile(rf'^(\s*{re.escape(field)}:\s*).*$', re.MULTILINE)

        def update_file(path):
            try:
                with open(path, 'r') as f:
                    content = f.read()
                new_content, count = pattern.subn(
                    lambda m: f'{m.group(1)}{vals_str}',
                    content,
                )
                if count == 0:
                    return False, f"'{field}' 항목을 찾지 못했습니다"
                with open(path, 'w') as f:
                    f.write(new_content)
                return True, None
            except PermissionError:
                return False, '쓰기 권한 없음'
            except Exception as exc:
                return False, str(exc)

        try:
            pkg_share = get_package_share_directory('quattro')
            install_path = os.path.join(pkg_share, 'config', self.active_yaml)
            # colcon 워크스페이스: install/<pkg>/share/<pkg> → 4단계 위가 workspace 루트
            workspace = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
            )
            src_path = os.path.join(workspace, 'src', 'quattro', 'config', self.active_yaml)
        except Exception as exc:
            sys.stdout.write(f'\033[91m경로 계산 실패: {exc}\033[0m\r\n')
            return

        for label, path in [('소스', src_path), ('인스톨', install_path)]:
            if not os.path.exists(path):
                sys.stdout.write(f'  \033[93m[{label}] 파일 없음: {path}\033[0m\r\n')
                continue
            ok, err = update_file(path)
            if ok:
                sys.stdout.write(f'  \033[92m✓ [{label}] 저장: {path}\033[0m\r\n')
            else:
                sys.stdout.write(f'  \033[91m✗ [{label}] 실패: {err}\033[0m\r\n')

        # 현재 세션 offset 갱신 → motor_angles가 모두 0.0도가 됨
        if field == 'offset_deg':
            self.backend.offset_deg = list(new_vals)
            self.motor_angles = [0.0] * 12
            sys.stdout.write('\033[96m  세션 offset 갱신 완료 — 모든 표시 각도 → 0.0 deg\033[0m\r\n')

        sys.stdout.flush()

    def run(self):
        # 하드웨어 초기화는 raw TTY 진입 전에 (일반 출력 사용)
        self.backend.setup()
        self.motor_angles = self.backend.sync()

        self.ttydev.enter()

        angles_str = '[' + ', '.join(f'{a:.1f}' for a in self.motor_angles) + ']'
        sys.stdout.write(f'\r\n초기 위치: {angles_str}\r\n')
        print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

        while rclpy.ok() and self.running:
            if not self.ttydev.ready(0.05):
                continue

            c = self.ttydev.getch()

            if c in ('\x1b', 'q', 'Q', '\x03'):
                self.running = False
                break

            if c in ('l', 'L'):
                self._show_calibration()
                continue

            if self.current_motor_id is None:
                if c.isdigit():
                    self.input_buffer += c
                    print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

                elif c in ('\r', '\n') and self.input_buffer:
                    try:
                        mid = int(self.input_buffer)
                        if 0 <= mid <= 11:
                            self.current_motor_id = mid
                        else:
                            raise ValueError()
                    except ValueError:
                        sys.stdout.write(
                            f'\r\n  잘못된 ID: "{self.input_buffer}" (0-11)\r\n'
                        )
                    self.input_buffer = ''
                    print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

                elif c == '\x7f' and self.input_buffer:
                    self.input_buffer = self.input_buffer[:-1]
                    print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

            else:
                if c in ('i', 'I', ']', '+', '='):
                    self.motor_angles[self.current_motor_id] += 1.0
                    self.backend.move(self.current_motor_id,
                                      self.motor_angles[self.current_motor_id])
                    print_status(self.current_motor_id, self.motor_angles)

                elif c in ('k', 'K', '[', '-', '_'):
                    self.motor_angles[self.current_motor_id] -= 1.0
                    self.backend.move(self.current_motor_id,
                                      self.motor_angles[self.current_motor_id])
                    print_status(self.current_motor_id, self.motor_angles)

                elif c in ('x', 'X'):
                    self.current_motor_id = None
                    self.input_buffer = ''
                    print_status(self.current_motor_id, self.motor_angles, self.input_buffer)

        self.backend.shutdown()
        sys.stdout.write('\r\nCalibrator 종료.\r\n')


# ── 진입점 ────────────────────────────────────────────────────────────────────
def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)

    # --ros-args -p calib_yaml:=xxx 가 지정된 경우 메뉴 건너뜀
    pre_yaml = ''
    for a in raw_args:
        if 'calib_yaml:=' in a:
            pre_yaml = a.split('calib_yaml:=', 1)[1]
            break

    if pre_yaml:
        robot_key = 'gim' if 'gim' in pre_yaml else 'quattro'
        chosen_yaml = pre_yaml
    else:
        robot_key, chosen_yaml = select_yaml_interactively()

    rclpy.init(args=args)
    node = None
    try:
        node = ManualMotorCalibrator(robot_key, chosen_yaml)
        node.run()
    except RuntimeError as e:
        sys.stderr.write(f'\nERROR: {e}\n')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
