# NOTE: Legacy control logic retained; project/topic naming normalized to Quattro during refactor.
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mit_publisher_ros2.py  —  GIM6010-8 CAN MIT 제어 노드

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GIM6010-8 MIT 제어 공식
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  τ_des = kp × (pos_des − pos_now) + kd × (vel_des − vel_now) + torque_ff

  변수 단위:
    pos_des / pos_now  : rad          유효 범위 −12.5 ~ +12.5
    vel_des / vel_now  : rad/s        유효 범위 −65.0 ~ +65.0
    kp                 : Nm/rad       유효 범위  0.0  ~ 500.0
    kd                 : Nm/(rad/s)   유효 범위  0.0  ~   5.0
    torque_ff          : Nm           유효 범위 −50.0 ~ +50.0
    τ_des              : Nm

  주의: pos_now, vel_now 피드백은 모터 내부 드라이버가 읽어 공식을 자동 계산합니다.
        호스트는 (pos_des, vel_des, kp, kd, torque_ff) 5 값을 CAN 으로 전송하기만 합니다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAN MIT 패킷 — 실수 → 정수 변환 공식 (8 바이트, TX)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  position  (16 bit): int = (pos  + 12.5) × 65535 / 25
  velocity  (12 bit): int = (vel  + 65.0) × 4095  / 130
  kp        (12 bit): int =  kp           × 4095  / 500
  kd        (12 bit): int =  kd           × 4095  / 5
  torque    (12 bit): int = (torq + 50.0) × 4095  / 100

  TX 바이트 구조:
    data[0] = pos[15:8]
    data[1] = pos[7:0]
    data[2] = vel[11:4]
    data[3] = vel[3:0] | kp[11:8]
    data[4] = kp[7:0]
    data[5] = kd[11:4]
    data[6] = kd[3:0]  | torq[11:8]
    data[7] = torq[7:0]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MIT feedback (0x008 RX) — 정수 → 실수 역변환 (출력축 기준)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  RX 바이트 구조:
    data[0]       = motor_id
    data[1~2]     = position  16bit  → −12.5 ~ +12.5 rad
    data[3]       = velocity  [11:4]
    data[4][7:4]  = velocity  [3:0]
    data[4][3:0]  = torque    [11:8]
    data[5]       = torque    [7:0]  → −50.0 ~ +50.0 Nm

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
초기 실물 테스트 권장값
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  vel_des   = 0.0  (Nm 대신 위치 제어만 사용)
  torque_ff = 0.0  (피드포워드 없이 시작)
  kp        = Isaac Lab stiffness 값(55)부터 시작 → 진동 시 줄임
  kd        = Isaac Lab damping   값(2)부터 시작  → 떨림 시 올림
  current_limit 은 보수적으로 유지 (20 A)
"""

import os
import time
import math
import yaml
import struct
import can
import json

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Float32MultiArray
from quattro_msgs.msg import JointAngles, JointPulse, IMUdata, QuattroCmd


# ─── CAN CMD ID ────────────────────────────────────────
CMD_SET_AXIS_STATE      = 0x007
CMD_MIT_CONTROL         = 0x008
CMD_ENCODER_ESTIMATES   = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_LIMITS          = 0x00F
CMD_CLEAR_ERRORS        = 0x018
CMD_SAVE_CONFIGURATION  = 0x01F

# ─── 모터 AXIS_STATE ───────────────────────────────────
AXIS_STATE_IDLE                       = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE  = 3
AXIS_STATE_MOTOR_CALIBRATION          = 4
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL        = 8

# ─── CONTROL_MODE / INPUT_MODE ─────────────────────────
CONTROL_MODE_VOLTAGE  = 0
CONTROL_MODE_TORQUE   = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3

INPUT_MODE_INACTIVE    = 0
INPUT_MODE_DIRECT      = 1
INPUT_MODE_VEL_RAMP    = 2
INPUT_MODE_POS_FILTER  = 3
INPUT_MODE_TRAP_TRAJ   = 5
INPUT_MODE_TORQUE_RAMP = 6
INPUT_MODE_MIT         = 9

# ─── CAN MIT TX 패킷 물리량 범위 ───────────────────────
P_MIN,  P_MAX  = -12.5, 12.5    # rad
V_MIN,  V_MAX  = -65.0, 65.0    # rad/s
KP_MIN, KP_MAX =   0.0, 500.0   # Nm/rad
KD_MIN, KD_MAX =   0.0,   5.0   # Nm/(rad/s)
T_MIN,  T_MAX  = -50.0, 50.0    # Nm

# GIM6010-8 기어비 — 0x009 encoder fallback 전용 (MIT feedback 은 출력축 기준이므로 불필요)
GIM_GEAR_RATIO = 8.0

# ─── MIT feedback (0x008 RX) 물리량 범위 — 출력축 기준 ─
MIT_POS_MIN, MIT_POS_MAX = -12.5, 12.5   # rad
MIT_VEL_MIN, MIT_VEL_MAX = -65.0, 65.0   # rad/s
MIT_TRQ_MIN, MIT_TRQ_MAX = -50.0, 50.0   # Nm

# feedback_stamp 기준: 이 시간 이상 수신 없으면 stale
FEEDBACK_STALE_SEC = 0.1


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """실수 x 를 [x_min, x_max] 범위에서 bits 비트 정수로 선형 변환.

    16-bit 예시 (position):
        int = (pos + 12.5) × 65535 / 25
    12-bit 예시 (velocity):
        int = (vel + 65.0) × 4095  / 130
    """
    span = x_max - x_min
    x = max(x_min, min(x_max, x))          # clamp
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, x_min: float, x_max: float, bits: int) -> float:
    """정수 value 를 [x_min, x_max] 범위의 실수로 역변환 (float_to_uint 역함수)."""
    span = x_max - x_min
    return value * span / ((1 << bits) - 1) + x_min


def parse_mit_feedback(data: bytes) -> tuple:
    """GIM6010-8 MIT feedback RX 패킷 파싱 (최소 6바이트 필요).

    반환: (motor_id, pos_rad, vel_radps, torque_nm)
    모두 출력축 기준 — 기어비 변환 불필요.
    """
    motor_id  = data[0]
    pos_int   = (data[1] << 8) | data[2]                  # 16bit
    vel_int   = (data[3] << 4) | (data[4] >> 4)           # 12bit
    trq_int   = ((data[4] & 0xF) << 8) | data[5]          # 12bit
    pos_rad   = uint_to_float(pos_int, MIT_POS_MIN, MIT_POS_MAX, 16)
    vel_radps = uint_to_float(vel_int, MIT_VEL_MIN, MIT_VEL_MAX, 12)
    torque_nm = uint_to_float(trq_int, MIT_TRQ_MIN, MIT_TRQ_MAX, 12)
    return motor_id, pos_rad, vel_radps, torque_nm


class MITPub(Node):
    def __init__(self):
        super().__init__('mit_publisher')   

        # ─── 파라미터 선언 ─────────────────────────────
        self.declare_parameter('can_interface_0', 'can0')   # 모터 0-5
        self.declare_parameter('can_interface_1', 'can1')   # 모터 6-11
        self.declare_parameter('bitrate', 500000)
        # kp/kd: launch 파일에서 명시적으로 덮어 씁니다.
        #   Bezier 보행(quattro_commander): mit_kp=60.0, mit_kd=0.8
        #   RL 정책(quattro_rl_interface)       : mit_kp=50.0, mit_kd=0.8
        self.declare_parameter('mit_kp', 60.0)          # Nm/rad
        self.declare_parameter('mit_kd',  0.8)          # Nm/(rad/s)
        self.declare_parameter('current_lim', 20.0)      # A (보수적)
        self.declare_parameter('calib_yaml', '')
        self.declare_parameter('center_on_start', False)
        self.declare_parameter('offset_on_start', False)
        self.declare_parameter('tx_rate_hz', 100.0)
        self.declare_parameter('hold_while_wait', False)
        self.declare_parameter('tx_on_no_cmd', True)
        self.declare_parameter('cmd_timeout_sec', 0.3)
        self.declare_parameter('direct_after_startup', False)

        self.can_interface_0 = self.get_parameter('can_interface_0').value
        self.can_interface_1 = self.get_parameter('can_interface_1').value
        self.bitrate         = int(self.get_parameter('bitrate').value)
        self.kp              = float(self.get_parameter('mit_kp').value)
        self.kd              = float(self.get_parameter('mit_kd').value)
        self.current_lim     = float(self.get_parameter('current_lim').value)
        self.tx_rate_hz      = float(self.get_parameter('tx_rate_hz').value)
        self.hold_while_wait = bool(self.get_parameter('hold_while_wait').value)
        self.tx_on_no_cmd    = bool(self.get_parameter('tx_on_no_cmd').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.direct_after_startup = bool(self.get_parameter('direct_after_startup').value)
        self.center_on_start = bool(self.get_parameter('center_on_start').value)
        self.offset_on_start = bool(self.get_parameter('offset_on_start').value)

        # ─── 퍼블리셔 ──────────────────────────────────
        self.pub_feedback = self.create_publisher(String, '/motor_feedback', 10)

        # ─── 상태 변수 ─────────────────────────────────
        self.target_run_vel     = 0.5
        self.max_velocity_rad_s = 1.0
        self.startup_mode       = True
        self.current_movement   = None

        self.internal_setpoint   = [0.0] * 12   # MIT 송신 목표값 [rad, 캘리브 적용]
        self.target_setpoint_rad = [0.0] * 12   # /quattro/joints_cal 수신 목표 [rad]
        self.last_update_time    = time.time()
        self.last_cmd_time       = 0.0
        self.have_cmd            = False

        self.dir        = [-1, -1, -1, -1,  1,  1,  1, -1, -1,  1,  1,  1]
        self.offset_deg = [0.0] * 12

        self.channels         = list(range(12))
        self.channel_to_joint = list(range(12))

        # ─── 피드백 상태 (MIT feedback 기반) ──────────
        self.display_degs       = [0.0] * 12   # IK 좌표계 각도 [deg], 실측값
        self.feedback_vel_radps = [0.0] * 12   # 관절 속도 [rad/s], dir 보정 후
        self.feedback_pos_rad   = [0.0] * 12   # 출력축 위치 [rad] (MIT 또는 0x009)
        self.feedback_torque_nm = [0.0] * 12   # 출력축 토크 [Nm] (MIT 전용, 0x009 시 0)
        self.feedback_stamp     = [0.0] * 12   # 모터별 마지막 수신 timestamp
        self.feedback_source    = 'NONE'        # 'MIT' | 'ENCODER_009' | 'NONE'

        self.torque_ff_nm = [0.0] * 12   # RL 정책 토크 피드포워드 [Nm]
        self.clamped_deg  = [0.0] * 12
        self.can_errors   = {}           # node_id → (timestamp, err_str)
        self.imu_latest_data    = {}
        self.direct_mode_logged = False

        # 범위 초과 경고 throttle: key → last_warn_time
        self._warn_throttle: dict = {}

        # ─── 캘리브레이션 로드 ─────────────────────────
        try:
            pkg_share = get_package_share_directory('quattro')
        except Exception:
            pkg_share = ''

        default_calib = (
            os.path.join(pkg_share, 'config', 'quattro_servo_calib.yaml') if pkg_share else ''
        )
        param_calib = self.get_parameter('calib_yaml').value
        self.calib_yaml = param_calib if param_calib else default_calib

        if self.calib_yaml and os.path.exists(self.calib_yaml):
            try:
                with open(self.calib_yaml, 'r') as f:
                    yf = yaml.safe_load(f) or {}
                if '/**' in yf:
                    params = yf['/**']['ros__parameters'].get('calibration', {})
                    self.dir        = params.get('direction',  self.dir)
                    self.offset_deg = params.get('offset_deg', self.offset_deg)
                else:
                    self.dir        = yf.get('direction',  self.dir)
                    self.offset_deg = yf.get('offset_deg', self.offset_deg)
                self.get_logger().info('캘리브레이션 로드 완료')
            except Exception as e:
                self.get_logger().warn(f'캘리브레이션 로드 실패: {e}')
        else:
            self.get_logger().warn('기본 캘리브레이션 사용')

        # ─── CAN 초기화 ────────────────────────────────
        self.bus0 = self._init_can_bus(self.can_interface_0)
        self.bus1 = self._init_can_bus(self.can_interface_1)
        if self.bus0 is None or self.bus1 is None:
            self.get_logger().error('CAN 초기화 실패.')
            raise RuntimeError('CAN Init Failed')

        # ─── 모터 초기화 시퀀스 ────────────────────────
        self.get_logger().info(
            f'전류 제한: {self.current_lim} A  |  kp={self.kp}  kd={self.kd}  '
            f'|  {self.can_interface_0}(0-5) / {self.can_interface_1}(6-11)'
        )
        for ch in self.channels:
            self._send_can_raw(ch, CMD_CLEAR_ERRORS);               time.sleep(0.02)
            self._set_limits(ch, self.current_lim, vel_lim=50.0);  time.sleep(0.02)
            self._set_mode_mit(ch);                                 time.sleep(0.02)
            self._set_axis_state(ch, AXIS_STATE_CLOSED_LOOP_CONTROL); time.sleep(0.02)

        self._sync_with_hardware()
        self._publish_feedback()

        if self.center_on_start:
            self.get_logger().info('영점으로 이동 (Raw 0)...')
            for ch in self.channels:
                self.send_mit_packet(ch, 0.0, 0.0, 1.0, 0.015, 0.0)
                self.internal_setpoint[ch]   = 0.0
                self.target_setpoint_rad[ch] = 0.0
                time.sleep(0.10)
        elif self.offset_on_start:
            self.get_logger().info('Offset Home 으로 이동...')
            for ch in self.channels:
                i = self.channel_to_joint[ch]
                v_deg = self.dir[i] * self.offset_deg[i]
                target_rad = math.radians(v_deg)
                self.send_mit_packet(i, target_rad, 0.0, self.kp, self.kd, 0.0)
                self.internal_setpoint[i]   = target_rad
                self.target_setpoint_rad[i] = target_rad
                time.sleep(0.10)

        self.get_logger().info('안정화 대기 (1 s)...')
        time.sleep(1.0)

        # ─── ROS 2 Pub/Sub ─────────────────────────────
        self.create_subscription(JointAngles,       '/quattro/joints_cal', self._cb_joint,     1)
        self.create_subscription(Float32MultiArray, '/quattro/torque_ff',  self._cb_torque_ff, 1)
        self.create_subscription(JointPulse,        '/quattro/pulse',      self._cb_pulse,     1)
        self.create_subscription(IMUdata,           'quattro/imu',         self._cb_imu,       1)
        self.create_subscription(QuattroCmd,           '/quattro_cmd',        self._cb_quattro_cmd,  1)

        self.tx_period = 1.0 / self.tx_rate_hz
        self.tx_timer  = self.create_timer(self.tx_period, self._tx_tick)

        self.get_logger().info(
            f'노드 준비.  TX {self.tx_rate_hz:.0f} Hz '
            f'({self.tx_rate_hz * 12:.0f} CAN frames/s)  '
            f'feedback_source 초기: {self.feedback_source}'
        )

    # ──────────────────────────────────────────────────
    # 종료
    # ──────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('종료 중...')
        try:
            for ch in self.channels:
                self._set_axis_state(ch, AXIS_STATE_IDLE)
            for bus in (getattr(self, 'bus0', None), getattr(self, 'bus1', None)):
                if bus is not None:
                    bus.shutdown()
        except Exception as e:
            self.get_logger().warn(f'종료 경고: {e}')
        return super().destroy_node()

    # ──────────────────────────────────────────────────
    # 경고 throttle 유틸
    # ──────────────────────────────────────────────────
    def _warn_throttled(self, key: str, msg: str, period: float = 2.0):
        """같은 key 의 경고를 period 초에 한 번만 출력."""
        now = time.time()
        if now - self._warn_throttle.get(key, 0.0) >= period:
            self.get_logger().warn(msg)
            self._warn_throttle[key] = now

    # ──────────────────────────────────────────────────
    # CAN 수신 — 운용: MIT feedback (0x008) 전용
    # ──────────────────────────────────────────────────
    def _listen_motor_feedback(self):
        """운용 중 CAN 수신 큐를 논블로킹으로 drain — 0x008 MIT feedback 전용.

        GIM6010-8 은 MIT command(0x008 TX) 수신 시 즉시 0x008 RX 로 회신한다.
        출력축 기준값이므로 기어비 변환 불필요.
        0x009 는 초기 동기화(_sync_with_hardware)에서만 사용하며 여기서는 무시한다.
        """
        for bus in (self.bus0, self.bus1):
            msg = bus.recv(timeout=0)
            while msg is not None:
                node_id = msg.arbitration_id >> 5
                cmd_id  = msg.arbitration_id & 0x01F

                if node_id in self.channels and cmd_id == CMD_MIT_CONTROL and len(msg.data) >= 6:
                    _, pos_rad, vel_radps, torque_nm = parse_mit_feedback(msg.data)
                    i = self.channel_to_joint[node_id]

                    self.feedback_pos_rad[node_id]   = pos_rad
                    self.feedback_vel_radps[node_id] = vel_radps / self.dir[i]
                    self.feedback_torque_nm[node_id] = torque_nm
                    self.feedback_stamp[node_id]     = time.time()
                    self.feedback_source             = 'MIT'

                    raw_deg = (math.degrees(pos_rad) / self.dir[i]) - self.offset_deg[i]
                    self.display_degs[node_id] = raw_deg
                    self.clamped_deg[node_id]  = raw_deg

                    if not self.have_cmd:
                        self.internal_setpoint[node_id]   = pos_rad
                        self.target_setpoint_rad[node_id] = pos_rad

                msg = bus.recv(timeout=0)

    # ──────────────────────────────────────────────────
    # CAN 수신 — 초기 동기화: encoder estimates (0x009) 전용
    # ──────────────────────────────────────────────────
    def _listen_encoder_estimates(self):
        """초기 동기화 전용 수신 함수 — 0x009 encoder estimates 만 처리.

        MIT TX 전이므로 0x008 reply 없음. GIM_GEAR_RATIO 로 출력축 변환 수행.
        """
        for bus in (self.bus0, self.bus1):
            msg = bus.recv(timeout=0)
            while msg is not None:
                node_id = msg.arbitration_id >> 5
                cmd_id  = msg.arbitration_id & 0x01F

                if node_id in self.channels and cmd_id == CMD_ENCODER_ESTIMATES and len(msg.data) == 8:
                    pos_turns, vel_rps = struct.unpack('<ff', msg.data)
                    joint_rad = pos_turns * 2.0 * math.pi / GIM_GEAR_RATIO
                    joint_vel = vel_rps   * 2.0 * math.pi / GIM_GEAR_RATIO
                    i = self.channel_to_joint[node_id]

                    self.feedback_pos_rad[node_id]   = joint_rad
                    self.feedback_vel_radps[node_id] = joint_vel / self.dir[i]
                    self.feedback_torque_nm[node_id] = 0.0
                    self.feedback_stamp[node_id]     = time.time()
                    self.feedback_source             = 'ENCODER_009'

                    raw_deg = (math.degrees(joint_rad) / self.dir[i]) - self.offset_deg[i]
                    self.display_degs[node_id] = raw_deg
                    self.clamped_deg[node_id]  = raw_deg

                    corrected_rad = math.radians(self.dir[i] * (raw_deg + self.offset_deg[i]))
                    self.internal_setpoint[node_id]   = corrected_rad
                    self.target_setpoint_rad[node_id] = corrected_rad

                msg = bus.recv(timeout=0)

    def _sync_with_hardware(self):
        """0x009 요청-응답으로 초기 관절 위치 동기화. MIT TX 시작 전 1회만 호출."""
        self.get_logger().info('인코더 동기화 중 (1 s)...')
        start = time.time()
        while (time.time() - start) < 1.0:
            for ch in self.channels:
                self._send_can_raw(ch, CMD_ENCODER_ESTIMATES)
            time.sleep(0.02)
            self._listen_encoder_estimates()
        self.get_logger().info(
            f'동기화 완료.  source={self.feedback_source}'
        )

    # ──────────────────────────────────────────────────
    # CAN 저수준 유틸
    # ──────────────────────────────────────────────────
    def _init_can_bus(self, channel: str):
        try:
            return can.interface.Bus(channel=channel, bustype='socketcan', bitrate=self.bitrate)
        except Exception as e:
            self.get_logger().error(f'CAN Open 에러 ({channel}): {e}')
            return None

    def _bus_for(self, node_id: int):
        """node_id 0-5 → bus0(can0), 6-11 → bus1(can1)."""
        return self.bus0 if node_id < 6 else self.bus1

    def _send_can_raw(self, node_id: int, cmd_id: int, data: bytes = b''):
        bus = self._bus_for(node_id)
        if bus is None:
            return
        try:
            msg = can.Message(
                arbitration_id=(node_id << 5) | cmd_id,
                data=data,
                is_extended_id=False,
            )
            bus.send(msg)
        except Exception as e:
            self.can_errors[node_id] = (time.time(), str(e))

    def _set_limits(self, node_id: int, current_lim: float, vel_lim: float = 50.0):
        self._send_can_raw(node_id, CMD_SET_LIMITS, struct.pack('<ff', vel_lim, current_lim))

    def _set_mode_mit(self, node_id: int):
        self._send_can_raw(
            node_id, CMD_SET_CONTROLLER_MODE,
            struct.pack('<II', CONTROL_MODE_POSITION, INPUT_MODE_MIT)
        )

    def _set_axis_state(self, node_id: int, state: int):
        self._send_can_raw(node_id, CMD_SET_AXIS_STATE, struct.pack('<I', state))

    # ──────────────────────────────────────────────────
    # MIT 패킷 생성 및 송신
    # ──────────────────────────────────────────────────
    def send_mit_packet(
        self,
        node_id:  int,
        p_rad:    float,    # 목표 위치   [rad]         −12.5 ~ +12.5
        v_rad:    float,    # 목표 속도   [rad/s]       −65.0 ~ +65.0
        kp:       float,    # 위치 이득   [Nm/rad]       0.0  ~ 500.0
        kd:       float,    # 댐핑 이득   [Nm/(rad/s)]   0.0  ~   5.0
        t_ff:     float,    # FF 토크     [Nm]          −50.0 ~ +50.0
    ):
        """GIM6010-8 MIT 제어 CAN 패킷을 만들어 전송.

        τ_des = kp×(p_rad − pos_now) + kd×(v_rad − vel_now) + t_ff

        패킷 구조 (8 바이트):
            data[0] = pos [15:8]
            data[1] = pos [7:0]
            data[2] = vel [11:4]
            data[3] = vel [3:0]  | kp[11:8]
            data[4] = kp  [7:0]
            data[5] = kd  [11:4]
            data[6] = kd  [3:0]  | torq[11:8]
            data[7] = torq[7:0]
        """
        if not (P_MIN  <= p_rad <= P_MAX):
            self._warn_throttled(
                f'pos_{node_id}',
                f'[joint {node_id}] pos={p_rad:.3f} rad  범위 초과 [{P_MIN}, {P_MAX}], clamp'
            )
        if not (V_MIN  <= v_rad <= V_MAX):
            self._warn_throttled(
                f'vel_{node_id}',
                f'[joint {node_id}] vel={v_rad:.3f} rad/s 범위 초과 [{V_MIN}, {V_MAX}], clamp'
            )
        if not (KP_MIN <= kp   <= KP_MAX):
            self._warn_throttled(
                f'kp_{node_id}',
                f'[joint {node_id}] kp={kp:.1f}  범위 초과 [{KP_MIN}, {KP_MAX}], clamp'
            )
        if not (KD_MIN <= kd   <= KD_MAX):
            self._warn_throttled(
                f'kd_{node_id}',
                f'[joint {node_id}] kd={kd:.3f}  범위 초과 [{KD_MIN}, {KD_MAX}], clamp'
            )
        if not (T_MIN  <= t_ff <= T_MAX):
            self._warn_throttled(
                f'torq_{node_id}',
                f'[joint {node_id}] torque={t_ff:.3f} Nm 범위 초과 [{T_MIN}, {T_MAX}], clamp'
            )

        p_int  = float_to_uint(p_rad, P_MIN,  P_MAX,  16)
        v_int  = float_to_uint(v_rad, V_MIN,  V_MAX,  12)
        kp_int = float_to_uint(kp,   KP_MIN, KP_MAX,  12)
        kd_int = float_to_uint(kd,   KD_MIN, KD_MAX,  12)
        t_int  = float_to_uint(t_ff, T_MIN,  T_MAX,   12)

        data = bytearray(8)
        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF
        data[2] = (v_int >> 4) & 0xFF
        data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
        data[4] = kp_int & 0xFF
        data[5] = (kd_int >> 4) & 0xFF
        data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
        data[7] = t_int & 0xFF

        self._send_can_raw(node_id, CMD_MIT_CONTROL, data)

    # ──────────────────────────────────────────────────
    # ROS 2 콜백
    # ──────────────────────────────────────────────────
    def _cb_imu(self, msg: IMUdata):
        self.imu_latest_data = {'r': msg.roll, 'p': msg.pitch}

    def _set_run_velocity_by_movement(self, movement: str):
        is_stepping = (movement == 'Stepping')
        new_vel = 6.0 if is_stepping else 1.0
        if math.isclose(self.target_run_vel, new_vel, abs_tol=1e-9):
            return
        self.target_run_vel = new_vel
        if not self.startup_mode:
            self.max_velocity_rad_s = new_vel

    def _cb_quattro_cmd(self, msg: QuattroCmd):
        self.current_movement = msg.movement
        self._set_run_velocity_by_movement(msg.movement)

    def _cb_joint(self, msg: JointAngles):
        """캘리브레이션이 적용된 관절 각도(degrees) 수신 → 내부 목표(rad)로 변환.

        display_degs 는 _listen_motor_feedback() 의 실측값으로만 갱신한다.
        여기서 명령값으로 덮어쓰지 않는다.
        """
        if self.current_movement is None:
            fallback = 'Viewing' if getattr(msg, 'step_or_view', False) else 'Stepping'
            self._set_run_velocity_by_movement(fallback)

        qraw = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]

        for ch in self.channels:
            i = self.channel_to_joint[ch]
            self.target_setpoint_rad[i] = math.radians(qraw[i])
            self.clamped_deg[i]         = qraw[i]

        self.last_cmd_time = time.time()
        self.have_cmd      = True

    def _cb_torque_ff(self, msg: Float32MultiArray):
        """토크 피드포워드 [Nm × 12] 수신 (RL Stage 4 용)."""
        data = msg.data
        if len(data) >= 12:
            for i in range(12):
                self.torque_ff_nm[i] = self.dir[i] * float(data[i])

    def _cb_pulse(self, jp: JointPulse):
        """개별 관절 디버그 명령 — 즉시 1회 송신."""
        try:
            mid = int(jp.servo_id)
            val = float(jp.servo_deg)

            if mid == 99:
                if val > 0.1:
                    self.target_run_vel     = val
                    self.max_velocity_rad_s = val
                return

            i = mid - 20 if 20 <= mid < 32 else mid
            if i in self.channels:
                v_deg      = self.dir[i] * (val + self.offset_deg[i])
                target_rad = math.radians(v_deg)
                self.target_setpoint_rad[i] = target_rad
                self.internal_setpoint[i]   = target_rad
                self.send_mit_packet(i, target_rad, 0.0, self.kp, self.kd, 0.0)
        except Exception as e:
            self.get_logger().warn(f'Pulse 오류: {e}')
        self._publish_feedback()

    # ──────────────────────────────────────────────────
    # TX 타이머 (100 Hz)
    # ──────────────────────────────────────────────────
    def _tx_tick(self):
        # ① CAN RX drain: MIT 0x008 feedback + 0x009 fallback 파싱
        self._listen_motor_feedback()

        now     = time.time()
        cmd_age = (now - self.last_cmd_time) if self.have_cmd else 999.0

        # ② MIT TX
        if self.have_cmd and cmd_age <= self.cmd_timeout_sec:
            self._step_and_send()
        else:
            if self.tx_on_no_cmd:
                self._send_hold_once()

    def _step_and_send(self):
        """초기 자세는 rate-limit 로 진입하고, 이후 옵션에 따라 direct 송신."""
        now = time.time()
        dt  = now - self.last_update_time
        self.last_update_time = now
        if dt > 0.1:
            dt = self.tx_period

        if self.direct_after_startup and not self.startup_mode and self.current_movement != 'Viewing':
            if not self.direct_mode_logged:
                self.get_logger().info(
                    'RL direct 송신 모드 진입: rate-limit 없이 target_setpoint_rad를 MIT 목표로 전송'
                )
                self.direct_mode_logged = True
            self._send_direct_targets()
            return

        diffs    = [self.target_setpoint_rad[ch] - self.internal_setpoint[ch]
                    for ch in self.channels]
        max_diff = max(abs(d) for d in diffs)

        if self.startup_mode and max_diff < 0.05:
            self.get_logger().info(
                f'초기 자세 도달. 속도 증가: {self.max_velocity_rad_s:.2f} → {self.target_run_vel:.2f}'
            )
            self.max_velocity_rad_s = self.target_run_vel
            self.startup_mode       = False

        max_step = self.max_velocity_rad_s * dt
        is_viewing = (self.current_movement == 'Viewing')

        for ch in self.channels:
            if is_viewing and max_diff > 1e-4:
                # View 모드: 모든 관절이 동시에 목표 도달 — 가장 먼 관절 속도 기준 비례 조절
                sync_step = max_step * (abs(diffs[ch]) / max_diff)
                step = math.copysign(min(sync_step, abs(diffs[ch])), diffs[ch])
            else:
                step = max(-max_step, min(max_step, diffs[ch]))
            next_rad = self.internal_setpoint[ch] + step
            self.internal_setpoint[ch] = next_rad
            self.send_mit_packet(ch, next_rad, 0.0, self.kp, self.kd, self.torque_ff_nm[ch])

        self._publish_feedback()

    def _send_direct_targets(self):
        """RL 보행용: smoothing 없이 최신 목표 관절각을 MIT 목표로 바로 송신."""
        for ch in self.channels:
            target_rad = self.target_setpoint_rad[ch]
            self.internal_setpoint[ch] = target_rad
            self.send_mit_packet(ch, target_rad, 0.0, self.kp, self.kd, self.torque_ff_nm[ch])

        self._publish_feedback()

    def _send_hold_once(self):
        for ch in self.channels:
            #임시테스트용
            self.send_mit_packet(ch, self.internal_setpoint[ch], 0.0, self.kp, self.kd, self.torque_ff_nm[ch])
        self._publish_feedback()
            #self.send_mit_packet(ch, self.internal_setpoint[ch], 0.0, self.kp, self.kd, 0.0)

    # ──────────────────────────────────────────────────
    # 피드백 퍼블리시 (/motor_feedback JSON)
    # ──────────────────────────────────────────────────
    def _publish_feedback(self):
        """관절 상태를 JSON 으로 발행.

        기존 필드 (하위 호환 유지):
            encoders        : IK 좌표계 각도 [deg]  — 실측값 (_listen_motor_feedback 갱신)
            encoder_vels    : 관절 속도 [rad/s]
            applied_targets : 현재 MIT 송신 목표 [rad]

        RL 전용 신규 필드:
            joint_pos_rad   : IK 좌표계 위치 [rad]  (dir/offset 보정 후)
            joint_vel_radps : IK 좌표계 속도 [rad/s]
            joint_torque_nm : 출력축 토크 [Nm]  (MIT feedback 시 유효, ENCODER_009 시 0)
            feedback_source : 'MIT' | 'ENCODER_009' | 'NONE'
            feedback_age_ms : 모터별 마지막 수신 후 경과 [ms]
            stale_flags     : True 이면 해당 관절 observation 신뢰 불가
        """
        now = time.time()
        fb  = {
            # 기존 필드
            'encoders':         [0.0]  * 12,
            'encoder_vels':     [0.0]  * 12,
            'applied_targets':  [0.0]  * 12,
            'offsets':          [0.0]  * 12,
            'errors':           ['OK'] * 12,
            'current_lim':      self.current_lim,
            'kp':               self.kp,
            'kd':               self.kd,
            'state_mode':       'STARTUP(Slow)' if self.startup_mode else 'NORMAL',
            'tx_rate':          int(self.tx_rate_hz),
            'speed_cur':        self.max_velocity_rad_s,
            'speed_tgt':        self.target_run_vel,
            # RL 전용 신규 필드
            'joint_pos_rad':    [0.0]   * 12,
            'joint_vel_radps':  [0.0]   * 12,
            'joint_torque_nm':  [0.0]   * 12,
            'feedback_source':  self.feedback_source,
            'feedback_age_ms':  [0.0]   * 12,
            'stale_flags':      [False] * 12,
        }

        for i in range(12):
            fb['encoders'][i]        = self.display_degs[i]
            fb['encoder_vels'][i]    = self.feedback_vel_radps[i]
            fb['applied_targets'][i] = self.internal_setpoint[i]
            fb['offsets'][i]         = self.offset_deg[i]

            # IK 좌표계 rad 변환: feedback_pos_rad → dir/offset 역적용
            raw_pos       = self.feedback_pos_rad[i]
            corrected_deg = (math.degrees(raw_pos) / self.dir[i]) - self.offset_deg[i]
            fb['joint_pos_rad'][i]   = math.radians(corrected_deg)
            fb['joint_vel_radps'][i] = self.feedback_vel_radps[i]
            fb['joint_torque_nm'][i] = self.feedback_torque_nm[i]

            age_ms = (now - self.feedback_stamp[i]) * 1000.0
            fb['feedback_age_ms'][i] = round(age_ms, 1)
            fb['stale_flags'][i]     = age_ms > (FEEDBACK_STALE_SEC * 1000.0)

            err_t, err_msg = self.can_errors.get(i, (0.0, ''))
            if (now - err_t) < 2.0:
                short = err_msg.split(']')[0] + ']' if ']' in err_msg else err_msg[:16]
                fb['errors'][i] = f'ERR {short}'

        out = String()
        out.data = json.dumps(fb)
        self.pub_feedback.publish(out)



def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MITPub()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
