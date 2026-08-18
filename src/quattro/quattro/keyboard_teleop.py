#!/usr/bin/env python3
import json
import math
import os
import select
import sys
import termios
import time
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from quattro_msgs.msg import IMUdata, JointAngles, JointPulse, JoyButtons, QuattroCmd


class KeyboardTeleop(Node):

    JOINT_NAMES = ['FLS', 'FLE', 'FLW', 'FRS', 'FRE', 'FRW',
                   'BLS', 'BLE', 'BLW', 'BRS', 'BRE', 'BRW']

    def __init__(self):
        super().__init__('keyboard_teleop')

        # --- 제어 상태 ---
        self.scale = 0.5
        self.vx = self.vy = self.wz = 0.0
        self.roll = self.pitch = self.yaw = self.z = 0.0
        self.rpy_step = 0.05
        self.current_max_vel = 1.0

        self.motion = 'Stop'
        self.movement = 'Stepping'
        self.pose_cmd = 'Normal'
        self.imu_auto_pose = False
        self.estop = False
        self.last_action = 'Ready'
        self.display_mode = 'quattro'  # 'quattro' or 'gim'

        # --- IMU 상태 ---
        self.imu_roll = self.imu_pitch = self.imu_yaw = 0.0
        self.imu_gyro = [0.0, 0.0, 0.0]
        self.imu_acc = [0.0, 0.0, 0.0]
        self.imu_fresh = False

        # --- 관절 상태 ---
        self.joint_targets = [0.0] * 12  # /quattro/joints_cal (quattro 캘리브레이션 적용)
        self.joint_raw = [0.0] * 12      # /quattro/joints_raw (raw IK)

        # --- Quattro 모터 피드백 ---
        self.joint_encoders = [0.0] * 12
        self.joint_applied = [0.0] * 12
        self.joint_offsets = [0.0] * 12
        self.joint_errors = ['OK'] * 12
        self.mit_state = '---'
        self.mit_tx_rate = 0
        self.feedback_fresh = False

        # --- 서보 캘리브레이션 (파라미터 서버) ---
        self.declare_parameter('calibration.direction',  [1] * 12)
        self.declare_parameter('calibration.offset_deg', [0.0] * 12)
        self.declare_parameter('calibration.neutral_us', [1500] * 12)
        self.declare_parameter('calibration.us_per_rad', 636.0)

        self.gim_dir       = list(self.get_parameter('calibration.direction').value)
        self.gim_offset    = list(self.get_parameter('calibration.offset_deg').value)
        self.gim_neutral_us = list(self.get_parameter('calibration.neutral_us').value)
        self.gim_us_per_rad = float(self.get_parameter('calibration.us_per_rad').value)

        # --- 발행자 ---
        # IMPORTANT: teleop is input-only. It publishes RAW commands and never
        # subscribes to /quattro/cmd, preventing a feedback loop through quattro_sm.
        self.cmd_pub = self.create_publisher(QuattroCmd, '/quattro/cmd_raw', 20)
        self.jb_pub = self.create_publisher(JoyButtons, '/joybuttons', 20)
        self.pulse_pub = self.create_publisher(JointPulse, '/quattro/pulse', 10)

        # --- 구독자 ---
        self.create_subscription(IMUdata,     '/quattro/imu',        self._cb_imu,           10)
        self.create_subscription(JointAngles, '/quattro/joints_cal', self._cb_joints_cal,    10)
        self.create_subscription(JointAngles, '/quattro/joints_raw',     self._cb_joints_raw,    10)
        self.create_subscription(String,      '/motor_feedback',  self._cb_motor_feedback, 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.last_display_time = 0.0

        # Input heartbeat: state machine watchdog expects a continuous command stream.
        # Keyboard events only change the stored command; the current command is
        # published continuously at 20 Hz on /quattro/cmd_raw.
        self.input_publish_hz = 20.0
        self.input_timer = self.create_timer(
            1.0 / self.input_publish_hz, self._publish_current_command
        )

        os.system('clear')

    # ------------------------------------------------------------------
    # 구독자 콜백
    # ------------------------------------------------------------------

    def _cb_quattro_cmd(self, msg):
        self.motion = msg.motion
        self.movement = msg.movement
        self.vx = msg.x_velocity
        self.vy = msg.y_velocity
        self.wz = msg.rate
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
        self.z = msg.z
        self.pose_cmd = msg.pose_cmd
        self.imu_auto_pose = msg.imu_auto_pose

    def _cb_imu(self, msg):
        try:
            self.imu_roll = msg.roll
            self.imu_pitch = msg.pitch
            self.imu_yaw = getattr(msg, 'yaw', 0.0)
            self.imu_gyro = [msg.gyro_x, msg.gyro_y, msg.gyro_z]
            self.imu_acc = [msg.acc_x, msg.acc_y, msg.acc_z]
            self.imu_fresh = True
        except Exception:
            pass

    def _cb_joints_cal(self, msg):
        self.joint_targets = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]

    def _cb_joints_raw(self, msg):
        self.joint_raw = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw,
        ]

    def _cb_motor_feedback(self, msg):
        try:
            data = json.loads(msg.data)
            encs = data.get('encoders', None)
            apps = data.get('applied_targets', None)
            offs = data.get('offsets', None)
            errs = data.get('errors', None)
            if isinstance(encs, list) and len(encs) == 12:
                self.joint_encoders = [float(x) for x in encs]
            if isinstance(apps, list) and len(apps) == 12:
                self.joint_applied = [math.degrees(float(x)) for x in apps]
            if isinstance(offs, list) and len(offs) == 12:
                self.joint_offsets = [float(x) for x in offs]
            if isinstance(errs, list) and len(errs) == 12:
                self.joint_errors = [str(x) for x in errs]
            self.mit_state = data.get('state_mode', '---')
            self.mit_tx_rate = data.get('tx_rate', 0)
            self.feedback_fresh = True
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 터미널 입력
    # ------------------------------------------------------------------

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        return sys.stdin.read(1) if rlist else ''

    def send_pulse(self, val):
        self.current_max_vel = max(0.5, min(20.0, self.current_max_vel + val))
        p = JointPulse()
        p.servo_id = 99
        p.servo_deg = float(self.current_max_vel)
        self.pulse_pub.publish(p)
        self.last_action = f'MaxVel: {self.current_max_vel:.1f} r/s'

    def _build_command(self):
        msg = QuattroCmd()
        msg.x_velocity = float(self.vx)
        msg.y_velocity = float(self.vy)
        msg.rate = float(self.wz)
        msg.roll = float(self.roll)
        msg.pitch = float(self.pitch)
        msg.yaw = float(self.yaw)
        msg.z = float(self.z)
        msg.motion = self.motion
        msg.movement = self.movement
        msg.pose_cmd = self.pose_cmd
        msg.imu_auto_pose = self.imu_auto_pose
        return msg

    def _publish_current_command(self):
        # E-stop forces a safe high-level command, but heartbeat continues.
        if self.estop:
            saved = (self.vx, self.vy, self.wz, self.motion)
            self.vx = self.vy = self.wz = 0.0
            self.motion = 'Stop'
            self.cmd_pub.publish(self._build_command())
            self.vx, self.vy, self.wz, self.motion = saved
        else:
            self.cmd_pub.publish(self._build_command())

    # ------------------------------------------------------------------
    # 화면 출력
    # ------------------------------------------------------------------

    def display_menu(self, force=False):
        now = time.time()
        if not force and (now - self.last_display_time < 0.1):
            return
        self.last_display_time = now

        RST  = '\033[0m'
        BOLD = '\033[1m'
        DIM  = '\033[2m'
        RED  = '\033[91m'
        GRN  = '\033[92m'
        YEL  = '\033[93m'
        CYN  = '\033[96m'
        MAG  = '\033[95m'
        W = 90

        estop_s  = f'{RED}● E-STOP{RST}' if self.estop else f'{GRN}○ 정상{RST}'
        mode_s   = f'{YEL}VIEWING{RST}' if self.movement == 'Viewing' else f'{CYN}STEPPING{RST}'
        mot_s    = f'{GRN}Go{RST}' if self.motion == 'Go' else f'{YEL}Stop{RST}'
        imu_s    = f'{YEL}ON{RST}' if self.imu_auto_pose else 'OFF'
        panel_s  = f'{MAG}[GIM]{RST}' if self.display_mode == 'gim' else f'{CYN}[QUATTRO]{RST}'

        o = '\r\033[H\033[J'
        o += f'{BOLD}{"=" * W}{RST}\r\n'
        o += f'{BOLD}{CYN}  QUATTRO DEBUG CONSOLE{RST}  {panel_s}'
        o += f'   {mode_s} | {mot_s} | Scale:{self.scale:.2f} | IMU:{imu_s} | {estop_s}\r\n'
        o += f'{BOLD}{"=" * W}{RST}\r\n'

        # 명령 상태
        o += f'{YEL}[ CMD  /quattro/cmd_raw ]{RST}\r\n'
        o += f'  Motion:{self.motion:<8} Movement:{self.movement:<12} Pose:{self.pose_cmd}\r\n'
        o += (f'  vx:{self.vx:+.2f}  vy:{self.vy:+.2f}  wz:{self.wz:+.2f}'
              f'    Roll:{self.roll:+.2f}  Pitch:{self.pitch:+.2f}'
              f'  Yaw:{self.yaw:+.2f}  Z:{self.z:+.2f}\r\n')
        o += '\r\n'

        # IMU
        ic = GRN if self.imu_fresh else ''
        o += f'{YEL}[ IMU  /quattro/imu ]{RST}\r\n'
        o += (f'{ic}  Roll:{self.imu_roll:+7.2f}  Pitch:{self.imu_pitch:+7.2f}'
              f'  Yaw:{self.imu_yaw:+7.2f}  (deg)\r\n{RST}')
        o += (f'{ic}  Gyro:[{self.imu_gyro[0]:+6.2f} {self.imu_gyro[1]:+6.2f}'
              f' {self.imu_gyro[2]:+6.2f}] deg/s'
              f'    Acc:[{self.imu_acc[0]:+6.2f} {self.imu_acc[1]:+6.2f}'
              f' {self.imu_acc[2]:+6.2f}] m/s2\r\n{RST}')
        o += '\r\n'

        # 모터 패널 (수정된 부분)
        if self.display_mode == 'gim':
            o += self._render_gim_panel(YEL, GRN, RED, RST)
        else:
            o += self._render_quattro_panel(YEL, RST)

        # 키 레퍼런스
        o += self._render_key_reference(YEL, CYN, MAG, DIM, RST, W)

        # 로그
        o += f'{BOLD}{"-" * W}{RST}\r\n'
        o += f'  LOG: {self.last_action}\r\n'
        o += f'{BOLD}{"=" * W}{RST}\r\n'

        sys.stdout.write(o)
        sys.stdout.flush()

    # MIT CAN 모터 패널 (GIM)
    def _render_gim_panel(self, YEL, GRN, RED, RST):
        fc = GRN if self.feedback_fresh else ''
        o = f'{YEL}[ GIM MOTORS  /quattro/joints_cal & /motor_feedback ]{RST}'
        o += f'  State:{fc}{self.mit_state}{RST}  TX:{self.mit_tx_rate}Hz\r\n'
        o += f'  {"Joint":<5} {"ID":>2}  {"Target°":>8}  {"Applied°":>9}  {"Enc°":>7}  {"Offset°":>8}  Status\r\n'
        o += f'  {"-" * 60}\r\n'
        for i in range(12):
            err = str(self.joint_errors[i])
            st = f'{RED}ERR{RST}' if ('ERR' in err or err == '1') else f'{GRN}OK{RST}'
            o += (f'  {self.JOINT_NAMES[i]:<5} {i:>2}'
                  f'  {self.joint_targets[i]:>8.2f}'
                  f'  {self.joint_applied[i]:>9.2f}'
                  f'  {self.joint_encoders[i]:>7.2f}'
                  f'  {self.joint_offsets[i]:>8.2f}'
                  f'  {st}\r\n')
        o += '\r\n'
        return o

    # RC 서보 패널 (QUATTRO)
    def _render_quattro_panel(self, YEL, RST):
        o = f'{YEL}[ QUATTRO SERVOS  /quattro/joints_raw (raw IK) + servo_calib ]{RST}\r\n'
        o += (f'  {"Joint":<5} {"ID":>2}  {"Raw°":>7}  {"Cal°":>7}'
              f'  {"Offset°":>8}  {"Dir":>4}  {"Neutral":>8}  {"Pulse(us)":>9}\r\n')
        o += f'  {"-" * 68}\r\n'
        for i in range(12):
            raw_deg = self.joint_raw[i]
            cal_deg = self.gim_dir[i] * (raw_deg + self.gim_offset[i])
            pulse_us = int(round(
                self.gim_neutral_us[i] + self.gim_us_per_rad * math.radians(cal_deg)
            ))
            pulse_us = max(500, min(2500, pulse_us))
            o += (f'  {self.JOINT_NAMES[i]:<5} {i:>2}'
                  f'  {raw_deg:>7.2f}'
                  f'  {cal_deg:>7.2f}'
                  f'  {self.gim_offset[i]:>8.2f}'
                  f'  {self.gim_dir[i]:>4d}'
                  f'  {self.gim_neutral_us[i]:>8d}'
                  f'  {pulse_us:>9d}\r\n')
        o += '\r\n'
        return o

    def _render_key_reference(self, YEL, CYN, MAG, DIM, RST, W):
        SEP = f'{DIM}{"─" * (W - 4)}{RST}'
        o = f'{YEL}[ KEY REFERENCE ]{RST}\r\n'
        o += f'  {SEP}\r\n'
        o += f'  {CYN}이동{RST}      w/x 전후진  a/d 좌우  q/e 회전  s 이동정지  SPACE 전체정지  0 E-Stop\r\n'
        o += f'  {CYN}자세{RST}      i/k Pitch   j/l Roll  u/o Yaw   r/f 높이     b 자세초기화   m IMU토글\r\n'
        o += f'  {CYN}파라미터{RST}  t/g 발높이   [/] 스케일  +/- 최대속도\r\n'
        o += f'  {CYN}특수자세{RST}  3 Sit  c Zero  5 0deg   v 모드전환(Stepping↔Viewing)  p Motion GO\r\n'
        o += f'  {MAG}패널{RST}      n  Quattro↔Gim 전환\r\n'
        o += f'  {SEP}\r\n'
        return o

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------

    def run(self):
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()

                if key:
                    if key in ('\x1b', '\x03'):
                        break

                    if key == '0':
                        self.estop = not self.estop
                        self.vx = self.vy = self.wz = 0.0
                        self.motion = 'Stop'
                        self.last_action = 'EMERGENCY STOP'

                    if not self.estop:
                        moved = False

                        if key == 'w':
                            self.vx = 0.0 if self.vx == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Fwd {'ON' if self.vx else 'OFF'}"
                            moved = True
                        elif key == 'x':
                            self.vx = 0.0 if self.vx == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Back {'ON' if self.vx else 'OFF'}"
                            moved = True
                        elif key == 'a':
                            self.vy = 0.0 if self.vy == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Left {'ON' if self.vy else 'OFF'}"
                            moved = True
                        elif key == 'd':
                            self.vy = 0.0 if self.vy == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Right {'ON' if self.vy else 'OFF'}"
                            moved = True
                        elif key == 'q':
                            self.wz = 0.0 if self.wz == 1.0 * self.scale else 1.0 * self.scale
                            self.last_action = f"Yaw L {'ON' if self.wz else 'OFF'}"
                            moved = True
                        elif key == 'e':
                            self.wz = 0.0 if self.wz == -1.0 * self.scale else -1.0 * self.scale
                            self.last_action = f"Yaw R {'ON' if self.wz else 'OFF'}"
                            moved = True
                        elif key == 's':
                            self.vx = self.vy = self.wz = 0.0
                            self.last_action = 'Move Stop'
                            moved = True

                        if moved:
                            self.motion = 'Go' if (self.vx or self.vy or self.wz) else 'Stop'

                        elif key == 'i':
                            self.pitch += self.rpy_step
                            self.last_action = f'Pitch: {self.pitch:.2f}'
                        elif key == 'k':
                            self.pitch -= self.rpy_step
                            self.last_action = f'Pitch: {self.pitch:.2f}'
                        elif key == 'j':
                            self.roll += self.rpy_step
                            self.last_action = f'Roll: {self.roll:.2f}'
                        elif key == 'l':
                            self.roll -= self.rpy_step
                            self.last_action = f'Roll: {self.roll:.2f}'
                        elif key == 'u':
                            self.yaw += self.rpy_step
                            self.last_action = f'Yaw: {self.yaw:.2f}'
                        elif key == 'o':
                            self.yaw -= self.rpy_step
                            self.last_action = f'Yaw: {self.yaw:.2f}'
                        elif key == 'r':
                            self.z = min(1.0, self.z + 0.1)
                            self.last_action = f'Height: {self.z:.2f}'
                        elif key == 'f':
                            self.z = max(-1.0, self.z - 0.1)
                            self.last_action = f'Height: {self.z:.2f}'
                        elif key == 'b':
                            self.roll = self.pitch = self.yaw = self.z = 0.0
                            self.last_action = 'Pose Reset'
                        elif key in (']', '}'):
                            self.scale = min(1.0, self.scale + 0.1)
                            if self.vx > 0:
                                self.vx = 1.0 * self.scale
                            elif self.vx < 0:
                                self.vx = -1.0 * self.scale
                            self.last_action = f'Scale: {self.scale:.2f}'
                        elif key in ('[', '{'):
                            self.scale = max(0.01, self.scale - 0.1)
                            if self.vx > 0:
                                self.vx = 1.0 * self.scale
                            elif self.vx < 0:
                                self.vx = -1.0 * self.scale
                            self.last_action = f'Scale: {self.scale:.2f}'
                        elif key in ('=', '+'):
                            self.send_pulse(0.5)
                        elif key == '-':
                            self.send_pulse(-0.5)
                        elif key == 'm':
                            self.imu_auto_pose = not self.imu_auto_pose
                            self.last_action = f"IMU: {'ON' if self.imu_auto_pose else 'OFF'}"
                        elif key == 'n':
                            self.display_mode = 'gim' if self.display_mode == 'quattro' else 'quattro'
                            self.last_action = f"Panel: {self.display_mode.upper()}"
                        elif key == 'v':
                            self.movement = 'Viewing' if self.movement == 'Stepping' else 'Stepping'
                            self.last_action = f'Mode -> {self.movement}'
                        elif key == '3':
                            self.movement = 'Viewing'
                            self.pose_cmd = 'Sit'
                            self.motion = 'Stop'
                            self.last_action = 'Sit Pose'
                        elif key == 'c':
                            self.movement = 'Viewing'
                            self.pose_cmd = 'Zero'
                            self.motion = 'Stop'
                            self.last_action = 'Zero Pose'
                        elif key == '5':
                            self.movement = 'Viewing'
                            self.pose_cmd = '0deg'
                            self.motion = '0deg'
                            self.last_action = '0deg Pose'
                        elif key == 'p':
                            self.motion = 'Go'
                            self.pose_cmd = 'Normal'
                            self.last_action = 'Motion GO'
                        elif key == ' ':
                            self.motion = 'Stop'
                            self.pose_cmd = 'Normal'
                            self.vx = self.vy = self.wz = 0.0
                            self.last_action = 'TOTAL STOP'

                    # QuattroCmd is published continuously by the 20 Hz timer.
                    # JoyButtons remains event-driven because it represents button edges.
                    jb = JoyButtons()
                    if key == 't':
                        jb.updown = 1
                        self.last_action = 'Clearance Up'
                    elif key == 'g':
                        jb.updown = -1
                        self.last_action = 'Clearance Down'

                    self.jb_pub.publish(jb)

                self.display_menu(force=bool(key))
                rclpy.spin_once(self, timeout_sec=0)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    try:
        node.run()
    except Exception:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()