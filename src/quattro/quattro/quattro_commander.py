#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import copy

# ROS 2 메시지
from quattro_msgs.msg import (
    QuattroCmd, JoyButtons, IMUdata, ContactData, JointAngles, GaitState, JointCommand
)
from geometry_msgs.msg import Twist

# quattro 프로젝트 내부 모듈
from quattro.Kinematics.QuattroKinematics import QuattroModel
#STANCE - phase
from quattro.GaitGenerator.Bezier import BezierGait, STANCE



def clip(v, lo, hi):
    return np.minimum(np.maximum(v, lo), hi)


def _quat_to_roll_pitch(qw: float, qx: float, qy: float, qz: float):
    """단위 쿼터니언 → (roll, pitch) [rad]."""
    roll  = math.atan2(2.0 * (qw * qx + qy * qz),
                       1.0 - 2.0 * (qx * qx + qy * qy))
    sin_p = float(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    pitch = math.asin(sin_p)
    return roll, pitch


class QuattroCommander(Node):
    def __init__(self):
        super().__init__('quattro_commander')

        self.get_logger().info("Initializing QuattroCommander Node (v2 Features + Param Calibration)...")

        # ----------------------------------------------------
        # 1. 파라미터 선언 및 로드
        # ----------------------------------------------------
        # (1) Controller Params
        self.STEPLENGTH_SCALE = self.get_ros_param("STEPLENGTH_SCALE", 0.05)
        self.Z_SCALE_CTRL     = self.get_ros_param("Z_SCALE_CTRL", 0.15)
        self.RPY_SCALE        = self.get_ros_param("RPY_SCALE", 0.785)
        self.SV_SCALE         = self.get_ros_param("SV_SCALE", 0.05)
        self.CHPD_SCALE       = self.get_ros_param("CHPD_SCALE", 0.0005)
        self.YAW_SCALE        = self.get_ros_param("YAW_SCALE", 1.25)
        
        # Twist 입력 (cmd_vel) 관련
        self.vx_scale = self.get_ros_param("cmd_vel_vx_scale", 1.0)
        self.vy_scale = self.get_ros_param("cmd_vel_vy_scale", 0.5)
        self.wz_scale = self.get_ros_param("cmd_vel_wz_scale", 0.6)
        # 정상 운용에서는 quattro_sm을 우회하지 않는다. 직접 Twist 입력은 디버그 전용.
        self.enable_twist_input = self.get_ros_param("enable_direct_twist_input", False)

        # (2) PID 제어 파라미터
        self.imu_auto_pose = self.get_ros_param("IMU_pose", False)
        self.Kp = self.get_ros_param("POSE_PID_Kp", 1.5)
        self.Ki = self.get_ros_param("POSE_PID_Ki", 0.1)
        self.Kd = self.get_ros_param("POSE_PID_Kd", 0.05)
        self.i_limit = self.get_ros_param("POSE_PID_I_Limit", 0.5)

        # (3) Quattro Params
        self.BaseStepVelocity     = self.get_ros_param("BaseStepVelocity", 0.001)
        self.BaseSwingPeriod      = self.get_ros_param("Tswing", 0.2)
        # 리스트 타입 파라미터는 기본값을 명시적으로 리스트로 주어야 함
        self.SwingPeriod_LIMITS   = list(self.get_ros_param("SwingPeriod_LIMITS", [0.1, 0.4]))
        self.BaseClearanceHeight  = self.get_ros_param("BaseClearanceHeight", 0.03)
        self.BasePenetrationDepth = self.get_ros_param("BasePenetrationDepth", 0.003)
        
        self.ClearanceHeight_LIMITS = list(self.get_ros_param("ClearanceHeight_LIMITS", [0.0, 0.1]))
        self.PenetrationDepth_LIMITS = list(self.get_ros_param("PenetrationDepth_LIMITS", [0.0, 0.05]))

        # (4) Robot Dimensions
        self.shoulder_length = self.get_ros_param("shoulder_length", 0.1165)
        self.elbow_length    = self.get_ros_param("elbow_length", 0.205)
        self.wrist_length    = self.get_ros_param("wrist_length", 0.197)
        self.hip_x           = self.get_ros_param("hip_x", 0.386)
        self.hip_y           = self.get_ros_param("hip_y", 0.1)
        self.foot_x          = self.get_ros_param("foot_x", 0.385)
        self.foot_y          = self.get_ros_param("foot_y", 0.333)
        self.height          = self.get_ros_param("height", 0.25)
        self.com_offset      = self.get_ros_param("com_offset", 0.0)
        
        self.control_rate_hz = float(self.get_ros_param("control_rate_hz", 100.0))
        self.dt = 1.0 / max(self.control_rate_hz, 1.0)

        # MIT actuator command 기본값. 상위 제어기가 필요하면 JointCommand에서 동적으로 바꿀 수 있다.
        self.actuator_kp = float(self.get_ros_param("actuator_kp", 60.0))
        self.actuator_kd = float(self.get_ros_param("actuator_kd", 0.8))
        self.default_torque_ff = float(self.get_ros_param("torque_ff_default", 0.0))

        # (5) 캘리브레이션 파라미터 (파일 직접 로드 X -> ROS Param O)
        # 기본값: 방향 1, 오프셋 0.0 (12개 관절)
        default_dirs = [1] * 12
        default_offsets = [0.0] * 12
        
        self.calib_dir = list(self.get_ros_param("calibration.direction", default_dirs))
        self.calib_offset = list(self.get_ros_param("calibration.offset_deg", default_offsets))

        # 관절 제한값 (로봇별로 params yaml에서 지정)
        self.joint_lo = list(self.get_ros_param("joint_lo", [-30.0, -5.0, -165.0] * 4))
        self.joint_hi = list(self.get_ros_param("joint_hi", [30.0, 125.0, 5.0] * 4))

        self.get_logger().info("Calibration Params Loaded.")

        

        # ----------------------------------------------------
        # 2. 내부 변수 초기화
        # ----------------------------------------------------
        self.quattro_cmd = QuattroCmd()
        self.jb = JoyButtons()
        self._init_quattro_cmd()

        self.StepVelocity     = copy.deepcopy(self.BaseStepVelocity)
        self.SwingPeriod      = copy.deepcopy(self.BaseSwingPeriod)
        self.ClearanceHeight  = copy.deepcopy(self.BaseClearanceHeight)
        self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.last_pid_time = self.last_time
        self.prev_command_pos_rad = None
        
        self.contacts = [0, 0, 0, 0]
        self.imu = [0.0]*8
        self.enable_contact = False

        # PID 제어용 적분항 초기화
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        self.current_roll_deg = 0.0
        self.current_pitch_deg = 0.0
        self.current_roll_rate_degs = 0.0
        self.current_pitch_rate_degs = 0.0

        # ----------------------------------------------------
        # 3. 모델 로드
        # ----------------------------------------------------
        self.quattro_model = QuattroModel(
            shoulder_length=self.shoulder_length, elbow_length=self.elbow_length,
            wrist_length=self.wrist_length, hip_x=self.hip_x, hip_y=self.hip_y,
            foot_x=self.foot_x, foot_y=self.foot_y,
            height=self.height, com_offset=self.com_offset
        )
        self.T_bf0 = self.quattro_model.WorldToFoot
        self.T_bf  = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.dt, Tswing=self.BaseSwingPeriod)

        # ----------------------------------------------------
        # 4. Pub/Sub 설정
        # ----------------------------------------------------
        qos_profile = QoSProfile(depth=1)
        self.sub_cmd = self.create_subscription(QuattroCmd, '/quattro/cmd', self._cb_quattro_cmd, qos_profile)
        self.sub_jb  = self.create_subscription(JoyButtons, '/joybuttons', self._cb_joy_buttons, qos_profile)
        self.sub_imu = self.create_subscription(IMUdata, '/quattro/imu', self._cb_imu, qos_profile)
        self.sub_cnt = self.create_subscription(ContactData, '/quattro/contact', self._cb_contact, qos_profile)
        self.ja_pub = self.create_publisher(JointAngles, '/quattro/joints_raw', qos_profile)
        self.ja_cal_pub = self.create_publisher(JointAngles, '/quattro/joints_cal', qos_profile)
        self.joint_cmd_pub = self.create_publisher(JointCommand, '/quattro/joint_command', qos_profile)
        self.gait_state_pub = self.create_publisher(GaitState, '/quattro/gait_state', qos_profile)


       


        # cmd_vel (Twist) 구독 추가
        if self.enable_twist_input:
            self.sub_cmdvel = self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        # 상위 command loop는 100 Hz 기본. Teensy의 내부 CAN loop는 이후 독립적으로 높일 수 있다.
        self.timer = self.create_timer(self.dt, self.move)
        self.get_logger().info("READY TO GO!")

    def get_ros_param(self, name, default=None):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _init_quattro_cmd(self):
        self.quattro_cmd.x_velocity = 0.0
        self.quattro_cmd.y_velocity = 0.0
        self.quattro_cmd.rate = 0.0
        self.quattro_cmd.roll = 0.0
        self.quattro_cmd.pitch = 0.0
        self.quattro_cmd.yaw = 0.0
        self.quattro_cmd.z = 0.0
        self.quattro_cmd.motion = "Stop"
        self.quattro_cmd.movement = "Stepping"
        self.quattro_cmd.pose_cmd = "Normal"

    # --- 콜백 함수들 ---
    def _cb_imu(self, imu: IMUdata):
        try:
            ROLL_OFFSET  = -1.0
            PITCH_OFFSET = 5.8

            # 쿼터니언이 유효하면 직접 roll/pitch 계산, 아니면 Euler 필드 사용
            quat_norm = math.sqrt(
                imu.quat_w**2 + imu.quat_x**2 + imu.quat_y**2 + imu.quat_z**2
            )
            if quat_norm > 0.9:
                roll_rad, pitch_rad = _quat_to_roll_pitch(
                    imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z
                )
                real_roll  = math.degrees(roll_rad)  - ROLL_OFFSET
                real_pitch = math.degrees(pitch_rad) - PITCH_OFFSET
            else:
                real_roll  = imu.roll  - ROLL_OFFSET
                real_pitch = imu.pitch - PITCH_OFFSET

            self.current_roll_deg        = real_roll
            self.current_pitch_deg       = real_pitch
            self.current_roll_rate_degs  = imu.gyro_x
            self.current_pitch_rate_degs = imu.gyro_y

            self.imu = [
                np.radians(real_roll), np.radians(real_pitch),
                np.radians(imu.gyro_x), np.radians(imu.gyro_y), np.radians(imu.gyro_z),
                imu.acc_x, imu.acc_y, imu.acc_z - 9.81
            ]
        except Exception:
            pass

    def _cb_contact(self, cnt: ContactData):
        self.contacts = [cnt.fl, cnt.fr, cnt.bl, cnt.br]

    def _cb_quattro_cmd(self, quattro_cmd: QuattroCmd):
        self.quattro_cmd = quattro_cmd
        self.imu_auto_pose = quattro_cmd.imu_auto_pose
        if not hasattr(self.quattro_cmd, 'pose_cmd') or not self.quattro_cmd.pose_cmd:
            self.quattro_cmd.pose_cmd = "Normal"

    def _cb_joy_buttons(self, jb: JoyButtons):
        self.jb = jb
        if self.jb.left_bump or self.jb.right_bump:
            self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
            self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)

    def _cb_cmd_vel(self, msg: Twist):
        """ROS 표준 /cmd_vel 메시지 처리"""
        vx = float(clip(msg.linear.x,  -1.0, 1.0)) * self.vx_scale
        vy = float(clip(msg.linear.y,  -1.0, 1.0)) * self.vy_scale
        wz = float(clip(msg.angular.z, -1.0, 1.0)) * self.wz_scale
        
        self.quattro_cmd.motion   = "Go"
        self.quattro_cmd.movement = "Stepping"
        self.quattro_cmd.pose_cmd = "Normal"
        self.quattro_cmd.x_velocity = vx
        self.quattro_cmd.y_velocity = vy
        self.quattro_cmd.rate       = wz
        self.quattro_cmd.roll  = 0.0
        self.quattro_cmd.pitch = 0.0
        self.quattro_cmd.yaw   = 0.0
        self.quattro_cmd.z     = 0.0

    def _apply_calibration(self, raw_degs):
        """IK 좌표계 각도(12개, degrees)에 관절 제한 및 캘리브레이션(dir, offset)을 적용."""
        return [
            self.calib_dir[i] * (float(np.clip(raw_degs[i], self.joint_lo[i], self.joint_hi[i])) + self.calib_offset[i])
            for i in range(12)
        ]

    def _make_ja_msg(self, degs, step_or_view=False):
        ja_msg = JointAngles()
        ja_msg.fls = degs[0];  ja_msg.fle = degs[1];  ja_msg.flw = degs[2]
        ja_msg.frs = degs[3];  ja_msg.fre = degs[4];  ja_msg.frw = degs[5]
        ja_msg.bls = degs[6];  ja_msg.ble = degs[7];  ja_msg.blw = degs[8]
        ja_msg.brs = degs[9];  ja_msg.bre = degs[10]; ja_msg.brw = degs[11]
        ja_msg.step_or_view = step_or_view
        return ja_msg

    def _publish_ja(self, raw_degs, step_or_view=False):
        """캘리브레이션 전 raw IK 각도(degrees)를 /quattro/joints_raw로 발행 (진단용)."""
        self.ja_pub.publish(self._make_ja_msg(raw_degs, step_or_view))

    def _publish_ja_cal(self, cal_degs, step_or_view=False):
        """캘리브레이션 적용 각도(degrees)를 /quattro/joints_cal로 발행 (모터 드라이버용)."""
        self.ja_cal_pub.publish(self._make_ja_msg(cal_degs, step_or_view))

    JOINT_NAMES = [
        'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
        'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
        'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
        'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
    ]

    def _publish_joint_command(self, cal_degs):
        """Calibrated joint target를 표준화된 actuator command로 발행한다.

        position/velocity는 SI 단위(rad, rad/s), kp/kd/torque_ff는 MIT 입력값이다.
        velocity는 목표 위치의 유한차분으로 생성하며 첫 프레임은 0으로 둔다.
        """
        q = [math.radians(float(v)) for v in cal_degs]
        if self.prev_command_pos_rad is None:
            qd = [0.0] * 12
        else:
            qd = [
                float(np.clip((q[i] - self.prev_command_pos_rad[i]) / self.dt, -65.0, 65.0))
                for i in range(12)
            ]
        self.prev_command_pos_rad = list(q)

        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = q
        msg.velocity = qd
        msg.kp = [self.actuator_kp] * 12
        msg.kd = [self.actuator_kd] * 12
        msg.torque_ff = [self.default_torque_ff] * 12
        self.joint_cmd_pub.publish(msg)

    def _publish_joint_outputs(self, raw_degs, step_or_view=False):
        """IK 결과를 진단 토픽 + calibrated command에 동시에 반영."""
        cal = self._apply_calibration(raw_degs)
        self._publish_ja(raw_degs, step_or_view)
        self._publish_ja_cal(cal, step_or_view)  # 기존 디버그 호환용
        self._publish_joint_command(cal)

    def _publish_gait_state(
        self,
        stance,
        phase,
        gait,
        t_stance,
        t_swing,
    ):
        msg = GaitState()
        msg.stance = [bool(v) for v in stance]
        msg.phase = [float(v) for v in phase]
        msg.gait = str(gait)

        msg.t_stance = float(t_stance)
        msg.t_swing = float(t_swing)

        self.gait_state_pub.publish(msg)

   


    

    

    # --- 메인 루프 (Timer) ---
    def move(self):
        # 1. 특수 자세 처리 (Sit, Zero, 0deg)
        if self.quattro_cmd.pose_cmd == "Sit":
            raw = [0.0, 120.0, -135.0] * 4
            self._publish_joint_outputs(raw)
            return

        elif self.quattro_cmd.pose_cmd == "Zero":
            raw = [0.0] * 12
            self._publish_joint_outputs(raw)
            return

        elif self.quattro_cmd.pose_cmd == "0deg":
            # 하드웨어 영점: 모터 인코더 물리적 0 위치
            # raw는 캘리브레이션 역산 (-dir*offset), cal은 0 직접 전송
            raw = [-self.calib_dir[i] * self.calib_offset[i] for i in range(12)]
            # 하드웨어 영점 명령은 calibrated target을 정확히 0으로 보낸다.
            self._publish_ja(raw)
            self._publish_ja_cal([0.0] * 12)
            self._publish_joint_command([0.0] * 12)
            return
        
        # 2. 일반 보행 및 관람 모드
        step_or_view = (self.quattro_cmd.movement != "Stepping")

        if self.quattro_cmd.motion != "Stop":
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = np.clip(
                copy.deepcopy(self.BaseSwingPeriod) +
                (-self.quattro_cmd.faster + -self.quattro_cmd.slower) * self.SV_SCALE,
                self.SwingPeriod_LIMITS[0], self.SwingPeriod_LIMITS[1]
            )

            if self.quattro_cmd.movement == "Stepping":
                # 보행 모드
                StepLength = self.quattro_cmd.x_velocity + abs(self.quattro_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0) * self.STEPLENGTH_SCALE
                LateralFraction = self.quattro_cmd.y_velocity * np.pi / 2.0
                YawRate = self.quattro_cmd.rate * self.YAW_SCALE
                
                pos = np.array([0.0, 0.0, 0.0])
                orn = np.array([0.0, 0.0, 0.0])
                # 보행 중에는 PID 적분항 초기화
                self.integral_roll = 0.0
                self.integral_pitch = 0.0
            else:
                # 관람 모드
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                self.ClearanceHeight  = copy.deepcopy(self.BaseClearanceHeight)
                self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
                self.StepVelocity     = copy.deepcopy(self.BaseStepVelocity)

                if self.imu_auto_pose:
                    # IMU 기반 자동 균형 제어 (PID 사용)
                    target_roll_deg = self.quattro_cmd.roll * (self.RPY_SCALE * 45.0)
                    target_pitch_deg = self.quattro_cmd.pitch * (self.RPY_SCALE * 45.0)
                    target_yaw_rad = self.quattro_cmd.yaw * self.RPY_SCALE
                    pos = np.array([0.0, 0.0, self.quattro_cmd.z * self.Z_SCALE_CTRL])

                    error_roll = target_roll_deg - self.current_roll_deg
                    error_pitch = target_pitch_deg - self.current_pitch_deg

                    now_pid = self.get_clock().now().nanoseconds / 1e9
                    pid_dt = float(np.clip(now_pid - self.last_pid_time, 0.001, 0.05))
                    self.last_pid_time = now_pid
                    self.integral_roll = clip(self.integral_roll + error_roll * pid_dt, -self.i_limit, self.i_limit)
                    self.integral_pitch = clip(self.integral_pitch + error_pitch * pid_dt, -self.i_limit, self.i_limit)

                    derivative_roll = -self.current_roll_rate_degs
                    derivative_pitch = -self.current_pitch_rate_degs

                    orn_roll_rad = -np.radians(
                        (self.Kp * error_roll) +
                        (self.Ki * self.integral_roll) +
                        (self.Kd * derivative_roll)
                    )
                    orn_pitch_rad = -np.radians(
                        (self.Kp * error_pitch) +
                        (self.Ki * self.integral_pitch) +
                        (self.Kd * derivative_pitch)
                    )
                    orn = np.array([orn_roll_rad, orn_pitch_rad, target_yaw_rad])
                else:
                    # 수동 제어
                    pos = np.array([0.0, 0.0, self.quattro_cmd.z * self.Z_SCALE_CTRL])
                    orn = np.array([
                        self.quattro_cmd.roll * self.RPY_SCALE,
                        self.quattro_cmd.pitch * self.RPY_SCALE,
                        self.quattro_cmd.yaw * self.RPY_SCALE
                    ])
                    self.integral_roll = 0.0
                    self.integral_pitch = 0.0
        else:
            # Stop 상태
            StepLength = 0.0; LateralFraction = 0.0; YawRate = 0.0
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])
            self.integral_roll = 0.0
            self.integral_pitch = 0.0

        # 사용자 입력에 의한 높이/깊이 조절
        self.ClearanceHeight  += self.jb.updown * self.CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * self.CHPD_SCALE

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        self.bzg.Tswing = self.SwingPeriod
        self.ClearanceHeight = np.clip(self.ClearanceHeight, self.ClearanceHeight_LIMITS[0], self.ClearanceHeight_LIMITS[1])
        self.PenetrationDepth = np.clip(self.PenetrationDepth, self.PenetrationDepth_LIMITS[0], self.PenetrationDepth_LIMITS[1])

        # 3. Trajectory 생성
        # 보폭이 줄어들 때 발 높이도 비례해서 낮춰 감속 중 스윙 발이 지면에 탁 닿는 현상 방지
        # threshold: STEPLENGTH_SCALE * 0.2 이상에서 full clearance
        _step_ratio = min(1.0, abs(StepLength) / max(self.STEPLENGTH_SCALE * 0.2, 1e-6))
        self.T_bf = self.bzg.GenerateTrajectory(
            StepLength, LateralFraction, YawRate, self.StepVelocity,
            self.T_bf0, self.T_bf,
            self.ClearanceHeight * _step_ratio, self.PenetrationDepth,
            self.contacts, dt
        )
        ##
        phase = list(self.bzg.Phases)

        stance = [
            s == STANCE
            for s in self.bzg.LegState
        ]

        is_gait_active = (
            self.quattro_cmd.motion != "Stop"
            and self.quattro_cmd.movement == "Stepping"
            and (
                abs(StepLength) > 1e-6
                or abs(YawRate) > 1e-6
            )
        )

        gait = "trot" if is_gait_active else "stand"

        if gait == "stand":
            stance = [True, True, True, True]
            phase = [0.0, 0.0, 0.0, 0.0]

            t_stance = 0.0
            t_swing = 0.0
        else:
            t_stance = self.bzg.current_tstance
            t_swing = self.bzg.current_tswing

        self._publish_gait_state(
            stance=stance,
            phase=phase,
            gait=gait,
            t_stance=t_stance,
            t_swing=t_swing,
        )
        ##
        T_bf_copy = copy.deepcopy(self.T_bf)
        
        # 4. 역기구학 (IK)
        joint_angles = self.quattro_model.IK(orn, pos, T_bf_copy)

        #MPC 테스트용 로그
        names = ["FL", "FR", "BL", "BR"]
        out = []

        #타입 확인 테스트 코드
        #self.get_logger().warn(
        #    f"T_bf type={type(T_bf_copy)}, value={T_bf_copy}",
        #    throttle_duration_sec=1.0
        #)

        for leg, name in enumerate(names):
            p_des = T_bf_copy[name][:3, 3]
            q_leg = joint_angles[leg]

            out.append(
                f"{name} T_bf=[{p_des[0]:+.3f}, {p_des[1]:+.3f}, {p_des[2]:+.3f}] "
                f"q=[{q_leg[0]:+.3f}, {q_leg[1]:+.3f}, {q_leg[2]:+.3f}]"
            )

        self.get_logger().info(" | ".join(out), throttle_duration_sec=1.0)

        # 5. raw는 진단용, calibrated 값은 JointCommand를 통해 Serial bridge로 전달
        raw_degs = [np.degrees(joint_angles[r][c]) for r in range(4) for c in range(3)]
        self._publish_joint_outputs(raw_degs, step_or_view)



        

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QuattroCommander()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ROS 2 강제 종료 시 발생하는 메시지 증발 에러를 조용히 무시합니다.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()