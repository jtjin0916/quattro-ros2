# NOTE: Legacy control logic retained; project/topic naming normalized to Quattro during refactor.
#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import json
import numpy as np
import scipy.sparse as sp
import osqp

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32MultiArray, String
from quattro_msgs.msg import IMUdata, GaitState


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('robot_mass', 9.5)
        self.declare_parameter('torque_limit', 0.2)
        self.declare_parameter('control_hz', 100.0)

        self.declare_parameter('shoulder_length', 0.0905)
        self.declare_parameter('elbow_length', 0.210)
        self.declare_parameter('wrist_length', 0.210)

        self.declare_parameter('hip_x', 0.405)
        self.declare_parameter('hip_y', 0.12)

        self.declare_parameter('foot_y', 0.301)
        self.foot_y = float(self.get_parameter('foot_y').value)

        self.hip_x = float(self.get_parameter('hip_x').value)
        self.hip_y = float(self.get_parameter('hip_y').value)

        self.robot_mass = float(self.get_parameter('robot_mass').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.control_hz = float(self.get_parameter('control_hz').value)

        self.shoulder_length = float(self.get_parameter('shoulder_length').value)
        self.elbow_length = float(self.get_parameter('elbow_length').value)
        self.wrist_length = float(self.get_parameter('wrist_length').value)

        self.q12 = np.zeros(12, dtype=np.float64)

        self.declare_parameter('enable_torque', False)
        self.enable_torque = bool(self.get_parameter('enable_torque').value)

        fz0 = -self.robot_mass * 9.81 / 4.0
        self.last_F_all = np.array([
            [0.0, 0.0, fz0],
            [0.0, 0.0, fz0],
            [0.0, 0.0, fz0],
            [0.0, 0.0, fz0],
        ], dtype=np.float64)

        self.roll = 0.0
        self.pitch = 0.0
        self.gyro = np.zeros(3, dtype=np.float64)
        self.imu_ready = False
        self.feedback_ready = False

        self.stance_mask = np.ones(4, dtype=bool)
        self.gait_phase = np.zeros(4, dtype=np.float64)
        self.gait_type = "stand"

        #목표 테스트하드코딩
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0

        self.desired_height = 0.30

        self.desired_vx = 0.0
        self.desired_vy = 0.0
        self.desired_vz = 0.0

        self.desired_wx = 0.0
        self.desired_wy = 0.0
        self.desired_wz = 0.0

        #발이 전환될 때 갑자기 발이 튀는 것을 방지 저역 통과 필터(수정중)
        self.estimated_height = self.desired_height
        self.height_filter_alpha = 0.1

        #관성 근사 함수
        self.body_inertia = self.approximate_body_inertia()
        self.body_inertia_inv = np.linalg.inv(self.body_inertia)

        # horizon (예측 모델)
        self.declare_parameter('mpc_horizon', 5)
        self.mpc_horizon = int(self.get_parameter('mpc_horizon').value)

        #게이트 예측
        self.gait_t_stance = 0.0
        self.gait_t_swing = 0.0

        #스케일 
        self.declare_parameter('mpc_force_xy_scale', 0.2)
        self.declare_parameter('mpc_force_z_scale', 0.2)

        self.mpc_force_xy_scale = float(
            self.get_parameter('mpc_force_xy_scale').value
        )
        self.mpc_force_z_scale = float(
            self.get_parameter('mpc_force_z_scale').value
        )

        #토크 변화율 제한
        self.last_tau_cmd = np.zeros(12, dtype=np.float64)

        self.declare_parameter('max_tau_step', 0.03)
        self.max_tau_step = float(
            self.get_parameter('max_tau_step').value
        )


        qos = QoSProfile(depth=1)

        self.create_subscription(
            String,
            '/motor_feedback',
            self.cb_motor_feedback,
            qos,
        )

        self.torque_pub = self.create_publisher(
            Float32MultiArray,
            '/quattro/torque_ff',
            qos,
        )

        self.timer = self.create_timer(
            1.0 / self.control_hz,
            self.control_loop,
        )

        self.create_subscription(
            IMUdata,
            "/quattro/imu",
            self.cb_imu,
            qos,
        )

        self.create_subscription(
            GaitState,
            '/quattro/gait_state',
            self.cb_gait_state,
            qos,
        )

        self.get_logger().info('mpc_controller ready')



    def cb_motor_feedback(self, msg: String):
        try:
            data = json.loads(msg.data)

            if 'joint_pos_rad' in data and len(data['joint_pos_rad']) >= 12:
                self.q12 = np.array(data['joint_pos_rad'][:12], dtype=np.float64)
                self.feedback_ready = True
        except Exception as e:
            self.get_logger().warn(
                f'motor_feedback parse failed: {e}',
                throttle_duration_sec=2.0,
            )

    def cb_imu(self, msg):

        self.roll = msg.roll
        self.pitch = msg.pitch

        self.gyro[0] = msg.gyro_x
        self.gyro[1] = msg.gyro_y
        self.gyro[2] = msg.gyro_z
        self.imu_ready = True

    def cb_gait_state(self, msg):
        self.stance_mask = np.asarray(
            msg.stance,
            dtype=bool,
        )
        self.gait_phase = np.asarray(
            msg.phase,
            dtype=np.float64,
        )
        self.gait_type = str(msg.gait)

        self.gait_t_stance = float(msg.t_stance)
        self.gait_t_swing = float(msg.t_swing)

    #forward kinematic 계산 부분 (좌표계 맞추기k, 기본 자세)
    #local이 0 이되어야지 자코비안이 들어가서 q가 변하므로 의미가 있음
    # 오프 셋 고려된 fk 구해짐
    def leg_fk_local(self, leg, q_leg):
            q0, q1, q2 = q_leg

            l_abad = self.shoulder_length
            l1 = self.elbow_length
            l2 = self.wrist_length

            is_left = leg in [0, 2]

            # IK에서 joint_angles = [-shoulder_angle, elbow_angle, wrist_angle]
            shoulder_angle = -q0
            elbow_angle = q1
            wrist_angle = q2

            # pitch plane
            r = l1 * math.cos(elbow_angle) + l2 * math.cos(elbow_angle + wrist_angle)
            x_local = -(l1 * math.sin(elbow_angle) + l2 * math.sin(elbow_angle + wrist_angle))

            # 기준 local 각도
            q0_ref = 0.0
            q1_ref = 0.899
            q2_ref = -1.532

            shoulder_ref = -q0_ref
            r0 = l1 * math.cos(q1_ref) + l2 * math.cos(q1_ref + q2_ref)

            x_local0 = -(l1 * math.sin(q1_ref) + l2 * math.sin(q1_ref + q2_ref))

            if is_left:
                y_local0 = l_abad * math.cos(shoulder_ref) + r0 * math.sin(shoulder_ref)
            else:
                y_local0 = -l_abad * math.cos(shoulder_ref) + r0 * math.sin(shoulder_ref)
            
            z_local0 = -(r0 * math.cos(shoulder_ref))


            if is_left:
                
                y_local = l_abad * math.cos(shoulder_angle) + r * math.sin(shoulder_angle)
                z_local = -(r * math.cos(shoulder_angle) - l_abad * math.sin(shoulder_angle))
            else:
                
                y_local = -l_abad * math.cos(shoulder_angle) + r * math.sin(shoulder_angle)
                z_local = -(r * math.cos(shoulder_angle) + l_abad * math.sin(shoulder_angle))

            #FK도  hip에서 foot 기준으로 설정
            return np.array([
                x_local - x_local0,
                y_local - y_local0,
                z_local - z_local0
            ], dtype=np.float64)

    def compute_leg_jacobian(self, leg, q_leg):
        eps = 1e-5
        J = np.zeros((3, 3), dtype=np.float64)

        for i in range(3):
            dq = np.zeros(3, dtype=np.float64)
            dq[i] = eps

            p_plus = self.leg_fk_local(leg, q_leg + dq)
            p_minus = self.leg_fk_local(leg, q_leg - dq)

            J[:, i] = (p_plus - p_minus) / (2.0 * eps)

        return J

    #토크 만들기
    def compute_mpc_torque_ff(
        self,
        F0_mpc,
        contact_mask,
    ):
        """
        Horizon MPC의 첫 입력 F0를
        관절 토크 tau = J^T F로 변환한다.
        """
        F0_mpc = np.asarray(
            F0_mpc,
            dtype=np.float64,
        )

        contact_mask = np.asarray(
            contact_mask,
            dtype=bool,
        )

        if F0_mpc.shape != (4, 3):
            raise ValueError(
                f"F0_mpc must have shape (4, 3), "
                f"got {F0_mpc.shape}"
            )

        if contact_mask.shape != (4,):
            raise ValueError(
                f"contact_mask must have shape (4,), "
                f"got {contact_mask.shape}"
            )

        tau12 = np.zeros(12, dtype=np.float64)


        for leg in range(4):
            # swing 다리는 MPC feedforward 토크를 넣지 않는다.
            if not contact_mask[leg]:
                continue

            q_leg = self.q12[
                leg * 3:(leg + 1) * 3
            ]

            J = self.compute_leg_jacobian(
                leg,
                q_leg,
            )

            F = F0_mpc[leg].copy()

            F[0] *= self.mpc_force_xy_scale
            F[1] *= self.mpc_force_xy_scale
            F[2] *= self.mpc_force_z_scale

            if not np.all(np.isfinite(F)):
                self.get_logger().warn(
                    f"Invalid MPC force: leg={leg}, F={F}",
                    throttle_duration_sec=1.0,
                )
                continue

            tau_leg = J.T @ F

            if not np.all(np.isfinite(tau_leg)):
                self.get_logger().warn(
                    f"Invalid MPC torque: "
                    f"leg={leg}, tau={tau_leg}",
                    throttle_duration_sec=1.0,
                )
                continue

            tau12[
                leg * 3:(leg + 1) * 3
            ] = tau_leg

        return np.clip(
            tau12,
            -self.torque_limit,
            self.torque_limit,
        )

    
    def log_fk_positions(self):
        names = ["FL", "FR", "BL", "BR"]

        out = []
        for leg in range(4):
            q_leg = self.q12[leg*3:(leg+1)*3]
            p = self.leg_fk_local(leg, q_leg)
            out.append(
                f"{names[leg]} p=[{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}]"
            )

        self.get_logger().info(" | ".join(out), throttle_duration_sec=1.0)

    
    #컨트롤 루프(로그 주로 여기서 찍어봄)
    def control_loop(self):
        if not self.feedback_ready:
            self.publish_zero_torque()
            return

        x = self.get_current_state()
        x_ref = self.get_desired_state()
        N = self.mpc_horizon

        # 기존 QP 힘 반복 예측 — 비교용
        F_test = self.solve_standing_qp_osqp()
        u_now = F_test.reshape(-1)
        U_test = np.tile(u_now, N)

        X_repeated, A_bar, B_bar, c_bar = self.predict_horizon(
            x0=x,
            U=U_test,
            horizon=N,
        )
        X_repeated = X_repeated.reshape(N, -1)

        # 실제 MPC
        U_opt, contact_schedule, U_ref = (
            self.solve_horizon_mpc(
                x0=x,
                x_ref=x_ref,
            )
        )
        F0_mpc = None

        if U_opt is not None:
            F0_mpc = U_opt[0].copy()

            # 현재 스텝의 실제 MPC contact schedule로 마스킹
            F0_mpc[~contact_schedule[0]] = 0.0

            X_mpc, _, _, _ = self.predict_horizon(
                x0=x,
                U=U_opt.reshape(-1),
                horizon=N,
            )
            X_mpc = X_mpc.reshape(N, -1)

            U_ref_matrix = U_ref.reshape(N, 4, 3)

            self.get_logger().info(
                f"MPC result "
                f"gait={self.gait_type}, "
                f"schedule={contact_schedule.astype(int).tolist()}, "
                f"Uref_z="
                f"{np.round(U_ref_matrix[:, :, 2], 1).tolist()}, "
                f"F0_sum_z={np.sum(F0_mpc[:, 2]):.2f}, "
                f"F0={np.round(F0_mpc, 2).tolist()}, "
                f"wx={np.round(X_mpc[:, 3], 2).tolist()}, "
                f"wy={np.round(X_mpc[:, 4], 2).tolist()}, "
                f"h={np.round(X_mpc[:, 9], 4).tolist()}",
                throttle_duration_sec=1.0,
            )

        if not self.enable_torque:
            self.publish_zero_torque()
            return

        # MPC 계산 실패 시 기존 힘을 쓰지 않고 안전하게 0 토크
        if F0_mpc is None:
            self.get_logger().warn(
                "MPC force unavailable; publishing zero torque",
                throttle_duration_sec=1.0,
            )
            self.publish_zero_torque()
            return

        raw_tau = self.compute_mpc_torque_ff(
            F0_mpc=F0_mpc,
            contact_mask=contact_schedule[0],
        )

        tau = np.clip(
            raw_tau,
            self.last_tau_cmd - self.max_tau_step,
            self.last_tau_cmd + self.max_tau_step,
        )

        tau = np.clip(
            tau,
            -self.torque_limit,
            self.torque_limit,
        )

        # swing 다리는 잔류 토크까지 즉시 제거
        for leg in range(4):
            if not contact_schedule[0][leg]:
                start = leg * 3
                end = start + 3

                tau[start:end] = 0.0


        self.last_tau_cmd = tau.copy()

        msg = Float32MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)

        self.get_logger().info(
            f"MPC torque "
            f"mask={contact_schedule[0].astype(int).tolist()}, "
            f"raw={np.round(raw_tau, 3).tolist()}, "
            f"limited={np.round(tau, 3).tolist()}",
            throttle_duration_sec=1.0,
        )

        self.get_logger().info(
            f"MPC torque "
            f"mask={contact_schedule[0].astype(int).tolist()}, "
            f"F0={np.round(F0_mpc, 2).tolist()}, "
            f"tau={np.round(tau, 3).tolist()}",
            throttle_duration_sec=1.0,
        )

    def leg_pos_body(self, leg, q_leg):
        p_delta = self.leg_fk_local(leg, q_leg)

        base_x = self.hip_x / 2.0 if leg in [0, 1] else -self.hip_x / 2.0
        base_y = self.foot_y / 2.0 if leg in [0, 2] else -self.foot_y / 2.0

        p_ref = np.array([base_x, base_y, -0.300], dtype=np.float64)
        return p_ref + p_delta
    
    def skew(self, r):
        x, y, z = r
        return np.array([
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ], dtype=np.float64)


    def solve_standing_qp_osqp(self):
        mu = 0.5
        fz_min = 2.0
        fz_max = 80.0
        reg = 1e-4

        A_wrench = np.zeros((6, 12), dtype=np.float64)

        for leg in range(4):
            q_leg = self.q12[leg * 3:(leg + 1) * 3]
            r = self.leg_pos_body(leg, q_leg)

            A_wrench[0:3, leg*3:(leg+1)*3] = np.eye(3)
            A_wrench[3:6, leg*3:(leg+1)*3] = self.skew(r)

        #imu pid
        Kp_roll = 2.0
        Kd_roll = 0.03
        Kp_pitch = 2.0
        Kd_pitch = 0.03
        Kd_yaw = 0.3

        if not self.imu_ready:
            tau_body = np.zeros(3)
        else:
            tau_body = np.array([
                #디버깅 다시 제대로 하기 kp des - now + kd des -now
                Kp_roll*(0.0-self.roll)  + Kd_roll*(0.0-self.gyro[0]),
                Kp_pitch*(0.0-self.pitch) + Kd_pitch*(0.0-self.gyro[1]),
                Kd_yaw*(0.0-self.gyro[2]),
            ])

        b = np.array([
            0,
            0,
            -self.robot_mass*9.81,

            tau_body[0],
            tau_body[1],
            tau_body[2],
        ])

        P = 2.0 * (A_wrench.T @ A_wrench + reg * np.eye(12))
        q = -2.0 * (A_wrench.T @ b)

        # 어느 다리에 힘을 주게 할 것인지
        A_con, l, u = self.build_contact_constraints(
            stance_mask=self.stance_mask,
            mu=mu,
            fz_min=fz_min,
            fz_max=fz_max,
        )
        solver = osqp.OSQP()
        solver.setup(
            P=sp.csc_matrix(P),
            q=q,
            A=sp.csc_matrix(A_con),
            l=l,
            u=u,
            verbose=False,
            polish=False,
            max_iter=1000,
            eps_abs=1e-3,
            eps_rel=1e-3,
        )

        res = solver.solve()
        #qp 실패했을 때 이전거를 쓰는게 아니라 마스크까지 씌워서 해야함 안그러면 못버팀
        if res.info.status_val not in [1, 2]:
            self.get_logger().warn(
                f"OSQP failed: {res.info.status}, use last solution",
                throttle_duration_sec=1.0,
            )
            fallback_F = self.last_F_all.copy()
            fallback_F[~self.stance_mask] = 0.0
            return fallback_F
        
        F_all = res.x.reshape(4,3)
        #혹시 모르니까 마스킹 하기
        F_all[~self.stance_mask] = 0.0

        F_all[:, 0] *= 0.5   # Fx 약화
        F_all[:, 1] *= 0.5   # Fy 약화

        self.last_F_all = F_all.copy()

        self.get_logger().info(
            f"QP F_all={np.round(F_all, 2).tolist()}",
            throttle_duration_sec=1.0,
        )

        return F_all
    
    def get_desired_state(self):
        return np.array([
            self.desired_roll,
            self.desired_pitch,
            self.desired_yaw,

            self.desired_wx,
            self.desired_wy,
            self.desired_wz,

            self.desired_vx,
            self.desired_vy,
            self.desired_vz,

            self.desired_height,
        ], dtype=np.float64)

    def get_current_state(self):
        """
        현재 로봇 몸체 상태를 MPC 상태 벡터 형태로 반환한다.

        초기 구현:
        [roll, pitch, yaw,
        wx, wy, wz,
         vx, vy, vz,
         height]
        """

        yaw = 0.0

        wx = self.gyro[0]
        wy = self.gyro[1]
        wz = self.gyro[2]

        # 아직 body velocity estimator가 없으므로 초기에는 0으로 둔다.
        vx = 0.0
        vy = 0.0
        vz = 0.0

        # 현재 body 높이 추정값.
        # 초기에는 고정값 또는 다리 FK 평균으로 계산할 수 있다.
        height = self.estimate_body_height()

        return np.array([
            self.roll,
            self.pitch,
            yaw,
            wx,
            wy,
            wz,
            vx,
            vy,
            vz,
            height,
        ], dtype=np.float64)
    
    #저역통과 필터 적용(fk를 이용한 현재 몸체 높이 추정하는 함수 - 부정확함)
    def estimate_body_height(self):
        heights = []

        for leg in range(4):
            if not self.stance_mask[leg]:
                continue

            q_leg = self.q12[leg * 3:(leg + 1) * 3]
            p_foot = self.leg_pos_body(leg, q_leg)

            height = -p_foot[2]

            if np.isfinite(height):
                heights.append(height)

        if not heights:
            return self.estimated_height

        raw_height = float(np.mean(heights))

        alpha = self.height_filter_alpha
        self.estimated_height = (
            (1.0 - alpha) * self.estimated_height
            + alpha * raw_height
        )

        return self.estimated_height
    
    #스텐스발만 만들기
    def build_contact_constraints(
        self,
        stance_mask,
        mu,
        fz_min,
        fz_max,
    ):
        stance_mask = np.asarray(
            stance_mask,
            dtype=bool,
        )

        if stance_mask.shape != (4,):
            raise ValueError(
                f"stance_mask must have shape (4,), "
                f"got {stance_mask.shape}"
            )

        A_list = []
        l_list = []
        u_list = []

        for leg in range(4):
            ix = leg * 3
            iy = ix + 1
            iz = ix + 2

            if stance_mask[leg]:
                # -fz_max <= Fz <= -fz_min
                row = np.zeros(12, dtype=np.float64)
                row[iz] = 1.0
                A_list.append(row)
                l_list.append(-fz_max)
                u_list.append(-fz_min)

                # Fx + mu*Fz <= 0
                row = np.zeros(12, dtype=np.float64)
                row[ix] = 1.0
                row[iz] = mu
                A_list.append(row)
                l_list.append(-np.inf)
                u_list.append(0.0)

                # -Fx + mu*Fz <= 0
                row = np.zeros(12, dtype=np.float64)
                row[ix] = -1.0
                row[iz] = mu
                A_list.append(row)
                l_list.append(-np.inf)
                u_list.append(0.0)

                # Fy + mu*Fz <= 0
                row = np.zeros(12, dtype=np.float64)
                row[iy] = 1.0
                row[iz] = mu
                A_list.append(row)
                l_list.append(-np.inf)
                u_list.append(0.0)

                # -Fy + mu*Fz <= 0
                row = np.zeros(12, dtype=np.float64)
                row[iy] = -1.0
                row[iz] = mu
                A_list.append(row)
                l_list.append(-np.inf)
                u_list.append(0.0)

            else:
                # swing 발은 Fx=Fy=Fz=0
                for index in (ix, iy, iz):
                    row = np.zeros(12, dtype=np.float64)
                    row[index] = 1.0

                    A_list.append(row)
                    l_list.append(0.0)
                    u_list.append(0.0)

        return (
            np.vstack(A_list),
            np.asarray(l_list, dtype=np.float64),
            np.asarray(u_list, dtype=np.float64),
        )
    
    def approximate_body_inertia(self):
        """
        전체 로봇을 하나의 직육면체 강체로 근사한 관성행렬.

        초기 Convex MPC 모델용 근사값:
        mass = 9.5 kg
        body size = 0.81 x 0.24 x 0.10 m
        """
        return np.diag([
            0.054,
            0.527,
            0.565,
        ]).astype(np.float64)
    
    def build_state_space_model(self):
        dt = 1.0 / self.control_hz

        nx = 10
        nu = 12

        A = np.eye(nx, dtype=np.float64)
        B = np.zeros((nx, nu), dtype=np.float64)
        c = np.zeros(nx, dtype=np.float64)

        # 현재 각도와 각속도가 degree, degree/s 단위
        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt

        # 높이 적분
        A[9, 8] = dt

        rad_to_deg = 180.0 / math.pi

        for leg in range(4):
            q_leg = self.q12[leg * 3:(leg + 1) * 3]
            r = self.leg_pos_body(leg, q_leg)

            col = leg * 3

            # 접촉력 → 선속도 변화
            B[6:9, col:col + 3] = (
                dt / self.robot_mass
            ) * np.eye(3)

            # 접촉력 모멘트 → 각속도 변화
            # rad/s → degree/s 단위 변환 포함
            B[3:6, col:col + 3] = (
                rad_to_deg
                * dt
                * self.body_inertia_inv
                @ self.skew(r)
            )

        # ΣFz=-mg일 때 선속도 변화가 상쇄되도록 설정
        c[8] = 9.81 * dt

        return A, B, c

    def predict_one_step(self, x, u):
        A, B, c = self.build_state_space_model()
        return A @ x + B @ u + c
    
    def build_prediction_matrices(self, A, B, c, horizon):
        """
        이산 선형 시스템

            x[k+1] = A x[k] + B u[k] + c

        를 horizon 길이만큼 쌓아서

            X = A_bar x0 + B_bar U + c_bar

        형태의 예측행렬을 만든다.
        """
        nx = A.shape[0]
        nu = B.shape[1]
        N = int(horizon)

        A_bar = np.zeros((nx * N, nx), dtype=np.float64)
        B_bar = np.zeros((nx * N, nu * N), dtype=np.float64)
        c_bar = np.zeros(nx * N, dtype=np.float64)

        # c 누적값:
        # x1: c
        # x2: A c + c
        # x3: A²c + Ac + c
        c_acc = np.zeros(nx, dtype=np.float64)

        for i in range(N):
            row = slice(i * nx, (i + 1) * nx)

            # x0가 xi+1에 미치는 영향
            A_power = np.linalg.matrix_power(A, i + 1)
            A_bar[row, :] = A_power

            # 상수항 누적
            c_acc = A @ c_acc + c
            c_bar[row] = c_acc

            # 각 과거 입력 uj가 xi+1에 미치는 영향
            for j in range(i + 1):
                col = slice(j * nu, (j + 1) * nu)

                power = i - j
                B_bar[row, col] = (
                    np.linalg.matrix_power(A, power) @ B
                )

        return A_bar, B_bar, c_bar
    
    #예측 함수
    def predict_horizon(self, x0, U, horizon=None):
        if horizon is None:
            horizon = self.mpc_horizon

        A, B, c = self.build_state_space_model()

        A_bar, B_bar, c_bar = self.build_prediction_matrices(
            A=A,
            B=B,
            c=c,
            horizon=horizon,
        )

        X = A_bar @ x0 + B_bar @ U + c_bar

        return X, A_bar, B_bar, c_bar
    
    def build_reference_horizon(self, x_ref, horizon):
        return np.tile(x_ref, horizon)
    
    def build_state_weight_matrix(self):
        return np.diag([
            40.0,   # roll
            40.0,   # pitch
            5.0,    # yaw

            2.0,    # wx
            2.0,    # wy
            1.0,    # wz

            1.0,    # vx
            1.0,    # vy
            3.0,    # vz

            80.0,   # height
        ]).astype(np.float64)
    
    def build_input_weight_matrix(self):
        return 1e-3 * np.eye(12, dtype=np.float64)
    

    def build_horizon_weight_matrices(self, Q, R, horizon):
        Q_bar = sp.block_diag([Q] * horizon).toarray()
        R_bar = sp.block_diag([R] * horizon).toarray()

        return Q_bar, R_bar
    
    def build_mpc_cost(
        self,
        x0,
        x_ref,
        A_bar,
        B_bar,
        c_bar,
        horizon,
        U_ref,
    ):
        Q = self.build_state_weight_matrix()
        R = self.build_input_weight_matrix()

        Q_bar, R_bar = self.build_horizon_weight_matrices(
            Q,
            R,
            horizon,
        )

        X_ref = self.build_reference_horizon(
            x_ref,
            horizon,
        )

        U_ref = np.asarray(
            U_ref,
            dtype=np.float64,
        ).reshape(-1)

        expected_size = 12 * horizon

        if U_ref.size != expected_size:
            raise ValueError(
                f"U_ref size must be {expected_size}, "
                f"got {U_ref.size}"
            )

        free_response = A_bar @ x0 + c_bar
        error = free_response - X_ref

        P = 2.0 * (
            B_bar.T @ Q_bar @ B_bar
            + R_bar
        )

        q = 2.0 * (
            B_bar.T @ Q_bar @ error
            - R_bar @ U_ref
        )

        P = 0.5 * (P + P.T)

        return P, q, X_ref
    
    def build_horizon_contact_constraints(
        self,
        contact_schedule,
        mu,
        fz_min,
        fz_max,
    ):
        contact_schedule = np.asarray(
            contact_schedule,
            dtype=bool,
        )

        if (
            contact_schedule.ndim != 2
            or contact_schedule.shape[1] != 4
        ):
            raise ValueError(
                f"contact_schedule must have shape (N, 4), "
                f"got {contact_schedule.shape}"
            )

        A_blocks = []
        l_blocks = []
        u_blocks = []

        for k in range(contact_schedule.shape[0]):
            A_step, l_step, u_step = (
                self.build_contact_constraints(
                    stance_mask=contact_schedule[k],
                    mu=mu,
                    fz_min=fz_min,
                    fz_max=fz_max,
                )
            )

            A_blocks.append(sp.csc_matrix(A_step))
            l_blocks.append(l_step)
            u_blocks.append(u_step)

        return (
            sp.block_diag(A_blocks, format="csc"),
            np.concatenate(l_blocks),
            np.concatenate(u_blocks),
        )
        

    def solve_horizon_mpc(self, x0, x_ref):
        N = self.mpc_horizon

        A, B, c = self.build_state_space_model()

        A_bar, B_bar, c_bar = (
            self.build_prediction_matrices(
                A=A,
                B=B,
                c=c,
                horizon=N,
            )
        )

        # 미래 접촉 상태 예측
        contact_schedule = (
            self.build_future_contact_schedule(N)
        )

        # 접촉 상태별 명목 중력보상력
        U_ref = self.build_nominal_force_horizon(
            contact_schedule
        )

        P, q, X_ref = self.build_mpc_cost(
            x0=x0,
            x_ref=x_ref,
            A_bar=A_bar,
            B_bar=B_bar,
            c_bar=c_bar,
            horizon=N,
            U_ref=U_ref,
        )

        A_con, l, u = (
            self.build_horizon_contact_constraints(
                contact_schedule=contact_schedule,
                mu=0.5,
                fz_min=2.0,
                fz_max=80.0,
            )
        )

        solver = osqp.OSQP()

        solver.setup(
            P=sp.csc_matrix(P),
            q=q,
            A=A_con,
            l=l,
            u=u,
            verbose=False,
            polish=False,
            max_iter=2000,
            eps_abs=1e-3,
            eps_rel=1e-3,
        )

        result = solver.solve()

        if result.info.status_val not in (1, 2):
            self.get_logger().warn(
                f"MPC failed: {result.info.status}",
                throttle_duration_sec=1.0,
            )

            return None, contact_schedule, U_ref

        U_opt = result.x.reshape(N, 4, 3)

        return U_opt, contact_schedule, U_ref
    
    def build_nominal_force_horizon(
        self,
        contact_schedule,
    ):
        contact_schedule = np.asarray(
            contact_schedule,
            dtype=bool,
        )

        N = contact_schedule.shape[0]

        U_ref = np.zeros(
            (N, 4, 3),
            dtype=np.float64,
        )

        total_support_force = (
            -self.robot_mass * 9.81
        )

        for k in range(N):
            stance_indices = np.flatnonzero(
                contact_schedule[k]
            )

            contact_count = stance_indices.size

            if contact_count == 0:
                continue

            nominal_fz = (
                total_support_force
                / contact_count
            )

            U_ref[k, stance_indices, 2] = nominal_fz

        return U_ref.reshape(-1)
    
    def build_future_contact_schedule(self, horizon):
        """
        현재 gait 상태로부터 horizon 각 스텝의 접촉 상태를 예측한다.

        반환:
            schedule.shape == (N, 4)
            True  = stance
            False = swing
        """
        N = int(horizon)
        dt = 1.0 / self.control_hz

        schedule = np.zeros((N, 4), dtype=bool)

        # stand에서는 phase 예측 없이 네 발 고정 접촉
        if self.gait_type == "stand":
            schedule[:, :] = True
            return schedule

        # trot timing 검증
        if self.gait_t_stance <= 0.0 or self.gait_t_swing <= 0.0:
            self.get_logger().warn(
                f"Invalid gait timing: "
                f"t_stance={self.gait_t_stance:.4f}, "
                f"t_swing={self.gait_t_swing:.4f}; "
                f"using current contact mask",
                throttle_duration_sec=1.0,
            )

            schedule[:, :] = self.stance_mask
            return schedule

        # 첫 입력은 현재 gait planner가 보낸 값을 그대로 사용
        schedule[0] = self.stance_mask.copy()

        for leg in range(4):
            initial_mode = bool(self.stance_mask[leg])
            initial_phase = float(
                np.clip(self.gait_phase[leg], 0.0, 1.0)
            )

            for k in range(1, N):
                future_time = k * dt
                mode = initial_mode

                current_duration = (
                    self.gait_t_stance
                    if mode
                    else self.gait_t_swing
                )

                # 현재 구간에서 남아 있는 시간
                remaining_time = (
                    1.0 - initial_phase
                ) * current_duration

                if future_time < remaining_time:
                    schedule[k, leg] = mode
                    continue

                # 현재 구간을 벗어났으면 다음 구간으로 이동
                future_time -= remaining_time
                mode = not mode

                while True:
                    mode_duration = (
                        self.gait_t_stance
                        if mode
                        else self.gait_t_swing
                    )

                    if future_time < mode_duration:
                        schedule[k, leg] = mode
                        break

                    future_time -= mode_duration
                    mode = not mode

        return schedule
    
    def publish_zero_torque(self):
        self.last_tau_cmd[:] = 0.0

        msg = Float32MultiArray()
        msg.data = [0.0] * 12
        self.torque_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPCController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_zero_torque()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()