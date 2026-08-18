#!/usr/bin/env python3

import math
import numpy as np
from quattro.Kinematics.LieAlgebra import TransToRp
import copy

STANCE = 0
SWING = 1

# 베지에 곡선 출처: https://dspace.mit.edu/handle/1721.1/98270
# 회전 로직 출처: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf


class BezierGait():
    def __init__(self, dSref=None, dt=0.01, Tswing=0.2):
        # 다리별 위상 지연(Phase Lag): 앞왼(FL), 앞오른(FR), 뒤왼(BL), 뒤오른(BR)
        # 기준 다리는 FL(앞왼발)이며, 항상 0임
        if dSref is None:
            dSref = [0.0, 0.5, 0.5, 0.0]
        self.dSref = list(dSref)
        self.Prev_fxyz = [0.0, 0.0, 0.0, 0.0]
        # 제어점의 개수는 n + 1 = 11 + 1 = 12개
        self.NumControlPoints = 11
        # 타임스텝
        self.dt = dt

        # 총 경과 시간
        self.time = 0.0
        # 착지(Touchdown) 시각
        self.TD_time = 0.0
        # 마지막 착지 이후 경과 시간
        self.time_since_last_TD = 0.0
        # 궤적 모드 (스윙/스탠스)
        self.StanceSwing = SWING
        # 기준 발의 스윙 위상 값 [0, 1]
        self.SwRef = 0.0
        self.Stref = 0.0
        # 기준 발이 착지했는지 여부
        self.TD = False

        # 스탠스(지지) 시간
        self.Tswing = Tswing

        # 기준 다리 인덱스
        self.ref_idx = 0

        # 모든 다리의 현재 위상
        self.Phases = [0.0] * 4

        # 각 다리의 현재 상태
        self.LegState = [STANCE] * 4

        #
        self.LegState = [STANCE] * 4
        #상태 저장용 함수
        self.current_tstance = 0.0
        self.current_tswing = Tswing
        self.current_tstride = Tswing

    def reset(self):
        """베지에 보행 생성기의 파라미터를 초기화합니다.
        """
        self.Prev_fxyz = [0.0, 0.0, 0.0, 0.0]

        # 총 경과 시간
        self.time = 0.0
        # 착지 시각
        self.TD_time = 0.0
        # 마지막 착지 이후 경과 시간
        self.time_since_last_TD = 0.0
        # 궤적 모드
        self.StanceSwing = SWING
        # 기준 발의 스윙 위상 값 [0, 1]
        self.SwRef = 0.0
        self.Stref = 0.0
        # 기준 발이 착지했는지 여부
        self.TD = False

    def GetPhase(self, index, Tstance, Tswing):
        """개별 다리의 위상을 반환합니다.

        참고: 원본 논문에서의 수정 사항:

        if ti < -Tswing:
           ti += Tstride

        이는 사용자가 Tstance > Tswing이 되는 보폭과 속도 조합을
        선택했을 때 위상 불연속성을 방지하기 위함입니다.

        :param index: 다리 인덱스, 필요한 위상 지연을 식별하는 데 사용
        :param Tstance: 현재 사용자가 지정한 스탠스(지지) 주기
        :param Tswing: 스윙 주기 (상수, 클래스 멤버)
        :return: 다리 위상, 그리고 다리가 스탠스 모드인지 스윙 모드인지 나타내는 StanceSwing(bool)
        """
        StanceSwing = STANCE
        Sw_phase = 0.0
        Tstride = Tstance + Tswing
        ti = self.Get_ti(index, Tstride)

        # 참고: 논문에는 이 로직이 빠져 있었음!!
        if ti < -Tswing:
            ti += Tstride

        # 스탠스(지지) 단계
        if ti >= 0.0 and ti <= Tstance:
            StanceSwing = STANCE
            if Tstance == 0.0:
                Stnphase = 0.0
            else:
                Stnphase = ti / float(Tstance)
            if index == self.ref_idx:
                # print("STANCE REF: {}".format(Stnphase))
                self.StanceSwing = StanceSwing
            return Stnphase, StanceSwing
        # 스윙(유영) 단계
        elif ti >= -Tswing and ti < 0.0:
            StanceSwing = SWING
            Sw_phase = (ti + Tswing) / Tswing
        elif ti > Tstance and ti <= Tstride:
            StanceSwing = SWING
            Sw_phase = (ti - Tstance) / Tswing
        # 스윙 끝에서의 착지
        if Sw_phase >= 1.0:
            Sw_phase = 1.0
        if index == self.ref_idx:
            # print("SWING REF: {}".format(Sw_phase))
            self.StanceSwing = StanceSwing
            self.SwRef = Sw_phase
            # 스윙 끝에서 기준 발 착지
            if self.SwRef >= 0.999:
                self.TD = True
            # else:
            #     self.TD = False
        return Sw_phase, StanceSwing

    def Get_ti(self, index, Tstride):
        """개별 다리의 시간 인덱스를 반환합니다.

        :param index: 다리 인덱스, 필요한 위상 지연을 식별하는 데 사용
        :param Tstride: 총 다리 운동 주기 (Tstance + Tswing)
        :return: 다리의 시간 인덱스
        """
        # 참고: 어떤 이유로 파이썬에서 수치적 문제가 발생하여
        # 기준 다리를 강제로 0으로 설정함
        if index == self.ref_idx:
            self.dSref[index] = 0.0
        return self.time_since_last_TD - self.dSref[index] * Tstride

    def Increment(self, dt, Tstride):
        """베지에 보행 생성기의 내부 시계(self.time)를 증가시킵니다.

        :param dt: 타임 스텝
        :param Tstride: 총 다리 운동 주기 (Tstance + Tswing)
        :return: 다리의 시간 인덱스
        """
        self.CheckTouchDown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        # print("T STRIDE: {}".format(Tstride))
        # 착지가 방금 발생했을 경우를 대비해 마지막에 시간을 증가시킴
        # time_since_last_TD가 0.0이 되도록 하기 위함
        self.time += dt

        # Tstride = Tswing이면, Tstance = 0임
        # 전체 초기화
        if Tstride < self.Tswing + dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def CheckTouchDown(self):
        """기준 다리의 착지가 발생했는지 확인하고,
           착지 시간을 재설정해야 하는지 판단합니다.
        """
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def BernSteinPoly(self, t, k, point):
        """위상(0->1), 제어점 번호(0-11), 제어점 값 자체를 기반으로
           번스타인 다항식 위의 점을 계산합니다.

           :param t: 위상 (phase)
           :param k: 포인트 번호
           :param point: 포인트 값
           :return: 베지에 곡선을 통과한 값
        """
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumControlPoints - k)

    def Binomial(self, k):
        """총 베지에 포인트 수에 대한 베지에 포인트 번호(k)의
           이항 정리를 풉니다.

           :param k: 베지에 포인트 번호
           :returns: 이항 해답 (Binomial solution)
        """
        return math.factorial(self.NumControlPoints) / (
            math.factorial(k) *
            math.factorial(self.NumControlPoints - k))

    def BezierSwing(self, phase, L, LateralFraction, clearance_height=0.04):
        """베지에(스윙) 구간의 발걸음 좌표를 계산합니다.

           :param phase: 현재 궤적 위상
           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param clearance_height: 스윙 단계에서의 발 높이(Clearance)

           :returns: 수정되지 않은 몸체에 대한 X,Y,Z 발 좌표
        """

        # 극좌표계 다리 좌표
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        # 베지에 곡선 포인트 (12개 점)
        # 참고: L은 보폭(STEP LENGTH)의 절반임
        # 전진 성분
        STEP = np.array([
            -L,  # 제어점 0, 보폭의 절반
            -L * 1.4,  # 제어점 1, 1과 0의 차이 = X축 들어올리기 속도
            -L * 1.5,  # 제어점 2, 3, 4는 다음을 위해 겹쳐 있음
            -L * 1.5,  # 팔로우 스루(동작 연결) 이후
            -L * 1.5,  # 방향 전환
            0.0,  # 내딛기(Protraction) 중 가속도 변화
            0.0,  # 그래서 3개를 포함함
            0.0,  # 겹쳐진 제어점: 5, 6, 7
            L * 1.5,  # 스윙 다리 당기기(Retraction)를 위한 방향 전환
            L * 1.5,  # 두 번 겹친 제어점 필요: 8, 9
            L * 1.4,  # 스윙 다리 당기기 속도 = 제어점 11 - 10
            L
        ])
        # 횡방향 움직임을 극좌표 곱셈으로 반영
        # LateralFraction은 0에서 멀어질수록 다리 움직임을 X에서 Y+ 또는 Y-로 전환함
        X = STEP * X_POLAR

        # 횡방향 움직임을 극좌표 곱셈으로 반영
        # LateralFraction은 0에서 멀어질수록 다리 움직임을 X에서 Y+ 또는 Y-로 전환함
        Y = STEP * Y_POLAR

        # 수직 성분
        Z = np.array([
            0.0,  # 리프트 0을 위한 두 번 겹친 제어점
            0.0,  # 힙(Hip)에 대한 속도 (점 0과 1)
            clearance_height * 0.9,  # 팔로우 스루에서 내딛기로 전환되는 동안의
            clearance_height * 0.9,  # 힘 방향 변화를 위한
            clearance_height * 0.9,  # 세 번 겹친 제어점 (2, 3, 4)
            clearance_height * 0.9,  # 궤적을 위한 두 번 겹친 제어점
            clearance_height * 0.9,  # 내딛기 중 방향 전환 (5, 6)
            clearance_height * 1.1,  # 궤적 중간에서의 최대 높이(Clearance), 점 7
            clearance_height * 1.1,  # 내딛기에서 당기기로의 부드러운 전환
            clearance_height * 1.1,  # 두 개의 제어점 (8, 9)
            0.0,  # 착지(Touchdown) 시 0을 위한 두 번 겹친 제어점
            0.0,  # 힙(Hip)에 대한 속도 (점 10과 11)
        ])

        stepX = 0.
        stepY = 0.
        stepZ = 0.
        # 제어점들에 대한 번스타인 다항식 합계
        for i in range(len(X)):
            stepX += self.BernSteinPoly(phase, i, X[i])
            stepY += self.BernSteinPoly(phase, i, Y[i])
            stepZ += self.BernSteinPoly(phase, i, Z[i])

        return stepX, stepY, stepZ

    def SineStance(self, phase, L, LateralFraction, penetration_depth=0.00):
        """사인파 스탠스(지지) 구간의 발걸음 좌표를 계산합니다.

           :param phase: 현재 궤적 위상
           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param penetration_depth: 스탠스 단계에서의 발 침투 깊이(가상)

           :returns: 수정되지 않은 몸체에 대한 X,Y,Z 발 좌표
        """
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        # +L에서 -L로 이동
        step = L * (1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        if L != 0.0:
            stepZ = -penetration_depth * np.cos(
                (np.pi * (stepX + stepY)) / (2.0 * L))
        else:
            stepZ = 0.0
        return stepX, stepY, stepZ

    def YawCircle(self, T_bf, index):
        """ Yaw 동작(회전)을 위해 필요한 궤적 평면의 회전을 계산합니다.

           :param T_bf: 기본 몸통-발 벡터
           :param index: 컨테이너 내의 발 인덱스
           :returns: phi_arc, Yaw 동작에 필요한 평면 회전 각도
        """

        # 다리 종류에 따른 발의 크기(거리)
        DefaultBodyToFoot_Magnitude = np.sqrt(T_bf[0]**2 + T_bf[1]**2)

        # 다리 종류에 따른 회전 각도
        DefaultBodyToFoot_Direction = np.arctan2(T_bf[1], T_bf[0])

        # 기본 좌표에 대한 이전 다리 좌표
        g_xyz = self.Prev_fxyz[index] - np.array([T_bf[0], T_bf[1], T_bf[2]])

        # 원을 계속 추적하기 위해 크기 조절
        g_mag = np.sqrt((g_xyz[0])**2 + (g_xyz[1])**2)
        th_mod = np.arctan2(g_mag, DefaultBodyToFoot_Magnitude)

        # 회전을 위해 발이 그리는 각도
        # FR(앞오른)과 BL(뒤왼)
        if index == 1 or index == 2:
            phi_arc = np.pi / 2.0 + DefaultBodyToFoot_Direction + th_mod
        # FL(앞왼)과 BR(뒤오른)
        else:
            phi_arc = np.pi / 2.0 - DefaultBodyToFoot_Direction + th_mod

        # print("INDEX {}: \t Angle: {}".format(
        #     index, np.degrees(DefaultBodyToFoot_Direction)))

        return phi_arc

    def SwingStep(self, phase, L, LateralFraction, YawRate, clearance_height,
                  T_bf, key, index):
        """사용자 입력(L, LateralFraction, YawRate)에서 분해된
           전진 및 회전 좌표의 조합을 사용하여 베지에(스윙) 구간의
           발걸음 좌표를 계산합니다.

           :param phase: 현재 궤적 위상
           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param YawRate: 원하는 몸통 Yaw 비율
           :param clearance_height: 스윙 단계에서의 발 높이(Clearance)
           :param T_bf: 기본 몸통-발 벡터
           :param key: 처리 중인 발을 나타냄
           :param index: 컨테이너 내의 발 인덱스

           :returns: 수정되지 않은 몸체에 대한 발 좌표
        """

        # 원의 접선 운동을 위한 Yaw 발 각도
        phi_arc = self.YawCircle(T_bf, index)

        # 전진 운동을 위한 발 좌표 구하기
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.BezierSwing(
            phase, L, LateralFraction, clearance_height)

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.BezierSwing(
            phase, YawRate, phi_arc, clearance_height)

        coord = np.array([
            X_delta_lin + X_delta_rot, Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])

        self.Prev_fxyz[index] = coord

        return coord

    def StanceStep(self, phase, L, LateralFraction, YawRate, penetration_depth,
                   T_bf, key, index):
        """사용자 입력(L, LateralFraction, YawRate)에서 분해된
           전진 및 회전 좌표의 조합을 사용하여 사인(스탠스) 구간의
           발걸음 좌표를 계산합니다.

           :param phase: 현재 궤적 위상
           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param YawRate: 원하는 몸통 Yaw 비율
           :param penetration_depth: 스탠스 단계에서의 발 침투 깊이
           :param T_bf: 기본 몸통-발 벡터
           :param key: 처리 중인 발을 나타냄
           :param index: 컨테이너 내의 발 인덱스

           :returns: 수정되지 않은 몸체에 대한 발 좌표
        """

        # 원의 접선 운동을 위한 Yaw 발 각도
        phi_arc = self.YawCircle(T_bf, index)

        # 전진 운동을 위한 발 좌표 구하기
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.SineStance(
            phase, L, LateralFraction, penetration_depth)

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.SineStance(
            phase, YawRate, phi_arc, penetration_depth)

        coord = np.array([
            X_delta_lin + X_delta_rot, Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])

        self.Prev_fxyz[index] = coord

        return coord

    def GetFootStep(self, L, LateralFraction, YawRate, clearance_height,
                    penetration_depth, Tstance, T_bf, index, key):
        """조회된 위상에 따라 궤적의 베지에 또는 사인 부분에서
           발걸음 좌표를 계산합니다.

           :param phase: 현재 궤적 위상
           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param YawRate: 원하는 몸통 Yaw 비율
           :param clearance_height: 스윙 단계에서의 발 높이(Clearance)
           :param penetration_depth: 스탠스 단계에서의 발 침투 깊이
           :param Tstance: 현재 사용자가 지정한 스탠스(지지) 주기
           :param T_bf: 기본 몸통-발 벡터
           :param index: 컨테이너 내의 발 인덱스
           :param key: 처리 중인 발을 나타냄

           :returns: 수정되지 않은 몸체에 대한 발 좌표
        """
        phase, StanceSwing = self.GetPhase(index, Tstance, self.Tswing)

        self.LegState[index] = StanceSwing

        self.Phases[index] = phase

        # print("LEG: {} \t PHASE: {}".format(index, stored_phase))
        if StanceSwing == STANCE:
            return self.StanceStep(phase, L, LateralFraction, YawRate,
                                   penetration_depth, T_bf, key, index)
        elif StanceSwing == SWING:
            return self.SwingStep(phase, L, LateralFraction, YawRate,
                                  clearance_height, T_bf, key, index)
        

    def GenerateTrajectory(self,
                           L,
                           LateralFraction,
                           YawRate,
                           vel,
                           T_bf_,
                           T_bf_curr,
                           clearance_height=0.06,
                           penetration_depth=0.01,
                           contacts=[0, 0, 0, 0],
                           dt=None):
        """각 발의 발걸음 좌표를 계산합니다.

           :param L: 보폭 (Step Length)
           :param LateralFraction: 횡방향 이동 정도를 결정
           :param YawRate: 원하는 몸통 Yaw 비율
           :param vel: 원하는 보행 속도
           :param clearance_height: 스윙 단계에서의 발 높이(Clearance)
           :param penetration_depth: 스탠스 단계에서의 발 침투 깊이
           :param contacts: 접촉 시 1, 아니면 0을 포함하는 배열
           :param dt: 타임 스텝

           :returns: 수정되지 않은 몸체에 대한 발 좌표
        """

        # 발을 땅에 딛고 있는 시간(Tstance)을 속도(vel)와 보폭(L)을 이용해 계산
        # vel 이 0이 아니면(움직이는 중이면)
        if vel != 0.0:
            # 시간 = 거리 / 속도
            # 보폭은 L(StepLength)의 절반이므로 2 * L 이 전체 보폭임
            Tstance = 2.0 * abs(L) / abs(vel)
        else:
            # 멈춰 있다면 모든 타이머와 상태를 초기화
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0

        # dt 및 YawRate 업데이트
        # dt가 입력되지 않았으면 클래스 내부 값(self.dt)을 사용
        if dt is None:
            dt = self.dt

        # 초당 회전 각도(Yawrate)에 dt를 곱해서 이번 스텝에 회전해야 할 각도(rad)로 변환
        YawRate *= dt


        # 계산된 지지 시간(Tstance)이 물리적으로 가능한지 검사
        # 실행 불가능한 경우: 지지 시간(Tstance)이 한 스탭(dt)보다 짧으면 계산 불가능하므로 멈춤
        if Tstance < dt:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0
            YawRate = 0.0
        # 지지 시간(Tstance)이 스윙 시간(self.Tswing)의 1.3배를 넘지 않게 자름
        # 너무 오래 지지하고 있으면 불안정해질 수 있으므로 제한
        elif Tstance > 1.3 * self.Tswing:
            Tstance = 1.3 * self.Tswing

        self.current_tstance = float(Tstance)
        self.current_tswing = float(self.Tswing)
        self.current_tstride = float(Tstance + self.Tswing)

        # 첫 번째 다리가 땅에 닿았고 로봇이 걷고 있는 상태
        if contacts[0] == 1 and Tstance > dt:
            # self.TD 플래그를 True로 켬
            self.TD = True

        # 내부 타이머를 dt만큼 증가
        self.Increment(dt, Tstance + self.Tswing)

        T_bf = copy.deepcopy(T_bf_)
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            # TODO: 이 부분을 더 깔끔하게 만들 것
            if key == "FL":
                self.ref_idx = i
                self.dSref[i] = 0.0
            if key == "FR":
                self.dSref[i] = 0.5
            if key == "BL":
                self.dSref[i] = 0.5
            if key == "BR":
                self.dSref[i] = 0.0
            _, p_bf = TransToRp(Tbf_in)
            if Tstance > 0.0:
                step_coord = self.GetFootStep(L, LateralFraction, YawRate,
                                              clearance_height,
                                              penetration_depth, Tstance, p_bf,
                                              i, key)
            else:
                step_coord = np.array([0.0, 0.0, 0.0])
            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]
        return T_bf