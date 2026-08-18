#!/usr/bin/env python3

import numpy as np
from quattro.Kinematics.LegKinematics import LegIK
from quattro.Kinematics.LieAlgebra import RpToTrans, TransToRp, TransInv, RPY, TransformVector
from collections import OrderedDict


class QuattroModel:
    # 파라미터 정의 및 초기화
    def __init__(self,
                 shoulder_length=0.055,
                 elbow_length=0.10652,
                 wrist_length=0.145,
                 hip_x=0.23,
                 hip_y=0.075,
                 foot_x=0.23,
                 foot_y=0.185,
                 height=0.20,
                 com_offset=0.016,
                 shoulder_lim=[-0.548, 0.548],
                 elbow_lim=[-2.17, 0.97],
                 wrist_lim=[-0.1, 2.59]):
        """
        Quattro 운동학 (Kinematics) 모델
        """
        # x 방향 COM(질량 중심) 오프셋
        self.com_offset = com_offset

        # 다리 파라미터 (Leg Parameters)
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length

        # 힙(Hips) 간의 거리
        self.hip_x = hip_x
        self.hip_y = hip_y

        # 기본 자세에서 발(Feet) 간의 거리
        self.foot_x = foot_x
        self.foot_y = foot_y

        # 몸체 높이
        self.height = height

        # 관절 제한값
        self.shoulder_lim = shoulder_lim
        self.elbow_lim = elbow_lim
        self.wrist_lim = wrist_lim


        # 다리 IK 솔버를 저장할 딕셔너리
        self.Legs = OrderedDict()
        self.Legs["FL"] = LegIK("LEFT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["FR"] = LegIK("RIGHT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["BL"] = LegIK("LEFT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["BR"] = LegIK("RIGHT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)

        # 힙과 발의 변환 행렬을 저장할 딕셔너리
        Rwb = np.eye(3)

        # 각 다리의 월드 -> 힙 행렬
        self.WorldToHip = OrderedDict()
        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)
        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)
        self.ph_BL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, self.ph_BL)
        self.ph_BR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, self.ph_BR)

        # 각 다리의 월드 -> 발 행렬
        self.WorldToFoot = OrderedDict()
        self.pf_FL = np.array([self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)
        self.pf_FR = np.array([self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)
        self.pf_BL = np.array([-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, self.pf_BL)
        self.pf_BR = np.array([-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, self.pf_BR)


    def HipToFoot(self, orn, pos, T_bf):
        """
        Quattro의 홈 위치를 기준으로 한 목표 위치와 방향(orientation)을 변환
        """
        # 회전 성분만 가져오기
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        pb = pos
        T_wb = RpToTrans(Rb, pb)

        # 벡터를 저장할 딕셔너리
        HipToFoot_List = OrderedDict()

        for i, (key, T_wh) in enumerate(self.WorldToHip.items()):
            # 순서: FL, FR, BL, BR
            # 벡터 성분 추출
            _, p_bf = TransToRp(T_bf[key])

            # 1단계, 각 다리에 대한 T_bh 구하기
            T_bh = np.dot(TransInv(T_wb), T_wh)

            # 2단계, 각 다리에 대한 T_hf 구하기
            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            _, p_hf = TransToRp(T_hf)

            HipToFoot_List[key] = p_hf

        return HipToFoot_List

    def IK(self, orn, pos, T_bf):
        """
        HipToFoot()을 사용하여 힙-발 벡터로 변환하고 LegIK 솔버에 입력
        """
        # com 오프셋만큼 x 수정
        pos[0] += self.com_offset

        # 4개의 다리, 다리당 3개의 관절
        joint_angles = np.zeros((4, 3))

        HipToFoot = self.HipToFoot(orn, pos, T_bf)

        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            # 3단계, 각 다리에 대해 T_hf로부터 관절 각도 계산
            joint_angles[i, :] = self.Legs[key].solve(p_hf)

        return joint_angles