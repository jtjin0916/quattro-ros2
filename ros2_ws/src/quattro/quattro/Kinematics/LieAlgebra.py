#!/usr/bin/env python3
import numpy as np

# 참고: 노스웨스턴 대학교의 Modern Robotics 코드 스니펫:
# 참조 https://github.com/NxRLab/ModernRobotics


def RpToTrans(R, p):
    """
    회전 행렬(Rotation Matrix)과 위치 벡터(Position Vector)를 
    동차 변환 행렬(Homogeneous Transformation Matrix)로 변환합니다.
    """
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]


def TransToRp(T):
    """
    동차 변환 행렬을 회전 행렬과 위치 벡터로 변환합니다.
    """
    T = np.array(T)
    return T[0:3, 0:3], T[0:3, 3]


def TransInv(T):
    """
    동차 변환 행렬의 역행렬을 구합니다.
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def Adjoint(T):
    """
    동차 변환 행렬의 수반(Adjoint) 표현을 계산합니다.
    """
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))], np.c_[np.dot(VecToso3(p), R), R]]


def VecToso3(omg):
    """
    3-벡터를 so(3) 표현(반대칭 행렬)으로 변환합니다.
    """
    return np.array([[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]],
                     [-omg[1], omg[0], 0]])


def RPY(roll, pitch, yaw):
    """
    Roll, Pitch, Yaw 변환 행렬을 생성합니다.
    """
    Roll = np.array([[1, 0, 0, 0], [0, np.cos(roll), -np.sin(roll), 0],
                     [0, np.sin(roll), np.cos(roll), 0], [0, 0, 0, 1]])
    Pitch = np.array([[np.cos(pitch), 0, np.sin(pitch), 0], [0, 1, 0, 0],
                      [-np.sin(pitch), 0, np.cos(pitch), 0], [0, 0, 0, 1]])
    Yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
                    [np.sin(yaw), np.cos(yaw), 0, 0], [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return np.matmul(np.matmul(Roll, Pitch), Yaw)


def RotateTranslate(rotation, position):
    """
    회전 후 평행이동을 수행하는 변환 행렬을 생성합니다.
    """
    trans = np.eye(4)
    trans[0, 3] = position[0]
    trans[1, 3] = position[1]
    trans[2, 3] = position[2]

    return np.dot(rotation, trans)


def TransformVector(xyz_coord, rotation, translation):
    """
    지정된 회전(Rotation) 후 평행이동(Translation) 행렬을 사용하여 벡터를 변환합니다.
    """
    xyz_vec = np.append(xyz_coord, 1.0)

    Transformed = np.dot(RotateTranslate(rotation, translation), xyz_vec)
    return Transformed[:3]