#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import time

# ROS 2 메시지 (사용자 환경에 맞게 임포트)
from quattro_msgs.msg import IMUdata

# Adafruit BNO08x 라이브러리
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        
        # Publisher 설정
        qos_profile = QoSProfile(depth=10)
        self.imu_pub = self.create_publisher(IMUdata, 'quattro/imu', qos_profile)
        
        self.get_logger().info("Initializing BNO085 IMU Sensor...")

        # I2C 설정 (기본적으로 I2C 버스 1 사용)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # BNO085 초기화
        try:
            self.bno = BNO08X_I2C(self.i2c)
            self.get_logger().info("BNO085 Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect BNO085: {e}")
            raise e

        # 센서 데이터 활성화 (가속도, 자이로, 쿼터니언)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # 주기로 데이터 발행
        self.timer = self.create_timer(0.01, self._on_timer)

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        쿼터니언(x, y, z, w)을 오일러 각(Roll, Pitch, Yaw) 단위인 라디안으로 변환
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def _on_timer(self):
        try:
            # 1. BNO085로부터 데이터 읽기
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x_rad, gyro_y_rad, gyro_z_rad = self.bno.gyro
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # 2. 쿼터니언을 오일러각(라디안)으로 변환
            roll_rad, pitch_rad, yaw_rad = self.euler_from_quaternion(quat_i, quat_j, quat_k, quat_real)

            # 3. 메시지 할당 및 변환
            msg = IMUdata()

            # 각도: Radian -> Degree 변환
            msg.roll = math.degrees(roll_rad)
            msg.pitch = math.degrees(pitch_rad)

            # 각속도: Radian/s -> Degree/s 변환
            msg.gyro_x = math.degrees(gyro_x_rad)
            msg.gyro_y = math.degrees(gyro_y_rad)
            msg.gyro_z = math.degrees(gyro_z_rad)

            # 가속도: m/s^2 그대로 전송
            msg.acc_x = accel_x
            msg.acc_y = accel_y
            msg.acc_z = accel_z

            # 쿼터니언: (w=real, x=i, y=j, z=k) 표준 표기
            msg.quat_w = quat_real
            msg.quat_x = quat_i
            msg.quat_y = quat_j
            msg.quat_z = quat_k

            self.imu_pub.publish(msg)

        except Exception as e:
            # BNO085는 간헐적으로 I2C 통신을 놓치는(RuntimeError) 경우가 발생할 수 있습니다.
            # 이 경우 노드가 죽지 않도록 예외 처리 후 무시하고 다음 루프로 넘깁니다.
            pass

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BNO085Node()
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