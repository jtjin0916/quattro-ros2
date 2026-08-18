import rclpy

class QuattroCommand:
    def __init__(self):
        self.motion = "Stop"   # "Go" or "Stop"
        self.movement = "Viewing" # "Stepping" or "Viewing"
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.rate = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.z = 0.0
        self.faster = 0.0
        self.slower = 0.0
        self.imu_auto_pose = False

class Quattro:
    def __init__(self):
        self.cmd = QuattroCommand()
        self.logger = rclpy.logging.get_logger("quattro_lib")

    def almost_equal(self, d1, d2, epsilon=1.0e-1):
        return abs(d1 - d2) < epsilon

    # IMU Auto 상태를 외부에서 업데이트하는 함수
    def update_imu_state(self, is_auto: bool):
        self.cmd.imu_auto_pose = is_auto

    def update_command(self, vx, vy, z, w, wx, wy):
        # 조이스틱 입력이 없고, 동시에 'IMU Auto 모드도 꺼져 있을 때만' Stop으로 전환
        if (self.almost_equal(vx, 0.0) and self.almost_equal(vy, 0.0) and 
            self.almost_equal(z, 0.0) and self.almost_equal(w, 0.0) and
            not self.cmd.imu_auto_pose):
            
            self.cmd.motion = "Stop"
            self.cmd.x_velocity = 0.0
            self.cmd.y_velocity = 0.0
            self.cmd.rate = 0.0
            self.cmd.roll = 0.0
            self.cmd.pitch = 0.0
            self.cmd.yaw = 0.0
            self.cmd.z = 0.0
            self.cmd.faster = 0.0
            self.cmd.slower = 0.0
        else:
            self.cmd.motion = "Go"
            if self.cmd.movement == "Stepping":
                # Stepping 모드: vx, vy, rate, z 사용
                self.cmd.x_velocity = vx
                self.cmd.y_velocity = vy
                self.cmd.rate = w
                self.cmd.z = z
                self.cmd.roll = 0.0
                self.cmd.pitch = 0.0
                self.cmd.yaw = 0.0
                # wx, wy를 사용하여 속도 조절 (Clearance Height 등)
                self.cmd.faster = 1.0 - wx
                self.cmd.slower = -(1.0 - wy)
            else:
                # Viewing 모드: RPY, Z 사용
                self.cmd.x_velocity = 0.0
                self.cmd.y_velocity = 0.0
                self.cmd.rate = 0.0
                self.cmd.roll = vy
                self.cmd.pitch = vx
                self.cmd.yaw = w
                self.cmd.z = z
                self.cmd.faster = 0.0
                self.cmd.slower = 0.0

    def switch_movement(self):
        # 이동 중에는 모드 전환 금지
        if (not self.almost_equal(self.cmd.x_velocity, 0.0) and 
            not self.almost_equal(self.cmd.y_velocity, 0.0) and 
            not self.almost_equal(self.cmd.rate, 0.0)):
            
            self.logger.warn(f"MAKE SURE VELOCITIES ARE 0.0 BEFORE SWITCHING! vx:{self.cmd.x_velocity:.2f}, vy:{self.cmd.y_velocity:.2f}, w:{self.cmd.rate:.2f}")
            self.logger.warn("STOPPING ROBOT...")

            self.cmd.motion = "Stop"
            self.cmd.x_velocity = 0.0
            self.cmd.y_velocity = 0.0
            self.cmd.rate = 0.0
            self.cmd.roll = 0.0
            self.cmd.pitch = 0.0
            self.cmd.yaw = 0.0
            self.cmd.z = 0.0
            self.cmd.faster = 0.0
            self.cmd.slower = 0.0
        else:
            # 모든 속도 초기화 후 모드 전환
            self.cmd.x_velocity = 0.0
            self.cmd.y_velocity = 0.0
            self.cmd.rate = 0.0
            self.cmd.roll = 0.0
            self.cmd.pitch = 0.0
            self.cmd.yaw = 0.0
            self.cmd.z = 0.0
            self.cmd.faster = 0.0
            self.cmd.slower = 0.0

            if self.cmd.movement == "Viewing":
                self.logger.info("SWITCHING TO STEPPING MOTION, COMMANDS NOW MAPPED TO VX|VY|W|Z.")
                self.cmd.movement = "Stepping"
                self.cmd.motion = "Stop"
            else:
                self.logger.info("SWITCHING TO VIEWING MOTION, COMMANDS NOW MAPPED TO R|P|Y|Z.")
                self.cmd.movement = "Viewing"
                self.cmd.motion = "Stop"

    def return_command(self):
        return self.cmd