#!/usr/bin/env python3
# -*- coding: utf_8 -*-

# ros2 run four_wheel_robot kinematics_node

# ros2 topic echo /wheel_speeds
# ros2 topic echo /steering_angles

# [참고] 4:1 기어비 적용 후 조향 틱 범위 (dxl_center=30720, steer_max_deg=45, max_steer_ticks=4096 기준)
#   정면(중앙): 30720
#   최대 좌측(+45°): 30720 + 4096 = 34816
#   최대 우측(-45°): 30720 - 4096 = 26624
# ros2 topic pub --once /steering_angles std_msgs/msg/Float32MultiArray "{data: [34816, 26624, 26624, 34816]}"


# =======================================================================================================================================
# =                                                            라이브러리                                                                 =
# =======================================================================================================================================

import rclpy                                    # ROS 2의 파이썬 클라이언트 라이브러리
from rclpy.node import Node                     # ROS 2 노드를 생성하기 위한 기본 Node 클래스
from geometry_msgs.msg import Twist             # 키보드나 조이스틱에서 날아오는 cmd_vel (x축 직진 속도, z축 회전 속도) 메시지를 받기 위해 사용합니다.
from std_msgs.msg import Float32MultiArray      # 4개의 바퀴 속도와 4개의 조향각을 배열 형태로 한 번에 묶어서 모터 제어기로 보내기 위해 사용합니다.
import math                                     # 삼각함수(탄젠트, 아크탄젠트 등)와 라디안/디그리 변환, 제곱근 계산 등 기구학 수식을 풀기 위해 가져옵니다.

# =======================================================================================================================================
# =                                                          노드 클래스 정의                                                              =
# =======================================================================================================================================
class KinematicsNode(Node):                                             # ROS 2의 Node 클래스를 상속받아 나만의 기구학 노드를 만듭니다.
    def __init__(self):
        super().__init__('kinematics_node')                             # 부모 클래스를 초기화하면서 이 노드의 이름을 kinematics_node로 등록합니다.

        # ================= [수정] ROS2 파라미터화 =================
        # 기존에는 차량 치수/속도 스케일이 전부 하드코딩되어 있어서, 이 값을 바꾸려면
        # 코드를 고치고 다시 빌드해야 했습니다. 또한 motor_driver_node의 MAX_RPM(=5)과
        # 여기 SPEED_MAX(=100)가 서로 다른 스케일이라 실제로는 항상 ±5로 뭉개지는
        # 심각한 버그가 있었습니다(모터 드라이버 노드 참고).
        # -> 파라미터로 빼서 launch 파일 한 곳에서 두 노드가 동일한 max_wheel_rpm 값을
        #    공유하도록 통일했습니다.
        self.declare_parameter('wheel_base', 0.685)        # 축거 (L)
        self.declare_parameter('track_width', 0.58719)     # 윤거 (W)

        # ================= [수정/4:1 기어비 반영] =================
        # 조향 모터에 4:1 감속기(기어비)가 새로 장착되어, 모터가 4바퀴 회전해야
        # 실제 바퀴(조향축)가 1바퀴 회전하는 구조로 바뀌었습니다.
        # 기존: 중앙값 2048, 최대 조향각(45도)일 때 오프셋 1024 틱 (감속기 없음)
        # 변경: 중앙값 30720, 최대 조향각(45도)일 때 오프셋 4096 틱 (감속기 4:1 반영,
        #       위치확장모드(Extended Position Mode) 기준)
        # -> 하드코딩 대신 'max_steer_ticks' 파라미터로 노출해서, 나중에 기어비가 또
        #    바뀌어도 이 값 하나만 바꾸면 되도록 했습니다.
        self.declare_parameter('dxl_center', 30720)        # 조향 중앙 틱값 (4:1 기어비 반영, 위치확장모드)
        self.declare_parameter('steer_min_deg', -45.0)     # 최소 조향각
        self.declare_parameter('steer_max_deg', 45.0)      # 최대 조향각
        self.declare_parameter('max_steer_ticks', 4096.0)  # 최대 조향각(steer_max_deg)에 도달하기 위한 틱 오프셋 (4:1 기어비 반영)
        # [중요] 이 값은 motor_driver_node의 'max_wheel_rpm' 파라미터와 반드시 동일해야
        # 합니다. launch 파일에서 같은 변수를 두 노드에 함께 전달하도록 구성되어 있습니다.
        self.declare_parameter('max_wheel_rpm', 100.0)

        # 차량 치수
        self.L = float(self.get_parameter('wheel_base').value)      # 축거 수식의 y
        self.W = float(self.get_parameter('track_width').value)     # 윤거 수식의 x

        # Dynamixel (조향) 설정
        self.DXL_CENTER = int(self.get_parameter('dxl_center').value)        # 조향 모터(다이나믹셀)가 정면(0도)을 바라볼 때의 엔코더 틱(Tick) 값입니다.
        self.DXL_MIN    = float(self.get_parameter('steer_min_deg').value)   # 최소 조향각
        self.DXL_MAX    = float(self.get_parameter('steer_max_deg').value)   # 최대 조향각
        self.MAX_STEER_TICKS = float(self.get_parameter('max_steer_ticks').value)  # steer_max_deg에서의 틱 오프셋 (4:1 기어비 반영)
        # [수정] 4:1 기어비 반영: 기존 "4096/360"(모터 1회전=360도 기준, 감속기 없음) 대신
        # "steer_max_deg에서 max_steer_ticks가 되도록" 선형 비례식으로 변경.
        # 예) steer_max_deg=45.0, max_steer_ticks=4096.0 이면 1도당 약 91.02 틱.
        self.TICKS_PER_DEG = self.MAX_STEER_TICKS / abs(self.DXL_MAX) if self.DXL_MAX != 0 else 0.0

        # ZLAC 구동 설정
        self.SPEED_MAX = float(self.get_parameter('max_wheel_rpm').value)  # 최대 속도(=목표 RPM 스케일, motor_driver_node와 동일해야 함)
        self.SPEED_MIN = -self.SPEED_MAX                                   # 최소 속도

        # Publisher, Subscriber
        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        self.steering_angle_pub = self.create_publisher(Float32MultiArray, 'steering_angles', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)

        self.get_logger().info("=== Kinematics Node Started (4 Wheel Independent Steering) ===")
        self.get_logger().info(f"L (y)={self.L:.3f}, W (x)={self.W:.3f}")
        self.get_logger().info(f"DXL Center={self.DXL_CENTER}, Steer Limit={self.DXL_MIN}° ~ {self.DXL_MAX}°, Ticks/Deg={self.TICKS_PER_DEG:.4f} (Max Offset={self.MAX_STEER_TICKS} ticks @ {self.DXL_MAX}°)")
        self.get_logger().info(f"Speed Range: {self.SPEED_MIN} ~ {self.SPEED_MAX}")

# =======================================================================================================================================
# =                                                          아커만 조향식 계산                                                             =
# =======================================================================================================================================

    # 매핑 함수 입력값(value)의 범위를 다른 범위로 비례 변환해 주는 함수
    def map_value(self, value, in_min, in_max, out_min, out_max):
        """입력값을 원하는 출력 범위로 선형 매핑합니다."""
        value = max(in_min, min(value, in_max))
        return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

    # 콜백 함수 핵심 기구학 로직
    def vel_callback(self, msg):
        # 콜백 함수가 실행되면 수신된 msg에서 전후 직진 속도를 v에, 좌우 회전 속도를 w에 저장
        v = msg.linear.x
        w = msg.angular.z
        # 로봇 치수 변수
        x = self.W
        y = self.L

# =======================================================================================================================================
# =                                                          직진, 후진식                                                                 =
# =======================================================================================================================================

        # 직진 상황 (angular.z == 0) 회전 명령이 없으므로 완전한 직진/후진 상태입니다.
        if w == 0:
            speed = self.map_value(v, -1.0, 1.0, self.SPEED_MIN, self.SPEED_MAX)    # 입력된 조이스틱 값(-1 ~ 1)을 모터 속도(-100 ~ 100)로 변환합니다.
            dxl = float(self.DXL_CENTER)                                            # 모든 바퀴가 정면을 바라보도록 다이나믹셀의 중심점 값을 부동소수점으로 저장합니다.
            wheel_speeds = [speed] * 4                                              # 4개 바퀴의 구동 속도를 모두 위에서 계산한 동일한 speed 값으로 설정합니다.
            steering_angles = [dxl] * 4                                             # 4개 바퀴의 조향 각도를 모두 정면(dxl)으로 설정합니다.

# =======================================================================================================================================
# =                                                          아커만 조향식                                                                 =
# =======================================================================================================================================
        else:
            speed = self.map_value(v, -1.0, 1.0, self.SPEED_MIN, self.SPEED_MAX)    # 입력 선속도(v)를 주행 모터 속도 범위로 매핑하여 기준 속도를 정합니다.
            alpha_deg = self.map_value(w, -1.0, 1.0, self.DXL_MIN, self.DXL_MAX)    # 조이스틱의 회전 입력값(w)을 물리적인 최대/최소 조향각(도, Degree)으로 매핑합니다.
            alpha_rad = math.radians(alpha_deg)                                     # 삼각함수 계산을 위해 도(Degree) 단위 각도를 라디안(Radian) 단위로 변환

            # [수정/치명적 버그] w(각속도)가 아주 작은 값(예: 0.00001)일 때 map_value 결과가
            # 부동소수점 반올림으로 alpha_deg가 정확히 0.0이 되어 math.tan(0)=0이 되면
            # 바로 아래 나눗셈에서 ZeroDivisionError가 발생해 노드가 죽습니다.
            # (이 커스텀 teleop은 우연히 이 값을 피해가지만, cmd_vel은 조이스틱/Nav2 등
            #  다른 소스에서도 들어올 수 있으므로 방어 코드가 반드시 필요합니다.)
            # -> 각도가 거의 0에 가까우면 부호를 유지한 채 아주 작은 값으로 클램프하여
            #    사실상 직진과 동일한 결과를 내면서도 0으로 나누는 상황 자체를 막습니다.
            MIN_ALPHA_RAD = 1e-4  # 약 0.006도. 다이나믹셀 1틱(0.088도)보다 훨씬 작아 실질적 영향 없음
            if abs(alpha_rad) < MIN_ALPHA_RAD:
                alpha_rad = MIN_ALPHA_RAD if alpha_rad >= 0 else -MIN_ALPHA_RAD

            R = abs((y / (2.0 * math.tan(alpha_rad)))) + (x / 2.0)                  # 로봇 중심에서의 회전반경(R)을 계산합니다.
            

# =======================================================================================================================================
# =                                                          좌회전 조향식 계산                                                             =
# =======================================================================================================================================

            if w > 0:
                steer_fl_rad = alpha_rad                                                                # 전륜 좌측: 기준 각도만큼 꺾음
                steer_rl_rad = -alpha_rad                                                               # 후륜 좌측: 전륜의 반대 방향으로 꺾음 (역상)
                steer_fr_rad = math.atan(y / ((2 * R) + x)) if (R + x / 2.0) != 0 else alpha_rad        # 전륜 우측: 회전 반경차를 고려하여 아크탄젠트로 내측 바퀴보다 작은 각도를 계산
                steer_rr_rad = -steer_fr_rad                                                            # 후륜 우측: 전륜 우측의 반대 방향 (역상)
                vel_fr = (speed * math.sqrt((R + x/2.0)**2 + (y/2.0)**2))/R if R != 0 else speed        # 바깥쪽 바퀴(FR) 속도: 회전 반경이 크므로 더 빠르게 회전하도록 보정 수식을 적용
                vel_fl = (speed * math.sqrt((R - x/2.0)**2 + (y/2.0)**2))/R if R != 0 else speed        # 안쪽 바퀴(FL) 속도: 회전 반경이 작으므로 기준보다 느리게 회전하도록 보정

# =======================================================================================================================================
# =                                                          우회전 조향식 계산                                                             =
# =======================================================================================================================================

            else:
                steer_fr_rad = alpha_rad                                                                # 전륜 우측: 기준 각도만큼 꺾음
                steer_rr_rad = -alpha_rad                                                               # 후륜 우측: 전륜의 반대 방향으로 꺾음 (역상)
                steer_rl_rad = math.atan(y / ((2 * R) + x)) if (R + x / 2.0) != 0 else alpha_rad        # 후륜 좌측: 애커만 수식을 적용하여 내측 바퀴보다 작은 각도를 계산
                steer_fl_rad = -steer_rl_rad                                                            # 전륜 좌측: 전륜 좌측의 반대 방향 (역상)
                vel_fl = (speed * math.sqrt((R + x/2.0)**2 + (y/2.0)**2))/R if R != 0 else speed        # 바깥쪽 바퀴(FL) 속도: 우회전 시 좌측 바퀴가 더 긴 거리를 가야 하므로 증속
                vel_fr = (speed * math.sqrt((R - x/2.0)**2 + (y/2.0)**2))/R if R != 0 else speed        # 안쪽 바퀴(FR) 속도: 우회전 시 내측인 우측 바퀴는 감속

            # 라디안 각도를 다이나믹셀 모터의 틱(Tick) 값으로 변환하는 내부 함수
            def get_dxl(rad):
                deg = math.degrees(rad)                                                                 # 라디안 -> 도(Degree)
                deg_clamped = max(self.DXL_MIN, min(self.DXL_MAX, deg))                                 # 물리적 한계치로 제한
                offset = deg_clamped * self.TICKS_PER_DEG                                               # 1도당 틱 수를 곱해 변화량 계산
                return float(int(round(self.DXL_CENTER + offset)))                                      # 중심값에 더하고 반올림하여 틱 생성

            # [수정] 회전 반경이 작을 때(=조향각이 클 때) 바깥쪽 바퀴는 sqrt(...)/R 보정으로
            # speed보다 훨씬 커질 수 있습니다 (예: 최대 조향각에서 약 1.5배까지 커질 수 있음).
            # 클램프 없이 그대로 나가면 SPEED_MAX(=max_wheel_rpm)를 넘는 값이 motor_driver_node로
            # 전달되어, 의도한 최대 속도보다 실제로 더 빠르게 도는 바퀴가 생길 수 있습니다.
            vel_fl = max(self.SPEED_MIN, min(self.SPEED_MAX, vel_fl))
            vel_fr = max(self.SPEED_MIN, min(self.SPEED_MAX, vel_fr))

            # 변환 함수를 호출하여 4개 조향 모터의 최종 틱 값을 배열로 구성합니다. (순서: FL, FR, RL, RR)
            steering_angles = [get_dxl(steer_fl_rad),get_dxl(steer_fr_rad),get_dxl(steer_rl_rad),get_dxl(steer_rr_rad)]
            # 속도 배열: 4WS 역상 특성상 대칭 구조이므로 후륜 속도를 전륜의 대응하는 속도와 같게 설정합니다.
            wheel_speeds = [vel_fl,vel_fr,vel_fl,vel_fr]

# =======================================================================================================================================
# =                                                          발행식                                                                      =
# =======================================================================================================================================

        # Steering Angle Publish
        angle_msg = Float32MultiArray()
        angle_msg.data = steering_angles
        self.steering_angle_pub.publish(angle_msg)

        # Wheel Speed Publish
        speed_msg = Float32MultiArray()
        speed_msg.data = wheel_speeds
        self.wheel_speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kinematics Node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()