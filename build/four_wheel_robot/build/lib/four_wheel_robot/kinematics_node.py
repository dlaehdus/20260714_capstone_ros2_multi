#!/usr/bin/env python3
# -*- coding: utf_8 -*-

# ros2 run four_wheel_robot kinematics_node

# ros2 topic echo /wheel_speeds
# ros2 topic echo /steering_angles


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

        # 차량 치수
        self.L = 0.685      # 축거 수식의 y
        self.W = 0.58719    # 윤거 수식의 x

        # Dynamixel (조향) 설정
        self.DXL_CENTER = 2048                                         # 조향 모터(다이나믹셀)가 정면(0도)을 바라볼 때의 엔코더 틱(Tick) 값입니다.
        self.DXL_MIN    = -45.0                                         # 최소 조향각
        self.DXL_MAX    = 45.0                                          # 최대 조향각
        self.TICKS_PER_DEG = 4096.0 / 360.0                             # 다이나믹셀 1도(Degree) 당 틱 수 (4096 / 360) - 새 로직 반영

        # ZLAC 구동 설정
        self.SPEED_MAX = 100.0                                          # 최대 속도
        self.SPEED_MIN = -100.0                                         # 최소 속도

        # Publisher, Subscriber
        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        self.steering_angle_pub = self.create_publisher(Float32MultiArray, 'steering_angles', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)

        self.get_logger().info("=== Kinematics Node Started (4 Wheel Independent Steering) ===")
        self.get_logger().info(f"L (y)={self.L:.3f}, W (x)={self.W:.3f}")
        self.get_logger().info(f"DXL Center={self.DXL_CENTER}, Steer Limit={self.DXL_MIN}° ~ {self.DXL_MAX}°")
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