# ros2 topic pub --once /steering_angles std_msgs/msg/Float32MultiArray "{data: [19285, 12285, 12285, 12285]}"

# ros2 run four_wheel_robot steering_driver_node

#!/usr/bin/env python3
# -*- coding: utf_8 -*-


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import * # Dynamixel SDK 라이브러리

# 다이나믹셀 제어 테이블 주소 (X 시리즈 기준)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# 제어 설정값
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
# DEVICE_NAME = '/dev/ttyACM5' # 실제 연결된 포트로 수정 필요 (ls /dev/ttyUSB*)
DEVICE_NAME ='/dev/serial/by-id/usb-CM-900_ROBOTIS_Virtual_COM_Port-if00'
OPERATING_MODE_EXTENDED_POSITION = 4 # 위치 확장 모드
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
INITIAL_POSITION = 2048 # 초기 위치 값

class SteeringDriverNode(Node):
    def __init__(self):
        super().__init__('steering_driver_node')

        # 모터 ID 설정
        self.dxl_ids = [1, 2, 3, 4]
        
        # 통신 포트 및 패킷 핸들러 초기화
        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # 포트 열기 및 통신 속도 설정
        if self.port_handler.openPort():
            self.get_logger().info(f"Succeeded to open the port: {DEVICE_NAME}")
        else:
            self.get_logger().error("Failed to open the port")
            return

        if self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info(f"Succeeded to change the baudrate: {BAUDRATE}")
        else:
            self.get_logger().error("Failed to change the baudrate")
            return

        # 모터 초기화 (토크 끄기 -> 모드 변경 -> 토크 켜기 -> 초기 위치 이동)
        for dxl_id in self.dxl_ids:
            self.init_dynamixel(dxl_id)

        # 상태 관리 변수
        self.last_msg_time = self.get_clock().now()
        self.watchdog_triggered = False

        # Subscriber: [모터1각도, 모터2각도, 모터3각도, 모터4각도] 수신
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'steering_angles', 
            self.angle_callback,
            10
        )

        # 3초 Watchdog 타이머 (0.1초 주기로 체크)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        
        self.get_logger().info("Steering Driver Node is Ready")

    # =========================================================
    # 다이나믹셀 초기화 함수 (에러 체크 포함)
    # =========================================================
    def init_dynamixel(self, dxl_id):
        try:
            # 1. Torque Off
            comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Torque Off 통신 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Torque Off Hardware Error: {self.packet_handler.getRxPacketError(error)}")

            # 2. Operating Mode 설정 (Extended Position Mode)
            comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_EXTENDED_POSITION)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Operating Mode 설정 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Operating Mode Hardware Error: {self.packet_handler.getRxPacketError(error)}")

            # 3. Torque On
            comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Torque On 통신 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Torque On Hardware Error: {self.packet_handler.getRxPacketError(error)}")

            # 4. 초기 위치 이동
            comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, INITIAL_POSITION)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Initial Position 설정 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Initial Position Hardware Error: {self.packet_handler.getRxPacketError(error)}")
            else:
                self.get_logger().info(f"Dynamixel ID {dxl_id}: Extended Position Mode & Torque On 완료")

        except Exception as e:
            self.get_logger().error(f"[ID:{dxl_id}] 초기화 중 예외 발생: {e}")

    # =========================================================
    # 각도 명령 수신 콜백 (에러 체크 강화)
    # =========================================================
    def angle_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.watchdog_triggered = False
        
        angles = msg.data
        for i, dxl_id in enumerate(self.dxl_ids):
            if i < len(angles):
                goal_pos = int(angles[i])
                
                comm_result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_pos
                )
                
                if comm_result != COMM_SUCCESS:
                    self.get_logger().error(f"[ID:{dxl_id}] Goal Position 통신 실패: {self.packet_handler.getTxRxResult(comm_result)} | 목표각: {goal_pos}")
                elif error != 0:
                    self.get_logger().error(f"[ID:{dxl_id}] Goal Position Hardware Error: {self.packet_handler.getRxPacketError(error)} | 목표각: {goal_pos}")
                # 성공 시 로그는 너무 많이 나올 수 있으니 주석 처리
                # else:
                #     self.get_logger().info(f"[ID:{dxl_id}] Goal Position {goal_pos} 설정 성공")

    # =========================================================
    # 3초 Watchdog 체크
    # =========================================================
    def watchdog_check(self):
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_diff > 333.0 and not self.watchdog_triggered:
            self.get_logger().error("⚠️ [Watchdog] 3초간 데이터 없음: 초기 위치(12285)로 복귀합니다.")
            for dxl_id in self.dxl_ids:
                self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, INITIAL_POSITION)
            self.watchdog_triggered = True

    # =========================================================
    # 안전 종료
    # =========================================================
    def destroy_node(self):
        self.get_logger().info("노드 종료: 모터를 초기화 위치로 이동 후 토크를 해제합니다.")
        for dxl_id in self.dxl_ids:
            # 초기 위치 이동
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, INITIAL_POSITION)
            # 토크 해제
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        
        self.port_handler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SteeringDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()