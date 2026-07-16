# [4:1 기어비 반영 후] 중앙=30720, 최대 조향(45도)=+-4096 틱 (26624 ~ 34816)
# ros2 topic pub --once /steering_angles std_msgs/msg/Float32MultiArray "{data: [30720, 30720, 30720, 30720]}"

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
# [수정/4:1 기어비 반영] 조향 감속기가 4:1로 바뀌면서 kinematics_node의 dxl_center와
# 동일하게 중앙(정면) 값을 2048 -> 30720 으로 변경했습니다.
# (반드시 kinematics_node의 'dxl_center' 파라미터 값과 같아야 합니다.)
INITIAL_POSITION = 30720 # 초기 위치 값 (조향 중앙, 4:1 기어비 반영)

class SteeringDriverNode(Node):
    def __init__(self):
        super().__init__('steering_driver_node')

        # [수정] 다른 노드들과 스타일을 통일하기 위해 포트 경로/보드레이트를 파라미터로 노출
        self.declare_parameter('device_name', DEVICE_NAME)
        self.declare_parameter('baudrate', BAUDRATE)
        device_name = self.get_parameter('device_name').value
        baudrate = self.get_parameter('baudrate').value

        # 모터 ID 설정
        self.dxl_ids = [1, 2, 3, 4]
        
        # 통신 포트 및 패킷 핸들러 초기화
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # [수정/치명적 버그] 기존에는 포트 연결/보드레이트 설정이 실패하면 그냥 return만
        # 하고 넘어갔습니다. 그러면 __init__이 여기서 조용히 끝나버려서 subscription과
        # watchdog_timer가 아예 생성되지 않는데, ROS 입장에서는 노드가 "정상적으로" 떠서
        # rclpy.spin()이 계속 살아있는 것처럼 보입니다. 즉 조향 하드웨어가 완전히 죽어도
        # 시작할 때 로그 한 줄만 찍히고, 이후로는 아무 경고도 없이 조향이 전혀 동작하지
        # 않는 '좀비 노드' 상태가 됩니다. 로봇은 구동만 되고 조향은 안 되는 매우 위험한
        # 상황을 아무도 알아채지 못할 수 있습니다.
        # -> RuntimeError를 발생시켜 노드 생성 자체를 실패시키고, 프로세스가 눈에 띄게
        #    종료되도록 수정했습니다 (main()에서 처리).
        if self.port_handler.openPort():
            self.get_logger().info(f"Succeeded to open the port: {device_name}")
        else:
            self.get_logger().error("Failed to open the port")
            raise RuntimeError(f"조향 포트 열기 실패: {device_name}")

        if self.port_handler.setBaudRate(baudrate):
            self.get_logger().info(f"Succeeded to change the baudrate: {baudrate}")
        else:
            self.get_logger().error("Failed to change the baudrate")
            raise RuntimeError(f"조향 포트 보드레이트 설정 실패: {baudrate}")

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
        # [수정/가장 치명적인 버그] 원래 333.0으로 되어 있었습니다. 주석과 로그 메시지는
        # "3초"라고 되어 있지만 실제로는 약 5분 33초가 지나야 안전 동작(중앙 복귀)이
        # 실행되었습니다. 즉 kinematics_node나 teleop_node가 죽거나 통신이 끊겨도
        # steering_angles가 마지막으로 받은 값 그대로 5분 넘게 유지된 채 로봇이 계속
        # 주행하게 되는 매우 위험한 상태였습니다. 3.0으로 수정합니다.
        if time_diff > 3.0 and not self.watchdog_triggered:
            # [수정] 로그 메시지에 하드코딩된 "12285"는 실제 INITIAL_POSITION(2048)과
            # 다른 값이라 디버깅 시 혼란을 줄 수 있어 실제 변수 값을 출력하도록 수정
            self.get_logger().error(f"⚠️ [Watchdog] 3초간 데이터 없음: 초기 위치({INITIAL_POSITION})로 복귀합니다.")
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
    node = None
    try:
        # [수정] 포트 연결 실패 시 SteeringDriverNode 생성자가 RuntimeError를 던지므로
        # 여기서 함께 감싸서 처리 (기존에는 node 생성과 spin이 분리되어 있어 실패 처리가 안 됐음)
        node = SteeringDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[steering_driver_node] 치명적 오류로 종료합니다: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()