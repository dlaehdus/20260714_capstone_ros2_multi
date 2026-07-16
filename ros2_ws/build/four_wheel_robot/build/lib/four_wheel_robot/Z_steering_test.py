#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 실행 예시:
# ros2 topic pub --once /steering_angles std_msgs/msg/Float32MultiArray "{data: [19285, 12285, 12285, 12285]}"
# ros2 run four_wheel_robot steering_driver_node

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *  # Dynamixel SDK 라이브러리

# 다이나믹셀 제어 테이블 주소 (X 시리즈 기준)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# 제어 설정값
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
# DEVICE_NAME = '/dev/ttyACM5'  # 실제 연결된 포트로 수정 필요 (ls /dev/ttyUSB*)
DEVICE_NAME = '/dev/serial/by-id/usb-CM-900_ROBOTIS_Virtual_COM_Port-if00'
OPERATING_MODE_EXTENDED_POSITION = 4  # 위치 확장 모드
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
INITIAL_POSITION = 2048  # 초기 위치 값 (절대좌표 기준)

# ---------------------------------------------------------------
# 다회전 위치 복구 관련 설정
# ---------------------------------------------------------------
ENCODER_RESOLUTION = 4096            # 1회전당 엔코더 분해능 (12bit, X시리즈 기준)
STATE_FILE = os.path.expanduser('~/.dxl_steering_state.json')  # 종료 시 위치 저장 파일
AUTOSAVE_PERIOD_SEC = 5.0            # 비정상 종료(크래시) 대비 주기 저장 (0으로 하면 비활성화)


def to_signed32(value):
    """dynamixel_sdk의 read 결과(부호없는 32bit)를 부호있는 정수로 변환"""
    if value > 0x7FFFFFFF:
        value -= 0x100000000
    return value


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

        # 이전 종료 시 저장된 절대 위치값 로드
        self.saved_state = self.load_state()

        # 모터별 오프셋(절대좌표 - 다이나믹셀 raw값) 및 현재 절대 위치 추적용 딕셔너리
        self.position_offset = {}
        self.current_absolute_position = {}

        # 모터 초기화 (토크 끄기 -> 모드 변경 -> raw 위치 읽고 오프셋 계산 -> 토크 켜기 -> 초기 위치 이동)
        for dxl_id in self.dxl_ids:
            self.init_dynamixel(dxl_id)

        # 상태 관리 변수
        self.last_msg_time = self.get_clock().now()
        self.watchdog_triggered = False

        # Subscriber: [모터1각도, 모터2각도, 모터3각도, 모터4각도] 수신 (절대좌표 기준)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'steering_angles',
            self.angle_callback,
            10
        )

        # 3초 Watchdog 타이머 (0.1초 주기로 체크)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        # 비정상 종료(크래시) 대비 주기적 저장 타이머
        if AUTOSAVE_PERIOD_SEC > 0:
            self.autosave_timer = self.create_timer(AUTOSAVE_PERIOD_SEC, self.autosave_check)

        self.get_logger().info("Steering Driver Node is Ready")

    # =========================================================
    # 상태 파일(마지막 절대 위치) 로드 / 저장
    # =========================================================
    def load_state(self):
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info(f"이전 위치 상태 로드 완료: {data}")
                    return data
            except Exception as e:
                self.get_logger().error(f"상태 파일 로드 실패: {e}")
        return {}

    def save_state(self):
        try:
            data = {str(k): v for k, v in self.current_absolute_position.items()}
            with open(STATE_FILE, 'w') as f:
                json.dump(data, f)
            self.get_logger().info(f"현재 위치 상태 저장 완료: {data}")
        except Exception as e:
            self.get_logger().error(f"상태 파일 저장 실패: {e}")

    def autosave_check(self):
        # 크래시 등 비정상 종료에 대비한 안전장치. destroy_node()에서도 별도로 저장함.
        self.save_state()

    # =========================================================
    # 다이나믹셀 초기화 함수 (에러 체크 + 다회전 위치 복구 포함)
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

            # 3. 현재 raw 위치 읽기 (전원 재인가 후 0~4095 사이 값으로 리셋되어 있음)
            dxl_present_position, comm_result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, ADDR_PRESENT_POSITION
            )
            raw_position = to_signed32(dxl_present_position)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Present Position 읽기 실패: {self.packet_handler.getTxRxResult(comm_result)}")
                raw_position = 0
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Present Position Hardware Error: {self.packet_handler.getRxPacketError(error)}")

            # 4. 저장된 이전 절대 위치와 비교하여 누적 바퀴 수(n) 및 오프셋 계산
            #    - saved_abs: 종료 직전까지 소프트웨어가 알고 있던 절대 위치
            #    - raw_position: 재부팅 후 다이나믹셀이 보고하는 0~4095 사이 값
            #    - n = round((saved_abs - raw_position) / 4096)  => 전원 꺼진 사이 몇 바퀴였는지 역산
            #    - recovered_abs = raw_position + n * 4096       => 복구된 절대 위치
            #    - offset = recovered_abs - raw_position (= n * 4096)
            #      앞으로 목표 절대위치(target)를 명령할 때는 항상
            #      "다이나믹셀에 실제로 보내는 값 = target - offset" 로 변환해서 보냄
            saved_abs = self.saved_state.get(str(dxl_id), INITIAL_POSITION)
            n = round((saved_abs - raw_position) / ENCODER_RESOLUTION)
            recovered_abs = raw_position + n * ENCODER_RESOLUTION
            offset = recovered_abs - raw_position

            self.position_offset[dxl_id] = offset
            self.current_absolute_position[dxl_id] = recovered_abs

            self.get_logger().info(
                f"[ID:{dxl_id}] raw={raw_position}, 저장된 절대값={saved_abs}, "
                f"복구된 절대값={recovered_abs}, offset={offset}"
            )

            # 5. Torque On
            comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Torque On 통신 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Torque On Hardware Error: {self.packet_handler.getRxPacketError(error)}")

            # 6. 초기 위치로 이동 (절대좌표 -> 오프셋 보정하여 실제 명령값 계산)
            goal_to_send = INITIAL_POSITION - offset
            comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_to_send)
            if comm_result != COMM_SUCCESS:
                self.get_logger().error(f"[ID:{dxl_id}] Initial Position 설정 실패: {self.packet_handler.getTxRxResult(comm_result)}")
            elif error != 0:
                self.get_logger().error(f"[ID:{dxl_id}] Initial Position Hardware Error: {self.packet_handler.getRxPacketError(error)}")
            else:
                self.current_absolute_position[dxl_id] = INITIAL_POSITION
                self.get_logger().info(f"Dynamixel ID {dxl_id}: Extended Position Mode & Torque On 완료")

        except Exception as e:
            self.get_logger().error(f"[ID:{dxl_id}] 초기화 중 예외 발생: {e}")

    # =========================================================
    # 각도 명령 수신 콜백 (절대좌표 -> 오프셋 보정 후 전송)
    # =========================================================
    def angle_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.watchdog_triggered = False

        angles = msg.data
        for i, dxl_id in enumerate(self.dxl_ids):
            if i < len(angles):
                target_absolute = int(angles[i])
                offset = self.position_offset.get(dxl_id, 0)
                goal_to_send = target_absolute - offset

                comm_result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_to_send
                )

                if comm_result != COMM_SUCCESS:
                    self.get_logger().error(f"[ID:{dxl_id}] Goal Position 통신 실패: {self.packet_handler.getTxRxResult(comm_result)} | 목표각(절대): {target_absolute}")
                elif error != 0:
                    self.get_logger().error(f"[ID:{dxl_id}] Goal Position Hardware Error: {self.packet_handler.getRxPacketError(error)} | 목표각(절대): {target_absolute}")
                else:
                    self.current_absolute_position[dxl_id] = target_absolute
                    # 성공 시 로그는 너무 많이 나올 수 있으니 주석 처리
                    # self.get_logger().info(f"[ID:{dxl_id}] Goal Position {target_absolute}(절대) 설정 성공")

    # =========================================================
    # 3초 Watchdog 체크
    # =========================================================
    def watchdog_check(self):
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_diff > 3.0 and not self.watchdog_triggered:
            self.get_logger().error(f"⚠️ [Watchdog] 3초간 데이터 없음: 초기 위치({INITIAL_POSITION})로 복귀합니다.")
            for dxl_id in self.dxl_ids:
                offset = self.position_offset.get(dxl_id, 0)
                goal_to_send = INITIAL_POSITION - offset
                self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_to_send)
                self.current_absolute_position[dxl_id] = INITIAL_POSITION
            self.watchdog_triggered = True

    # =========================================================
    # 안전 종료 (마지막 절대 위치를 파일에 저장)
    # =========================================================
    def destroy_node(self):
        self.get_logger().info("노드 종료: 모터를 초기화 위치로 이동 후 토크를 해제합니다.")
        for dxl_id in self.dxl_ids:
            offset = self.position_offset.get(dxl_id, 0)
            goal_to_send = INITIAL_POSITION - offset
            # 초기 위치 이동
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal_to_send)
            self.current_absolute_position[dxl_id] = INITIAL_POSITION
            # 토크 해제
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # 다음 부팅 시 사용할 절대 위치값 저장
        self.save_state()

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