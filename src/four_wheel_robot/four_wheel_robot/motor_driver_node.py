#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=================================
원본 대비 개선 사항:
  [안전]
    - 드라이버 하드웨어 워치독(0x2000, Communication offline time) 명시적 설정
      -> ROS 프로세스가 죽어도 드라이버 자체가 통신 끊김을 감지해 정지
    - 초기화 시 fault(알람) clear 후 enable (이전 알람 잔존 문제 방지)
    - 목표 속도 범위 클리핑 (-3000 ~ 3000 rpm, 매뉴얼 기준)
    - 런타임 중 주기적 에러코드(0x20A5/0x20A6) 모니터링
  [통신 신뢰성]
    - 포트 연결 실패 시에도 masters 리스트 슬롯을 유지 (인덱스 밀림 버그 수정)
    - Modbus 타임아웃을 1.0s -> 0.1s 로 단축 (콜백 블로킹 방지)
    - 예외 처리 범위 확장 (ModbusError 뿐 아니라 SerialException 등 포함)
  [데이터 정합성]
    - wheel_speeds 배열 길이가 기대값과 다르면 전체 명령 무시 + 경고
  [구조]
    - 포트/ID/baudrate/watchdog 시간 등을 ROS2 파라미터로 노출

[추가 리뷰 수정 사항]
  - (치명적) MAX_RPM 하드코딩이 5였던 버그 수정 -> max_wheel_rpm 파라미터로 변경,
    kinematics_node와 launch 파일에서 같은 값을 공유하도록 통일
  - max_wheel_rpm이 ZLAC 매뉴얼 절대 한계치(3000)를 넘지 않도록 2중 클램프 추가
  - 에러코드가 여러 비트 동시에 켜져도 전부 읽을 수 있도록 디코딩 로직 개선
  - _clamp_rpm을 truncation(int())에서 반올림(round())으로 변경

터미널 테스트 예시:
  ros2 topic pub --once /wheel_speeds std_msgs/msg/Float32MultiArray "{data: [50.0, -50.0, 50.0, -50.0]}"
  ros2 run four_wheel_robot motor_driver_node
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


# ===================== 레지스터 주소 (ZLAC8015D 매뉴얼 기준) =====================
REG_OFFLINE_TIME   = 0x2000   # 드라이버 자체 하드웨어 통신 워치독 (ms)
REG_CONTROL_MODE   = 0x200D
REG_CONTROL_WORD   = 0x200E   # 0x05 quick stop, 0x06 clear fault, 0x07 stop, 0x08 enable
REG_TARGET_VEL     = 0x2088   # L, R 연속 2 레지스터
REG_ERROR_L        = 0x20A5
REG_ERROR_R        = 0x20A6

CTRL_CLEAR_FAULT = 0x06
CTRL_STOP        = 0x07
CTRL_ENABLE      = 0x08
MODE_VELOCITY    = 0x03

# [수정/치명적 버그] 기존 코드는 MAX_RPM = 5 로 하드코딩되어 있었습니다.
# 주석에는 "매뉴얼상 속도모드 목표속도 범위 -3000~3000 r/min" 이라고 적혀 있지만
# 실제 값은 5였기 때문에, kinematics_node가 ±100 스케일로 보내는 wheel_speeds가
# _clamp_rpm()에서 전부 ±5로 뭉개져 사실상 로봇이 거의 움직이지 못하는 상태였습니다
# (두 노드가 서로 다른 스케일을 쓰고 있었던 것이 근본 원인).
# -> ROS2 파라미터 'max_wheel_rpm'으로 바꾸고, kinematics_node의 동일 이름 파라미터와
#    launch 파일에서 같은 값을 공유하도록 하여 두 노드의 속도 스케일을 통일했습니다.
# ABSOLUTE_MAX_RPM은 ZLAC8015D 매뉴얼상 속도모드의 물리적 절대 한계치이며,
# max_wheel_rpm 파라미터를 실수로 과도하게 높게 설정해도 이 값을 넘지 않도록 하는
# 2중 안전장치입니다. (실제 주행 전 반드시 감속기/바퀴 규격에 맞는 안전한 rpm으로 검증할 것)
ABSOLUTE_MAX_RPM = 3000

ERROR_MESSAGES = {
    0x0000: "정상", 0x0001: "과전압", 0x0002: "저전압", 0x0004: "과전류",
    0x0008: "과부하", 0x0010: "전류추종오류", 0x0020: "위치추종오류",
    0x0040: "속도추종오류", 0x0080: "기준전압오류", 0x0100: "EEPROM오류",
    0x0200: "홀센서오류", 0x0400: "모터과온",
}


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # ================= ROS2 파라미터 (launch/yaml에서 오버라이드 가능) =================
        self.declare_parameter('ports', [
            '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BCD9D7ABCD-if00',
            '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BCD9D7ABCD-if06',
        ])
        self.declare_parameter('driver_ids', [1, 1])
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('modbus_timeout_sec', 0.1)      # 통신 응답 타임아웃
        self.declare_parameter('watchdog_timeout_sec', 3.0)    # 소프트웨어 워치독 (2중 안전장치)
        self.declare_parameter('driver_offline_time_ms', 1000) # 드라이버 하드웨어 워치독
        self.declare_parameter('acc_time_ms', 500)
        self.declare_parameter('dec_time_ms', 500)
        # [수정] kinematics_node와 동일한 이름/의미의 파라미터. launch 파일에서 반드시
        # kinematics_node의 max_wheel_rpm과 같은 값을 넣어줘야 속도 스케일이 맞습니다.
        self.declare_parameter('max_wheel_rpm', 100.0)

        ports = self.get_parameter('ports').value
        ids = self.get_parameter('driver_ids').value
        self.baudrate = self.get_parameter('baudrate').value
        self.modbus_timeout = self.get_parameter('modbus_timeout_sec').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self.offline_time_ms = self.get_parameter('driver_offline_time_ms').value
        self.acc_ms = self.get_parameter('acc_time_ms').value
        self.dec_ms = self.get_parameter('dec_time_ms').value

        # [수정] 파라미터 값이 하드웨어 절대 한계치(ABSOLUTE_MAX_RPM)를 넘지 못하도록 2중 클램프
        configured_max = abs(float(self.get_parameter('max_wheel_rpm').value))
        self.max_rpm = min(configured_max, ABSOLUTE_MAX_RPM)
        if configured_max > ABSOLUTE_MAX_RPM:
            self.get_logger().warn(
                f"max_wheel_rpm({configured_max})이 하드웨어 절대 한계치({ABSOLUTE_MAX_RPM})를 "
                f"초과하여 {self.max_rpm}(으)로 제한합니다."
            )

        if len(ports) != len(ids):
            self.get_logger().error("ports와 driver_ids 파라미터 길이가 다릅니다. 종료합니다.")
            raise RuntimeError("ports/driver_ids length mismatch")

        self.driver_configs = [{'port': p, 'id': i} for p, i in zip(ports, ids)]

        # masters: 연결 성공/실패와 관계없이 driver_configs와 항상 같은 길이/순서를 유지
        # (포트 하나가 실패해도 나머지 포트의 인덱스가 밀리지 않도록 하기 위함)
        self.masters = []
        for config in self.driver_configs:
            self.masters.append(self._connect_and_init(config))

        connected = sum(1 for m in self.masters if m['master'] is not None)
        self.get_logger().info(f"드라이버 {connected}/{len(self.masters)}개 연결 및 초기화 완료")

        # Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.speed_callback, 10
        )

        # 소프트웨어 워치독 (드라이버 하드웨어 워치독의 2중 안전장치)
        self.last_speed_time = self.get_clock().now()
        self.motor_stopped = False
        self.no_command_timer = self.create_timer(0.5, self.check_no_command)

        # 런타임 에러코드 모니터링 (1초 주기)
        self.fault_monitor_timer = self.create_timer(1.0, self.check_faults)

        self.get_logger().info("motor_driver_node 대기 중")

    # =========================================================
    # 연결 + 초기화 (포트 1개 단위)
    # =========================================================
    def _connect_and_init(self, config):
        """성공하면 {'config':..., 'master':master} 반환,
        실패해도 {'config':..., 'master':None} 을 반환해서 인덱스 정합성을 유지한다."""
        port_name, d_id = config['port'], config['id']
        entry = {'config': config, 'master': None}
        try:
            ser = serial.Serial(port=port_name, baudrate=self.baudrate,
                                 bytesize=8, parity='N', stopbits=1, xonxoff=0)
            master = modbus_rtu.RtuMaster(ser)
            master.set_timeout(self.modbus_timeout)
            master.set_verbose(False)

            # 1. 드라이버 하드웨어 워치독 설정 (ROS 프로세스가 죽어도 살아있는 안전장치)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            REG_OFFLINE_TIME, output_value=self.offline_time_ms)

            # 2. 이전 알람(fault) 클리어 -> 그 다음 모드/enable 설정
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            REG_CONTROL_WORD, output_value=CTRL_CLEAR_FAULT)
            time.sleep(0.05)

            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            REG_CONTROL_MODE, output_value=MODE_VELOCITY)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            0x2080, output_value=self.acc_ms)  # Acc(Left)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            0x2081, output_value=self.acc_ms)  # Acc(Right)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            0x2082, output_value=self.dec_ms)  # Dec(Left)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            0x2083, output_value=self.dec_ms)  # Dec(Right)
            master.execute(d_id, cst.WRITE_SINGLE_REGISTER,
                            REG_CONTROL_WORD, output_value=CTRL_ENABLE)

            entry['master'] = master
            self.get_logger().info(f"포트 연결 및 초기화 성공: {port_name} (ID:{d_id})")
        except Exception as e:
            # ModbusError뿐 아니라 SerialException 등 통신 관련 예외를 모두 포괄
            self.get_logger().error(f"포트 연결 실패 [{port_name}]: {e}")
        return entry

    # =========================================================
    # 저수준 제어 헬퍼
    # =========================================================
    def _to_u16(self, v):
        return v + 0x10000 if v < 0 else v

    def _clamp_rpm(self, v):
        # [수정] 모듈 상수 MAX_RPM(=5) 대신 파라미터 기반 self.max_rpm 사용.
        # 또한 int(v)는 항상 0 방향으로 잘라내는 truncation이라 작은 속도 명령이
        # 실제보다 더 많이 깎이는 경향이 있어 round()로 반올림하도록 수정.
        clamped = max(-self.max_rpm, min(self.max_rpm, v))
        return int(round(clamped))

    def control_word(self, master, driver_id, word):
        try:
            master.execute(driver_id, cst.WRITE_SINGLE_REGISTER,
                            REG_CONTROL_WORD, output_value=word)
        except Exception as e:
            self.get_logger().error(f"Control Word 에러: {e}")

    def speed_mode_speed_set_sync(self, master, driver_id, speed_l, speed_r):
        speed_l = self._clamp_rpm(speed_l)
        speed_r = self._clamp_rpm(speed_r)
        try:
            master.execute(driver_id, cst.WRITE_MULTIPLE_REGISTERS, REG_TARGET_VEL,
                            output_value=[self._to_u16(speed_l), self._to_u16(speed_r)])
        except Exception as e:
            self.get_logger().error(f"Sync Speed 에러 (ID:{driver_id}): {e}")

    # =========================================================
    # 속도 명령 콜백
    # =========================================================
    def speed_callback(self, msg):
        speeds = msg.data
        expected_len = len(self.masters) * 2

        # 배열 길이가 안 맞으면 일부만 처리하지 말고 전체를 무시 (부분 반영 시 궤적 왜곡 방지)
        if len(speeds) != expected_len:
            self.get_logger().warn(
                f"wheel_speeds 길이 불일치: 수신 {len(speeds)}개, 기대 {expected_len}개 -> 명령 무시"
            )
            return

        self.last_speed_time = self.get_clock().now()

        if self.motor_stopped:
            self.get_logger().info("명령 수신 재개: 모터 재-enable")
            for m_info in self.masters:
                if m_info['master'] is None:
                    continue
                self.control_word(m_info['master'], m_info['config']['id'], CTRL_ENABLE)
            self.motor_stopped = False

        for i, m_info in enumerate(self.masters):
            if m_info['master'] is None:
                continue  # 연결 안 된 포트는 스킵 (인덱스는 그대로 유지됨)

            idx = i * 2
            speed_L = speeds[idx]
            speed_R = speeds[idx + 1]

            self.speed_mode_speed_set_sync(
                m_info['master'], m_info['config']['id'], speed_L, speed_R
            )

    # =========================================================
    # 소프트웨어 워치독 (드라이버 하드웨어 워치독의 2중 안전장치)
    # =========================================================
    def check_no_command(self):
        now = self.get_clock().now()
        diff = (now - self.last_speed_time).nanoseconds / 1e9

        if diff > self.watchdog_timeout and not self.motor_stopped:
            self.get_logger().warn(
                f"{self.watchdog_timeout}초 동안 명령 없음 -> 전체 정지"
            )
            for m_info in self.masters:
                if m_info['master'] is None:
                    continue
                master = m_info['master']
                d_id = m_info['config']['id']
                self.speed_mode_speed_set_sync(master, d_id, 0, 0)
                self.control_word(master, d_id, CTRL_STOP)
            self.motor_stopped = True

    # =========================================================
    # [수정] 에러코드 비트 디코딩 (여러 에러가 동시에 발생해도 전부 표시)
    # 기존에는 ERROR_MESSAGES.get(err, hex(err))로 단일 값만 조회했기 때문에,
    # 예를 들어 과전압(0x0001)+과전류(0x0004)가 동시에 나서 0x0005가 되면
    # 사전에 없는 값이라 그냥 "0x5"로만 찍혀서 원인 파악이 어려웠습니다.
    # =========================================================
    def _decode_error_bits(self, code):
        if code == 0:
            return "정상"
        msgs = [msg for bit, msg in ERROR_MESSAGES.items() if bit != 0 and (code & bit)]
        return " + ".join(msgs) if msgs else hex(code)

    # =========================================================
    # 런타임 에러코드 모니터링
    # =========================================================
    def check_faults(self):
        for m_info in self.masters:
            if m_info['master'] is None:
                continue
            master = m_info['master']
            d_id = m_info['config']['id']
            port_name = m_info['config']['port']
            try:
                err_l = master.execute(d_id, cst.READ_HOLDING_REGISTERS,
                                        REG_ERROR_L, quantity_of_x=1)[0]
                err_r = master.execute(d_id, cst.READ_HOLDING_REGISTERS,
                                        REG_ERROR_R, quantity_of_x=1)[0]
                if err_l != 0 or err_r != 0:
                    self.get_logger().error(
                        f"[{port_name}] 드라이버 에러 감지! "
                        f"L: {self._decode_error_bits(err_l)}, "
                        f"R: {self._decode_error_bits(err_r)}"
                    )
            except Exception as e:
                self.get_logger().error(f"[{port_name}] 에러코드 조회 실패: {e}")

    # =========================================================
    # 초기 연결 확인 (진단용)
    # =========================================================
    def check_motor_connection(self):
        self.get_logger().info("모터 연결 상태 확인을 시작합니다...")
        for m_info in self.masters:
            port_name = m_info['config']['port']
            if m_info['master'] is None:
                self.get_logger().error(f"[{port_name}] 연결되지 않음 (초기화 실패)")
                continue
            master = m_info['master']
            d_id = m_info['config']['id']
            try:
                master.execute(d_id, cst.READ_HOLDING_REGISTERS,
                                REG_OFFLINE_TIME, quantity_of_x=1)
                self.get_logger().info(f"[{port_name}] ID:{d_id} 연결 정상")
            except modbus_tk.modbus.ModbusTimeoutError:
                self.get_logger().error(f"[{port_name}] 연결 실패 - Timeout (케이블/포트/baudrate 확인)")
            except modbus_tk.modbus.ModbusError as e:
                self.get_logger().error(f"[{port_name}] 연결 실패 - Modbus 에러(ID 확인 필요): {e}")
            except Exception as e:
                self.get_logger().error(f"[{port_name}] 알 수 없는 오류: {e}")

    # =========================================================
    # 안전 종료
    # =========================================================
    def destroy_node(self):
        self.get_logger().info("노드 종료: 모든 모터 정지")
        for m_info in self.masters:
            if m_info['master'] is None:
                continue
            master = m_info['master']
            d_id = m_info['config']['id']
            try:
                self.speed_mode_speed_set_sync(master, d_id, 0, 0)
                self.control_word(master, d_id, CTRL_STOP)
                master.close()
            except Exception as e:
                self.get_logger().error(f"종료 중 정지 실패: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    node.check_motor_connection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
