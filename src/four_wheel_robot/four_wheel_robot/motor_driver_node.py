# 터미널에 아래 코드로 실험해보면 됌
# ros2 topic pub --once /wheel_speeds std_msgs/msg/Float32MultiArray "{data: [50.0, -50.0]}"
# ros2 topic pub --once /wheel_speeds std_msgs/msg/Float32MultiArray "{data: [50.0, -50.0, 50.0, -50.0]}"

# ros2 run four_wheel_robot motor_driver_node

# ROS통신 관련 라이브러리
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

# 모터 통신관련 라이브러리
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

# 통신속도
BAUDRATE = 115200

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # =========================================================
        # 확장성을 위한 포트 설정 배열
        # 사용할 포트와 해당 포트에 연결된 드라이버의 ID(보통 1)를 적어줍니다
        # 나중에 바퀴가 추가되면 여기에 포트만 쉼표로 계속 추가하면 됩니다
        # =========================================================
        self.driver_configs = [ # ls -l /dev/serial/by-id/
            # 전륜 구동 모터 드라이버 (Quad Serial의 첫 번째 채널 예시)
            
            {'port': '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BCD9D7ABCD-if00', 'id': 1},
            
            # 후륜 구동 모터 드라이버가 추가된다면 아래와 같이 추가 (if02 등)
            {'port': '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BCD9D7ABCD-if06', 'id': 1},
        ]

        # 생성된 Modbus Master 객체들을 저장할 리스트
        self.masters = []

        # 각 포트마다 독립적인 통신 객체(Master)를 생성하고 초기화합니다.
        for config in self.driver_configs:
            port_name = config['port']
            d_id = config['id']
            try:
                # 1. 시리얼 포트 열기
                ser = serial.Serial(port=port_name, baudrate=BAUDRATE, bytesize=8, parity='N', stopbits=1, xonxoff=0)
                
                # 2. Modbus 마스터 생성
                master = modbus_rtu.RtuMaster(ser)
                master.set_timeout(1.0)
                master.set_verbose(False)
                
                # 3. 드라이버 초기 세팅 (속도 제어 모드, 모터 Enable)
                self.control_mode(master, d_id, 0x03)
                self.control_word(master, d_id, 0x08)
                
                # 4. 성공한 마스터 객체를 리스트에 저장
                self.masters.append({'config': config, 'master': master})
                self.get_logger().info(f"포트 연결 및 초기화 성공: {port_name} (ID: {d_id})")

            except Exception as e:
                self.get_logger().error(f"포트 연결 실패 [{port_name}]: {e}")

        # Subcriber: kinematics_node에서 발행하는 [속도1, 속도2, ...] 배열 수신
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_speeds', 
            self.speed_callback,
            10
        )
        self.safety_subscription = self.create_subscription(
            Int32,
            'control_mode',
            self.control_mode_callback,
            10
        )

        # 3초 동안 메시지 미수신 감지를 위한 타이머와 마지막 수신 시간
        self.last_speed_time = self.get_clock().now()
        self.no_command_timer = self.create_timer(0.5, self.check_no_command)  # 0.5초마다 체크
        self.motor_stopped = False  # 모터가 정지 상태인지 여부
        self.emergency_stop = False

        self.get_logger().info("motor_driver_node 다중 포트 모드 대기 중")

    # =========================================================
    # Modbus 제어 함수들 (이제 특정 master 객체를 인자로 받음)
    # =========================================================
    def control_mode(self, master, driver_id, mode):
        try:
            master.execute(driver_id, cst.WRITE_SINGLE_REGISTER, 0x200D, output_value=mode)
        except modbus_tk.modbus.ModbusError as e:
            self.get_logger().error(f"Mode Set 에러: {e}")

    def control_word(self, master, driver_id, word):
        try:
            master.execute(driver_id, cst.WRITE_SINGLE_REGISTER, 0x200E, output_value=word)
        except modbus_tk.modbus.ModbusError as e:
            self.get_logger().error(f"Control Word 에러: {e}")

    def speed_mode_speed_set_sync(self, master, driver_id, speed_l, speed_r):
        try:
            master.execute(driver_id, cst.WRITE_MULTIPLE_REGISTERS, 0x2088, output_value=[speed_l, speed_r])
        except modbus_tk.modbus.ModbusError as e:
            self.get_logger().error(f"Sync Speed 에러: {e}")

    # =========================================================
    # 콜백 함수 (속도 데이터 수신 시 실행)
    # =========================================================
    def control_mode_callback(self, msg):
        if int(msg.data) == 0:
            self.emergency_stop = True
            self.stop_motors()
        else:
            self.emergency_stop = False

    def speed_callback(self, msg):
        if self.emergency_stop:
            return

        # msg.data 형태: [1번바퀴, 2번바퀴, 3번바퀴, 4번바퀴...]
        speeds = msg.data

        self.last_speed_time = self.get_clock().now()

        # 정지 상태였다면 다시 Enable
        if self.motor_stopped:
            self.get_logger().info("wheel_speeds 메시지 수신 재개: 모터를 다시 Enable 합니다.")
            for m_info in self.masters:
                try:
                    master = m_info['master']
                    d_id = m_info['config']['id']
                    self.control_word(master, d_id, 0x08)  # Enable
                except Exception as e:
                    self.get_logger().error(f"모터 Enable 실패: {e}")
            self.motor_stopped = False

        # 연결된 각 마스터(포트)마다 2개씩 속도 데이터를 잘라서 보냄
        for i, m_info in enumerate(self.masters):
            idx = i * 2 # 0, 2, 4... 인덱스
            
            if idx + 1 < len(speeds):
                speed_L = int(speeds[idx])
                speed_R = int(speeds[idx + 1])
                
                master = m_info['master']
                d_id = m_info['config']['id']
                port_name = m_info['config']['port']

                # 해당 포트로 명령 전송
                self.speed_mode_speed_set_sync(master, d_id, speed_L, speed_R)
                # 디버깅용 출력이 필요하면 아래 주석 해제
                # self.get_logger().info(f"[{port_name}] 명령 전송: L={speed_L}, R={speed_R}")
            else:
                self.get_logger().warn(f"수신된 속도 데이터가 부족합니다.")

    # =========================================================
    # 3초 동안 속도 명령 미수신 시 모터 정지 기능
    # =========================================================
    def stop_motors(self):
        for m_info in self.masters:
            try:
                master = m_info['master']
                d_id = m_info['config']['id']
                self.speed_mode_speed_set_sync(master, d_id, 0, 0)
                self.control_word(master, d_id, 0x07)  # Disable
            except Exception as e:
                self.get_logger().error(f"모터 정지 명령 전송 실패: {e}")
        self.motor_stopped = True

    def check_no_command(self):
        now = self.get_clock().now()
        diff = (now - self.last_speed_time).nanoseconds / 1e9  # 초 단위

        if diff > 3.0 and not self.motor_stopped and not self.emergency_stop:
            self.get_logger().warn("3초 동안 wheel_speeds 메시지를 수신하지 못했습니다. 모든 모터를 정지합니다.")
            self.stop_motors()

    # =========================================================
    # 연결 확인 함수 (통신 문제 / ID 문제 구별)
    # =========================================================
    def check_motor_connection(self):
        self.get_logger().info("모터 연결 상태 확인을 시작합니다...")
        for m_info in self.masters:
            master = m_info['master']
            d_id = m_info['config']['id']
            port_name = m_info['config']['port']
            
            try:
                # 간단한 레지스터 읽기 시도로 연결 및 ID 확인
                response = master.execute(d_id, cst.READ_HOLDING_REGISTERS, 0x2000, quantity_of_x=1)
                self.get_logger().info(f"[{port_name}] ID:{d_id} 연결 정상 (응답 수신 성공)")
                
            except modbus_tk.modbus.ModbusTimeoutError:
                self.get_logger().error(f"[{port_name}] 연결 실패 - 통신 문제 (Timeout): 케이블, 포트, baudrate 확인 필요")
            except modbus_tk.modbus.ModbusError as e:
                if "illegal" in str(e).lower() or "function" in str(e).lower():
                    self.get_logger().error(f"[{port_name}] 연결 실패 - ID 문제 가능성 높음 (Illegal Function/Data Address): 드라이버 ID({d_id}) 또는 레지스터 주소 확인")
                else:
                    self.get_logger().error(f"[{port_name}] 연결 실패 - Modbus 에러 (ID 또는 기타): {e}")
            except Exception as e:
                self.get_logger().error(f"[{port_name}] 연결 확인 중 알 수 없는 오류: {e}")

    # =========================================================
    # 안전 종료 로직
    # =========================================================
    def destroy_node(self):
        self.get_logger().info("노드 종료: 모든 포트의 모터를 정지합니다.")
        for m_info in self.masters:
            try:
                master = m_info['master']
                d_id = m_info['config']['id']
                
                # 속도 0으로 만들고 모터 비활성화(0x07)
                self.speed_mode_speed_set_sync(master, d_id, 0, 0)
                self.control_word(master, d_id, 0x07)
                master.close() # 시리얼 포트 닫기
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    # 노드 시작 시 한 번 연결 확인 수행
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