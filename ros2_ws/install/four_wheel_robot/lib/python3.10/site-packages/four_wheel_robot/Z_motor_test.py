#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZLAC8015D 서보 드라이버 - 바퀴 1개씩 개별 테스트 스크립트
====================================================

목적:
  4륜 로봇에서 드라이버(포트)가 여러 개 연결되어 있을 때,
  ROS2 없이 시리얼 포트 하나 + 모터(L/R)를 지정해서
  "통신 연결 확인 -> 상태(전압/온도/에러) 조회 -> 저속 회전 테스트"
  를 한 바퀴씩 순서대로 검증할 수 있게 만든 스크립트입니다.

필요 라이브러리:
  pip install modbus_tk pyserial --break-system-packages

사용 예시:
  # 대화형 메뉴로 실행 (포트/속도 등을 직접 입력)
  python3 zlac8015d_wheel_test.py

  # 커맨드라인 인자로 바로 실행 (왼쪽 모터를 30rpm으로 3초간)
  python3 zlac8015d_wheel_test.py --port /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_XXXX-if00 --side L --rpm 30 --duration 3

주의:
  - 실제 바퀴가 지면에 닿아 있으면 로봇이 움직입니다.
    가능하면 로봇을 들어올리거나 바퀴가 자유롭게 헛돌 수 있는 상태에서 테스트하세요.
  - 기본 baudrate/ID는 매뉴얼 기준 각각 115200, 1 입니다.
    (RS485 통신 매뉴얼 표에는 RS485 Node ID 기본값이 4로 표기된 곳도 있어
     실제 장비의 기본값과 다를 수 있으니, 안 붙으면 --id 값을 바꿔보세요.)
"""

import sys
import time
import argparse

try:
    import serial
    import modbus_tk
    import modbus_tk.defines as cst
    from modbus_tk import modbus_rtu
except ImportError as e:
    print("필요한 라이브러리가 없습니다. 아래 명령으로 설치하세요:")
    print("  pip install modbus_tk pyserial --break-system-packages")
    sys.exit(1)


# ===================== 레지스터 주소 (ZLAC8015D RS485 매뉴얼 4장 기준) =====================
REG_CONTROL_MODE    = 0x200D   # 3: 속도모드
REG_CONTROL_WORD    = 0x200E   # 0x05 quick stop, 0x06 clear fault, 0x07 stop, 0x08 enable
REG_ACC_TIME_L      = 0x2080
REG_ACC_TIME_R      = 0x2081
REG_DEC_TIME_L      = 0x2082
REG_DEC_TIME_R      = 0x2083
REG_TARGET_VEL_L    = 0x2088   # -3000 ~ 3000 rpm
REG_TARGET_VEL_R    = 0x2089

REG_SW_VERSION      = 0x20A0
REG_DC_VOLTAGE      = 0x20A1   # unit: 0.01V
REG_MOTOR_STATE     = 0x20A2   # 상위8bit=Left, 하위8bit=Right (0:정지 1:구동)
REG_HALL_STATE      = 0x20A3
REG_MOTOR_TEMP      = 0x20A4   # unit: 1'C, 상위8bit=Left, 하위8bit=Right
REG_ERROR_L         = 0x20A5
REG_ERROR_R         = 0x20A6
REG_ACTUAL_VEL_L     = 0x20AB  # unit: 0.1 rpm
REG_ACTUAL_VEL_R     = 0x20AC
REG_ACTUAL_TORQUE_L  = 0x20AD  # unit: 0.1A
REG_ACTUAL_TORQUE_R  = 0x20AE

CONTROL_MODE_VELOCITY = 3
CTRL_QUICK_STOP  = 0x05
CTRL_CLEAR_FAULT = 0x06
CTRL_STOP        = 0x07
CTRL_ENABLE      = 0x08

MAX_RPM = 3000  # 매뉴얼상 속도모드 목표속도 범위

ERROR_MESSAGES = {
    0x0000: "정상 (No error)",
    0x0001: "과전압 (Over voltage)",
    0x0002: "저전압 (Under voltage)",
    0x0004: "과전류 (Over current)",
    0x0008: "과부하 (Over load)",
    0x0010: "전류 추종 오류 (Current following error)",
    0x0020: "위치 추종 오류 (Position following error)",
    0x0040: "속도 추종 오류 (Velocity following error)",
    0x0080: "기준 전압 오류 (Reference voltage error)",
    0x0100: "EEPROM 오류",
    0x0200: "홀센서 오류 (Hall error)",
    0x0400: "모터 과온 (Motor over temperature)",
}


def to_signed16(v):
    """unsigned 16bit 값을 signed 16bit로 변환"""
    return v - 0x10000 if v >= 0x8000 else v


def to_unsigned16(v):
    """음수를 modbus 전송용 unsigned 16bit 값으로 변환"""
    return v + 0x10000 if v < 0 else v


class ZlacTester:
    def __init__(self, port, baudrate=115200, slave_id=1, timeout=1.0):
        self.port_name = port
        self.slave_id = slave_id
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=8, parity='N', stopbits=1, xonxoff=0
        )
        self.master = modbus_rtu.RtuMaster(self.ser)
        self.master.set_timeout(timeout)
        self.master.set_verbose(False)

    # ---------- low level ----------
    def write_reg(self, addr, value):
        self.master.execute(
            self.slave_id, cst.WRITE_SINGLE_REGISTER,
            addr, output_value=to_unsigned16(value)
        )

    def write_regs(self, addr, values):
        values = [to_unsigned16(v) for v in values]
        self.master.execute(
            self.slave_id, cst.WRITE_MULTIPLE_REGISTERS,
            addr, output_value=values
        )

    def read_regs(self, addr, count=1):
        return self.master.execute(
            self.slave_id, cst.READ_HOLDING_REGISTERS,
            addr, quantity_of_x=count
        )

    # ---------- high level ----------
    def check_connection(self):
        print(f"\n[{self.port_name}] ID {self.slave_id} 연결 확인 중...")
        try:
            sw = self.read_regs(REG_SW_VERSION, 1)[0]
            dc = self.read_regs(REG_DC_VOLTAGE, 1)[0]
            print(f"  -> 통신 성공. SW 버전: {sw}, DC 전압: {dc * 0.01:.2f} V")
            return True
        except modbus_tk.modbus.ModbusTimeoutError:
            print("  -> 실패: 응답 없음 (Timeout). 케이블 / 포트 / baudrate / ID 확인 필요")
        except modbus_tk.modbus.ModbusError as e:
            print(f"  -> 실패: Modbus 에러 - {e} (ID 또는 레지스터 주소 문제일 가능성)")
        except Exception as e:
            print(f"  -> 실패: 알 수 없는 오류 - {e}")
        return False

    def read_status(self):
        try:
            state = self.read_regs(REG_MOTOR_STATE, 1)[0]
            hall = self.read_regs(REG_HALL_STATE, 1)[0]
            temp = self.read_regs(REG_MOTOR_TEMP, 1)[0]
            err_l = self.read_regs(REG_ERROR_L, 1)[0]
            err_r = self.read_regs(REG_ERROR_R, 1)[0]
            vel_l = to_signed16(self.read_regs(REG_ACTUAL_VEL_L, 1)[0]) * 0.1
            vel_r = to_signed16(self.read_regs(REG_ACTUAL_VEL_R, 1)[0]) * 0.1
            torque_l = to_signed16(self.read_regs(REG_ACTUAL_TORQUE_L, 1)[0]) * 0.1
            torque_r = to_signed16(self.read_regs(REG_ACTUAL_TORQUE_R, 1)[0]) * 0.1
        except Exception as e:
            print(f"  상태 조회 실패: {e}")
            return

        temp_l = (temp >> 8) & 0xFF
        temp_r = temp & 0xFF

        print("  ---- 상태 ----")
        print(f"  Motor state raw: 0x{state:04X} (상위8bit=L, 하위8bit=R / 0:정지 1:구동)")
        print(f"  Hall state  raw: 0x{hall:04X}")
        print(f"  온도(L/R): {temp_l}'C / {temp_r}'C")
        print(f"  Left  실제속도: {vel_l:6.1f} rpm | 실제토크: {torque_l:5.1f} A | "
              f"에러: {ERROR_MESSAGES.get(err_l, hex(err_l))}")
        print(f"  Right 실제속도: {vel_r:6.1f} rpm | 실제토크: {torque_r:5.1f} A | "
              f"에러: {ERROR_MESSAGES.get(err_r, hex(err_r))}")

    def clear_fault(self):
        self.write_reg(REG_CONTROL_WORD, CTRL_CLEAR_FAULT)
        time.sleep(0.05)

    def setup_velocity_mode(self, acc_ms=500, dec_ms=500):
        self.write_reg(REG_CONTROL_MODE, CONTROL_MODE_VELOCITY)
        self.write_reg(REG_ACC_TIME_L, acc_ms)
        self.write_reg(REG_ACC_TIME_R, acc_ms)
        self.write_reg(REG_DEC_TIME_L, dec_ms)
        self.write_reg(REG_DEC_TIME_R, dec_ms)
        self.write_reg(REG_CONTROL_WORD, CTRL_ENABLE)
        time.sleep(0.1)

    def set_velocity(self, side, rpm):
        rpm = max(-MAX_RPM, min(MAX_RPM, int(rpm)))  # 범위 클리핑
        addr = REG_TARGET_VEL_L if side == 'L' else REG_TARGET_VEL_R
        self.write_reg(addr, rpm)

    def stop(self):
        try:
            self.write_reg(REG_TARGET_VEL_L, 0)
            self.write_reg(REG_TARGET_VEL_R, 0)
            self.write_reg(REG_CONTROL_WORD, CTRL_STOP)
        except Exception:
            pass

    def close(self):
        self.stop()
        try:
            self.master.close()
        except Exception:
            pass

    def monitor_hall(self, interval=0.1):
        """
        모터를 Enable하지 않고 Hall 상태만 계속 모니터링.
        손으로 축을 돌리면서 Hall 값이 변하는지 확인한다.
        Ctrl+C 로 종료.
        """
        print("\n=== Hall Sensor Monitor ===")
        print("모터는 Enable하지 않습니다.")
        print("손으로 모터 축(또는 바퀴)을 천천히 돌려보세요.")
        print("Hall 값이 바뀌면 센서는 살아있는 것입니다.")
        print("Ctrl+C 로 종료\n")

        prev = None

        try:
            while True:
                hall = self.read_regs(REG_HALL_STATE, 1)[0]

                left = (hall >> 8) & 0xFF
                right = hall & 0xFF

                changed = ""
                if prev is not None and hall != prev:
                    changed = "  <=== CHANGED"

                print(
                    f"Hall Raw: 0x{hall:04X}   "
                    f"L=0x{left:02X}   "
                    f"R=0x{right:02X}"
                    f"{changed}"
                )

                prev = hall
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\nHall 모니터 종료.")


def run_single_wheel_test(port, slave_id, baudrate, side, rpm, duration):
    tester = ZlacTester(port, baudrate=baudrate, slave_id=slave_id)
    try:
        if not tester.check_connection():
            return

        tester.clear_fault()
        tester.setup_velocity_mode()

        print(f"\n[테스트] {side}측 모터를 {rpm} rpm 으로 {duration}초간 회전시킵니다.")
        print("주의: 바퀴가 지면에 닿아있으면 로봇이 움직입니다. 안전한 상태인지 확인하세요.")
        try:
            input("준비되면 Enter, 취소하려면 Ctrl+C 를 누르세요...")
        except KeyboardInterrupt:
            print("\n취소되었습니다.")
            return

        tester.set_velocity(side, rpm)
        start = time.time()
        while time.time() - start < duration:
            tester.read_status()
            time.sleep(0.5)

        print("\n정지합니다...")
        tester.stop()
        time.sleep(0.5)
        tester.read_status()
    except KeyboardInterrupt:
        print("\n중단됨. 모터를 정지합니다.")
    finally:
        tester.close()


def interactive_menu():
    print("=== ZLAC8015D 모터 / 드라이버 단일 바퀴 테스트 ===")
    port = input("시리얼 포트 경로 입력 "
                 "(예: /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_XXXX-if00): ").strip()
    baud = input("Baudrate (기본 115200): ").strip()
    baud = int(baud) if baud else 115200
    sid = input("드라이버 ID (기본 1): ").strip()
    sid = int(sid) if sid else 1

    while True:
        side = input(
            "\n테스트할 모터 선택  [L]eft / [R]ight / [S]tatus만 확인  /[H]all Monitor / [Q]uit: "
        ).strip().upper()
        if side == 'Q':
            break
        if side == 'S':
            tester = ZlacTester(port, baudrate=baud, slave_id=sid)
            try:
                if tester.check_connection():
                    tester.read_status()
            finally:
                tester.close()
            continue
        if side == 'H':
            tester = ZlacTester(port, baudrate=baud, slave_id=sid)
            try:
                if tester.check_connection():
                    tester.monitor_hall(interval=0.1)
            finally:
                tester.close()
            continue
        if side not in ('L', 'R'):
            print("잘못된 입력입니다.")
            continue

        rpm_in = input("목표 속도(rpm, 기본 50, 음수도 가능): ").strip()
        rpm = float(rpm_in) if rpm_in else 50
        dur_in = input("회전 시간(초, 기본 3): ").strip()
        dur = float(dur_in) if dur_in else 3

        run_single_wheel_test(port, sid, baud, side, rpm, dur)


def main():
    parser = argparse.ArgumentParser(description="ZLAC8015D 단일 바퀴 테스트")
    parser.add_argument('--port', help='시리얼 포트 경로')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--id', type=int, default=1, dest='slave_id')
    parser.add_argument('--side', choices=['L', 'R'], help='L 또는 R (지정하면 바로 테스트 실행)')
    parser.add_argument('--rpm', type=float, default=50)
    parser.add_argument('--duration', type=float, default=3)
    args = parser.parse_args()

    if args.port and args.side:
        run_single_wheel_test(args.port, args.slave_id, args.baud,
                               args.side, args.rpm, args.duration)
    else:
        interactive_menu()


if __name__ == "__main__":
    main()
