#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time      # 시간 지연(sleep) 및 실시간 타이머 계산을 위한 모듈
import serial    # RS485/시리얼 통신을 하기 위한 PySerial 라이브러리
import sys       # 시스템 관련 함수 및 입력 버퍼 비우기용
import select    # 키보드 입력 비동기 감지용 (Linux/macOS 환경용)
from dataclasses import dataclass  # 데이터를 구조화하여 관리하기 위한 dataclass 데코레이터

HEADER = bytes((0xFF, 0xFE))  # 누리로봇 프로토콜의 시작을 알리는 고정된 헤더 2바이트
MODE_ACC_SPEED = 0x03         # 모터의 '가감속 속도 제어' 명령 모드 번호
MODE_SET_ONOFF = 0x0C         # 모터의 '토크 On/Off (구동 활성화/비활성화)' 명령 모드 번호
REVERSE_DIRECTION = False     # 물리적 배선 때문에 모터가 반대로 돌 때 True로 바꾸는 사용자 설정 플래그

# 실행 도중 중복 명령이 들어오는 것을 막기 위한 플래그 변수
is_running = False

def to_be16(v: int) -> bytes:
    v = max(0, min(65533, int(v)))
    return bytes(((v >> 8) & 0xFF, v & 0xFF))

def calc_checksum(frame: bytes) -> int:
    s = 0
    for i, b in enumerate(frame):
        if i in (0, 1, 4):
            continue
        s = (s + b) & 0xFF
    return (~s) & 0xFF

def build_frame(dev_id: int, mode: int, data: bytes = b'') -> bytes:
    size = 1 + 1 + len(data)
    buf = bytearray(6 + len(data))
    buf[0:2] = HEADER         
    buf[2] = dev_id & 0xFF    
    buf[3] = size & 0xFF      
    buf[5] = mode & 0xFF      
    if data:
        buf[6:] = data        
    buf[4] = calc_checksum(buf)  
    return bytes(buf)         

@dataclass
class LinkConfig:
    port: str
    baud: int = 9600
    timeout: float = 0.5

class Link:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self.ser = None

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.cfg.port, baudrate=self.cfg.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=self.cfg.timeout
            )
            print(f"✓ 연결 성공: {self.cfg.port}")
            return True
        except Exception as e:
            print(f"✗ 연결 실패: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def write(self, frame: bytes):
        if self.ser and self.ser.is_open:
            self.ser.write(frame)
            self.ser.flush()

def print_info():
    print("\n" + "="*60)
    print("【 누리로봇 단축키 제어 프로그램 】")
    print("="*60)
    print("  u : 정회전 높아짐 (6000 RPM, 40초 구동 후 강력 정지)")
    print("  j : 역회전 낮아짐 (6000 RPM, 40초 구동 후 강력 정지)")
    print("  q : 프로그램 안전 종료")
    print("-"*60)
    print("※ 동작 중에는 다른 키 입력이 완전히 무시(중복 방지)됩니다.")
    if REVERSE_DIRECTION:
        print("⚠️  [주의] 방향 반전(REVERSE_DIRECTION)이 활성화 상태입니다.")
    print("="*60 + "\n")

def flush_input_buffer():
    """ 모터 구동 중 사용자가 마구 누른 키보드 잔여 입력을 지워주는 함수 """
    if sys.platform != "win32":
        import tcflush, termios
        try:
            tcflush(sys.stdin, termios.TCIFLUSH)
        except:
            pass

def get_key_noblocking():
    """ 엔터 없이 키보드 입력을 받아오는 함수 (Linux 환경 표준) """
    if sys.platform == "win32":
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore').lower()
    else:
        # Linux/macOS 비동기 입력 체크
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1).lower()
    return None

def countdown_timer(duration_sec: float):
    """ 실시간 남은 시간 카운트다운 """
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        remaining = duration_sec - elapsed
        
        if remaining <= 0:
            print("\r  ⏳ 00:00 ✅ 구동 완료. 정지 시퀀스 진입...", flush=True)
            break
        
        minutes = int(remaining) // 60
        seconds = int(remaining) % 60
        deciseconds = int((remaining % 1) * 10)
        
        print(f"\r  ⏳ {minutes:02d}:{seconds:02d}.{deciseconds} [동작 중 - 키 입력 잠금]", end="", flush=True)
        time.sleep(0.1)
    print()

def execute_macro(link: Link, dev_id: int, direction_cw: bool):
    """ u, j 키 입력 시 실행되는 6000RPM / 40초 / 10회 정지 매크로 함수 """
    global is_running
    is_running = True  # 중복 실행 방지 락(Lock) 활성화
    
    speed_rpm = 6500.0
    duration_sec = 40.0
    
    speed_raw = int(speed_rpm * 10)
    time_raw_hw = min(int(duration_sec * 10), 255) # 하드웨어 프로토콜 한계치(25.5초) 예외 처리
    
    if REVERSE_DIRECTION:
        direction_cw = not direction_cw
        
    direction_str = "정회전(CW)" if direction_cw else "역회전(CCW)"
    print(f"\n▶ : {speed_rpm} RPM / {direction_str} / {duration_sec}초 동안")
    
    # [방향(1B), 속도(2B), 도달시간(1B)] 프레임 빌드 및 전송
    data = bytes([0x01 if direction_cw else 0x00]) + to_be16(speed_raw) + bytes([time_raw_hw])
    link.write(build_frame(dev_id, MODE_ACC_SPEED, data))
    
    # 40초 대기 및 타이머 표시
    countdown_timer(duration_sec)
    
    # ★ 핵심 요구사항: 정지 명령 10번 연속 전송으로 유실 원천 차단 ★
    print(" [안전 대책] 정지 명령 10회 연속 전송 시작")
    stop_data = bytes([0x00]) + to_be16(0) + bytes([0x01])
    stop_frame = build_frame(dev_id, MODE_ACC_SPEED, stop_data)
    
    for i in range(10):
        link.write(stop_frame)
        print(f"  └─ 정지 신호 송신 ({i+1}/10)")
        time.sleep(0.05)  # 모터 회로가 연속 패킷을 처리할 수 있도록 50ms 미세 간격 부여
        
    print("✓ 모든 정지 명령 전송 완료. 시스템이 안전 상태입니다.\n")
    
    # 구동 중에 사용자가 난타한 키보드 버퍼를 깨끗이 비워 튀는 현상 방지
    flush_input_buffer()
    is_running = False  # 락 해제 (새로운 명령 수신 가능)

# ====================== 메인 스크립트 ======================
def main():
    # 리눅스 터미널 엔터 없이 입력받기 위한 초기 설정 (터미널 속성 보존)
    if sys.platform != "win32":
        import tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(sys.stdin.fileno())

    PORT = "/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BCD9B6ABCD-if06"
    DEV_ID = 0

    link = Link(LinkConfig(port=PORT))
    if not link.open():
        if sys.platform != "win32":
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return

    try:
        # 모터 토크 제어 On (활성화)
        link.write(build_frame(DEV_ID, MODE_SET_ONOFF, bytes([0x00])))
        time.sleep(0.3)
        
        print_info()

        while True:
            # 비동기로 키보드 한 글자 가져오기
            key = get_key_noblocking()
            
            if key:
                if is_running:
                    # 모터가 돌고 있는 중(is_running이 True)이면 입력된 키를 무시하고 루프 통과
                    continue
                
                if key == 'q':
                    print("\n프로그램 종료 요청을 수신했습니다.")
                    break
                    
                elif key == 'u':
                    # u 누르면 정회전 (True) 매크로 시작
                    execute_macro(link, DEV_ID, direction_cw=True)
                    print_info() # 매크로 끝난 후 안내창 다시 출력
                    
                elif key == 'j':
                    # j 누르면 역회전 (False) 매크로 시작
                    execute_macro(link, DEV_ID, direction_cw=False)
                    print_info()

            time.sleep(0.05) # CPU 과부하 방지를 위한 미세 휴식

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 강제 중단되었습니다.")
    except Exception as e:
        print(f"\n✗ 런타임 오류 발생: {e}")
    finally:
        # 최종 안전 정지 시퀀스 (종료 시에도 5번 연속 정지)
        print("\n[종료 안전 대책] 최종 안전 정지 명령 전송 중...")
        stop_frame = build_frame(DEV_ID, MODE_ACC_SPEED, bytes([0x00]) + to_be16(0) + bytes([0x01]))
        for _ in range(5):
            link.write(stop_frame)
            time.sleep(0.02)
        
        # 모터 토크 Off (릴리즈)
        link.write(build_frame(DEV_ID, MODE_SET_ONOFF, bytes([0x01])))
        link.close()
        
        # 리눅스 터미널 원래 속성으로 복원
        if sys.platform != "win32":
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
        print("프로그램이 안전하게 종료되었습니다. ✓")

if __name__ == "__main__":
    main()