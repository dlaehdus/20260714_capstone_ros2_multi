#!/usr/bin/env python3

# cmd_vel 토픽으로 발행되는 메시지를 실시간으로 출력

# python3 /home/limdoyeon/capstone_4_1/src/four_wheel_robot/four_wheel_robot/teleop_node.py

# ros2 topic echo /cmd_vel

# 실행 예시 (노트북 IP를 반드시 지정해야 네트워크 워치독이 동작합니다):
# ros2 run four_wheel_robot teleop_node --ros-args -p remote_host_ip:=192.168.0.xxx

# =======================================================================================================================================
# =                                                          라이브러리                                                                   =
# =======================================================================================================================================

import rclpy                            # ros2 파이썬 클라이언트 라이브러리
from rclpy.node import Node             # 독립된 노드 사용
from geometry_msgs.msg import Twist     # 로봇 속도 명령을 담는 표준 메세지 타입
from std_msgs.msg import Int32          # 제어 모드 정보를 전달하기 위한 메시지 타입
from pynput import keyboard             # 키보드 입력을 감지하고 모니터링 하기 위한 라이브러리
import tkinter as tk                    # 파이썬에서 GUI를 그리기 위한 라이브러리
from tkinter import ttk                 # 파이썬에서 GUI를 그리기 위한 라이브러리
import threading                        # GUI/ROS/ping 워치독을 동시에 돌리기 위한 멀티스레딩 라이브러리
import subprocess                       # ping 명령 실행용
import time                             # 워치독 루프의 sleep 용

# =======================================================================================================================================
# =                                                          초기선언                                                                     =
# =======================================================================================================================================

class FourWheelSteeringTeleop(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_teleop')                  # Node 클래스를 상속받아 'four_wheel_steering_teleop'이라는 이름의 ROS 2 노드를 생성

        # ROS 2 파라미터(기본값)를 선언
        self.declare_parameter('max_linear_speed', 1.0)                 # 최대 속도
        self.declare_parameter('max_angular_speed', 1.0)                # 최대 회전각
        self.declare_parameter('linear_accel', 1.0)                     # 선형가속도
        self.declare_parameter('angular_accel', 1.0)                    # 각가속도
        self.declare_parameter('publish_rate', 50.0)                    # 초당 데이터 발행 주기

        # [추가/안전 기능] 네트워크(ping) 기반 연결 감시 파라미터
        # 키 이벤트 패턴으로 연결 끊김을 추측하던 기존 input_idle_timeout 방식은
        # 두 개 이상의 키를 순차로 눌렀다 뗄 때(w를 누른 채 d를 눌렀다 뗌) OS의
        # auto-repeat 슬롯이 넘어가 원래 키의 이벤트가 다시 오지 않는 근본적 한계가
        # 있어 제거했습니다. 대신 원격 노트북과의 실제 네트워크 연결을 별도 스레드에서
        # ping으로 직접 확인합니다. remote_host_ip를 비워두면 워치독이 비활성화되니
        # 반드시 노트북의 IP를 넣어주세요.
        self.declare_parameter('remote_host_ip', '100.98.94.91')          # 노트북(원격 클라이언트) IP
        self.declare_parameter('ping_interval', 1.0)          # ping 주기(초)
        self.declare_parameter('ping_timeout', 0.5)           # ping 응답 대기(초)
        self.declare_parameter('ping_fail_threshold', 1)      # 연속 실패 허용 횟수 (이 이상 실패 시 정지)

        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.lin_accel = self.get_parameter('linear_accel').value
        self.ang_accel = self.get_parameter('angular_accel').value
        self.rate = self.get_parameter('publish_rate').value

        self.remote_host_ip = self.get_parameter('remote_host_ip').value
        self.ping_interval = float(self.get_parameter('ping_interval').value)
        self.ping_timeout = float(self.get_parameter('ping_timeout').value)
        self.ping_fail_threshold = int(self.get_parameter('ping_fail_threshold').value)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int32, 'control_mode', 10)

        # 속도 변수
        self.current_v = 0.0        # linear.x
        self.current_w = 0.0        # angular.z
        self.current_y = 0.0        # linear.y (모드 2용)
        self.target_v = 0.0
        self.target_w = 0.0
        self.target_y = 0.0
        self.keys_pressed = set()
        self.current_mode = 1       # 1: Ackermann, 2: Crab, 3: Spin

        # [추가/안전 기능] 네트워크(ping) 워치독 상태
        # network_alive: 원격 노트북이 ping에 응답하고 있는지 여부 (연결 감지 주 방어선)
        # ping_fail_count: 연속 ping 실패 횟수
        self.network_alive = True
        self.ping_fail_count = 0
        self.network_stop_triggered = False

        if self.remote_host_ip:
            self.ping_thread = threading.Thread(target=self._ping_watchdog_loop, daemon=True)
            self.ping_thread.start()
            self.get_logger().info(f"네트워크 워치독 시작: {self.remote_host_ip} 대상 ping 감시")
        else:
            self.get_logger().warn(
                "⚠️ remote_host_ip 파라미터가 비어있어 네트워크 워치독이 비활성화되었습니다. "
                "'--ros-args -p remote_host_ip:=<노트북IP>' 로 실행하세요."
            )

        # Tkinter 창
        self.root = tk.Tk()
        self.root.title("4축 조향 로봇 Teleop - linear.y 고정")
        self.root.geometry("500x580")
        self.root.resizable(False, False)
        self.create_gui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # 키보드 리스너
        self.listener = keyboard.Listener(on_press=self._safe_on_press, on_release=self._safe_on_release)
        self.listener.start()
        self.update_period_ms = max(10, int(1000.0 / self.rate))
        self.publish_mode()
        self.get_logger().info("4축 조향 로봇 Teleop 시작 (1: Ackermann, 2: Crab, 3: Spin)")

# =======================================================================================================================================
# =                                                          GUI 창                                                                     =
# =======================================================================================================================================

    def create_gui(self):
        ttk.Label(self.root, text="4축 조향 로봇 실시간 제어", font=("Arial", 18, "bold")).pack(pady=15)

        lin_frame = ttk.LabelFrame(self.root, text=" Linear Velocity (m/s) ", padding=15)
        lin_frame.pack(fill="x", padx=30, pady=10)

        self.label_lx = ttk.Label(lin_frame, text="linear.x  :  0.000 m/s", font=("Consolas", 14), foreground="red")
        self.label_ly = ttk.Label(lin_frame, text="linear.y  :  0.000 m/s", font=("Consolas", 14))
        self.label_lz = ttk.Label(lin_frame, text="linear.z  :  0.000 m/s", font=("Consolas", 14))

        self.label_lx.pack(anchor="w", pady=5)
        self.label_ly.pack(anchor="w", pady=5)
        self.label_lz.pack(anchor="w", pady=5)

        ang_frame = ttk.LabelFrame(self.root, text=" Angular Velocity (rad/s) ", padding=15)
        ang_frame.pack(fill="x", padx=30, pady=10)

        self.label_ax = ttk.Label(ang_frame, text="angular.x :  0.000 rad/s", font=("Consolas", 14))
        self.label_ay = ttk.Label(ang_frame, text="angular.y :  0.000 rad/s", font=("Consolas", 14))
        self.label_az = ttk.Label(ang_frame, text="angular.z :  0.000 rad/s", font=("Consolas", 14), foreground="red")

        self.label_ax.pack(anchor="w", pady=8)
        self.label_ay.pack(anchor="w", pady=8)
        self.label_az.pack(anchor="w", pady=8)

        self.status_label = ttk.Label(self.root, text="눌린 키: -   |   동작: 정지", font=("Arial", 12), foreground="blue")
        self.status_label.pack(pady=20)
        self.mode_label = ttk.Label(self.root, text="모드: Ackermann", font=("Arial", 12), foreground="green")
        self.mode_label.pack(pady=5)

        # 네트워크 상태 표시
        self.net_label = ttk.Label(self.root, text="네트워크: 감시 안 함", font=("Arial", 11), foreground="gray")
        self.net_label.pack(pady=3)

        ttk.Button(self.root, text="창 닫기 & 정지", command=self.on_closing).pack(pady=10)
        ttk.Label(self.root, text="[1] Ackermann / [2] Crab / [3] Spin / [Space] 비상정지", font=("Arial", 10), foreground="gray").pack(pady=2)

# =======================================================================================================================================
# =                                                     네트워크(ping) 워치독                                                              =
# =======================================================================================================================================

    def _ping_watchdog_loop(self):
        # [추가/안전 기능] 원격 노트북과의 실제 네트워크 연결을 별도 스레드에서
        # 주기적으로 ping하여 직접 확인합니다. 키 입력 패턴과 완전히 무관하므로
        # 다중 키 조작 중 오탐이 발생하지 않고, 실제로 와이파이가 끊기면 ping
        # 실패가 누적되어 확실하게 감지됩니다.
        while rclpy.ok():
            try:
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', str(self.ping_timeout), self.remote_host_ip],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    if self.ping_fail_count >= self.ping_fail_threshold:
                        self.get_logger().info("네트워크 연결 복구됨 (ping 응답 정상)")
                    self.ping_fail_count = 0
                    self.network_alive = True
                    self.network_stop_triggered = False
                else:
                    self.ping_fail_count += 1
            except Exception as e:
                self.get_logger().warn(f"ping 실행 중 예외: {e}")
                self.ping_fail_count += 1

            if self.ping_fail_count >= self.ping_fail_threshold:
                if not self.network_stop_triggered:
                    self.get_logger().error(
                        f"⚠️ [네트워크 단절] {self.remote_host_ip} 응답 없음 "
                        f"(연속 {self.ping_fail_count}회 실패) -> 즉시 비상정지"
                    )
                self.network_alive = False

            time.sleep(self.ping_interval)

# =======================================================================================================================================
# =                                                          키 입력                                                                     =
# =======================================================================================================================================

    def _safe_on_press(self, key):
        try:
            self.on_press(key)
        except Exception as e:
            self.get_logger().warn(f"키 입력 처리 중 예외(press): {e}")

    def _safe_on_release(self, key):
        try:
            self.on_release(key)
        except Exception as e:
            self.get_logger().warn(f"키 입력 처리 중 예외(release): {e}")

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.keys_pressed.clear()
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_y = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            self.current_y = 0.0
            return

        try:
            if key is None:
                return
            if hasattr(key, 'char') and key.char is not None:
                k = key.char.lower()
                if k in ['1', '2', '3']:
                    self.current_mode = int(k)
                    self.keys_pressed.clear()
                    self.target_v = 0.0
                    self.target_w = 0.0
                    self.target_y = 0.0
                    self.current_v = 0.0
                    self.current_w = 0.0
                    self.current_y = 0.0
                    self.publish_mode()
                    return
                if k in ['w', 's', 'a', 'd']:
                    self.keys_pressed.add(k)
                    self.update_target()
            else:
                self.get_logger().debug(f"특수 키 입력: {key}")
        except Exception as e:
            self.get_logger().warn(f"키 입력 처리 중 예외: {e}")

    def on_release(self, key):
        try:
            if key is None:
                return
            if hasattr(key, 'char') and key.char is not None:
                k = key.char.lower()
                self.keys_pressed.discard(k)
                self.update_target()
        except Exception as e:
            self.get_logger().warn(f"키 해제 처리 중 예외: {e}")

# =======================================================================================================================================
# =                                                          입력값 발행                                                                  =
# =======================================================================================================================================

    def update_target(self):
        v = 0.0
        w = 0.0
        y = 0.0

        if self.current_mode == 1:
            if 'w' in self.keys_pressed:
                v += self.max_v
            if 's' in self.keys_pressed:
                v -= self.max_v
            if 'a' in self.keys_pressed:
                w += self.max_w
            if 'd' in self.keys_pressed:
                w -= self.max_w
        elif self.current_mode == 2:
            if 'w' in self.keys_pressed:
                v += self.max_v
            if 's' in self.keys_pressed:
                v -= self.max_v
            if 'a' in self.keys_pressed:
                y -= self.max_v
            if 'd' in self.keys_pressed:
                y += self.max_v
        else:
            if 'a' in self.keys_pressed:
                w += self.max_w
            if 'd' in self.keys_pressed:
                w -= self.max_w

        self.target_v = v
        self.target_w = w
        self.target_y = y

    def _tk_update_loop(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.001)
            self.update()
        except Exception as e:
            self.get_logger().warn(f"Teleop loop error: {e}")
        finally:
            if self.root is not None and self.root.winfo_exists():
                self.root.after(self.update_period_ms, self._tk_update_loop)

    def update(self):
        dt = 1.0 / self.rate

        # [안전 기능] 네트워크(ping) 워치독
        # remote_host_ip가 설정되어 있고 network_alive가 False면 (ping 연속 실패)
        # 즉시(램프 없이) 비상정지합니다. 키 입력 패턴과 무관하므로 w+d 같은
        # 다중 키 조작 중에도 오탐이 발생하지 않습니다.
        if self.remote_host_ip and not self.network_alive:
            if self.keys_pressed or self.target_v != 0.0 or self.target_w != 0.0 or self.target_y != 0.0 \
                    or self.current_v != 0.0 or self.current_w != 0.0 or self.current_y != 0.0:
                self.network_stop_triggered = True
                self.keys_pressed.clear()
                self.target_v = 0.0
                self.target_w = 0.0
                self.target_y = 0.0
                self.current_v = 0.0
                self.current_w = 0.0
                self.current_y = 0.0

        try:
            self.current_v = self._ramp(self.current_v, self.target_v, self.lin_accel * dt)
            self.current_w = self._ramp(self.current_w, self.target_w, self.ang_accel * dt)
            self.current_y = self._ramp(self.current_y, self.target_y, self.lin_accel * dt)

            twist = Twist()
            twist.linear.x = self.current_v
            twist.linear.y = self.current_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.current_w
            self.publisher.publish(twist)

            if self.root is not None and self.root.winfo_exists():
                self.root.after(0, self._safe_update_gui, twist)
        except Exception as e:
            self.get_logger().error(f"Teleop update 중 예외: {e}")
            self.current_v = 0.0
            self.current_w = 0.0
            self.current_y = 0.0
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_y = 0.0
            self.keys_pressed.clear()
            stop = Twist()
            self.publisher.publish(stop)

# =======================================================================================================================================
# =                                                          계산부                                                                      =
# =======================================================================================================================================

    def _ramp(self, current, target, step):
        if current < target:
            return min(current + step, target)
        else:
            return max(current - step, target)

    def _safe_update_gui(self, twist):
        try:
            if self.root is None or not self.root.winfo_exists():
                return
            self.label_lx.config(text=f"linear.x  : {twist.linear.x:7.3f} m/s")
            self.label_ly.config(text=f"linear.y  : {twist.linear.y:7.3f} m/s")
            self.label_lz.config(text=f"linear.z  : {twist.linear.z:7.3f} m/s")
            self.label_az.config(text=f"angular.z : {twist.angular.z:7.3f} rad/s")

            mode_name = {1: "Ackermann", 2: "Crab Driving", 3: "Zero Turn"}.get(self.current_mode, "Unknown")
            self.mode_label.config(text=f"모드: {mode_name}")

            keys_str = ''.join(sorted(self.keys_pressed)) if self.keys_pressed else "-"
            status = "전진" if twist.linear.x > 0.05 else "후진" if twist.linear.x < -0.05 else "정지"
            if abs(twist.angular.z) > 0.05:
                status += " + 회전"
            if abs(twist.linear.y) > 0.05:
                status += " + 측면"

            if self.network_stop_triggered:
                status += "  ⚠ 네트워크 단절로 자동 정지됨 (와이파이 확인 필요)"
                self.status_label.config(foreground="red")
            else:
                self.status_label.config(foreground="blue")

            # 네트워크 상태 라벨 갱신
            if self.remote_host_ip:
                if self.network_alive:
                    self.net_label.config(text=f"네트워크: 정상 ({self.remote_host_ip})", foreground="green")
                else:
                    self.net_label.config(
                        text=f"네트워크: 단절 의심 ({self.remote_host_ip}, 연속실패 {self.ping_fail_count}회)",
                        foreground="red"
                    )
            else:
                self.net_label.config(text="네트워크: 감시 안 함 (remote_host_ip 미설정)", foreground="gray")

            self.status_label.config(text=f"눌린 키: {keys_str}   |   동작: {status}")
        except Exception as e:
            self.get_logger().warn(f"GUI 업데이트 중 예외: {e}")

    def publish_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_closing(self):
        stop = Twist()
        self.publisher.publish(stop)
        self.listener.stop()
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelSteeringTeleop()

    try:
        node.root.after(node.update_period_ms, node._tk_update_loop)
        node.root.mainloop()
    finally:
        try:
            node.listener.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()