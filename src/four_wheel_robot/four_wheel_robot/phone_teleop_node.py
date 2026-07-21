#!/usr/bin/env python3

# cmd_vel 토픽으로 발행되는 메시지를 실시간으로 출력
# python3 /home/limdoyeon/capstone_4_1/src/four_wheel_robot/four_wheel_robot/phone_teleop_node.py
# ros2 topic echo /cmd_vel
# ros2 topic hz /cmd_vel   <- 발행 주기가 안정적인지 확인할 때 사용

# 실행 예시 (노트북 IP를 반드시 지정해야 네트워크 워치독이 동작합니다):
# ros2 run four_wheel_robot phone_teleop_node --ros-args -p remote_host_ip:=192.168.0.xxx

# [GUI 확대] 이 버전은 1/4 축소 버전 대비 GUI 크기를 2배로 키운 버전입니다
# (즉 원본 기준으로는 1/2 크기). 창 크기, 캔버스, 조이스틱 반지름, 손잡이 크기,
# 여백, 폰트 크기를 모두 축소판의 2배로 스케일했습니다
# (버튼 width/height는 문자 단위라 그대로 유지).

# =======================================================================================================================================
# =                                                          라이브러리                                                                   =
# =======================================================================================================================================

import rclpy                            # ros2 파이썬 클라이언트 라이브러리
from rclpy.node import Node             # 독립된 노드 사용
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist     # 로봇 속도 명령을 담는 표준 메세지 타입
from std_msgs.msg import Int32          # 제어 모드 정보를 전달하기 위한 메시지 타입
import tkinter as tk                    # 파이썬에서 GUI를 그리기 위한 라이브러리
from tkinter import ttk                 # 파이썬에서 GUI를 그리기 위한 라이브러리
import threading                        # GUI/ROS/ping 워치독을 동시에 돌리기 위한 멀티스레딩 라이브러리
import subprocess                       # ping 명령 실행용
import time                             # 워치독 루프의 sleep 용
import math                             # 조이스틱 벡터 계산용

# =======================================================================================================================================
# =                                                          초기선언                                                                     =
# =======================================================================================================================================

class FourWheelSteeringTeleop(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_teleop')

        # ROS 2 파라미터(기본값)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_accel', 1.0)
        self.declare_parameter('angular_accel', 1.0)
        self.declare_parameter('publish_rate', 50.0)

        # 네트워크(ping) 기반 연결 감시 파라미터
        self.declare_parameter('remote_host_ip', '100.107.95.7')
        self.declare_parameter('ping_interval', 1.0)
        self.declare_parameter('ping_timeout', 0.5)
        self.declare_parameter('ping_fail_threshold', 1)

        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.lin_accel = self.get_parameter('linear_accel').value
        self.ang_accel = self.get_parameter('angular_accel').value
        self.rate = self.get_parameter('publish_rate').value

        self.remote_host_ip = self.get_parameter('remote_host_ip').value
        self.ping_interval = float(self.get_parameter('ping_interval').value)
        self.ping_timeout = float(self.get_parameter('ping_timeout').value)
        self.ping_fail_threshold = int(self.get_parameter('ping_fail_threshold').value)

        # [수정] cmd_vel / control_mode 는 "최신 값이 항상 중요한" 스트리밍 토픽이라
        # depth를 1로 낮췄습니다. 네트워크가 잠깐 막혔다가 뚫릴 때 큐에 쌓인 오래된
        # 메시지들을 순서대로 몰아서 처리하는 대신, 항상 최신 명령 하나만 남도록 합니다.
        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', stream_qos)
        self.mode_pub = self.create_publisher(Int32, 'control_mode', 10)

        # 속도 변수
        self.current_v = 0.0        # linear.x
        self.current_w = 0.0        # angular.z
        self.current_y = 0.0        # linear.y (모드 2용)
        self.target_v = 0.0
        self.target_w = 0.0
        self.target_y = 0.0
        self.current_mode = 1       # 1: Ackermann, 2: Crab, 3: Spin

        # [추가] target_v/w/y 는 Tk 메인 스레드(터치 드래그 이벤트)에서 쓰고,
        # 제어 루프(_control_loop)는 별도 ROS 스핀 스레드에서 읽으므로
        # 세 값이 항상 같은 순간의 조합으로 읽히도록 락으로 보호합니다.
        self._target_lock = threading.Lock()

        # 드래그 상태
        self.dragging = False

        # 네트워크(ping) 워치독 상태
        self.network_alive = True
        self.ping_fail_count = 0
        self.network_stop_triggered = False

        if self.remote_host_ip:
            self.ping_thread = threading.Thread(target=self._ping_watchdog_loop, daemon=True)
            self.ping_thread.start()
            self.get_logger().info(f"네트워크 워치독 시작: {self.remote_host_ip} 대상 ping 감시")
        else:
            self.get_logger().warn(
                "⚠️ remote_host_ip 파라미터가 비어있어 네트워크 워치독이 비활성화되었습니다."
            )

        # [GUI 확대] 창 크기를 375x250(1/4 축소판)의 2배인 750x500으로 확대
        self.root = tk.Tk()
        self.root.title("4축 조향 로봇 Teleop - 조이스틱")
        self.root.geometry("750x500")
        self.root.resizable(False, False)
        self.create_gui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # =====================================================================
        # [핵심 수정] cmd_vel 발행 루프를 GUI 렌더링에서 완전히 분리합니다.
        #
        # 기존 구조: tkinter의 root.after()가 rclpy.spin_once() + update()(램프+발행)를
        # 같은 콜백에서 처리했습니다. NoMachine으로 폰에 화면을 스트리밍하면 조이스틱을
        # 드래그할 때마다 캔버스 다시 그리기가 네트워크로 전송되면서 로컬 마우스보다
        # 훨씬 무거워지고, 이 렌더링이 밀리면 root.after() 콜백 자체가 밀립니다.
        # 그러면 update()에서 쓰던 dt(=1/rate 고정값)가 실제 경과 시간과 어긋나면서
        # 속도가 계단식으로 튀어 조향이 뚝뚝 끊기는 현상으로 나타났습니다.
        #
        # 수정 구조: rclpy.spin()을 별도 스레드에서 돌리고, ROS 타이머(_control_loop)가
        # "실측 dt"로 램프 계산 + cmd_vel 발행을 전담합니다. GUI는 입력을 받아서
        # target_v/w/y를 갱신하고 화면을 그리는 역할만 하므로, NoMachine 렌더링이
        # 아무리 지연되어도 로봇으로 나가는 cmd_vel 주기에는 영향을 주지 않습니다.
        # =====================================================================
        self.last_control_time = self.get_clock().now()
        self.control_timer = self.create_timer(1.0 / self.rate, self._control_loop)

        self.publish_mode()
        self.get_logger().info("4축 조향 로봇 Teleop(조이스틱) 시작")

# =======================================================================================================================================
# =                                                          GUI 창                                                                     =
# =======================================================================================================================================

    def create_gui(self):
        # [GUI 확대] 좌측: 조이스틱 캔버스, 우측: 모드/상태/버튼들을 배치하는 가로 레이아웃
        # (여백/패딩도 축소판의 2배로 확대)
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side="left", fill="both", expand=True)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side="right", fill="y", padx=(16, 0))

        ttk.Label(right_frame, text="4축 조향 로봇\n조이스틱 제어", font=("Arial", 12, "bold"),
                  justify="center").pack(pady=(0, 10))

        # 모드 선택 버튼 (기존 키 1/2/3 대체)
        mode_frame = ttk.Frame(right_frame)
        mode_frame.pack(pady=6)
        self.mode_buttons = {}
        for m, label in [(1, "Ackermann"), (2, "Crab"), (3, "Spin")]:
            btn = tk.Button(mode_frame, text=label, width=12, height=2, font=("Arial", 8, "bold"),
                             command=lambda mm=m: self.set_mode(mm))
            btn.pack(pady=4, fill="x")
            self.mode_buttons[m] = btn
        self._refresh_mode_buttons()

        # 상태 표시 라벨
        self.status_label = ttk.Label(right_frame, text="동작: 정지", font=("Arial", 8), foreground="blue")
        self.status_label.pack(pady=8)

        self.label_lx = ttk.Label(right_frame, text="linear.x  :  0.000 m/s", font=("Consolas", 8), foreground="red")
        self.label_ly = ttk.Label(right_frame, text="linear.y  :  0.000 m/s", font=("Consolas", 8))
        self.label_az = ttk.Label(right_frame, text="angular.z :  0.000 rad/s", font=("Consolas", 8), foreground="red")
        self.label_lx.pack(anchor="w", pady=2)
        self.label_ly.pack(anchor="w", pady=2)
        self.label_az.pack(anchor="w", pady=2)

        self.mode_label = ttk.Label(right_frame, text="모드: Ackermann", font=("Arial", 8), foreground="green")
        self.mode_label.pack(pady=6)

        self.net_label = ttk.Label(right_frame, text="네트워크: 감시 안 함", font=("Arial", 6), foreground="gray")
        self.net_label.pack(pady=4)

        # 비상정지 버튼
        stop_btn = tk.Button(right_frame, text="■ 비상정지", font=("Arial", 12, "bold"),
                              fg="white", bg="#cc3333", height=3, width=16, command=self.emergency_stop)
        stop_btn.pack(pady=10)

        ttk.Button(right_frame, text="창 닫기 & 정지", command=self.on_closing).pack(pady=4)

        # [GUI 확대] 조이스틱 영역 (축소판 250 -> 500, 즉 원본 대비 정확히 1/2 크기)
        # Ackermann 모드: 2축 원형 조이스틱 (기존 방식 그대로 유지)
        # Crab / Spin 모드: 좌우로만 움직이는 1축 슬라이더로 전환
        self.joy_size = 500
        self.joy_center = self.joy_size / 2
        self.joy_radius = self.joy_size / 2 - 30      # 2축 모드 바깥 원 반지름 (축소판 15 -> 30)
        self.knob_radius = 46                          # 손잡이(knob) 반지름 (축소판 23 -> 46)

        self.canvas = tk.Canvas(left_frame, width=self.joy_size, height=self.joy_size,
                                 bg="#eeeeee", highlightthickness=1, highlightbackground="#999999")
        self.canvas.pack(expand=True)

        self._draw_joystick_track()

        self.canvas.bind("<ButtonPress-1>", self._on_joy_press)
        self.canvas.bind("<B1-Motion>", self._on_joy_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_joy_release)

    def _refresh_mode_buttons(self):
        for m, btn in self.mode_buttons.items():
            btn.config(relief="sunken" if m == self.current_mode else "raised",
                       bg="#a9d18e" if m == self.current_mode else "#f0f0f0")

    def _draw_joystick_track(self):
        # [추가] 모드에 따라 조이스틱 트랙을 다시 그립니다.
        # Ackermann(1): 원형 2축 트랙
        # Crab(2), Spin(3): 가로 막대 1축 트랙 (세로 움직임 없음)
        self.canvas.delete("all")

        if self.current_mode == 1:
            # 원형 트랙
            self.canvas.create_oval(
                self.joy_center - self.joy_radius, self.joy_center - self.joy_radius,
                self.joy_center + self.joy_radius, self.joy_center + self.joy_radius,
                outline="#888888", width=1
            )
            self.canvas.create_line(self.joy_center, self.joy_center - self.joy_radius,
                                     self.joy_center, self.joy_center + self.joy_radius, fill="#cccccc")
            self.canvas.create_line(self.joy_center - self.joy_radius, self.joy_center,
                                     self.joy_center + self.joy_radius, self.joy_center, fill="#cccccc")
        else:
            # [추가/1축] 가로로 긴 트랙만 그려서 좌우로만 움직인다는 걸 시각적으로 표시
            # (축소판 18 -> 36)
            track_half_height = 36
            self.canvas.create_rectangle(
                self.joy_center - self.joy_radius, self.joy_center - track_half_height,
                self.joy_center + self.joy_radius, self.joy_center + track_half_height,
                outline="#888888", width=1, fill="#e5e5e5"
            )
            self.canvas.create_line(self.joy_center, self.joy_center - track_half_height,
                                     self.joy_center, self.joy_center + track_half_height, fill="#cccccc")
            label = "Crab: 좌/우 = 측면 이동" if self.current_mode == 2 else "Spin: 좌/우 = 제자리 회전"
            self.canvas.create_text(self.joy_center, self.joy_center - track_half_height - 20,
                                     text=label, font=("Arial", 10, "bold"), fill="#555555")

        # 손잡이(knob) - 항상 중앙에서 시작
        self.knob = self.canvas.create_oval(
            self.joy_center - self.knob_radius, self.joy_center - self.knob_radius,
            self.joy_center + self.knob_radius, self.joy_center + self.knob_radius,
            fill="#3366cc", outline=""
        )

# =======================================================================================================================================
# =                                                     조이스틱 입력 처리                                                                  =
# =======================================================================================================================================

    def set_mode(self, m):
        # 모드 버튼 탭 -> 즉시 정지 후 모드 전환 + 트랙 다시 그리기(2축<->1축)
        self.current_mode = m
        self._stop_all()
        self.dragging = False
        self._refresh_mode_buttons()
        self._draw_joystick_track()
        self.publish_mode()

    def _on_joy_press(self, event):
        self.dragging = True
        self._update_from_pointer(event.x, event.y)

    def _on_joy_drag(self, event):
        if self.dragging:
            self._update_from_pointer(event.x, event.y)

    def _on_joy_release(self, event):
        self.dragging = False
        with self._target_lock:
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_y = 0.0
        self._reset_knob_position()

    def _reset_knob_position(self):
        self.canvas.coords(
            self.knob,
            self.joy_center - self.knob_radius, self.joy_center - self.knob_radius,
            self.joy_center + self.knob_radius, self.joy_center + self.knob_radius
        )

    def _update_from_pointer(self, x, y):
        max_dist = self.joy_radius

        if self.current_mode == 1:
            # [기존 방식 유지] Ackermann: 2축(원형) 조이스틱
            dx = x - self.joy_center
            dy = y - self.joy_center
            dist = math.hypot(dx, dy)

            if dist > max_dist:
                scale = max_dist / dist
                dx *= scale
                dy *= scale

            self.canvas.coords(
                self.knob,
                self.joy_center + dx - self.knob_radius, self.joy_center + dy - self.knob_radius,
                self.joy_center + dx + self.knob_radius, self.joy_center + dy + self.knob_radius
            )

            norm_x = dx / max_dist if max_dist > 0 else 0.0
            norm_y = -dy / max_dist if max_dist > 0 else 0.0

            with self._target_lock:
                self.target_v = norm_y * self.max_v
                self.target_w = -norm_x * self.max_w
                self.target_y = 0.0

        else:
            # [추가/1축] Crab / Spin: 좌우로만 움직이는 1축 슬라이더
            # y 좌표는 무시하고 x축 이동량만 사용, knob은 항상 중앙 높이에 고정
            dx = x - self.joy_center
            if dx > max_dist:
                dx = max_dist
            elif dx < -max_dist:
                dx = -max_dist

            self.canvas.coords(
                self.knob,
                self.joy_center + dx - self.knob_radius, self.joy_center - self.knob_radius,
                self.joy_center + dx + self.knob_radius, self.joy_center + self.knob_radius
            )

            norm_x = dx / max_dist if max_dist > 0 else 0.0

            with self._target_lock:
                if self.current_mode == 2:
                    # Crab: 좌우 = 측면 이동만, 전후진은 사용 안 함
                    self.target_v = 0.0
                    self.target_y = norm_x * self.max_v
                    self.target_w = 0.0
                else:
                    # Spin: 좌우 = 제자리 회전만
                    self.target_v = 0.0
                    self.target_y = 0.0
                    self.target_w = -norm_x * self.max_w

    def emergency_stop(self):
        self.dragging = False
        self._stop_all()
        self._reset_knob_position()

    def _stop_all(self):
        with self._target_lock:
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_y = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_y = 0.0

# =======================================================================================================================================
# =                                                     네트워크(ping) 워치독                                                              =
# =======================================================================================================================================

    def _ping_watchdog_loop(self):
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
# =                                                     제어 루프 (ROS 타이머, GUI와 분리됨)                                                =
# =======================================================================================================================================

    def _control_loop(self):
        """publish_rate 주기의 ROS 타이머 콜백. GUI 렌더링/NoMachine 지연과 무관하게
        항상 정해진 주기로 실행되며, dt도 실제 경과시간을 측정해서 사용합니다."""
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        # 컴퓨터가 잠깐 멈췄다 돌아오는 등 dt가 비정상적으로 커지는 경우
        # 가감속이 한 번에 과도하게 튀지 않도록 상한을 둡니다.
        dt = min(dt, 0.2)

        moving = False
        if self.remote_host_ip and not self.network_alive:
            with self._target_lock:
                moving = (self.target_v != 0.0 or self.target_w != 0.0 or self.target_y != 0.0
                          or self.current_v != 0.0 or self.current_w != 0.0 or self.current_y != 0.0)
                if moving:
                    self.network_stop_triggered = True
                    self.target_v = 0.0
                    self.target_w = 0.0
                    self.target_y = 0.0
            if moving:
                self.dragging = False

        with self._target_lock:
            target_v = self.target_v
            target_w = self.target_w
            target_y = self.target_y

        try:
            self.current_v = self._ramp(self.current_v, target_v, self.lin_accel * dt)
            self.current_w = self._ramp(self.current_w, target_w, self.ang_accel * dt)
            self.current_y = self._ramp(self.current_y, target_y, self.lin_accel * dt)

            twist = Twist()
            twist.linear.x = self.current_v
            twist.linear.y = self.current_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.current_w
            self.publisher.publish(twist)

            # GUI 갱신은 반드시 Tk 메인 스레드에서만 수행 (root.after(0, ...)로 큐잉)
            if self.root is not None and self.root.winfo_exists():
                self.root.after(0, self._safe_update_gui, twist)
        except Exception as e:
            self.get_logger().error(f"제어 루프 중 예외: {e}")
            with self._target_lock:
                self.target_v = 0.0
                self.target_w = 0.0
                self.target_y = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            self.current_y = 0.0
            self.dragging = False
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
            self.label_az.config(text=f"angular.z : {twist.angular.z:7.3f} rad/s")

            mode_name = {1: "Ackermann", 2: "Crab Driving", 3: "Zero Turn"}.get(self.current_mode, "Unknown")
            self.mode_label.config(text=f"모드: {mode_name}")

            status = "전진" if twist.linear.x > 0.05 else "후진" if twist.linear.x < -0.05 else "정지"
            if abs(twist.angular.z) > 0.05:
                status += " + 회전"
            if abs(twist.linear.y) > 0.05:
                status += " + 측면"

            if self.network_stop_triggered:
                status += "  ⚠ 네트워크 단절로 자동 정지됨"
                self.status_label.config(foreground="red")
            else:
                self.status_label.config(foreground="blue")

            if self.remote_host_ip:
                if self.network_alive:
                    self.net_label.config(text=f"네트워크: 정상 ({self.remote_host_ip})", foreground="green")
                else:
                    self.net_label.config(
                        text=f"네트워크: 단절 의심 ({self.remote_host_ip}, 연속실패 {self.ping_fail_count}회)",
                        foreground="red"
                    )
            else:
                self.net_label.config(text="네트워크: 감시 안 함", foreground="gray")

            self.status_label.config(text=f"동작: {status}")
        except Exception as e:
            self.get_logger().warn(f"GUI 업데이트 중 예외: {e}")

    def publish_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_closing(self):
        stop = Twist()
        self.publisher.publish(stop)
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelSteeringTeleop()

    # [수정/핵심] ROS 스핀(타이머/구독 콜백 처리)을 GUI와 별도 스레드에서 돌립니다.
    # 이렇게 하면 tkinter 메인루프가 NoMachine 렌더링으로 아무리 바빠도
    # cmd_vel을 발행하는 _control_loop 타이머는 정해진 주기대로 계속 돕니다.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()