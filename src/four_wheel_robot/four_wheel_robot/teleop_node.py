#!/usr/bin/env python3

# cmd_vel 토픽으로 발행되는 메시지를 실시간으로 출력
# ros2 topic echo /cmd_vel

# =======================================================================================================================================
# =                                                          라이브러리                                                                   =
# =======================================================================================================================================

import rclpy                            # ros2 파이썬 클라이언트 라이브러리
from rclpy.node import Node             # 독립된 노드 사용
# https://velog.io/@bbolddagu/ROS2-%EB%AA%A8%EB%93%88-%EA%B0%9C%EB%85%90
from geometry_msgs.msg import Twist     # 로봇 속도 명령을 담는 표준 메세지 타입
from std_msgs.msg import Int32          # 제어 모드 정보를 전달하기 위한 메시지 타입
from pynput import keyboard             # 키보드 입력을 감지하고 모니터링 하기 위한 라이브러리
import tkinter as tk                    # 파이썬에서 GUI를 그리기 위한 라이브러리
from tkinter import ttk                 # 파이썬에서 GUI를 그리기 위한 라이브러리
import threading                        # GUI와 ROS통신이 동시에 돌아가도록 분리하는 멀티스레딩 작업 라이브러리

# =======================================================================================================================================
# =                                                          초기선언                                                                     =
# =======================================================================================================================================

class FourWheelSteeringTeleop(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_teleop')                  # Node 클래스를 상속받아 'four_wheel_steering_teleop'이라는 이름의 ROS 2 노드를 생성

        # ROS 2 파라미터(기본값)를 선언
        # 이 노드(Node)에서 max_linear_speed라는 이름의 파라미터를 사용하겠다고 ROS 2 시스템에 등록(선언)하는 과정
        self.declare_parameter('max_linear_speed', 1.0)                 # 최대 속도
        self.declare_parameter('max_angular_speed', 1.0)                # 최대 회전각
        self.declare_parameter('linear_accel', 1.0)                     # 선형가속도
        self.declare_parameter('angular_accel', 1.0)                    # 각가속도
        self.declare_parameter('publish_rate', 50.0)                    # 초당 데이터 발행 주기

        # 파라미터를 선언만 해두면 ROS 2 시스템 내부에만 존재할 뿐, 파이썬 코드 안에서 계산할 때 바로 쓸 수는 없습니다. 
        # 그래서 그 값을 가져와서 파이썬 변수에 저장하는 과정이 필요합니다.
        # .value: 가져온 객체 중에서 실제 데이터 값만 빼오는 명령어입니다.
        # self.max_v =: 클래스 내부 변수(self.max_v)에 그 값을 대입
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.lin_accel = self.get_parameter('linear_accel').value
        self.ang_accel = self.get_parameter('angular_accel').value
        self.rate = self.get_parameter('publish_rate').value

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)    # cmd_vel이라는 이름의 토픽으로 Twist 메시지를 쏘는 발행자(Publisher)를 만듭니다
                                                                        # 메시지를 보관할 대기줄의 칸을 최대 10개로 제한
        self.mode_pub = self.create_publisher(Int32, 'control_mode', 10) # 제어 모드 토픽 발행자

        # 속도 변수
        # current: 로봇의 현재 속도 (가속/감속이 반영되는 중인 값)
        # target: 사용자가 키보드를 눌러 도달하고 싶은 목표 속도
        self.current_v = 0.0        # linear.x
        self.current_w = 0.0        # angular.z
        self.current_y = 0.0        # linear.y (모드 2용)
        self.target_v = 0.0
        self.target_w = 0.0
        self.target_y = 0.0
        self.keys_pressed = set()   # 현재 어떤 키가 눌려 있는지 중복 없이 저장하는 집합(set)입니다.
        self.current_mode = 1       # 1: Ackermann, 2: Crab, 3: Spin
        
        # Tkinter를 이용해 가로 900, 세로 580 크기의 창을 띄웁니다. 크기 조절은 안 되게 고정
        self.root = tk.Tk()
        self.root.title("4축 조향 로봇 Teleop - linear.y 고정")
        self.root.geometry("500x580")
        self.root.resizable(False, False)
        self.create_gui()
        # [수정/안전 버그] 기존에는 창 우측 상단의 OS 기본 'X' 버튼으로 닫으면
        # on_closing()이 호출되지 않고 Tk 창만 그냥 사라졌습니다. 이 경우 정지(Twist())
        # 발행이 되지 않기 때문에, 만약 'w'나 'a' 같은 키를 누른 상태에서 'X'를 눌러
        # 창을 닫으면 로봇은 마지막으로 발행된 속도 명령을 계속 유지한 채 멈추지 않을
        # 수 있습니다. 아래 protocol 등록으로 'X' 버튼도 on_closing()을 타도록 통일합니다.
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # 키보드가 눌렸을 때(on_press)와 떼졌을 때(on_release)를 감지하는 백그라운드 리스너를 시작합니다.
        self.listener = keyboard.Listener(on_press=self._safe_on_press, on_release=self._safe_on_release)
        self.listener.start()
        self.update_period_ms = max(10, int(1000.0 / self.rate))
        self.publish_mode()
        self.get_logger().info("4축 조향 로봇 Teleop 시작 (1: Ackermann, 2: Crab, 3: Spin)")

# =======================================================================================================================================
# =                                                          GUI 창                                                                     =
# =======================================================================================================================================

    def create_gui(self):
        # ttk.Label(...): 화면에 수정할 수 없는 고정된 글자(라벨)를 만듭니다.
        # self.root: 이 글자를 메인 창(self.root)에 넣겠다는 뜻입니다.
        # text="...": 화면에 표시될 실제 글자 내용입니다.
        # font=("Arial", 18, "bold"): 글꼴은 Arial, 크기는 18포인트, 두껍게(bold) 설정합니다.
        # .pack(pady=15): 만들어진 라벨을 화면에 실제로 배치합니다.
        # pady=15: 위아래(Y축)로 15픽셀만큼 여백을 주어 다른 요소들과 너무 붙지 않게 만듭니다.
        ttk.Label(self.root, text="4축 조향 로봇 실시간 제어", font=("Arial", 18, "bold")).pack(pady=15)

        # 선속도(x, y, z)를 보여줄 테두리 달린 상자(프레임)를 만듭니다.
        # 시각적으로 구역을 나눌 수 있도록 테두리가 있는 빈 상자(프레임)를 만듭니다.
        lin_frame = ttk.LabelFrame(self.root, text=" Linear Velocity (m/s) ", padding=15)
        lin_frame.pack(fill="x", padx=30, pady=10)

        # 위에서 만든 lin_frame 상자 안에 들어갈 라벨 3개를 만듭니다
        # self.label_lx: 나중에 키보드를 누를 때마다 이 라벨 안의 숫자를 바꿔야 하므로, 클래스 변수(self.)로 지정해 둡니다.
        # font=("Consolas", 14): 숫자가 계속 변하므로 모든 글자의 폭이 일정한 고정폭 글꼴
        # foreground="red": linear.x 글자색만 빨간색으로 강조합니다.
        self.label_lx = ttk.Label(lin_frame, text="linear.x  :  0.000 m/s", font=("Consolas", 14), foreground="red")
        self.label_ly = ttk.Label(lin_frame, text="linear.y  :  0.000 m/s", font=("Consolas", 14))
        self.label_lz = ttk.Label(lin_frame, text="linear.z  :  0.000 m/s", font=("Consolas", 14))

        # 만든 라벨 3개를 lin_frame 상자 내부에서 수직으로 차례대로 배치합니다.
        self.label_lx.pack(anchor="w", pady=5)
        self.label_ly.pack(anchor="w", pady=5)
        self.label_lz.pack(anchor="w", pady=5)

        # 회전 속도를 담을 두 번째 테두리 상자를 만듭니다.
        ang_frame = ttk.LabelFrame(self.root, text=" Angular Velocity (rad/s) ", padding=15)
        ang_frame.pack(fill="x", padx=30, pady=10)

        self.label_ax = ttk.Label(ang_frame, text="angular.x :  0.000 rad/s", font=("Consolas", 14))
        self.label_ay = ttk.Label(ang_frame, text="angular.y :  0.000 rad/s", font=("Consolas", 14))
        self.label_az = ttk.Label(ang_frame, text="angular.z :  0.000 rad/s", font=("Consolas", 14), foreground="red")

        self.label_ax.pack(anchor="w", pady=8)
        self.label_ay.pack(anchor="w", pady=8)
        self.label_az.pack(anchor="w", pady=8)

        # 상태 표시
        self.status_label = ttk.Label(self.root, text="눌린 키: -   |   동작: 정지", font=("Arial", 12), foreground="blue")
        self.status_label.pack(pady=20)
        self.mode_label = ttk.Label(self.root, text="모드: Ackermann", font=("Arial", 12), foreground="green")
        self.mode_label.pack(pady=5)

        ttk.Button(self.root, text="창 닫기 & 정지", command=self.on_closing).pack(pady=10)
        # [추가] 비상 정지 키 안내
        ttk.Label(self.root, text="[1] Ackermann / [2] Crab / [3] Spin / [Space] 비상정지", font=("Arial", 10), foreground="gray").pack(pady=2)

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

    # 키보드를 누를 때 (on_press)
    def on_press(self, key):
        # [추가/안전 기능] 스페이스바를 누르면 가속/감속 램프(ramp)를 무시하고
        # 즉시 목표 속도와 현재 속도를 모두 0으로 만드는 비상 정지 기능을 추가했습니다.
        # 기존에는 정지하려면 키를 떼고 ang/lin_accel에 따라 서서히 감속될 때까지
        # 기다려야 했는데, 실사용 로봇에서는 즉각적인 정지 수단이 반드시 필요합니다.
        if key == keyboard.Key.space:
            self.keys_pressed.clear()
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_y = 0.0
            self.current_v = 0.0
            self.current_w = 0.0
            self.current_y = 0.0
            return

        # def on_press(self, key):: 키보드가 눌렸을 때 실행되는 함수입니다. 어떤 키가 눌렸는지 정보가 key 변수로 들어옵니다.
        # try:: 특수키(예: Shift, Ctrl)를 누르면 에러가 발생할 수 있으므로, 에러를 방지하기 위해 예외 처리를 시작합니다.
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

    # 키보드에서 손을 뗄 때 (on_release)
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

    # 목표 속도 계산하기 (update_target)
    def update_target(self):
        v = 0.0                         # 임시 선속도(x)를 0.0으로 초기화합니다.
        w = 0.0                         # 임시 각속도(z)를 0.0으로 초기화합니다.
        y = 0.0                         # 임시 측면 속도(y)를 0.0으로 초기화합니다.

        if self.current_mode == 1:
            # w/s: 전진/후진, a/d: 회전
            if 'w' in self.keys_pressed:
                v += self.max_v
            if 's' in self.keys_pressed:
                v -= self.max_v
            if 'a' in self.keys_pressed:
                w += self.max_w
            if 'd' in self.keys_pressed:
                w -= self.max_w
        elif self.current_mode == 2:
            # w/s: 전진/후진, a/d: 좌/우 측면 이동
            if 'w' in self.keys_pressed:
                v += self.max_v
            if 's' in self.keys_pressed:
                v -= self.max_v
            if 'a' in self.keys_pressed:
                y -= self.max_v
            if 'd' in self.keys_pressed:
                y += self.max_v
        else:
            # a/d: 제자리 회전
            if 'a' in self.keys_pressed:
                w += self.max_w
            if 'd' in self.keys_pressed:
                w -= self.max_w

        # 계산된 임시 값들을 클래스의 진짜 목표 속도 변수에 덮어씁니다.
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

    # 실시간 주기적 업데이트 (update)
    # 이 함수는 타이머에 의해 1초에 50번씩 자동으로 계속 실행됩니다.
    def update(self):
        # dt = 1.0 / self.rate: 주기 사이의 시간 간격을 계산합니다.
        # self.rate가 50이면 dt는 1.0 / 50.0 = 0.02초입니다.
        dt = 1.0 / self.rate

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

    # current: 현재 로봇의 실제 속도입니다.
    # target: 사용자가 키보드를 눌러 도달하려는 목표 속도입니다.
    # step: 한 주기(0.02초) 동안 변화할 속도의 변화량입니다. (가속도 $\times$ 시간)
    def _ramp(self, current, target, step):
        if current < target:
            # 현재 속도가 목표 속도보다 작다면 현재 속도에 변화량(step)을 더합니다.
            # 만약 더한 값이 목표 속도를 초과해 버리면 안 되므로, 둘 중 더 작은 값을 선택해 목표 속도에 딱 멈추도록 제한
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

            mode_name = {1: "Ackermann", 2: "Crab", 3: "Spin"}.get(self.current_mode, "Unknown")
            self.mode_label.config(text=f"모드: {mode_name}")

            keys_str = ''.join(sorted(self.keys_pressed)) if self.keys_pressed else "-"
            status = "전진" if twist.linear.x > 0.05 else "후진" if twist.linear.x < -0.05 else "정지"
            if abs(twist.angular.z) > 0.05:
                status += " + 회전"
            if abs(twist.linear.y) > 0.05:
                status += " + 측면"

            self.status_label.config(text=f"눌린 키: {keys_str}   |   동작: {status}")
        except Exception as e:
            self.get_logger().warn(f"GUI 업데이트 중 예외: {e}")

    def publish_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_closing(self):
        # [수정/버그] 여기서 destroy_node()를 호출하면 main()의 finally 블록에서
        # 다시 한 번 destroy_node()가 호출되어 이미 destroy된 노드를 또 종료하려다가
        # 예외가 발생할 수 있었습니다 (이중 종료).
        # -> 정지 명령 발행 + 리스너 정지 + GUI 종료만 담당하고,
        #    실제 rclpy 노드 종료는 main()에서 한 번만 수행하도록 정리했습니다.
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