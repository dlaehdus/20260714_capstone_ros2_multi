#!/usr/bin/env python3

# cmd_vel 토픽으로 발행되는 메시지를 실시간으로 출력
# ros2 topic echo /cmd_vel

# =======================================================================================================================================
# =                                                          라이브러리                                                                   =
# =======================================================================================================================================

import rclpy                            # ros2 파이썬 클라이언트 라이브러리
from rclpy.node import Node             # 독립된 노드 사용
from geometry_msgs.msg import Twist     # 로봇 속도 명령을 담는 표준 메세지 타입
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

        # 속도 변수
        # current: 로봇의 현재 속도 (가속/감속이 반영되는 중인 값)
        # target: 사용자가 키보드를 눌러 도달하고 싶은 목표 속도
        self.current_v = 0.0        # linear.x
        self.current_w = 0.0        # angular.z
        self.target_v = 0.0
        self.target_w = 0.0
        self.keys_pressed = set()   # 현재 어떤 키가 눌려 있는지 중복 없이 저장하는 집합(set)입니다.
        
        # Tkinter를 이용해 가로 900, 세로 580 크기의 창을 띄웁니다. 크기 조절은 안 되게 고정
        self.root = tk.Tk()
        self.root.title("4축 조향 로봇 Teleop - linear.y 고정")
        self.root.geometry("500x580")
        self.root.resizable(False, False)
        self.create_gui()

        # 키보드가 눌렸을 때(on_press)와 떼졌을 때(on_release)를 감지하는 백그라운드 리스너를 시작합니다.
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        # 1초에 50번(1.0 / 50.0 = 0.02초)씩 주기적으로 update 함수를 실행하는 타이머입니다.
        self.timer = self.create_timer(1.0 / self.rate, self.update)
        self.get_logger().info("4축 조향 로봇 Teleop 시작 (linear.y = 0 고정)")

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
        self.status_label.pack(pady=30)

        ttk.Button(self.root, text="창 닫기 & 정지", command=self.on_closing).pack(pady=10)

# =======================================================================================================================================
# =                                                          키 입력                                                                     =
# =======================================================================================================================================

    # 키보드를 누를 때 (on_press)
    def on_press(self, key):
        # def on_press(self, key):: 키보드가 눌렸을 때 실행되는 함수입니다. 어떤 키가 눌렸는지 정보가 key 변수로 들어옵니다.
        # try:: 특수키(예: Shift, Ctrl)를 누르면 에러가 발생할 수 있으므로, 에러를 방지하기 위해 예외 처리를 시작합니다.
        try:
            # k = key.char.lower(): 사용자가 누른 알파벳 문자를 추출하고(char), 대문자로 입력되더라도 모두 소문자(lower())로 통일하여 변수 k에 담습니다.
            k = key.char.lower()
            # if k in ['w', 's', 'a', 'd']:: 누른 키가 우리가 조종에 쓸 w, s, a, d 중 하나인지 확인합니다.
            if k in ['w', 's', 'a', 'd']:
                # self.keys_pressed.add(k): 해당 키를 집합(set)에 추가합니다. (예: w를 누르면 집합은 {'w'}가 됨)
                self.keys_pressed.add(k)
                # self.update_target(): 키가 눌렸으니 목표 속도를 새로 계산하라고 명령합니다.
                self.update_target()
        except:
            pass

    # 키보드에서 손을 뗄 때 (on_release)
    def on_release(self, key):
        try:
            k = key.char.lower()
            # self.keys_pressed.discard(k): 손을 뗀 키(k)를 집합에서 안전하게 제거합니다.
            # discard()는 지우려는 키가 집합에 없더라도 에러를 내지 않는 안전한 명령어입니다.
            self.keys_pressed.discard(k)
            # self.update_target(): 키에서 손을 뗐으니 목표 속도를 다시 0으로 줄이거나 바꾸기 위해 계산을 요청합니다.
            self.update_target()
        except:
            pass

# =======================================================================================================================================
# =                                                          입력값 발행                                                                  =
# =======================================================================================================================================

    # 목표 속도 계산하기 (update_target)
    def update_target(self):
        v = 0.0                         # 임시 선속도(x)를 0.0으로 초기화합니다.
        w = 0.0                         # 임시 각속도(z)를 0.0으로 초기화합니다.
        # w가 있으면: 전진하므로 최대 속도(max_v)를 더합니다.
        # s가 있으면: 후진하므로 최대 속도(max_v)만큼 뺍니다.
        # a가 있으면: 좌회전하므로 최대 회전각(max_w)을 더합니다.
        # d가 있으면: 우회전하므로 최대 회전각(max_w)만큼 뺍니다.
        if 'w' in self.keys_pressed:
            v += self.max_v
        if 's' in self.keys_pressed:
            v -= self.max_v
        if 'a' in self.keys_pressed:
            w += self.max_w
        if 'd' in self.keys_pressed:
            w -= self.max_w
        # 계산된 임시 값들을 클래스의 진짜 목표 속도 변수(self.target_v, self.target_w)에 덮어씁니다.
        self.target_v = v
        self.target_w = w


    # 실시간 주기적 업데이트 (update)
    # 이 함수는 타이머에 의해 1초에 50번씩 자동으로 계속 실행됩니다.
    def update(self):
        # dt = 1.0 / self.rate: 주기 사이의 시간 간격을 계산합니다.
        # self.rate가 50이면 dt는 1.0 / 50.0 = 0.02초입니다.
        dt = 1.0 / self.rate

        # Ramp (부드러운 가속/감속) _ramp(...): 현재 속도에서 목표 속도로 한 걸음씩 다가갑니다.
        # self.lin_accel * dt: 가속도에 0.02초를 곱해 한 걸음의 크기를 정합니다.
        # 예를 들어, 현재 0.0이고 목표가 1.0이라면, 한 번에 1.0이 되는 게 아니라 가속도 폭에 맞춰 0.04 -> 0.08 -> 0.12 ... 이런 식으로 부드럽게 증가
        self.current_v = self._ramp(self.current_v, self.target_v, self.lin_accel * dt)
        self.current_w = self._ramp(self.current_w, self.target_w, self.ang_accel * dt)

        # Twist 메시지 생성 및 발행
        # twist = Twist(): ROS 2로 발행할 새로운 Twist 메시지 상자를 하나 만듭니다
        twist = Twist()
        # twist.linear.x = ...: 부드럽게 가속이 계산된 current_v와 current_w를 각각 알맞은 축에 집어넣습니다.
        twist.linear.x = self.current_v
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_w
        # self.publisher.publish(twist): 이 메시지를 cmd_vel 토픽을 통해 로봇에게 실제로 쏩니다.
        self.publisher.publish(twist)
        # GUI 업데이트
        self.root.after(0, self.update_gui, twist)

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

    # GUI 화면 새로고침 (update_gui)
    def update_gui(self, twist):
        # 화면에 떠 있는 라벨의 글자를 변경하는 Tkinter 명령어입니다.
        self.label_lx.config(text=f"linear.x  : {twist.linear.x:7.3f} m/s")
        self.label_ly.config(text=f"linear.y  : {twist.linear.y:7.3f} m/s")
        self.label_lz.config(text=f"linear.z  : {twist.linear.z:7.3f} m/s")
        self.label_az.config(text=f"angular.z : {twist.angular.z:7.3f} rad/s")

        keys_str = ''.join(sorted(self.keys_pressed)) if self.keys_pressed else "-"
        status = "전진" if twist.linear.x > 0.05 else "후진" if twist.linear.x < -0.05 else "정지"
        if abs(twist.angular.z) > 0.05:
            status += " + 회전"

        self.status_label.config(text=f"눌린 키: {keys_str}   |   동작: {status}")

    def on_closing(self):
        stop = Twist()
        self.publisher.publish(stop)
        self.listener.stop()
        self.root.quit()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelSteeringTeleop()

    def ros_spin():
        rclpy.spin(node)

    thread = threading.Thread(target=ros_spin, daemon=True)
    thread.start()

    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()