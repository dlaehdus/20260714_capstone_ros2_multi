# 1. 워크스페이스 폴더로 이동
# cd ~/ros2_ws
# cd CapstoneDesign/ros2_ws/

# 2. 패키지 빌드 (four_wheel_robot 패키지만 선택해서 빌드)
# colcon build --packages-select four_wheel_robot

# 3. 빌드가 정상적으로 끝났다면, 현재 터미널에 환경 등록
# source install/local_setup.bash

# ros2 launch four_wheel_robot four_wheel_robot.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

# =====================================================================================
# [수정] kinematics_node는 ±100 스케일로 wheel_speeds를 보내는데 motor_driver_node는
# 이를 ±5 rpm으로 클램프해버리는 버그가 있었습니다 (두 노드가 서로 다른 값을 하드코딩).
# 이제 두 노드 모두 'max_wheel_rpm' 파라미터를 받도록 바뀌었으므로, 아래 변수 하나로
# 두 노드에 동시에 같은 값을 넘겨 값이 어긋날 가능성 자체를 없앴습니다.
# ⚠️ 실제 주행 전 ZLAC8015D + 감속기 + 바퀴 규격에 맞는 안전한 최대 rpm인지 반드시 확인하세요.
# =====================================================================================
MAX_WHEEL_RPM = 5.0

def generate_launch_description():
    return LaunchDescription([
        
        # Teleop Node (키보드 GUI)
        Node(
            package='four_wheel_robot',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[
                {'max_linear_speed': 1.0},
                {'max_angular_speed': 1.0},
                {'linear_accel': 1.0},
                {'angular_accel': 1.0},
                {'publish_rate': 50.0},
            ]
        ),

        # Kinematics Node
        Node(
            package='four_wheel_robot',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[
                {'wheel_base': 0.685},
                {'track_width': 0.58719},
                # [수정/4:1 기어비 반영] 조향 감속기 장착으로 중앙값과 최대조향 틱 오프셋 변경
                # steering_driver_node.py의 INITIAL_POSITION과 반드시 동일해야 합니다.
                {'dxl_center': 30720},
                {'steer_min_deg': -45.0},
                {'steer_max_deg': 45.0},
                {'max_steer_ticks': 4096.0},        # 45도(steer_max_deg)에서의 틱 오프셋 (4:1 기어비 반영)
                {'max_wheel_rpm': MAX_WHEEL_RPM},   # motor_driver_node와 동일 값 공유 (핵심 수정)
            ]
        ),
    
        # Motor Driver Node
        Node(
            package='four_wheel_robot',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
            parameters=[
                {'max_wheel_rpm': MAX_WHEEL_RPM},   # kinematics_node와 동일 값 공유 (핵심 수정)
            ]
        ),

        # Steering Driver Node
        Node(
            package='four_wheel_robot',
            executable='steering_driver_node',
            name='steering_driver_node',
            output='screen',
        ),
    ])