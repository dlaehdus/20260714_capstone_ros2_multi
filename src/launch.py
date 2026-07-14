# 1. 워크스페이스 폴더로 이동
# cd ~/ros2_ws

# 2. 패키지 빌드 (four_wheel_robot 패키지만 선택해서 빌드)
# colcon build --packages-select four_wheel_robot

# 3. 빌드가 정상적으로 끝났다면, 현재 터미널에 환경 등록
# source install/local_setup.bash

# ros2 launch four_wheel_robot four_wheel_robot.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

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
                {'max_angular_speed': 1.0}
            ]
        ),

        # Kinematics Node
        Node(
            package='four_wheel_robot',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
        ),

        # Motor Driver Node
        Node(
            package='four_wheel_robot',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
        ),

        # Steering Driver Node
        # Node(
        #     package='four_wheel_robot',
        #     executable='steering_driver_node',
        #     name='steering_driver_node',
        #     output='screen',
        # ),
    ])