import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/limdoyeon/CapstoneDesign/ros2_ws/install/four_wheel_robot'
