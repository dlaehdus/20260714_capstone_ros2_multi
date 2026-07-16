# cd ~/ros2_ws

# colcon build --packages-select four_wheel_robot --symlink-install

# source install/setup.bash

# ros2 launch four_wheel_robot four_wheel_robot.launch.py

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'four_wheel_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch 파일 설치 (가장 중요!)
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='limdoyeon',
    maintainer_email='your_email@example.com',
    description='4 Wheel Independent Steering Robot Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = four_wheel_robot.teleop_node:main',
            'kinematics_node = four_wheel_robot.kinematics_node:main',
            'motor_driver_node = four_wheel_robot.motor_driver_node:main',
            'steering_driver_node = four_wheel_robot.steering_driver_node:main',
        ],
    },
)