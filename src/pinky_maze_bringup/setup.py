"""
src/pinky_maze_bringup/setup.py - pinky_maze_bringup 패키지 설치 설정 (colcon build 가 읽음)

- ros2 pkg create 로 만든 기본 내용 + launch, config 폴더 설치
- 실행 파일 (노드) 은 없음, launch 와 설정 파일만 가진 패키지
"""

import os                                                                           # 설치 경로 결합
from glob import glob                                                               # 파일 목록 찾기

from setuptools import find_packages, setup                                         # 파이썬 패키지 설치 도구

package_name = 'pinky_maze_bringup'                                                 # ROS 2 패키지 이름

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 가 패키지를 찾을 수 있게 하는 표시 파일
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch, config 파일 설치 (ros2 launch 로 찾을 수 있게 함)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),                                            # robot_sim.launch.py 등
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),                                                 # velocity_smoother, sim_sensors
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='solidSnakesado',
    maintainer_email='52153542+solidSnakesado@users.noreply.github.com',
    description='Launch and config for Pinky maze',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [                                                        # 실행 파일 없음
        ],
    },
)
