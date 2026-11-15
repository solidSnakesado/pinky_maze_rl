"""
src/pinky_maze_sim/setup.py - pinky_maze_sim 패키지 설치 설정 (colcon build 가 읽음)

- ros2 pkg create 로 만든 기본 내용 + 실행 파일 sim_bridge 등록
- 실행 파일 이름: ros2 run pinky_maze_sim sim_bridge
"""

from setuptools import find_packages, setup                     # 파이썬 패키지 설치 도구

package_name = 'pinky_maze_sim'                                 # ROS 2 패키지 이름

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),                   # pinky_maze_sim/ 폴더 설ㅊ;
    data_files=[
        # ROS 2 가 패키지를 찾을 수 있게 하는 표시 파일
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='solidSnakesado',
    maintainer_email='52153542+solidSnakesado@users.noreply.github.com',
    description='Robot-side sim nodes for Pinky maze',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # 실행 파일 등록: "이름 = 모듈 경로:함수"
    entry_points={
        'console_scripts': [
            "sim_bridge = pinky_maze_sim.sim_bridge:main",      # 시뮬 브리지 노드
        ],
    },
)
