"""로봇 쪽(시뮬) 실행 파일.

핑키 프로의 MuJoCo launch(브리지 + robot_state_publisher)를 수정 없이 포함하고,
ROS 표준 노드를 더한다.
  - image_transport republish: 원본 영상 -> JPEG 압축 영상
  - nav2 velocity_smoother: /cmd_vel_nav -> 속도·가속 한계 -> /cmd_vel,
    명령이 끊기면 정지 명령
  - nav2 lifecycle_manager: velocity_smoother 자동 시작
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_share = get_package_share_directory('pinky_mujoco')
    bringup_share = get_package_share_directory('pinky_maze_bringup')
    default_world = os.path.join(mujoco_share, 'worlds', 'room.yaml')
    smoother_yaml = os.path.join(bringup_share, 'config', 'velocity_smoother.yaml')

    world = LaunchConfiguration('world')
    viewer = LaunchConfiguration('viewer')
    use_gpu = LaunchConfiguration('use_gpu')

    # 핑키 프로 시뮬. world를 절대 경로로 넘겨 Gazebo 패키지 없이 동작하게 함
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mujoco_share, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'world': world, 'viewer': viewer, 'use_gpu': use_gpu}.items())

    # 원본 영상 -> 압축 영상 (/camera/image_raw/compressed)
    republish = Node(
        package='image_transport', executable='republish',
        name='camera_republish', output='screen',
        parameters=[{'in_transport': 'raw', 'out_transport': 'compressed',
                     'use_sim_time': True}],
        remappings=[('in', '/camera/image_raw'),
                    ('out/compressed', '/camera/image_raw/compressed')])

    # 제어 PC 명령(/cmd_vel_nav) -> 한계 적용 -> 바퀴(/cmd_vel). Nav2 bringup과 같은 연결
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[smoother_yaml],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')])

    # Nav2 노드는 lifecycle 노드라 관리자가 켜야 동작함
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_robot', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['velocity_smoother']}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to a MuJoCo world file'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='Open the MuJoCo viewer window'),
        DeclareLaunchArgument('use_gpu', default_value='false',
                              description='true: hardware OpenGL, false: software'),
        sim, republish, smoother, lifecycle,
    ])
