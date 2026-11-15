"""
src/pinky_maze_bringup/launch/robot_sim.launch.py - 로봇 쪽 (시뮬) 실행 파일: 실물 핑키 본체가 하는 역할을 시뮬에서 대신

- 실행하는 노드
    sim_bridge (pinky_maze_sim)         MuJoCo 시뮬 + 라이다/오도메트리/카메라/IMU/초음파/적외선
    robot_state_publisher               핑키 프로 URDF 로 로봇 좌표계 /tf 발행
    camera_republish (image_transport)  원본 영상 -> JPEG 압축 영상 (제어 PC 는 압축 영상 수신)
    velocity_smoother (Nav2)            제어 PC 명령 /cmd_vel_nav -> 속도/가속 한계 -> /cmd_vel,
                                        명령이 0.2초 끊기면 정지 명령
    lifecycle_manager_robot (Nav2)      velocity_smoother 자동 시작
- 핑키 프로 파일은 수정하지 않고, URDF 펼치기 함수 (expand_urdf) 만 불러다 사용
- 설정 파일: config/velocity_smoother.yaml, config/sim_sensors.yaml

실행:
  ros2 launch pinky_maze_bringup robot_sim.launch.py
  월드 지정: world:=/절대/경로/world.yaml,  창 없이: viewer:=false
"""

# ==================== 표준 라이브러리 ======================
import os                                                                           # 경로 결합, 환경 변수
from pathlib import Path                                                            # URDF 저장 폴더

# ==================== ROS 2 launch =========================
from ament_index_python.packages import get_package_share_directory                 # 패키지 설치 경로
from launch import LaunchDescription                                                # 실행 구성
from launch.actions import DeclareLaunchArgument, OpaqueFunction                    # 인자, 실행 시 계산
from launch.substitutions import LaunchConfiguration                                # 인자 값 읽기
from launch_ros.actions import Node                                                 # ROS 노드 실행

# =========================================================
# 시뮬 브리지 + robot_state_publisher (인자 값이 필요해 실행 시점에 만듦)
# =========================================================
def robot_nodes(context):
    from pinky_mujoco.description import expand_urdf                                # 핑키 프로 URDF 펼치기

    # 핑키 프로 URDF (카메라 8도, 화면 -25도 기울기: 핑키 프로 launch 기본값과 같음)
    urdf        = expand_urdf(mappings={"cam_tilt_deg": "8", "screen_tilt_deg": "-25"})
    model_dir   = Path(os.environ.get("PINKY_MODEL_DIR",
                                      str(Path.home() / ".cache/pinky_mujoco")))
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_path   = model_dir / "robot.urdf"
    urdf_path.write_text(urdf)                                                      # 브리지가 읽을 파일

    def arg(name):                                                                  # launch 인자 -> 문자열
        return LaunchConfiguration(name).perform(context)

    use_gpu     = arg("use_gpu").lower() == "true"
    bridge_args = ["--urdf-file", str(urdf_path), "--output-dir", str(model_dir),
                   "--world", arg("world"), "--sensor-config", arg("sensor_config")]
    if arg("viewer").lower() == "true":
        bridge_args.append("--viewer")                                              # MuJoCo 창 표시
    env = {}
    if not use_gpu:
        # 핑키 프로 launch 와 같은 소프트웨어 렌더링 설정 (GPU 없이 동일하게 동작)
        bridge_args.append("--simple-render")
        env = {"MUJOCO_GL": "glfw", "LIBGL_ALWAYS_SOFTWARE": "1",
               "__GLX_VENDOR_LIBRARY_NAME": "mesa"}
    return [
        Node(package="pinky_maze_sim", executable="sim_bridge",
             name="pinky_mujoco",                                                   # 핑키 프로와 같은 노드 이름
             arguments=bridge_args, output="screen", additional_env=env,
             parameters=[{"use_sim_time": True}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
    ]

# =========================================================
# 실행 구성
# =========================================================
def generate_launch_description():
    mujoco_share    = get_package_share_directory("pinky_mujoco")
    bringup_share   = get_package_share_directory("pinky_maze_bringup")
    config          = os.path.join(bringup_share, "config")                         # 설정 파일 폴더

    # 원본 영상 -> 압축 영상 (/camera/image_raw/compressed)
    republish = Node(
        package="image_transport", executable="republish",
        name="camera_republish", output="screen",
        parameters=[{"in_transport": "raw", "out_transport": "compressed",
                     "use_sim_time": True}],
        remappings=[("in", "/camera/image_raw"),
                    ("out/compressed", "/camera/image_raw/compressed")])

    # 제어 PC 명령 (/cmd_vel_nav) -> 한계 적용 -> 바퀴 (/cmd_vel), Nav2 bringup 과 같은 연결
    smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen",
        parameters=[os.path.join(config, "velocity_smoother.yaml")],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")])

    # Nav2 노드는 lifecycle 노드라 관리자가 켜야 동작함
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_robot", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["velocity_smoother"]}])

    return LaunchDescription([
        DeclareLaunchArgument("world",
                              default_value=os.path.join(mujoco_share, "worlds", "room.yaml"),
                              description="Absolute path to a MuJoCo world file"),
        DeclareLaunchArgument("sensor_config",
                              default_value=os.path.join(config, "sim_sensors.yaml"),
                              description="Maze sensor YAML file"),
        DeclareLaunchArgument("viewer", default_value="true",
                              description="Open the MuJoCo viewer window"),
        DeclareLaunchArgument("use_gpu", default_value="false",
                              description="true: hardware OpenGL, false: software"),
        OpaqueFunction(function=robot_nodes),                                       # 브리지 + /tf
        republish, smoother, lifecycle,
    ])
