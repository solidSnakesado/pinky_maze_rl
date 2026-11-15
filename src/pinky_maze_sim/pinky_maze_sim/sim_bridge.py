"""
src/pinky_maze_sim/pinky_maze_sim/sim_bridge.py - 핑키 미로 시뮬 브리지 (로봇 쪽): 핑키 프로 브리지 + 실물 센서 토픽

- 핑키 프로 (pinky_mujoco, 014a09f) 의 Bridge 를 상속, 핑키 프로 파일은 수정하지 않음
- 핑키 프로 브리지가 내는 토픽은 그대로 (/scan, /odom, /tf, /clock, /ground_truth, 카메라)
- 추가 토픽 (이름/형식은 실물 노드 pinky_imu_bno055, pinky_sensor_adc 와 같음)
    imu_raw             sensor_msgs/Imu            100Hz, 각속도는 실물처럼 deg/s
    us_sensor/range     sensor_msgs/Range           30Hz, 초음파 거리 (m)
    ir_sensor/range     std_msgs/UInt16MultiArray   30Hz, 바닥 적외선 원시 ADC 값 3개
- 라이다 /scan 에는 sensor_model.MazePhysics 가 잡음을 더함
- 시각은 시뮬 시간 (/clock) 기준

실행 (보통은 launch 에서 실행):
  ros2 launch pinky_maze_bringup robot_sim.launch.py
  직접 실행: ros2 run pinky_maze_sim sim_bridge --sensor-config sim_sensors.yaml --viewer
"""

# ==================== 표준 라이브러리 ======================
import argparse                                                                         # 명령행 인자
from contextlib                   import nullcontext                                    # 창 없을 때 빈 lock
import math                                                                             # rad -> deg
import os                                                                               # 환경 변수
from pathlib                      import Path                                           # 파일 경로
import time                                                                             # 실시간 루프 시계

# ==================== ROS 2 / 외부 라이브러리 ==============
from ament_index_python.packages  import get_package_share_directory                    # 패키지 설치 경로
from pinky_mujoco.bridge          import Bridge, stamp                                  # 핑키 프로 브리지
from pinky_mujoco.model           import build_model                                    # 핑키 프로 모델 생성
import rclpy                                                                            # ROS 2 파이썬
from sensor_msgs.msg              import Imu, Range                                     # IMU, 초음파 메시지
from std_msgs.msg                 import UInt16MultiArray                               # 적외선 메시지

# ==================== 이 패키지 (pinky_maze_sim) ==========
from .sensor_model                import add_sensors, load_sensor_config, MazePhysics   # 센서 추가/측정

COVARIANCE = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]                           # 실물 IMU 노드와 같은 값

# =========================================================
# 핑키 프로 Bridge + 실물 센서 토픽
# =========================================================
class MazeBridge(Bridge):
    """핑키 프로 Bridge 를 상속해 imu_raw, us_sensor/range, ir_sensor/range 를 더함"""

    def __init__(self, physics):
        super().__init__(physics)                                                         # 핑키 프로 토픽 생성
        cfg             = physics.cfg
        self.imu_pub    = self.create_publisher(Imu, "imu_raw", 10)                     # 실물과 같은 이름
        self.us_pub     = self.create_publisher(Range, "us_sensor/range", 10)
        self.ir_pub     = self.create_publisher(UInt16MultiArray, "ir_sensor/range", 10)
        self.imu_period = 1.0 / cfg["imu"]["rate"]                                      # IMU 발행 간격 (시뮬 초)
        self.adc_period = 1.0 / cfg["adc_rate"]                                         # 초음파/적외선 간격
        self.next_imu   = 0.0                                                           # 다음 발행 시각
        self.next_adc   = 0.0
        self.get_logger().info(
            f"Maze sensors: imu_raw {cfg['imu']['rate']:g} Hz, us_sensor/range and "
            f"ir_sensor/range {cfg['adc_rate']:g} Hz, lidar noise {cfg['lidar']['noise_std']} m")

    # ---------------------------------------------------------
    # 한 주기 (bridge_hz, 100 Hz) 진행
    # ---------------------------------------------------------
    def advance(self):
        """핑키 프로 한 주기 진행 후, 시뮬 시간에 맞춰 추가 센서 발행."""
        super().advance()                                                               # 물리/핑키 프로 토픽
        t = self.physics.data.time                                                      # 시뮬 시간 (초)
        if t + 1e-9 >= self.next_imu:                                                   # 1e-9: 실수 오차 여유
            self.next_imu = max(self.next_imu + self.imu_period, t)                     # 밀려도 몰아서 안 냄
            self.publish_imu(stamp(t))
        if t + 1e-9 >= self.next_adc:
            self.next_adc = max(self.next_adc + self.adc_period, t)
            self.publish_adc(stamp(t))

    # ---------------------------------------------------------
    # IMU -> imu_raw
    # ---------------------------------------------------------
    def publish_imu(self, now):
        """실물 pinky_imu_bno055 노드와 같은 형식"""
        gyro, accel, quat                   = self.physics.imu()
        msg                                 = Imu()
        msg.header.stamp                    = now
        msg.header.frame_id                 = self.physics.cfg["imu"]["link"]           # imu_link
        o                                   = msg.orientation
        o.w, o.x, o.y, o.z                  = map(float, quat)                          # MuJoCo 순서 w x y z
        w                                   = msg.angular_velocity
        w.x, w.y, w.z                       = (math.degrees(float(v)) for v in gyro)    # 실물 노드는 deg/s
        acc                                 = msg.linear_acceleration
        acc.x, acc.y, acc.z                 = (float(v) for v in accel)                 # m/s^2 (중력 포함)
        msg.orientation_covariance          = COVARIANCE
        msg.angular_velocity_covariance     = COVARIANCE
        msg.linear_acceleration_covariance  = COVARIANCE
        self.imu_pub.publish(msg)

    # ---------------------------------------------------------
    # 초음파 -> us_sensor/range, 적외선 -> ir_sensor/range
    # ---------------------------------------------------------
    def publish_adc(self, now):
        """실물 pinky_sensor_adc 노드와 같은 형식 (같은 주기로 함께 발행)"""
        us                  = self.physics.cfg["ultrasonic"]
        msg                 = Range()
        msg.header.stamp    = now
        msg.header.frame_id = us["link"]                                                # ultrasonic_link
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = float(us["field_of_view"])                                # 0.26 rad
        msg.min_range       = float(us["min_range"])
        msg.max_range       = float(us["max_range"])
        msg.range           = float(self.physics.ultrasonic())                          # ADC 양자화된 거리
        self.us_pub.publish(msg)
        self.ir_pub.publish(UInt16MultiArray(data=self.physics.ir()))                   # ADC 값 3개

# =========================================================
# 명령행 인자 (핑키 프로 브리지와 같은 인자 + --sensor-config)
# =========================================================
def parse_args(args):
    p = argparse.ArgumentParser(description="Pinky maze sim bridge")
    p.add_argument("--sensor-config", required=True, help="Maze sensor YAML file")
    p.add_argument("--viewer", action="store_true", help="Open the MuJoCo viewer window")
    p.add_argument("--no-camera", action="store_true", help="Do not render/publish the camera")
    p.add_argument("--simple-render", action="store_true",
                   help="No shadows/reflections/MSAA (software OpenGL)")
    p.add_argument("--urdf-file", help="Expanded URDF file")
    p.add_argument("--physics-config", help="MuJoCo-only YAML override file")
    p.add_argument("--world", help="World file (.yaml / .world / .sdf)")
    p.add_argument("--output-dir", default=os.environ.get(
        "PINKY_MODEL_DIR", str(Path.home() / ".cache/pinky_mujoco")))                   # 모델 파일 저장 폴더
    return p.parse_known_args(args)                                                     # 나머지는 ROS 인자

# =========================================================
# 모델 생성 -> 노드 실행 -> 실시간 루프
# =========================================================
def main(args=None):
    options, ros_args = parse_args(args)

    # 핑키 프로 모델 생성 -> 센서 추가 -> 물리 엔진 생성
    cfg     = load_sensor_config(options.sensor_config)
    share   = get_package_share_directory("pinky_mujoco")
    urdf    = Path(options.urdf_file).read_text() if options.urdf_file else None
    scene   = build_model(share, options.output_dir, urdf, options.physics_config, options.world,
                          simple_render=options.simple_render)                        # scene.xml
    physics = MazePhysics(add_sensors(scene, cfg), options.physics_config, cfg)
    camera  = physics.settings.get("camera")
    if options.simple_render and camera:
        camera["simple_render"] = True                                                # 카메라도 단순 렌더
    if options.no_camera and camera:
        camera["enabled"] = False

    rclpy.init(args=ros_args)
    node    = MazeBridge(physics)
    viewer  = None
    try:
        if options.viewer:                                                            # MuJoCo 창 (선택)
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(physics.model, physics.data)
            viewer.cam.lookat[:] = [0, 0, 0]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 5.5, 125, -55
            viewer.opt.geomgroup[1] = 0                                               # 충돌 형상 숨김
        # 핑키 프로 브리지와 같은 실시간 루프 (bridge_hz 마다 한 주기)
        deadline = time.monotonic()
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            rclpy.spin_once(node, timeout_sec=0.0)                                    # 명령 수신
            with viewer.lock() if viewer else nullcontext():
                node.advance()                                                        # 물리 + 토픽 발행
            if viewer and node.tick % 3 == 0:
                viewer.sync()                                                         # 창은 3주기마다 갱신
            deadline += physics.bridge_period
            delay     = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)                                                     # 실시간 맞추기
            elif delay < -0.25:
                deadline = time.monotonic()                                           # 많이 밀리면 재기준
    except KeyboardInterrupt:
        pass
    finally:
        try:                                                                          # 종료 중 Ctrl + C 무시
            if viewer:
                viewer.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
