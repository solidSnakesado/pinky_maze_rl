"""
src/pinky_maze_sim/pinky_maze_sim/sensor_model.py - 핑키 프로 MuJoCo 모델에 실물 센서 (IMU, 초음파, 바닥 적외선) 를 추가

- 핑키 프로 (pinky_mujoco, 014a09f) 의 build_model / Physics 는 수정하지 않고 불러다 사용
- 순서: 핑키 프로가 만든 scene.xml 읽기 -> 센서 위치 (site)/IMU 센서/바닥 테이프 추가
        -> scene_sensors.xml 로 저장 -> MazePhysics 로 물리 엔진 생성
- 센서별 측정 방법 (모두 물리 엔진 값 사용)
    IMU     : MuJoCo 표준 센서 (gyro, accelerometer, framequat) 를 imu_link 에 부착
    초음파   : ultrasonic_link 에서 시야각 안으로 광선 여러 개 -> 가장 가까운 거리
    적외선   : ir_*_link 에서 바닥으로 광선 -> 바닥 테이프 geom 에 맞으면 테이프 값
    라이다   : 핑키 프로 광선 측정값에 거리 잡음/측정 누락 추가
- 잡음/ADC 값은 sim_sensors.yaml 에서 읽음 (실물 측정 후 조정)

사용 (sim_bridge.py 에서):
  cfg = load_sensor_config("sim_sensors.yaml")
  physics = MazePhysics(add_sensors(scene_xml, cfg), None, cfg)
"""

# ==================== 표준 라이브러리 ======================
from pathlib import Path                                                            # 파일 경로
import xml.etree.ElementTree as ET                                                  # MuJoCo 모델 (XML) 편집

# ==================== 외부 라이브러리 ======================
import numpy as np                                                                  # 광선 방향, 잡음 계산
from pinky_mujoco.model import Physics                                              # 핑키 프로 물리 엔진
import yaml                                                                         # 센서 설정 파일

# 바닥 테이프 전용 geom 그룹 (충돌 없음, 적외선 광선만 검사)
# 핑키 프로는 0 (월드), 1 (로봇 충돌), 2 (로봇 외형), 4 (라이다용 숨김 벽) 를 사용
TAPE_GROUP = 3

# =========================================================
# 센서 설정 파일 읽기
# =========================================================
def load_sensor_config(path):
    """sim_sensors.yaml -> dict."""
    return yaml.safe_load(Path(path).read_text())

# =========================================================
# 모델 XML 에서 이름으로 몸체 (body) 찾기
# =========================================================
def _body(root, name):
    """URDF 링크 이름과 같은 MuJoCo body 반환 (없으면 오류)."""
    for body in root.iter("body"):                                                  # 모든 body 순회
        if body.get("name") == name:
            return body
    raise ValueError(f"Body not found in MuJoCo model: {name}")

# =========================================================
# scene.xml -> 센서 추가 -> scene_sensors.xml
# =========================================================
def add_sensors(scene_path, cfg):
    """scene.xml 에 센서 site, IMU 센서, 바닥 테이프를 더해 scene_sensors.xml 로 저장."""
    scene_path  = Path(scene_path)
    tree        = ET.parse(scene_path)                                              # 핑키 프로가 만든 모델
    root        = tree.getroot()

    # 센서 위치 (site): URDF의 센서 링크에 작은 점을 붙임
    ET.SubElement(_body(root, cfg["imu"]["link"]), "site",
                  name="imu_site", size="0.002")                                    # IMU
    ET.SubElement(_body(root, cfg["ultrasonic"]["link"]), "site",
                  name="us_site", size="0.002")                                     # 초음파
    for i, link in enumerate(cfg["ir"]["links"]):                                   # 적외선 3개 (발행 순서)
        ET.SubElement(_body(root, link), "site", name=f"ir_site_{i}", size="0.002")

    # IMU: MuJoCo 표준 센서 3개 (값은 data.sensordata 에 들어옴)
    sensors = root.find("sensor")
    if sensors is None:                                                             # 핑키 프로 모델에는 없음
        sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="imu_site")                # 각속도 (rad/s)
    ET.SubElement(sensors, "accelerometer", name="imu_accel",
                  site="imu_site")                                                  # 가속도 (중력 포함, m/s^2)
    ET.SubElement(sensors, "framequat", name="imu_quat",
                  objtype="site", objname="imu_site")                               # 자세 (w x y z)

    # 바닥 테이프: [x, y, 길이, 폭, yaw] (월드 좌표 m, rad), 두께 1mm, 충돌 없음
    world = root.find("worldbody")
    for i, (x, y, length, width, yaw) in enumerate(cfg["ir"].get("tapes") or []):
        ET.SubElement(world, "geom", name=f"floor_tape_{i}", type="box",
                      pos=f"{x} {y} 0.0005",                                        # 바닥 위 0.5 mm 중심
                      size=f"{length / 2} {width / 2} 0.0005",                      # MuJoCo 크기는 절반 값
                      euler=f"0 0 {yaw}", group=str(TAPE_GROUP),
                      contype="0", conaffinity="0",                                 # 바퀴와 충돌하지 않음
                      rgba="0.1 0.1 0.1 1")                                         # 검정 테이프

    out = scene_path.with_name("scene_sensors.xml")                                 # 같은 폴더 (메시 경로 유지)
    tree.write(out, encoding="unicode")
    return out

# =========================================================
# 핑키 프로 Physics + 추가 센서 측정 + 잡음
# =========================================================
class MazePhysics(Physics):
    """핑키 프로 Physics 를 상속해 IMU/초음파/적외선 측정과 잡음을 더함"""

    def __init__(self, model_path, config_file, cfg):
        super().__init__(model_path, config_file)                                   # 핑키 프로 초기화 그대로
        self.cfg        = cfg
        self.rng        = np.random.default_rng(cfg.get("seed", 0))                 # 잡음 난수 (재현 가능)
        m               = self.model
        self.imu_adr    = {k: int(m.sensor(f"imu_{k}").adr[0])                      # sensordata 안 위치
                           for k in ("gyro", "accel", "quat")}
        self.gyro_bias  = self.rng.normal(0.0, cfg["imu"]["gyro_bias_std"], 3)      # 시작 시 치우침
        self.us_id      = m.site("us_site").id                                      # 초음파 site 번호
        self.ir_ids     = [m.site(f"ir_site_{i}").id
                           for i in range(len(cfg["ir"]["links"]))]
        us              = cfg["ultrasonic"]
        half            = us["field_of_view"] / 2
        angles = np.linspace(-half, half, us["rays"])                               # 시야각 안 광선 각도
        self.us_vectors = np.column_stack(
            (np.cos(angles), np.sin(angles), np.zeros(len(angles))))                # site 기준 방향 (+x 앞)
        self.tape_group = np.zeros(6, dtype=np.uint8)                               # 광선이 볼 geom 그룹
        self.tape_group[TAPE_GROUP] = 1                                             # 테이프만 봄

    # ---------------------------------------------------------
    # 광선 여러 개 -> 거리 (맞은 것이 없으면 inf)
    # ---------------------------------------------------------
    def _rays(self, origin, vectors, group, cutoff):
        """핑키 프로 라이다와 같은 mj_multiRay 사용."""
        n           = len(vectors)
        distances   = np.empty(n, dtype=np.float64)
        geom_ids    = np.empty(n, dtype=np.int32)                                   # 맞은 geom (-1 = 없음)
        self.mj.mj_multiRay(self.model, self.data, origin,
                            np.ascontiguousarray(vectors).ravel(),
                            group, True, -1, geom_ids, distances, None, n, cutoff)
        distances[(geom_ids < 0) | (distances < 0) | (distances > cutoff)] = np.inf
        return distances

    # ---------------------------------------------------------
    # 라이다: 핑키 프로 측정값 + 잡음
    # ---------------------------------------------------------
    def scan(self):
        """핑키 프로 라이다 측정값에 거리 잡음과 측정 누락을 더함 (단위 m)."""
        distances               = super().scan()                                    # 360개, 없음은 inf
        lidar                   = self.cfg["lidar"]
        hit                     = np.isfinite(distances)                            # 맞은 광선만 잡음
        distances[hit]         += self.rng.normal(0.0, lidar["noise_std"], int(hit.sum()))
        too_close               = hit & (distances < self.settings["lidar"]["range_min"])
        distances[too_close]    = np.inf                                            # 최소 거리 아래는 무효
        drop                    = self.rng.random(len(distances)) < lidar["dropout"]
        distances[drop]         = np.inf
        return distances

    # ---------------------------------------------------------
    # IMU: (각속도 rad/s, 가속도 m/s^2, 자세 쿼터니언 w x y z), 몸체 기준
    # ---------------------------------------------------------
    def imu(self):
        """물리 엔진 센서 값 + 치우침 + 잡음."""
        s, c, a = self.data.sensordata, self.cfg["imu"], self.imu_adr
        gyro    = (s[a["gyro"]:a["gyro"] + 3] + self.gyro_bias
                   + self.rng.normal(0.0, c["gyro_noise_std"], 3))                  # 각속도
        accel   = (s[a["accel"]:a["accel"] + 3]
                   + self.rng.normal(0.0, c["accel_noise_std"], 3))                 # 가속도
        quat    = s[a["quat"]:a["quat"] + 4].copy()                                 # 자세 (잡음 없음)
        return gyro, accel, quat

    # ---------------------------------------------------------
    # 초음파: 거리 (m), 실물 노드와 같은 ADC 계산식으로 양자화
    # ---------------------------------------------------------
    def ultrasonic(self):
        """실물 노드 계산식: range = ADC / 4096 - 0.03 (ADC 0~4095)."""
        c           = self.cfg["ultrasonic"]
        rotation    = self.data.site_xmat[self.us_id].reshape(3, 3)                 # site 방향 (월드 기준)
        distances   = self._rays(self.data.site_xpos[self.us_id],
                                 self.us_vectors @ rotation.T,                      # 광선을 월드 방향으로
                                 self.ray_group, c["max_range"])                    # 벽만 봄 (라이다와 같음)
        distance    = float(distances.min())                                        # 가장 가까운 물체
        if np.isfinite(distance):
            distance += self.rng.normal(0.0, c["noise_std"])
        else:
            distance = c["max_range"]                                               # 아무것도 없음
        adc         = int(np.clip(round((distance + 0.03) * 4096), 0, 4095))        # 거리 -> ADC
        return adc / 4096.0 - 0.03                                                  # ADC -> 거리 (실물과 같음)

    # ---------------------------------------------------------
    # 바닥 적외선: ADC 값 목록 (cfg["ir"]["links"] 순서)
    # ---------------------------------------------------------
    def ir(self):
        """센서 아래에 테이프가 있으면 tape_adc, 없으면 floor_adc (+ 잡음)"""
        c       = self.cfg["ir"]
        down    = np.array([[0.0, 0.0, -1.0]])                                      # 바로 아래 방향
        values  = []
        for site in self.ir_ids:
            dist    = self._rays(self.data.site_xpos[site], down,
                              self.tape_group, c["max_distance"])[0]                # 테이프만 검사
            value   = c["tape_adc"] if np.isfinite(dist) else c["floor_adc"]
            value  += self.rng.normal(0.0, c["noise_std"])
            values.append(int(np.clip(round(value), 0, 4095)))                      # 12비트 ADC 범위
        return values
