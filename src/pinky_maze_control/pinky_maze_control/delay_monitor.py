"""
src/pinky_maze_control/pinky_maze_control/delay_monitor.py - 제어 PC 가 받는 센서 토픽의 지연과 주기를 감시

- 역할
    로봇 쪽 센서 토픽 (라이다, 오도메트리, IMU, 초음파, 적외선, 압축 영상) 을 받아
    토픽마다 "수신 주기"와 "지연 (받은 시각 - 메시지 시간 도장)" 을 계산하고
    ROS 표준 진단 토픽 /diagnostics 로 OK / WARN / ERROR 를 발행
- 방법 (diagnostic_updater 표준 구조)
    Updater: 1초마다 모든 진단 결과를 모아 /diagnostics 발행
    FrequencyStatus (표준): 수신 주기, 기대 주기 +-tolerance 밖이면 WARN, 수신 없으면 ERROR
    DelayStatus (이 파일): 지연, max_delay 초과 시 ERROR (표준 TimeStampStatus 와 같은 기준/항목)
    HeaderlessTopicDiagnostic: header 없는 토픽 (ir_sensor/range) 은 주기만 감시
- 표준 TimeStampStatus 를 쓰지 않는 이유
    TimeStampStatus 는 노드 시계가 아닌 별도 시계 (실제 PC 시간) 로 지연을 계산
    시뮬 (use_sim_time: true) 에서 시뮬 시간 도장과 비교하면 항상 ERROR 가 됨
    -> DelayStatus 는 노드 시계 (node.get_clock()) 를 써서 시뮬/실물 모두 맞게 계산
- 실물에서는 라즈베리 파이와 제어 PC 시계를 chrony 로 동기화해야 지연 값이 의미 있음

실행:
  ros2 run pinky_maze_control delay_monitor --ros-args \\
    --params-file install/pinky_maze_bringup/share/pinky_maze_bringup/config/delay_monitor.yaml \\
    -p use_sim_time:=true
결과 보기:
    ros2 topic echo /diagnostics
"""

# ==================== ROS 2 외부 라이브러리 ================
import rclpy                                                                        # ROS 2 파이썬
from rclpy.node                     import Node                                     # 노드 기본 클래스
from rclpy.parameter                import Parameter                                # 파라미터 타입 지정
from rclpy.qos                      import qos_profile_sensor_data                  # 센서용 QoS (best effort)
from rosidl_runtime_py.utilities    import get_message                              # "패키지/msg/타입" -> 메시지 클래스
from diagnostic_msgs.msg            import DiagnosticStatus                         # OK / WARN / ERROR 값
from diagnostic_updater             import (DiagnosticTask, FrequencyStatusParam,   # 진단 작업 기본 클래스, 주기 기준
                                            HeaderlessTopicDiagnostic, Updater)     # 토픽 진단 묶음, 진단 발행기

# =========================================================
# 지연 감시 작업 (표준 TimeStampStatus 와 같은 기준, 노드 시계 사용)
# =========================================================
class DelayStatus(DiagnosticTask):
    """메시지 시간 도장과 노드 시계의 차이 (지연) 를 감시"""

    def __init__(self, node, min_acceptable, max_acceptable, name="Timestamp Status"):
        DiagnosticTask.__init__(self, name)
        self.clock          = node.get_clock()                                      # use_sim_time 을 따르는 노드 시계
        self.min_acceptable = min_acceptable                                        # 이보다 작으면 미래 시간 도장 (s)
        self.max_acceptable = max_acceptable                                        # 이보다 크면 지연 초과 (s)
        self.early_count    = 0                                                     # 미래 시간 도장 누적 횟수
        self.late_count     = 0                                                     # 지연 초과 누적 횟수
        self.zero_count     = 0                                                     # 시간 도장 0 누적 횟수
        self.reset_window()

    def reset_window(self):                                                         # 진단 발행 주기마다 초기화
        self.min_delta = 0.0
        self.max_delta = 0.0
        self.sum_delta = 0.0
        self.num_delta = 0
        self.zero_seen = False

    def tick(self, stamp):
        """메시지 하나를 받을 때마다 호출 (stamp: builtin_interfaces/Time)"""
        t = stamp.sec + stamp.nanosec * 1e-9                                        # 시간 도장 (s)
        if t == 0.0:
            self.zero_seen = True                                                   # 시간 도장을 안 찍은 메시지
            return
        delta = self.clock.now().nanoseconds * 1e-9 - t                             # 지연 = 받은 시각 - 시간 도장
        if self.num_delta == 0 or delta > self.max_delta:
            self.max_delta = delta
        if self.num_delta == 0 or delta < self.min_delta:
            self.min_delta = delta
        self.sum_delta += delta
        self.num_delta += 1

    def run(self, stat):
        """Updater 가 1초마다 호출, 결과를 stat 에 채움"""
        stat.summary(DiagnosticStatus.OK, "Timestamps are reasonable.")
        if self.num_delta == 0:
            stat.summary(DiagnosticStatus.WARN, "No data since last update.")
        else:
            if self.min_delta < self.min_acceptable:
                stat.summary(DiagnosticStatus.ERROR, "Timestamps too far in future seen.")
                self.early_count += 1
            if self.max_delta > self.max_acceptable:
                stat.summary(DiagnosticStatus.ERROR, "Timestamps too far in past seen")
                self.late_count += 1
            if self.zero_seen:
                stat.summary(DiagnosticStatus.ERROR, "Zero timestamp seen.")
                self.zero_count += 1

        mean = self.sum_delta / self.num_delta if self.num_delta else 0.0           # 평균 지연 (s)
        stat.add("Earliest timestamp delay:", "%f" % self.min_delta)                # 아래 항목 이름은 표준과 같음
        stat.add("Latest timestamp delay", "%f" % self.max_delta)
        stat.add("Mean timestamp delay", "%f" % mean)                               # 표준에 없는 항목 (평균)
        stat.add("Earliest acceptable timestamp delay", "%f" % self.min_acceptable)
        stat.add("Latest acceptable timestamp delay", "%f" % self.max_acceptable)
        stat.add("Late diagnostic update count", "%i" % self.late_count)
        stat.add("Early diagnostic update count", "%i" % self.early_count)
        stat.add("Zero seen diagnostic update count", "%i" % self.zero_count)
        self.reset_window()
        return stat

# =========================================================
# header 있는 토픽 진단 (주기 + 지연), 표준 TopicDiagnostic 과 같은 구성
# =========================================================
class StampedTopicDiagnostic(HeaderlessTopicDiagnostic):
    """주기 (표준 FrequencyStatus) + 지연 (DelayStatus) 을 한 진단 항목으로 묶음"""

    def __init__(self, name, diag, freq, node, min_acceptable, max_acceptable):
        HeaderlessTopicDiagnostic.__init__(self, name, diag, freq)                  # 주기 작업 추가 + Updater 등록
        self.delay = DelayStatus(node, min_acceptable, max_acceptable)
        self.addTask(self.delay)                                                    # 지연 작업 추가

    def tick(self, stamp):                                                          # 메시지 수신 시 호출
        self.delay.tick(stamp)
        HeaderlessTopicDiagnostic.tick(self)                                        # 주기 카운터

# =========================================================
# 지연 측정 노드
# =========================================================
class DelayMonitor(Node):
    """설정 파일의 토픽 목록을 구독해 토픽별 진단을 /diagnostics 로 발행"""

    def __init__(self):
        super().__init__("delay_monitor")

        # ---------- 공통 파라미터 (delay_monitor.yaml) ----------
        hw_id           = self.declare_parameter("hardware_id", "pinky_maze").value # 진단 장치 이름
        min_delay       = self.declare_parameter("min_delay", -1.0).value           # 표준 기본값 -1 s
        max_delay       = self.declare_parameter("max_delay", 0.2).value            # smoother 타임아웃과 같은 값
        tolerance       = self.declare_parameter("freq_tolerance", 0.1).value       # 주기 허용 오차 (표준 0.1)
        window          = self.declare_parameter("window_size", 5).value            # 주기 계산 창 (진단 횟수)
        verbose         = self.declare_parameter("verbose", True).value             # OK 아닌 진단을 로그로 출력
        keys            = self.declare_parameter("topic_keys", Parameter.Type.STRING_ARRAY).value

        self.updater            = Updater(self)                                     # 1초마다 /diagnostics 발행
        self.updater.setHardwareID(hw_id)
        self.updater.verbose    = verbose
        self.diags              = []                                                # 진단 객체 보관
        self.subs               = []                                                # 구독 객체 보관

        # ---------- 토픽별 진단 + 구독 ----------
        for key in keys:
            topic   = self.declare_parameter(f"{key}.topic", Parameter.Type.STRING).value
            mtype   = self.declare_parameter(f"{key}.type", Parameter.Type.STRING).value
            rate    = self.declare_parameter(f"{key}.rate", Parameter.Type.DOUBLE).value
            stamped = self.declare_parameter(f"{key}.stamped", True).value          # header 유무
            freq    = FrequencyStatusParam({"min": rate, "max": rate}, tolerance, window)
            if stamped:
                diag = StampedTopicDiagnostic(topic, self.updater, freq, self, min_delay, max_delay)
                cb = lambda msg, d=diag: d.tick(msg.header.stamp)                   # noqa: E731
            else:
                diag = HeaderlessTopicDiagnostic(topic, self.updater, freq)
                cb = lambda msg, d=diag: d.tick()                                   # noqa: E731
            self.diags.append(diag)
            self.subs.append(self.create_subscription(get_message(mtype), topic, cb,
                                                      qos_profile_sensor_data))
            self.get_logger().info(f"monitor {topic} ({mtype}, {rate:g} Hz"
                                   f"{', delay' if stamped else ''})")
        self.get_logger().info(f"delay limit {max_delay:g} s, rate tolerance {tolerance:g}, "
                               f"use_sim_time {self.get_parameter('use_sim_time').value}")

# =========================================================
# 실행 진입점
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = DelayMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:                                                       # Ctrl+C 종료
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()