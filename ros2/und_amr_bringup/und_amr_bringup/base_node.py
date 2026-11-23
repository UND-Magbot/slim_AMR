#!/usr/bin/env python3
"""
und_amr_base — STM32 구동부 ROS2 브리지

    /cmd_vel  (geometry_msgs/Twist)  →  차동구동 역기구학 → 바퀴 rpm
                                     →  시리얼 "s 1 <rpm>" / "s 2 <rpm>"
    STM32 피드백("f" 응답 파싱)       →  /odom, /joint_states, TF(odom→base_link)

설계 메모
    오도메트리를 펌웨어가 아니라 여기서 계산한다. 펌웨어의 기존 계산은
    삼륜(애커만) 기구학이라 2륜 차동구동에 맞지 않고, 윤거·바퀴직경이
    바뀔 때마다 펌웨어를 다시 구워야 한다. 파라미터를 yaml 로 빼두면
    재빌드 없이 튜닝할 수 있고 EKF 융합에도 유리하다.

    펌웨어의 축 부호는 이미 "+ = 로봇 전진" 으로 통일되어 있으므로
    (l2db4830.c 의 s_dir), 여기서는 부호를 다시 뒤집지 않는다.
"""

import collections
import math
import os
import re
import select
import termios
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import json

from sensor_msgs.msg import BatteryState, Imu, JointState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 펌웨어 L2DB_PrintFeedback() 출력 형식
# 배터리(타보스 LV-25V25AH, CAN2) — 펌웨어가 1초마다 한 줄 낸다(2026-09-29).
# 원시 단위: v,rc 는 x100, i 는 x100 (+충전/-방전), ae,t 는 x10, age 는 ms.
BAT_RE = re.compile(
    r"\[BAT\] v=(?P<v>\d+) i=(?P<i>-?\d+) st=(?P<st>[0-9A-Fa-f]+) "
    r"soc=(?P<soc>\d+) soh=(?P<soh>\d+) rc=(?P<rc>\d+) ae=(?P<ae>\d+) "
    r"t=(?P<t>-?\d+) ttf=(?P<ttf>\d+) tte=(?P<tte>\d+) cyc=(?P<cyc>\d+) "
    r"age=(?P<age>\d+)")
# 상태 비트 (타보스 통신규약 6.2)
BAT_FLAGS = ["과전압", "저전압", "충전 시 과전류", "사용 시 과전류", "고온", "저온",
             "BMU 오류"]
BAT_STALE_MS = 3000        # 이보다 오래된 답이면 배터리와 통신이 끊긴 것
BAT_DESIGN_AH = 25.0

FEEDBACK_RE = re.compile(
    # node 번호는 여러 자리일 수 있다 — 2026-10-26 드라이브가 16/17 이 되자
    #   'node\d' 가 안 맞아 피드백(바퀴 위치 · 속도)을 통째로 못 읽었다.
    r"axis(?P<axis>\d)\s+node\d+\s+(?P<online>\S+)\s+(?P<enabled>\S+)\s+"
    r"cmd=\s*(?P<cmd>-?\d+)\s*rpm\s+act=\s*(?P<act>-?\d+)\s*rpm\s+"
    r"pos=\s*(?P<pos>-?\d+)\s+status=0x(?P<status>[0-9A-Fa-f]+)\s+"
    r"err=0x(?P<err>[0-9A-Fa-f]+)\s+(?P<volt>-?\d+)V"
    # 2026-09-29 펌웨어부터 모터 전류(Iq). 옛 펌웨어에는 없다.
    r"(?:\s+iq=(?P<iq>[-+]?\d+(?:\.\d+)?)A)?"
)


class SerialPort:
    """termios 기반 최소 시리얼. pyserial 의존을 피한다."""

    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        speed = getattr(termios, "B%d" % baud)
        cc = list(termios.tcgetattr(self.fd)[6])
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(
            self.fd, termios.TCSANOW,
            [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, speed, speed, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._lock = threading.Lock()
        self.checksum = False
        self.crc = False

    def read(self, timeout=0.1):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except OSError:
            return b""

    # 줄 뒤에 붙이는 빈 줄 수. 펌웨어는 빈 줄을 무시한다.
    #
    # ⚠ STM32 쪽 UART 수신 인터럽트가 10ms 타이머보다 우선순위가 낮아,
    #   타이머 처리가 길어지면 몇 바이트가 하드웨어에서 사라진다(overrun,
    #   펌웨어가 조용히 지운다). 그것이 줄바꿈과 다음 줄 머리를 삼키면
    #   "s 1 -13" + "s 2 -14" 가 "s 1 -1314" 가 되어 한쪽 바퀴에 상한
    #   400 rpm 이 걸렸다(2026-09-28, 드라이브 피드백 400/233/177/167 등,
    #   그 순간 차체가 지령 없이 7도 돌았다). 빈 줄을 채워 두면 줄이
    #   합쳐지려면 한 번에 12바이트 넘게 사라져야 한다. 실측 유실은 6~7바이트.
    #   근본 해결은 펌웨어(인터럽트 우선순위/FIFO)다.
    PAD = 8
    # 줄 끝에 " #<바이트 합 mod 256>" 을 붙인다. 펌웨어가 맞지 않는 줄을 버린다
    # (2026-09-28 펌웨어부터). 합쳐진 줄은 뒷줄의 확인값을 달고 오므로 걸린다.
    # 옛 펌웨어는 '#' 뒤를 무시하므로 켜 두어도 동작은 하지만 보호는 없다.
    checksum = False

    @staticmethod
    def with_checksum(line):
        return "%s #%d" % (line, sum(line.encode()) & 0xFF)

    @staticmethod
    def crc16(data):
        """CRC-16/CCITT-FALSE. 펌웨어 L2DB_Crc16 과 같은 식."""
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc

    def with_crc(self, line):
        return "%s %%%04X" % (line, self.crc16(line.encode()))

    def write_line(self, line):
        if self.crc:
            line = self.with_crc(line)
        elif self.checksum:
            line = self.with_checksum(line)
        with self._lock:
            os.write(self.fd, (line + "\r\n" + "\n" * self.PAD).encode())

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class BaseNode(Node):

    def __init__(self):
        super().__init__("base_node")

        p = self.declare_parameters("", [
            ("serial_port", "/dev/ttyUSB0"),
            ("baudrate", 115200),
            # 지령 줄 확인값. 2026-09-28 펌웨어를 구운 뒤에 켠다.
            ("line_checksum", False),
            # 2026-09-30 펌웨어(%%CRC16, w 명령)를 구운 뒤에 켠다. 옛 펌웨어는
            # 둘 다 모른다 — 먼저 켜면 모든 주행 지령이 거부된다.
            ("line_crc", False),       # 확인값을 1바이트 합 대신 CRC-16 으로
            ("pair_cmd", False),       # 두 바퀴를 'w <축1> <축2>' 한 줄로
            ("wheel_separation", 0.40),
            ("wheel_diameter", 0.16),
            ("wheel_diameter_ratio", 1.0),
            ("rpm_dither", True),
            # 소수 rpm 지령 (펌웨어 'v' 명령, 0x60FF). 켜면 디더링을 안 쓴다.
            ("fine_rpm", False),
            ("gear_ratio", 1.0),
            ("counts_per_wheel_rev", 4096),
            ("max_wheel_rpm", 200),
            # 가속 제한 [m/s^2, rad/s^2]. 0 이면 제한 없음(예전 동작).
            # 0.35 면 0.2 m/s 까지 0.57초에 오른다 — 사람 눈에 '스르르' 다.
            ("max_accel_mps2", 0.35),
            ("max_accel_rps2", 1.20),
            ("max_decel_mps2", 0.90),
            ("max_decel_rps2", 3.00),
            # 전진하면서 도는 동안의 최소 회전 반경 [m]. 0 이면 제한 없음.
            ("min_turn_radius_m", 1.5),
            # 자이로 받아치기 이득. 0 이면 끔. (옛 방식, 아래 yaw_hold_* 로 대체)
            ("rate_damping_kd", 0.0),
            # 주행 중 차체 회전 잡기 (2026-10-02). 0 이면 끔. 실행 중 바꿀 수 있다.
            ("odom_yaw_rate_var", 0.10),   # /odom 회전속도 분산 (EKF 가중)
            ("yaw_hold_kp", 0.0),
            ("yaw_hold_tau_s", 0.15),      # 지령 -> 바퀴 회전 반응 시간
            ("yaw_hold_max_rps", 0.15),    # 잡기 보정 상한 [rad/s]
            ("max_linear_mps", 1.0),
            # 리프트를 올린 채(렉을 든 채) 달릴 때의 상한 (2026-10-07).
            # 처음 값이다 — 실제 렉을 들고 재서 맞춘다.
            ("lift_max_linear_mps", 0.20),
            ("lift_accel_mps2", 0.05),
            ("max_angular_rps", 1.5),
            ("cmd_timeout_s", 0.5),
            # 어느 드라이브 축이 어느 쪽 바퀴인지, 그리고 그 축의 양수 지령이
            # 로봇을 앞으로 보내는지. 베이스를 다시 만들면 여기가 바뀐다.
            #   2026-09-10 신형 SLIM: 축1=우, 축2=좌, 양쪽 다 극성 반대.
            #   손으로 앞으로 밀었을 때 엔코더가 음수로 나오는 것으로 확인.
            ("left_axis", 1),
            ("right_axis", 2),
            ("left_sign", 1.0),
            ("right_sign", 1.0),
            # 두 축 피드백을 '같은 주기' 로 볼 시간차 한계 [s].
            # 두 줄이 잇달아 오므로 정상이면 10ms 안쪽이다.
            ("feedback_pair_tol_s", 0.020),
            ("poll_hz", 20.0),
            ("publish_odom_tf", True),
            # 켤 때 드라이브를 자동으로 사용 상태로 만든다 (2026-11-19). 아침에
            # 로봇을 켠 뒤 수동 enable 을 잊어 첫 왕복이 시간 초과로 실패했다.
            ("auto_enable", True),
            ("auto_enable_window_s", 60.0),
            ("odom_frame", "odom"),
            ("base_frame", "base_link"),
        ])
        g = self.get_parameter
        self.sep = g("wheel_separation").value
        # 좌우 바퀴 유효직경이 미세하게 다르다. wheel_diameter 는 두 바퀴의
        # 평균, wheel_diameter_ratio 는 (좌 직경 / 우 직경) 이다.
        #   D_l = D * 2k/(1+k),  D_r = D * 2/(1+k)     (평균이 D 로 보존된다)
        # 비율이 1.0 이면 기존 동작과 완전히 같다.
        _d = g("wheel_diameter").value
        _k = g("wheel_diameter_ratio").value
        self.radius = _d / 2.0                      # 평균 반경 (표시/참고용)
        # 반경은 '축'이 아니라 '쪽'에 붙는다. 축이 바뀌어도 좌우 직경비는
        # 물리적인 좌우를 따라가야 한다.
        self.radius_side = {
            "l": _d * (2.0 * _k / (1.0 + _k)) / 2.0,
            "r": _d * (2.0 / (1.0 + _k)) / 2.0,
        }
        self.side_axis = {"l": int(g("left_axis").value),
                          "r": int(g("right_axis").value)}
        self.side_sign = {"l": float(g("left_sign").value),
                          "r": float(g("right_sign").value)}
        if set(self.side_axis.values()) != {1, 2}:
            raise ValueError("left_axis / right_axis 는 1 과 2 여야 한다")
        self.gear = g("gear_ratio").value
        self.cpr = float(g("counts_per_wheel_rev").value)
        self.max_rpm = g("max_wheel_rpm").value
        self.acc_v = float(g("max_accel_mps2").value) or 1e9
        self.acc_w = float(g("max_accel_rps2").value) or 1e9
        self.dec_v = float(g("max_decel_mps2").value) or 1e9
        self.dec_w = float(g("max_decel_rps2").value) or 1e9
        self.min_radius = float(g("min_turn_radius_m").value)
        self.rate_kd = float(g("rate_damping_kd").value)
        self.odom_w_var = float(g("odom_yaw_rate_var").value)
        self.yaw_kp = float(g("yaw_hold_kp").value)
        self.yaw_tau = float(g("yaw_hold_tau_s").value)
        self.yaw_max = float(g("yaw_hold_max_rps").value)
        self._w_ref = 0.0
        self._hold = None               # 잡기 루프가 낼 (v, w, 받은 시각)
        self._gyro_sum, self._gyro_n = 0.0, 0
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.02, self._yaw_hold_tick)
        self._gyro_z, self._gyro_t, self._w_sent = 0.0, 0.0, 0.0
        self.create_subscription(Imu, "/imu/base_link", self._on_imu,
                                 qos_profile_sensor_data)
        self.ramp_v = 0.0
        self.ramp_w = 0.0
        self.ramp_t = time.monotonic()
        self.max_v = g("max_linear_mps").value
        self.lift_max_v = float(g("lift_max_linear_mps").value)
        self.lift_acc_v = float(g("lift_accel_mps2").value) or 1e9
        self.max_w = g("max_angular_rps").value
        self.cmd_timeout = g("cmd_timeout_s").value
        self.publish_tf = g("publish_odom_tf").value
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value

        port = g("serial_port").value
        self.ser = SerialPort(port, g("baudrate").value)
        self.ser.checksum = bool(g("line_checksum").value)
        self.ser.crc = bool(g("line_crc").value)
        self.get_logger().info("시리얼 %s @ %d 열림" % (port, g("baudrate").value))

        # 상태
        self.x = self.y = self.th = 0.0
        self.act_rpm = {1: 0.0, 2: 0.0}       # 참고/진단용
        self.diag = {}                        # 축별 online/enabled/err/volt
        self.enc = {1: None, 2: None}         # 최신 엔코더 카운트
        self.enc_prev = {1: None, 2: None}
        self.wheel_pos = {"l": 0.0, "r": 0.0}  # 적분한 바퀴 각도 [rad]
        self.last_cmd_t = 0.0
        self.last_odom_t = None
        self.last_sent = (None, None)
        self._bad = {1: 0, 2: 0}              # 지령 불일치 연속 횟수
        self.resent = 0                       # 다시 보낸 누적 횟수
        self.pair_tol = g("feedback_pair_tol_s").value
        self.pair_skip = 0                    # 짝이 안 맞아 건너뛴 주기 수
        self._skip_log = 0
        self._buf = ""
        self.dither = g("rpm_dither").value
        # ⚠ 정수 rpm 은 회전으로 치면 약 0.1 rad/s 단위라, 다리 사이의 작은
        #   조향(0.01~0.04 rad/s)을 디더링으로 흉내 냈고 지연과 겹쳐 ±2~3도로
        #   흔들렸다(2026-09-28). 드라이브의 0x60FF 는 1 rpm 당 약 1118 단위를
        #   받으므로 0.01 rpm 단위로 보낸다. 2026-09-28 16:31 펌웨어부터 지원.
        self.fine = bool(g("fine_rpm").value)
        # 'w' 는 0.01 rpm 단위만 받는다. fine_rpm 이 꺼져 있으면 두 줄로 보낸다.
        self.pair = bool(g("pair_cmd").value) and self.fine
        self._res = {"l": 0.0, "r": 0.0}      # 정수 양자화 잔차

        self.pub_odom = self.create_publisher(Odometry, "odom", 20)
        self.pub_js = self.create_publisher(JointState, "joint_states", 20)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_bat = self.create_publisher(BatteryState, "/battery_state", 10)
        # BatteryState 에 없는 값(SOH, 남은 시간, 상태 비트 등)까지 담은 JSON
        self.pub_bat_detail = self.create_publisher(String, "/battery_detail", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        # 최소 회전 반경을 적용하지 않는 지령. 가감속 제한은 그대로다.
        # 렉 한가운데서 기어가며 캐스터를 뒤집을 때, 캐스터가 차체를 돌리는
        # 것을 받아쳐야 하는데 0.03 m/s 에서는 반경 제한(w <= v/1.5)이 회전을
        # 0.02 rad/s 로 묶었다(2026-09-28). 그 한 동작만 이 길로 보낸다.
        self.create_subscription(
            Twist, "cmd_vel_noradius",
            lambda m: self.on_cmd_vel(m, radius=False), 10)

        # STM32 콘솔 명령을 ROS 서비스로 노출한다. UI/상위 시스템이
        # 시리얼을 직접 열지 않고도 모터를 준비·해제·복구할 수 있다.
        self.create_service(Trigger, "~/enable", self._srv_enable)
        self.create_service(Trigger, "~/release", self._srv_release)
        self.create_service(Trigger, "~/clear_errors", self._srv_clear)
        # 가속 제한을 건너뛰고 즉시 세운다. 비상정지가 쓴다.
        self.create_service(Trigger, "~/stop_now", self._srv_stop_now)
        # 리프팅 (STM32 PC0 올림 / PE1 내림, 2026-10-07 · 핀 2026-10-30). 펌웨어가 정해진
        # 시간이 지나면 출력을 스스로 내린다. 상태는 [LIFT] 줄에서 읽는다.
        self.create_service(Trigger, "~/lift_up", self._srv_lift_up)
        self.create_service(Trigger, "~/lift_down", self._srv_lift_down)
        self.create_service(Trigger, "~/lift_stop", self._srv_lift_stop)
        self.pub_lift = self.create_publisher(String, "/lift_state", 10)
        # up / down(움직이는 중) top / bottom / stopped / unknown.
        # 펌웨어는 리셋하면 위치를 잊지만 리프트는 그 자리에 있다. 렉을 든
        # 채 base_node 가 다시 떠도 느린 주행이 유지되도록 파일에 남긴다.
        self.lift_state = self._lift_load()
        self.create_timer(1.0, self._pub_lift)

        threading.Thread(target=self.serial_reader, daemon=True).start()

        hz = g("poll_hz").value
        self.create_timer(1.0 / hz, self.tick)
        self._ae_until = time.monotonic() + float(g("auto_enable_window_s").value)
        self._ae_tries = 0
        self._ae_last = -1e9
        self._ae_warned = False
        self._ae_timer = (self.create_timer(1.0, self._auto_enable_tick)
                          if bool(g("auto_enable").value) else None)
        self.get_logger().info(
            "차체: 윤거 %.4f m, 바퀴직경 좌 %.4f / 우 %.4f m (비율 %.5f), "
            "감속비 %.2f, %.0f counts/회전"
            % (self.sep, self.radius_side["l"] * 2.0, self.radius_side["r"] * 2.0,
               _k, self.gear, self.cpr))
        self.get_logger().info(
            "바퀴 배선: 좌=축%d(부호 %+.0f) / 우=축%d(부호 %+.0f)"
            % (self.side_axis["l"], self.side_sign["l"],
               self.side_axis["r"], self.side_sign["r"]))
        self.get_logger().info(
            "분해능: %.4f mm/count"
            % (2.0 * math.pi * self.radius / self.cpr * 1000.0))

    # ---------------- 지령 ----------------

    # 한 바퀴가 이보다 빠르면 엔코더 값이 잘못된 것으로 본다 [m/s].
    # 모터 상한 200rpm 이 1.8 m/s 다.
    ODOM_V_MAX = 3.0

    # api_node 가 직접 모는 통로(/cmd_vel_noradius)가 살아 있으면 /cmd_vel 은
    # 무시한다 [s].
    #
    # ⚠ nav2 목표를 끊은 뒤에도 velocity_smoother·collision_monitor 가 약 1초
    #   동안 /cmd_vel 에 0 을 계속 낸다. 그 사이 api_node 가 목표점까지 기어
    #   들어가는 지령을 내면 둘이 번갈아 들어가 덜컹였다(2026-09-29 정밀 도착).
    #   비상정지는 ~/stop_now 서비스가 따로 세운다(이 우선순위를 푼다).
    FREE_HOLD_S = 0.3

    def _on_params(self, params):
        """yaw_hold_* 는 실행 중에 바꿀 수 있다(세기를 올려 가며 시험)."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "yaw_hold_kp":
                self.yaw_kp = float(p.value)
            elif p.name == "yaw_hold_tau_s":
                self.yaw_tau = float(p.value)
            elif p.name == "yaw_hold_max_rps":
                self.yaw_max = float(p.value)
        return SetParametersResult(successful=True)

    def _on_imu(self, m):
        self._gyro_sum += m.angular_velocity.z
        self._gyro_n += 1
        self._gyro_z = m.angular_velocity.z
        self._gyro_t = time.monotonic()

    def on_cmd_vel(self, msg, radius=True):
        now = time.monotonic()
        if getattr(self, "_fault", None) is not None:
            # 모터 이상 중 - 주행 지령은 버리고 계속 세워 둔다 (_check_fault)
            if now - getattr(self, "_fault_zero_t", 0.0) > 0.5:
                self._fault_zero_t = now
                self.send_wheels(0, 0)
            return
        if not radius:
            self._free_t = now
        elif now - getattr(self, "_free_t", 0.0) < self.FREE_HOLD_S:
            return
        v = max(-self.max_v, min(self.max_v, msg.linear.x))
        w = max(-self.max_w, min(self.max_w, msg.angular.z))
        # 리프트가 움직이는 10초 동안은 세운다 (2026-10-07). 어느 경로(nav2 · 수동 ·
        # 도킹)로 와도 여기를 지난다.
        # ⚠ 2026-11-12 렉을 든 상태(top/stopped) 감속(lift_max_linear_mps 0.20 ·
        #   lift_accel_mps2 0.05)은 뺐다 — 사용자: "리프트해도 속도는 같아야해". 리프트를
        #   중간에 멈춘 'stopped' 도 든 상태로 쳐서 설정 0.6 이 0.2 로 묶였다.
        lift = getattr(self, "lift_state", "unknown")
        if lift in ("up", "down"):
            v = w = 0.0

        # 가속 제한.
        #
        # ⚠ 예전에는 지령을 그대로 rpm 으로 바꿔 던졌다. 0.2 m/s 로 출발하면
        #   0 에서 22 rpm 으로 한 번에 뛴다. 그 순간 두 바퀴가 받는 부하가
        #   다르면(캐스터가 어디를 향하고 있느냐에 따라 달라진다) 튀어 나가는
        #   정도가 달라져 차체가 돌아간다. 직진 6회 실측에서 좌우 바퀴 차이가
        #   매번 출발 직후 0.19~0.21m 지점에서 최대였다(2026-09-16).
        #
        #   nav2 경로에는 velocity_smoother 가 걸려 있지만, 수동 조종과 렉
        #   진입은 그 경로를 안 거쳐 아무 제한이 없었다.
        #
        # 정지도 완만하게 한다. 다만 감속은 가속보다 세게 허용하고,
        # 비상정지·통신 끊김은 이 경로를 안 거친다 —
        # 비상정지는 ~/stop_now 서비스로, 통신 끊김은 아래 감시가 즉시 세운다.
        now = time.monotonic()
        dt = now - self.ramp_t
        self.ramp_t = now
        if dt <= 0.0 or dt > 0.5:
            self.ramp_v, self.ramp_w = v, w      # 오래 끊겼으면 새로 시작
        else:
            # 줄이는 쪽은 더 빠르게 허용한다. 세우는 일은 늦으면 위험하다.
            av = self.acc_v if abs(v) >= abs(self.ramp_v) else self.dec_v
            aw = self.acc_w if abs(w) >= abs(self.ramp_w) else self.dec_w
            dv = av * dt
            dw = aw * dt
            self.ramp_v += max(-dv, min(dv, v - self.ramp_v))
            self.ramp_w += max(-dw, min(dw, w - self.ramp_w))
        v, w = self.ramp_v, self.ramp_w

        # 최소 회전 반경.
        #
        # ⚠ 회전 반경은 v/w 다. 가속하는 동안 v 가 아직 작은데 w 가 제 값으로
        #   나가면 반경이 확 줄어 제자리에서 홱 도는 것처럼 보인다 —
        #   0.01 m/s 에 0.05 rad/s 면 반경 0.2m 다. 속도가 다 오르면 같은
        #   값이 3m 가 된다. 그래서 '출발할 때만 급회전' 이 생겼다(2026-09-16).
        #
        # 제자리 회전(전진 지령이 0)은 이 제한을 안 받는다. 그건 반경이
        # 0 인 것이 목적이다.
        if radius and self.min_radius > 0.0 and abs(self.ramp_v) > 1e-6:
            wm = abs(self.ramp_v) / self.min_radius
            w = max(-wm, min(wm, w))

        # 자이로 받아치기 — 주행 중 캐스터가 차체를 돌리기 시작하는 순간 바퀴에서
        # 먼저 잡는다. 실제 회전(자이로)에서 직전에 보낸 회전을 뺀 것이 바깥
        # 힘이 만든 회전이다. 그 KD 배를 반대로 더한다.
        #
        # ⚠ nav2 는 캐스터에 밀린 것을 0.5초쯤 늦게 알아채고 크게 되돌려, 속도가
        #   오르는 동안 ±5도 S자를 그렸다(2026-09-29 왕복 기록: 지령 +3~+8 도/s
        #   인데 차체는 -3~-4 도/s). 도킹(api_node, KD 0.6)에서 같은 방법으로
        #   흔들림이 줄었고 0.8·1.0 은 스스로 흔들렸다. 그 도킹은 자기 받아치기가
        #   있으므로 여기서는 /cmd_vel 경로(nav2·조이스틱)에만, 주행 중에만 건다
        #   — 제자리 회전(도착 후 방향 맞추기)은 그대로 둔다.
        if (radius and self.rate_kd > 0.0 and abs(v) > 0.02
                and now - self._gyro_t < 0.2):
            w -= self.rate_kd * (self._gyro_z - self._w_sent)
            w = max(-self.max_w, min(self.max_w, w))

        self._w_sent = w

        # 주행 중 차체 회전 잡기 — 캐스터가 차체를 휘두르는 만큼 바퀴로 맞받는다.
        #
        # ⚠ 0.35 m/s 에서 조향 계단을 주면 바퀴(엔코더)는 0.15~0.25초 만에 따라와
        #   1.27배까지 가는데, 차체(자이로)는 그보다 더 돌고 되튀었다(2026-10-02).
        #   9/29 받아치기(위 rate_kd)는 자이로를 '방금 보낸 지령' 과 비교해, 바퀴가
        #   반응하는 동안의 정상적인 늦음까지 방해로 보고 거꾸로 꺾었다. 여기서는
        #   지령을 바퀴 반응 시간(tau)만큼 늦춘 '차체가 돌아야 할 회전' 과 비교해,
        #   그보다 더(덜) 도는 몫만 바퀴로 되잡는다.
        # ⚠ 처음엔 지령이 올 때(20Hz)만 고쳐 캐스터 떨림보다 늦었다. 잡기가 켜져
        #   있고 달리는 중이면 지령은 저장만 하고, 50Hz 루프(_yaw_hold_tick)가
        #   최신 자이로로 고쳐서 바퀴에 낸다.
        if radius and self.yaw_kp > 0.0 and abs(v) > 0.05:
            self._hold = (v, w, time.monotonic())
            return
        self._hold = None
        self._w_ref = w
        self._send_vw(v, w)

    def _yaw_hold_tick(self):
        """50Hz. 저장된 지령에 자이로 잡기를 더해 바퀴에 낸다(on_cmd_vel 참고)."""
        now = time.monotonic()
        last, self._hold_last = getattr(self, "_hold_last", now), now
        g_n = self._gyro_n
        g = self._gyro_sum / g_n if g_n else self._gyro_z
        self._gyro_sum, self._gyro_n = 0.0, 0
        h = self._hold
        if h is None or self.yaw_kp <= 0.0:
            return
        v, w, t = h
        if now - t > 0.15:
            self._hold = None           # 지령이 끊겼다 — 감시(tick)가 세운다
            return
        dt = min(0.1, max(0.0, now - last))
        if dt <= 0.0 or now - self._gyro_t > 0.2:
            self._send_vw(v, w)
            return
        a = dt / (self.yaw_tau + dt)
        self._w_ref += a * (w - self._w_ref)
        corr = self.yaw_kp * (self._w_ref - g)
        corr = max(-self.yaw_max, min(self.yaw_max, corr))
        self._send_vw(v, max(-self.max_w, min(self.max_w, w + corr)))

    def _send_vw(self, v, w):
        # 차동구동 역기구학
        v_l = v - w * self.sep / 2.0
        v_r = v + w * self.sep / 2.0

        rpm_l = self.mps_to_rpm(v_l, "l")
        rpm_r = self.mps_to_rpm(v_r, "r")

        # 한쪽이 한계를 넘으면 양쪽을 같은 비율로 줄여 회전 반경을 보존한다
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak > self.max_rpm:
            k = self.max_rpm / peak
            rpm_l *= k
            rpm_r *= k

        q_l, q_r = self.quantize(rpm_l * self.side_sign["l"],
                                 rpm_r * self.side_sign["r"])
        ax = {self.side_axis["l"]: q_l, self.side_axis["r"]: q_r}
        self.send_wheels(ax[1], ax[2])
        self.last_cmd_t = time.monotonic()

    def _srv_stop_now(self, req, res):
        """가속 제한을 건너뛰고 즉시 0 rpm. 비상정지·안전 계통이 쓴다."""
        self._free_t = 0.0          # /cmd_vel 의 0 도 다시 받는다
        self._hold = None           # 잡기 루프가 옛 지령을 다시 내지 않게
        self.ramp_v = 0.0
        self.ramp_w = 0.0
        # ⚠ 같은 값 0.2초 거르기를 건너뛴다. 방금 보낸 0 한 줄이 통신에서
        #   빠졌다면 이 정지도 걸러져 아무것도 안 나갔다(2026-09-30 검토).
        self.last_send_t = 0.0
        self.send_wheels(0, 0)
        res.success = True
        res.message = "즉시 정지"
        return res

    @staticmethod
    def centi_to_rpm(c):
        """펌웨어가 상태 줄에 내는 정수 rpm (0.5 는 0 에서 먼 쪽으로)."""
        return (c + 50) // 100 if c >= 0 else -((-c + 50) // 100)

    def quantize(self, rpm_l, rpm_r):
        """정수 rpm 으로 만들되 버려지는 소수를 다음 주기로 넘긴다(오차확산).

        드라이브 목표속도 오브젝트가 1 rpm 단위라, 좌우 직경차 보정량
        (0.5% 미만 = 25rpm 에서 0.1rpm) 이 단순 반올림에서 통째로 사라진다.
        잔차를 누적해 가끔 1 rpm 을 더 얹으면 시간 평균이 원하는 값이 된다.

        지령이 0 이면 잔차와 무관하게 반드시 0 을 내보낸다. 안 그러면
        정지 지령에 바퀴가 1 rpm 으로 계속 기어간다."""
        if self.fine:
            return int(round(rpm_l * 100.0)), int(round(rpm_r * 100.0))
        if not self.dither:
            return int(round(rpm_l)), int(round(rpm_r))
        out = []
        for key, val in (("l", rpm_l), ("r", rpm_r)):
            if val == 0.0:
                self._res[key] = 0.0
                out.append(0)
                continue
            v = val + self._res[key]
            n = int(round(v))
            # 잔차가 발산하지 않도록 한 스텝 이내로 제한
            self._res[key] = max(-0.5, min(0.5, v - n))
            out.append(n)
        return out[0], out[1]

    def mps_to_rpm(self, mps, side=None):
        # 바퀴 원주 = 2*pi*r  →  회전수/분 = v*60/(2*pi*r), 감속기 있으면 곱
        r = self.radius if side is None else self.radius_side[side]
        return mps * 60.0 / (2.0 * math.pi * r) * self.gear

    def rpm_to_mps(self, rpm, side=None):
        r = self.radius if side is None else self.radius_side[side]
        return rpm / 60.0 * (2.0 * math.pi * r) / self.gear

    # 같은 지령이라도 이 주기마다 다시 보낸다 [s].
    #
    # ⚠ 같은 값이면 안 보내다 보니, 한 줄이 통신에서 사라지면 그 바퀴가 직전
    #   지령에 그대로 남았다. 소수 rpm 에서는 0.3 rpm(2.5%) 차이가 드라이브
    #   보고(정수 rpm)로는 '일치' 라 아무도 못 고쳤다 — 회전 지령 0 으로 1.2m
    #   가는 동안 차체가 8~10도 돌았다(2026-09-29 직진 시험). 곧게 가는 구간
    #   (지령이 일정)마다 생길 수 있다. 사라져도 이 시간 안에 복구된다.
    RESEND_S = 0.2

    def send_wheels(self, rpm_l, rpm_r):
        now = time.monotonic()
        if ((rpm_l, rpm_r) == self.last_sent
                and now - getattr(self, "last_send_t", 0.0) < self.RESEND_S):
            return                      # 같은 값이면 잠깐은 다시 안 보낸다
        self.last_send_t = now
        hist = getattr(self, "sent_hist", None)
        if hist is None:
            # ⚠ 5 -> 20. 아래 verify_cmd 는 '최근 0.4초 안에 보낸 값' 과 비교하는데
            #   20Hz 로 바뀌는 가속 중에는 5개가 0.25초치뿐이라, 출발 직후 피드백
            #   (아직 0)을 불일치로 보고 매번 다시 보냈다(2026-09-30 로그).
            hist = self.sent_hist = collections.deque(maxlen=20)
        hist.append((time.monotonic(), rpm_l, rpm_r))
        self.last_sent = (rpm_l, rpm_r)
        cmd = "v" if self.fine else "s"   # fine 이면 값은 0.01 rpm 단위
        # 정지(0)는 두 번 보낸다. 한 줄이 사라지면 그 바퀴만 돌아 차체가 돈다
        # (2026-09-28: 축2 가 0.5초 -5 rpm 에 머물러 4도 돌았다).
        for _ in range(2 if (rpm_l, rpm_r) == (0, 0) else 1):
            if self.pair:
                # 한 줄이면 두 바퀴가 같이 바뀌거나 같이 안 바뀐다. 두 줄일 때는
                # 뒤 줄(축2)만 빠지는 일이 4배 많았다(2026-09-30, 불일치 191:787).
                self.ser.write_line("w %d %d" % (rpm_l, rpm_r))
            else:
                self.ser.write_line("%s 1 %d" % (cmd, rpm_l))
                self.ser.write_line("%s 2 %d" % (cmd, rpm_r))

    def verify_cmd(self):
        """보낸 지령이 실제로 들어갔는지 드라이브 피드백으로 확인하고 고친다.

        시리얼에서 한 글자만 씹혀도 "s 1 17" 이 "s 1 1" 이 되어 그 바퀴가
        거의 선다. 그런데 send_wheels 는 같은 값이면 다시 보내지 않으므로,
        한 번 어긋나면 지령값이 바뀔 때까지 틀린 채로 남는다.
        실제로 직진 중 한쪽이 17 대신 1 로 들어가 로봇이 홱 돌았다
        (2026-09-11, 0.603m 지점).

        피드백은 우리가 보낸 시점보다 늦을 수 있으므로, 두 주기 연속으로
        어긋날 때만 다시 보낸다."""
        if self.last_sent == (None, None):
            return
        for ax in (1, 2):
            sent = self.last_sent[ax - 1]
            want = self.centi_to_rpm(sent) if self.fine else sent
            # 피드백은 지령보다 늦게, 더 드물게 온다. 지령이 매 주기 바뀌면
            # (가감속, 소수 rpm) 몇 주기 전 값을 보고 있는 것이 정상이다 —
            # 그것까지 불일치로 보면 감속할 때마다 다시 보냈다(2026-09-28).
            # 최근 0.4초 안에 보낸 지령 중 하나와 맞으면 통과. 줄이 합쳐진
            # 수백 rpm 은 여전히 걸린다.
            # ⚠ 시간 제한이 없으면, 한 줄이 통째로 사라져 드라이브가 옛 값에
            #   머물 때 그 옛 값이 기록에 남아 있는 한 영영 못 고친다(같은
            #   값이면 다시 보내지 않으므로 기록도 안 밀린다).
            now = time.monotonic()

            def sent_within(age):
                return {self.centi_to_rpm(h[ax]) if self.fine else h[ax]
                        for h in getattr(self, "sent_hist", ())
                        if now - h[0] <= age}
            recent = sent_within(0.4)
            recent_fast = sent_within(0.15)   # 감속 쪽은 피드백 한두 번만 봐준다
            d = self.diag.get(ax)
            if not d or d.get("cmd_rpm") is None:
                self._bad[ax] = 0
                continue
            # ⚠ 드라이브가 보낸 값보다 빠르게 돌고 있으면(정지·감속이 안 먹음)
            #   봐주지 않는다. 늦게 따라오는 것은 올릴 때만 정상이다.
            #   2026-09-28: 0 을 보냈는데 축2 가 -5 에 머물렀고, -5 가 0.4초 안에
            #   보낸 값이라 정상으로 봐서 0.5초 방치 — 차체가 4도 돌았다.
            slower = abs(d["cmd_rpm"]) > abs(want)
            if d["cmd_rpm"] == want or (
                    d["cmd_rpm"] in (recent_fast if slower else recent)):
                self._bad[ax] = 0
                continue
            self._bad[ax] += 1
            # 크게 어긋났으면 두 주기를 기다리지 않는다. 줄이 합쳐져 수백 rpm 이
            # 들어간 경우라 기다리는 동안 바퀴가 튄다(2026-09-28).
            big = abs(d["cmd_rpm"] - want) > 10
            if self._bad[ax] >= 2 or big:
                self._bad[ax] = 0
                self.resent += 1
                self.ser.write_line("%s %d %d" % ("v" if self.fine else "s",
                                                  ax, sent))
                self.get_logger().warn(
                    "축%d 지령 불일치: 보낸 값 %d / 드라이브 %d — 다시 보냄 (누적 %d회)"
                    % (ax, want, d["cmd_rpm"], self.resent))

    # ---------------- 서비스 ----------------

    def _srv_enable(self, req, res):
        """양축 초기화 — 알람 클리어 → 속도모드 → 램프 → 사용."""
        self.send_wheels(0, 0)
        self.ser.write_line("i")
        res.success = True
        res.message = "양축 초기화 명령 전송"
        return res

    # 기동 직후 한 번만 드라이브를 켠다. 이미 켜져 있으면(base_node 만 다시 뜬 경우)
    # 건드리지 않고, 알람 · 끊김이 있으면 켜지 않는다(_check_fault 와 같은 기준).
    # 기동 뒤 auto_enable_window_s 안에 3번까지 시도하고 그 뒤로는 손대지 않는다 —
    # 운행 중에 누가 해제(z)했거나 알람으로 선 것을 몰래 되살리지 않기 위해서다.
    AE_TRIES = 3
    AE_RETRY_S = 3.0

    def _auto_enable_tick(self):
        now = time.monotonic()
        d = [self.diag.get(1), self.diag.get(2)]
        fresh = all(x and now - x.get("t", -1e9) < 1.0 for x in d)
        if fresh and all(x.get("enabled") for x in d):
            if self._ae_tries:
                self.get_logger().info("드라이브 자동 켜기 완료 (%d번째 시도)" % self._ae_tries)
            else:
                self.get_logger().info("드라이브가 이미 켜져 있음 — 자동 켜기 생략")
            self._ae_done()
            return
        if now > self._ae_until or (self._ae_tries >= self.AE_TRIES
                                    and now - self._ae_last > self.AE_RETRY_S):
            self.get_logger().error(
                "드라이브 자동 켜기 실패 (%s) — 확인 뒤 /base_node/enable 로 켜야 한다"
                % ("드라이브 응답 없음" if not fresh else
                   "알람 · 끊김" if self._ae_tries == 0 else "%d번 시도" % self._ae_tries))
            self._ae_done()
            return
        if not fresh or now - self._ae_last < self.AE_RETRY_S:
            return
        if (getattr(self, "_fault", None) is not None
                or any((not x.get("online")) or x.get("error") for x in d)):
            if not self._ae_warned:
                self._ae_warned = True
                self.get_logger().warn("드라이브 알람 · 끊김 중이라 자동 켜기를 미룬다")
            return
        self._ae_tries += 1
        self._ae_last = now
        self.get_logger().info("드라이브 자동 켜기 (%d/%d)" % (self._ae_tries, self.AE_TRIES))
        self.send_wheels(0, 0)
        self.ser.write_line("i")

    def _ae_done(self):
        if self._ae_timer is not None:
            self._ae_timer.cancel()
            self._ae_timer = None

    def _srv_lift_up(self, req, res):
        self.ser.write_line("l up")
        res.success = True
        res.message = "리프팅 올림"
        return res

    def _srv_lift_down(self, req, res):
        self.ser.write_line("l down")
        res.success = True
        res.message = "리프팅 내림"
        return res

    def _srv_lift_stop(self, req, res):
        self.ser.write_line("l stop")
        res.success = True
        res.message = "리프팅 정지"
        return res

    def _on_lift_line(self, line):
        """펌웨어 [LIFT] 줄로 리프팅 상태를 갱신한다 (lift.c 의 printf 와 짝)."""
        if "up start" in line:
            st = "up"
        elif "down start" in line:
            st = "down"
        elif "up done" in line:
            st = "top"
        elif "down done" in line:
            st = "bottom"
        elif "stop after" in line:
            st = "stopped"
        else:
            return
        self.lift_state = st
        try:
            with open(self.LIFT_FILE, "w") as f:
                f.write(st)
        except OSError:
            pass
        self._pub_lift()

    LIFT_FILE = os.path.expanduser("~/.und_amr_lift_state")
    LIFT_LOADED = ("top", "stopped")     # 렉을 들고 있을 수 있는 상태

    def _lift_load(self):
        try:
            with open(self.LIFT_FILE) as f:
                st = f.read().strip()
        except OSError:
            return "unknown"
        # 움직이던 중에 꺼졌다면 어디서 멈췄는지 모른다 — 중간으로 본다
        return "stopped" if st in ("up", "down") else (st or "unknown")

    def _pub_lift(self):
        self.pub_lift.publish(String(data=json.dumps({"state": self.lift_state})))

    def _srv_release(self, req, res):
        self.ser.write_line("z")
        res.success = True
        res.message = "축 해제(프리) 명령 전송"
        return res

    # ⚠ 2026-11-17 축1 알람 0x4000(엔코더 통신)이 직진 빠져나오기 중에 걸려, 오른쪽 바퀴는 서고 왼쪽만
    #   돌아 로봇이 -1.0~-1.3 rad/s 로 한 바퀴 반을 돌았다. 알람은 화면에 '모터 이상' 으로만 떴다.
    #   이제 어느 축이든 알람 · 끊김이면 바로 두 바퀴를 세우고, 풀릴 때(두 축 정상 FAULT_CLEAR_S)까지
    #   주행 지령을 받지 않는다.
    FAULT_CLEAR_S = 1.0

    def _check_fault(self, axis):
        d = self.diag.get(axis) or {}
        bad = (not d.get("online", True)) or bool(d.get("error"))
        f = getattr(self, "_fault", None)
        now = time.monotonic()
        if bad:
            self._fault_ok_t = None
            if f is None:
                self._fault = {"axis": axis, "error": d.get("error"), "t": now}
                self.ramp_v = 0.0
                self.ramp_w = 0.0
                self._hold = None
                self.last_sent = (None, None)          # 같은 값 거르기를 건너뛴다
                self.send_wheels(0, 0)
                self.get_logger().error(
                    "모터 이상 — 축%d %s (0x%04X). 두 바퀴를 세우고 알람이 풀릴 때까지 주행 지령을 막는다"
                    % (axis, "끊김" if not d.get("online", True) else "알람", int(d.get("error") or 0)))
            return
        if f is None:
            return
        ok_all = all(dd.get("online") and not dd.get("error") for dd in self.diag.values())
        if not ok_all:
            return
        if getattr(self, "_fault_ok_t", None) is None:
            self._fault_ok_t = now
        elif now - self._fault_ok_t >= self.FAULT_CLEAR_S:
            self.get_logger().warn("모터 이상 풀림 — 주행 지령 다시 받음")
            self._fault = None
            self._fault_ok_t = None

    def _srv_clear(self, req, res):
        self.ser.write_line("a")
        res.success = True
        res.message = "알람 클리어 명령 전송"
        return res

    # ---------------- 피드백 ----------------

    def serial_reader(self):
        while rclpy.ok():
            chunk = self.ser.read()
            if not chunk:
                continue
            self._buf += chunk.decode("utf-8", "replace")
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                # 펌웨어가 줄을 버리거나 CAN 쓰기에 실패하면 알린다. 예전에는
                # 흘려버려 정지 지령이 왜 안 먹었는지 알 수 없었다(2026-09-28).
                if ("checksum mismatch" in line or "write failed" in line
                        or "rejected" in line or "not enabled" in line
                        or "[UART1]" in line or "[DRIVE]" in line
                        or "[CAN2]" in line or "WRITE-FAIL" in line
                        or "[LIFT]" in line):
                    self.get_logger().warn("펌웨어: %s" % line.strip())
                if "[LIFT]" in line:
                    self._on_lift_line(line)
                    continue
                if "[BAT]" in line:
                    self._on_battery_line(line)
                    continue
                m = FEEDBACK_RE.search(line)
                if m:
                    axis = int(m.group("axis"))
                    self.act_rpm[axis] = float(m.group("act"))
                    self.enc[axis] = int(m.group("pos"))
                    self.diag[axis] = {
                        "online": m.group("online") == "online",
                        "enabled": m.group("enabled") == "enabled",
                        "cmd_rpm": int(m.group("cmd")),
                        "act_rpm": int(m.group("act")),
                        "position": int(m.group("pos")),
                        "status": int(m.group("status"), 16),
                        "error": int(m.group("err"), 16),
                        "voltage": int(m.group("volt")),
                        "iq": (float(m.group("iq")) if m.group("iq")
                               is not None else None),
                        # 오도메트리 시간축으로 쓴다. 벽시계(time.time)는 NTP 가
                        # 뒤로 돌리면 dt 가 음수가 되어 오도메트리가 멈췄다가
                        # 한꺼번에 튄다 — 단조 시계를 쓴다(2026-09-29 검토).
                        "t": time.monotonic(),
                    }
                    self._check_fault(axis)
            if len(self._buf) > 512:
                self._buf = ""

    def _on_battery_line(self, line):
        m = BAT_RE.search(line)
        now = time.time()
        if not m or int(m.group("age")) > BAT_STALE_MS:
            self.pub_bat_detail.publish(String(data=json.dumps(
                {"ok": False, "t": now,
                 "error": "배터리 응답 없음" if "none" in line or not m
                 else "배터리 응답이 %.1f초 지났습니다"
                 % (int(m.group("age")) / 1000.0)})))
            return
        g = {k: int(v, 16) if k == "st" else int(v)
             for k, v in m.groupdict().items()}
        d = {"ok": True, "t": now,
             "voltage": g["v"] / 100.0, "current": g["i"] / 100.0,
             "soc": g["soc"], "soh": g["soh"],
             "remaining_ah": g["rc"] / 100.0, "energy_wh": g["ae"] / 10.0,
             "temperature": g["t"] / 10.0,
             "time_to_full_min": g["ttf"], "time_to_empty_min": g["tte"],
             "cycles": g["cyc"], "status": g["st"],
             "flags": [n for b, n in enumerate(BAT_FLAGS) if g["st"] >> b & 1],
             "age_ms": g["age"], "design_ah": BAT_DESIGN_AH}
        self.pub_bat_detail.publish(String(data=json.dumps(d, ensure_ascii=False)))
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = d["voltage"]
        b.current = d["current"]
        b.temperature = d["temperature"]
        b.charge = d["remaining_ah"]
        b.capacity = (d["remaining_ah"] / (d["soc"] / 100.0)
                      if d["soc"] > 0 else float("nan"))
        b.design_capacity = BAT_DESIGN_AH
        b.percentage = d["soc"] / 100.0
        b.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if d["current"] > 0.2 else
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if d["current"] < -0.2
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
        st = g["st"]
        b.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE if st & 0x01 else
            BatteryState.POWER_SUPPLY_HEALTH_DEAD if st & 0x02 else
            BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT if st & 0x10 else
            BatteryState.POWER_SUPPLY_HEALTH_COLD if st & 0x20 else
            BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE if st & 0x4C else
            BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        b.location = "main"
        self.pub_bat.publish(b)

    def tick(self):
        # cmd_vel 이 끊기면 정지 (놓친 명령으로 계속 달리는 사고 방지)
        if self.last_cmd_t and \
                (time.monotonic() - self.last_cmd_t) > self.cmd_timeout:
            # 가속 제한을 건너뛰고 즉시 세운다. 다음 지령은 0 에서 다시 오른다.
            self.ramp_v = 0.0
            self.ramp_w = 0.0
            self.send_wheels(0, 0)
            self.last_cmd_t = 0.0

        self.verify_cmd()
        if self.pair_skip - self._skip_log >= 200:
            self._skip_log = self.pair_skip
            self.get_logger().warn(
                "피드백 짝 불일치로 건너뛴 주기 %d 회 — 시리얼 유실이 잦다"
                % self.pair_skip)
        self.ser.write_line("f")        # 다음 주기용 피드백 요청
        self.update_odom()
        self.publish_diagnostics()

    # L2DB4830 알람 비트 (매뉴얼 표 F1-2, 객체 0x2601:00)
    ERR_BITS = {
        0: "internal", 1: "encoder ABZ", 2: "encoder UVW", 3: "encoder count",
        4: "over-temperature", 5: "bus over-voltage", 6: "bus under-voltage",
        7: "output short", 8: "brake resistor", 9: "following error",
        11: "I2T overload", 14: "motor search failed",
    }

    def publish_diagnostics(self):
        if not self.diag:
            return
        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()
        for axis in sorted(self.diag):
            d = self.diag[axis]
            st = DiagnosticStatus()
            st.name = "motor/axis%d" % axis
            st.hardware_id = "L2DB4830-node%d" % axis
            if not d["online"]:
                st.level = DiagnosticStatus.ERROR
                st.message = "offline"
            elif d["error"]:
                st.level = DiagnosticStatus.ERROR
                st.message = ", ".join(
                    v for b, v in self.ERR_BITS.items() if d["error"] >> b & 1) \
                    or "error 0x%04X" % d["error"]
            elif not d["enabled"]:
                st.level = DiagnosticStatus.WARN
                st.message = "shaft free"
            else:
                st.level = DiagnosticStatus.OK
                st.message = "ok"
            st.values = [KeyValue(key=k, value=str(v))
                         for k, v in d.items() if k != "t"]
            da.status.append(st)
        self.pub_diag.publish(da)

    def wheel_delta_m(self, side, cur=None):
        """직전 호출 이후 그 쪽 바퀴가 굴러간 거리 [m]. 전진이 양수.

        rpm 피드백(1rpm 분해능, 30ms 샘플)이 아니라 엔코더 카운트를 쓴다.
        4096 counts/회전 이므로 분해능이 0.088° 이고 적분 오차가 쌓이지 않는다."""
        axis = self.side_axis[side]
        if cur is None:
            cur = self.enc[axis]
        if cur is None:
            return None
        prev = self.enc_prev[axis]
        self.enc_prev[axis] = cur
        if prev is None:
            return None
        d_rev = (cur - prev) / self.cpr / self.gear * self.side_sign[side]
        self.wheel_pos[side] += d_rev * 2.0 * math.pi
        return d_rev * 2.0 * math.pi * self.radius_side[side]

    def update_odom(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        # 두 축 피드백이 '같은 주기' 의 것인지 먼저 본다.
        #
        # MCU 는 'f' 한 번에 두 축 줄을 잇달아 보낸다(115200 에서 약 6ms 간격).
        # 한쪽 줄이 유실되면 그 축만 이전 주기 값이 남는데, 그대로 빼면
        # 한 주기치 이동량이 통째로 '회전' 으로 둔갑한다.
        # 0.15m/s 에서 한 주기(50ms)면 7.5mm, 윤거 179mm 로 나누면 2.4도다.
        # 실제로 직진 중 방향이 rms 5도로 떨었고, 자이로는 1도였다
        # (2026-09-11, tools/odom_smooth.py).
        #
        # 짝이 안 맞으면 이번 주기는 통째로 건너뛴다. last_odom_t 와
        # enc_prev 를 둘 다 그대로 두므로, 다음에 짝이 맞을 때 그 구간까지
        # 한 번에 적분된다. 이동량은 잃지 않고 잡음만 빠진다.
        d1, d2 = self.diag.get(1), self.diag.get(2)
        if not d1 or not d2:
            return
        if abs(d1["t"] - d2["t"]) > self.pair_tol:
            self.pair_skip += 1
            return

        # ⚠ 시간은 '계산하는 순간' 이 아니라 두 축 피드백이 들어온 시각으로 잰다.
        #   계산(tick 20Hz)과 피드백(20Hz)이 조금씩 어긋나, 새 피드백이 없는
        #   주기는 이동량 0 이라 속도 0, 다음 주기는 두 번치가 들어와 속도
        #   두 배가 됐다 — 달리는 중 0.4 -> 0 -> 0.8~1.0 m/s 로 180초에 45번
        #   널뛰었고, EKF 가 그 속도를 받아 위치가 들쭉날쭉해져 nav2 가 그것을
        #   고치느라 S자로 조향했다(지령이 차체보다 0.3~0.45초 먼저 흔들림,
        #   2026-09-29). 새 피드백이 없으면 건너뛴다(다음에 한 번에 적분된다).
        t = (d1["t"] + d2["t"]) / 2.0
        # 엔코더 값도 같은 피드백 줄에서 꺼낸다. self.enc 를 따로 읽으면 그
        # 사이 읽기 스레드가 한 축만 새 값으로 바꿔 시각과 값이 어긋날 수 있다.
        pos = {"l": d1["position"] if self.side_axis["l"] == 1 else d2["position"],
               "r": d1["position"] if self.side_axis["r"] == 1 else d2["position"]}
        if self.last_odom_t is None:
            self.last_odom_t = t
            return
        dt = t - self.last_odom_t
        if dt <= 1e-3:
            return                      # 새 피드백이 아직 없다
        if dt > 1.0:
            # 오래 비었다. 그동안의 카운트를 한 주기에 몰아넣으면 속도가
            # 튀므로, 기준점만 맞추고 이번 구간은 버린다.
            self.last_odom_t = t
            self.wheel_delta_m("l", pos["l"])
            self.wheel_delta_m("r", pos["r"])
            return
        self.last_odom_t = t

        d_l = self.wheel_delta_m("l", pos["l"])
        d_r = self.wheel_delta_m("r", pos["r"])
        if d_l is None or d_r is None:
            return                      # 아직 기준 카운트가 없다

        v_l = d_l / dt
        v_r = d_r / dt
        # ⚠ 물리적으로 불가능한 이동은 버린다(기준만 새 값으로 맞춰진 상태).
        #   펌웨어는 부팅 직후 드라이브 위치를 읽기 전까지 pos=0 을 보고한다.
        #   펌웨어를 구운 뒤 0 -> 195000 카운트가 한 주기에 들어와 초속 수백 m 로
        #   계산됐고, EKF 가 그것을 받아 위치가 수백만 m 로 날아가 SLAM 이
        #   죽었다(2026-09-30 15:16~15:39).
        if max(abs(v_l), abs(v_r)) > self.ODOM_V_MAX:
            self.odom_reject = getattr(self, "odom_reject", 0) + 1
            self.get_logger().warn(
                "오도메트리 튐 버림: 좌 %.1f / 우 %.1f m/s (%.0fms) — 누적 %d회"
                % (v_l, v_r, dt * 1000, self.odom_reject),
                throttle_duration_sec=1.0)   # 계속 튀면 20Hz 로 쏟아진다
            return
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.sep

        # 중점 적분: 회전 중 직선 근사 오차를 줄인다
        self.th += w * dt
        self.x += v * math.cos(self.th - w * dt / 2.0) * dt
        self.y += v * math.sin(self.th - w * dt / 2.0) * dt

        q = Quaternion()
        q.z = math.sin(self.th / 2.0)
        q.w = math.cos(self.th / 2.0)

        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = q
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        # 자세(pose) yaw 분산은 크게 둔다(EKF 는 /odom 의 속도만 쓴다). 회전
        # 속도 분산은 odom_yaw_rate_var 로 정한다 — 2026-10-02 부터 바퀴 위주.
        od.pose.covariance[0] = 0.02
        od.pose.covariance[7] = 0.02
        od.pose.covariance[35] = 0.10
        od.twist.covariance[0] = 0.02
        od.twist.covariance[35] = self.odom_w_var
        self.pub_odom.publish(od)

        if self.publish_tf:
            tr = TransformStamped()
            tr.header.stamp = now.to_msg()
            tr.header.frame_id = self.odom_frame
            tr.child_frame_id = self.base_frame
            tr.transform.translation.x = self.x
            tr.transform.translation.y = self.y
            tr.transform.rotation = q
            self.tf.sendTransform(tr)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [self.wheel_pos["l"], self.wheel_pos["r"]]
        js.velocity = [v_l / self.radius_side["l"], v_r / self.radius_side["r"]]
        # 참고: self.act_rpm 은 진단용으로만 남겨둔다
        self.pub_js.publish(js)

    def shutdown(self):
        try:
            self.ser.write_line("x")
            time.sleep(0.1)
            self.ser.close()
        except OSError:
            pass


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
