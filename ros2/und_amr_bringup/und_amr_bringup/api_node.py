#!/usr/bin/env python3
"""
und_amr_api — ROS2 위에 얹는 REST API 계층

AutoXing 로봇(axbot)의 엔드포인트 구조를 참고했다. 목적은 같다:
ROS 내부(토픽/액션/TF)를 모르는 상위 시스템이나 웹 UI 가
HTTP 만으로 로봇을 조회·제어할 수 있게 하는 것.

    GET   /                     운용 웹 UI
    GET   /api                  API 인덱스 (JSON)
    GET   /device/info          모델·버전·하드웨어
    GET   /chassis/status       제어모드·비상정지·모터 상태
    PATCH /chassis/status       {"control_mode": "auto|manual"}
    GET   /chassis/pose         현재 위치 (odom 기준)
    POST  /chassis/twist        {"linear":0.1,"angular":0.0}  수동 주행
    GET   /battery-state        배터리 BMS 값 (끊기면 모터 드라이버 모선 전압)
    GET   /battery-history      배터리 이력 ?hours=6 (최대 72)
    GET   /route/status         지점 왕복 상태
    POST  /route/start          지점 왕복 {a, b, repeat(0=무한), dwell}
    POST  /route/cancel         지점 왕복 멈춤
    GET   /scan                 최신 라이다 스캔
    GET   /maps/                저장된 맵 목록
    GET   /nodes                ROS 노드 목록
    GET   /topics               ROS 토픽 목록
    POST  /services/estop       {"enable": true|false}
    POST  /services/clear_errors

실행:
    ros2 run und_amr_bringup api_node
    ros2 run und_amr_bringup api_node --ros-args -p port:=8090
"""

import datetime
import io
import json
import shutil
import statistics
import signal
import subprocess
import sys
import math
import os
import threading
import time

import yaml
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from rclpy.qos import DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import tf2_ros as _tf2
from sensor_msgs.msg import Image, Imu, LaserScan
from und_amr_bringup.cart_match import (best_rectangles, cluster_scan,
                                        leg_candidates)
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
from std_srvs.srv import Empty as EmptySrv

try:
    from slam_toolbox.srv import (Clear, DeserializePoseGraph, Pause, SaveMap,
                                  SerializePoseGraph)
    HAS_SLAM = True
except ImportError:
    HAS_SLAM = False

try:
    import numpy as np
    from PIL import Image as PilImage
    from PIL import ImageDraw
    HAS_IMAGING = True
except ImportError:
    HAS_IMAGING = False

from ament_index_python.packages import get_package_share_directory

from und_amr_bringup.web_ui import PAGE
from und_amr_bringup.charger_dock import ChargerDockMixin
from und_amr_bringup.narrow_poi import NarrowPoiMixin
from und_amr_bringup import scan_icp

API_VERSION = "0.1.0"


def wrap_pi(d):
    return math.atan2(math.sin(d), math.cos(d))


class ApiNode(ChargerDockMixin, NarrowPoiMixin, Node):

    def __init__(self):
        super().__init__("api_node")
        self.declare_parameter("port", 8090)
        self.declare_parameter("map_dir",
                               os.path.expanduser("~/ros2_ws/maps"))
        self.port = self.get_parameter("port").value
        self.map_dir = self.get_parameter("map_dir").value
        self.declare_parameter("record_bag", True)
        self.bag_enabled = bool(self.get_parameter("record_bag").value)

        self.lock = threading.Lock()
        self.odom = None
        self.scan = None
        # 다리 표면->중심 보정 반지름. cart_detector 의 leg_radius 와 같은
        # 값을 써야 한다. 다르면 여기서 잰 규격이 검출기에서 안 맞는다.
        self.leg_radius = 0.020
        self.motors = {}
        self.control_mode = "auto"
        self.estop = False
        self.logs = deque(maxlen=300)
        self.cam_jpeg = None
        self.cam_info = {}
        self._cam_last = 0.0
        self._tf_err = None
        self.map_png = None
        # 편집 전 원본과 편집 목록. map_png 는 이 둘을 합성한 결과다.
        self.map_base_png = None
        # 위치추정용 합성본. 가상벽은 빼고 지우개만 반영한 것
        self.map_loc_png = None
        self.map_edits = []
        self.pois = []
        # 작업(POI 이동 -> 인식 -> 진입) 상태. 타이머는 이것 하나뿐이다.
        self.job = {"state": "idle", "message": "", "poi": None,
                    "cart": None, "step": "", "remain": None}
        self._job_timer = None
        self._job_t = 0.0
        self._job_seen = 0
        self._job_hits = []
        self._job_seen_t = 0.0
        self._job_anchor = None
        self._job_start_xy = None
        self._job_budget = 0.0
        self._job_poi = None
        self._job_cart = None   # 지도에 못박아 둔 대차 자세
        self._job_bad = 0
        self._job_last = None
        self._job_entry_yaw = None   # 대차로 들어가는 방향 (지도 기준)
        self._job_restand = 0        # 설 자리를 고쳐 다시 간 횟수
        self._job_repose = 0
        self.map_meta = {}
        self._map_seq = 0
        # 지금 들고 있는 지도의 출처. "live" = SLAM 이 발행 중, "saved" = 저장된
        # 지도를 메인 맵으로 채택한 것. SLAM 이 내려갔을 때 무엇을 지우고
        # 무엇을 남길지 가르는 기준이다.
        self.map_source = None
        self.last_saved = None      # 이번 세션에서 마지막으로 저장한 이름
        # 지금 띄운 SLAM 의 성격.
        #   "mapping"      - 지도를 그린다 (async_slam_toolbox_node)
        #   "localization" - 저장된 지도 위에서 위치만 추정한다. 지도는 안 늘어난다
        # 대시보드/매핑 화면에 로봇 위치를 찍으려면 map->odom TF 가 필요하고,
        # 그건 SLAM 이 떠 있어야 나온다. 그래서 매핑이 끝나도 위치추정용으로는
        # 계속 띄워 둔다.
        self.slam_mode = None
        self._lik_cache = None   # (map seq, 거리장) 캐시

        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        # ⚠ 방향 유지에는 /odom 을 쓰면 안 된다. 바퀴 오도메트리는 미끄러져서
        #   돈 것을 못 본다(실측: 엔코더로 -0.25도인데 실제 -1.76도).
        #   EKF 출력은 자이로가 들어가 있어 미끄러진 회전까지 본다.
        #   지도 자세(map)는 SLAM 보정이 계단으로 들어와 조향이 튄다.
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self._on_ekf, 20)
        self.create_subscription(LaserScan, "/scan", self._on_scan, best)
        # 카메라 장애물 스캔 (라이다 높이 밖 가까운 물건). 제자리 회전 공간 검사에 같이 쓴다.
        self._cam_obst = {}
        for key, topic in (("down", "/camera/obstacle_scan"),
                           ("front", "/camera/obstacle_scan_front")):
            self.create_subscription(
                LaserScan, topic,
                (lambda k: (lambda m: self._cam_obst.__setitem__(k, (m, time.monotonic()))))(key),
                best)
        # 자이로 — 도킹 중 캐스터가 차체를 돌리는 것을 도는 순간 받아친다
        self._gyro_z, self._gyro_t = 0.0, 0.0
        self.create_subscription(Imu, "/imu/base_link", self._on_imu, best)
        self.create_subscription(DiagnosticArray, "/diagnostics",
                                 self._on_diag, 10)
        # /map 은 latched(TRANSIENT_LOCAL) 로 발행된다. QoS 를 맞추지 않으면
        # 이미 나온 맵을 못 받고 다음 갱신까지 빈 화면이 된다.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.tf_buffer = Buffer()
        # ⚠ 2026-11-23 '저속 출발 중 위치(odom) 없음 (TF 가 1.5초 전 것)' 으로 왕복이 다섯 번 멈췄다. 밖에서 재면 TF 시각차는
        #   0.02초 이하, api_node 주 실행기 20Hz 타이머도 멀쩡히 돌았다. /tf 는 초당 85개(map->odom 50, odom 25, 바퀴 20)인데
        #   rclpy 는 실행기가 한 번 돌 때 구독마다 메시지를 하나만 꺼낸다 — 주 실행기가 조금만 바빠도 /tf 대기열(100개)이 차서
        #   오래된 것부터 처리하느라 버퍼의 최신 TF 가 1초 넘게 밀렸다. TF 는 전용 노드·전용 스레드에서 받는다
        #   (node=None 이면 tf2_ros 가 따로 노드를 만든다. self 를 넘기면 두 실행기가 같은 노드를 돌게 되어 안 됨).
        # ⚠ 위 전용 스레드만으로는 안 고쳐졌다(같은 날 계측). 진짜 원인: 카메라 원본 영상(장당 0.9MB, 두 대)이 api_node 의
        #   UDP 수신 소켓(net.core.rmem_max 212KB 기본값)을 넘쳐 쉬고 있어도 초당 24개 패킷을 버린다. /tf 도 같은 소켓으로
        #   오는데 RELIABLE 이라 잃은 패킷을 재전송받을 때까지 뒤 메시지를 안 넘겨준다 -> 0.3~1.5초 아무것도 안 오다 몰아서 옴
        #   (그 순간 모든 스레드는 놀고 있었고, 다른 프로세스는 정상). /tf 는 BEST_EFFORT 로 받는다 — 하나 잃으면 버리고
        #   다음 것을 받는다(초당 25~50개라 상관없음). 근본은 rmem_max 를 키우는 것(관리자 권한 필요).
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True, qos=tf_qos)

        # 대차 인식 결과. cart_detector 가 laser 프레임으로 내보내므로
        # 화면 지도에 그리려면 map 좌표로 옮겨 줘야 한다.
        self.carts = None
        self.carts_t = 0.0
        self._job_cart_tf_cache = {}
        self._carts_tf = (None, None)   # 직전에 성공한 TF (map←base, base←laser)
        self.create_subscription(StringMsg, "/carts", self._on_carts, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 50)
        # 배터리 BMS (타보스, CAN2 -> STM32 -> base_node /battery_detail)
        self.bat = None
        self.bat_t = 0.0
        self.bat_hist = deque(maxlen=self.BAT_HIST_MAX)
        self._bat_hist_load()
        self.create_subscription(StringMsg, "/battery_detail", self._on_bat, 10)
        # 리프팅 상태 (base_node /lift_state, 2026-10-07)
        self.lift = None
        self.lift_t = 0.0
        self.create_subscription(StringMsg, "/lift_state", self._on_lift, 10)
        self.create_timer(self.BAT_HIST_S, self._bat_sample)
        self.create_timer(self.BAT_SAVE_S, self._bat_hist_save)
        if HAS_IMAGING:
            self.create_subscription(Image, "/camera/camera/color/image_raw",
                                     self._on_image, best)
            # 아래 카메라 컬러 (2026-11-13, 화면에 두 카메라를 반반으로)
            self.create_subscription(Image, "/camera_down/camera_down/color/image_raw",
                                     lambda m: self._on_image(m, "down"), best)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        # 반경 제한 없는 지령 (base_node). 캐스터 미리 뒤집기에서만 쓴다.
        self.pub_twist_free = self.create_publisher(
            Twist, "/cmd_vel_noradius", 10)
        # nav2 속도 제한 (controller_server 가 듣는다). 출발 'nav2' 방식에서 쓴다.
        self.pub_speed_limit = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.depart_mode = "nav2"       # "nav2"(정렬 뒤 nav2 저속, 기본) | "creep"(예전: api_node 가 기어감)
        self._slow_timer = None
        # 위치 재조정용. slam_toolbox 위치추정 모드가 /initialpose 를 구독한다.
        # nav2 경로계획용 지도. slam_toolbox 의 /map 에는 가상벽이 없다
        # (일부러 뺐다 - 라이다가 볼 수 없는 벽을 위치추정 지도에 넣으면
        #  맞는 자세의 점수가 깎인다). 그래서 경로계획에는 가상벽을 얹은
        # 지도를 따로 내보낸다. nav2 global costmap 이 이것을 구독한다.
        _qos_map = QoSProfile(depth=1)
        _qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        _qos_map.reliability = ReliabilityPolicy.RELIABLE
        self.pub_map_nav = self.create_publisher(
            OccupancyGrid, "/map_nav", _qos_map)
        self._map_nav_seq = -1
        # nav2 로 목표를 보내는 통로. nav2 가 안 떠 있으면 그냥 실패로 답한다.
        self.nav_cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 2026-11-17 지점 POI 로 가기 전에 경로가 들어오는 방향을 본다 (narrow_poi.sp_request)
        from nav2_msgs.action import ComputePathToPose
        self.plan_cli = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        # 목표 앞에서 nav2 를 먼저 끊는다 (_nav_watch 설명)
        self.create_timer(1.0 / self.NAV_WATCH_HZ, self._nav_watch)
        # 막혀 선 원인 기록 · 20초 알림 (_stop_watch 설명)
        self.create_timer(0.5, self._stop_watch)
        # 주 실행기 멈춤 감시 (_stall_watch 설명)
        self._hb = time.monotonic()
        self.create_timer(0.1, self._hb_tick)
        threading.Thread(target=self._stall_watch, daemon=True).start()
        self._tfw_setup()
        # 지점 왕복 (대시보드). 브라우저를 닫아도 돌도록 여기서 돈다.
        self.route = {"state": "idle", "message": ""}
        self.create_timer(self.ROUTE_TICK_S, self._route_tick)
        # 충전소 후진 도킹 (charger_dock.py, 2026-11-12)
        self.cdock = {"state": "idle"}
        # ⚠ 2026-11-13 nav_seq 는 첫 이동 명령에서야 생겼다. 재시작 뒤 첫 명령이 충전소
        #   도킹(이미 보여 nav2 를 안 씀)이면 도킹 타이머가 없는 nav_seq 를 읽고 api_node 가
        #   죽어 화면이 새로고침됐다(사용자: "도킹 넣으면 새로고침되는데"). 처음부터 둔다.
        self.nav_seq = getattr(self, "nav_seq", 0)
        self.cleave = {"state": "idle"}
        self.create_timer(1.0 / self.CD_HZ, self._cdock_tick)
        # 도착 뒤 방향만 제자리에서 맞춘다. 맵 기준 닫힌 루프로 돈다.
        self._turning = False
        self._turn_timer = None
        self._turn_pass = 0
        self._turn_ok_n = self.TURN_OK
        self._turn_max_n = self.TURN_PASS_MAX
        self._turn_phase = "settle"
        self._turn_dir = 1.0
        self._turn_t = 0.0
        self.nav_state = "idle"     # idle / sending / active / done / failed / canceled
        self.nav_msg = ""
        self.nav_goal = None        # 진행 중인 goal handle
        self.nav_target = None      # (x, y, yaw)
        # 자율주행 속도. 저장해 두고 nav2 가 뜰 때마다 다시 밀어 넣는다.
        self._spd_cli = None
        self._speed_sent = False
        self.ekf_yaw = None
        self._jog = None          # (시각, 전진, 회전) 수동 지령
        self._jog_ref = None      # 붙들고 있는 방향 [rad]
        self.nav_speed = self._speed_load()
        self.create_timer(5.0, self._speed_watch)
        self.create_timer(1.0 / self.JOG_HZ, self._jog_tick)
        # nav2 가 낸 경로. 화면에 그려 주면 로봇이 어디로 갈 작정인지 보인다.
        self.nav_path = []
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.pub_initpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        # 수동 위치 고정 (2026-11-13): EKF(odom->base) 자세를 직접 넣는다.
        self.pub_ekf_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/set_pose", 10)
        self.loc_hold = None
        self.srv = {n: self.create_client(Trigger, "/base_node/" + n)
                    for n in ("enable", "release", "clear_errors",
                              "stop_now", "lift_up", "lift_down",
                              "lift_stop")}
        # 도킹의 보정 가능량과 적분 상한도 실제 모터 제한을 따라야 한다.
        self._base_limits = None
        self._base_limits_t = 0.0
        self._base_limits_future = None
        self._base_limits_cli = self.create_client(
            GetParameters, "/base_node/get_parameters")
        self.create_timer(2.0, self._job_refresh_base_limits)
        # 카메라는 드라이버가 진짜 하드웨어 리셋 서비스를 제공한다.
        self.cam_reset_cli = self.create_client(
            EmptySrv, "/camera/camera/hw_reset")

        # slam_toolbox 서비스. 노드가 없으면 클라이언트만 만들어지고
        # 호출 시 "SLAM 미실행"으로 응답한다.
        self.slam = {}
        if HAS_SLAM:
            self.slam = {
                "save": self.create_client(SaveMap, "/slam_toolbox/save_map"),
                # 위치를 재조정하면 이전 위치에서 쌓인 스캔 버퍼를 비워야
                # 새 위치와 싸우지 않는다.
                "clear_buffer": self.create_client(
                    EmptySrv, "/slam_toolbox/clear_localization_buffer"),
                "serialize": self.create_client(
                    SerializePoseGraph, "/slam_toolbox/serialize_map"),
                "deserialize": self.create_client(
                    DeserializePoseGraph, "/slam_toolbox/deserialize_map"),
                "pause": self.create_client(Pause,
                                            "/slam_toolbox/pause_new_measurements"),
                "clear": self.create_client(Clear, "/slam_toolbox/clear_changes"),
            }
        self.mapping_paused = False
        # SLAM 노드는 api_node 가 자식 프로세스로 직접 띄우고 내린다.
        # slam_toolbox 에 '지도 비우기' 서비스가 없어 백지 재시작에 필요하다.
        self.slam_proc = None
        self._slam_busy = 0         # 띄우기·내리기 중이면 감시가 손대지 않는다
        self._slam_busy_lock = threading.Lock()
        self._slam_restarts = []
        self.create_timer(1.0, self._slam_watch)
        self._smoother_lock = threading.Lock()
        self.create_timer(5.0, self._smoother_ensure)
        self.active_map = None      # 현재 SLAM 에 적용된 지도 이름
        self.resumed_from = None    # '이어서 매핑' 을 시작한 지도 이름
        # 매핑 세션 기록: rosbag 프로세스와 주행 통계
        self.bag_proc = None
        self.bag_tmp = os.path.join(self.map_dir, ".session_bag")
        self.sess = None
        try:
            self.slam_cfg = os.path.join(
                get_package_share_directory("und_amr_bringup"),
                "config", "slam.yaml")
        except Exception:                                    # noqa: BLE001
            self.slam_cfg = ""

        self.get_logger().info("REST API : http://0.0.0.0:%d" % self.port)

        # 지난 메인 맵 복구는 서비스/TF 가 자리잡은 뒤에 한 번만 돌린다.
        self._restore_timer = self.create_timer(3.0, self._restore_once)
        # 붙을 WiFi 가 없으면 핫스팟으로. 기동 뒤 한 번만 본다.
        # 무선이 붙는 데 시간이 걸리므로 넉넉히 기다린다.
        self.declare_parameter("ap_fallback", True)
        self.declare_parameter("ap_fallback_sec", 45.0)
        self.ap_fallback = bool(self.get_parameter("ap_fallback").value)
        self.ap_fallback_done = None
        self._ap_timer = self.create_timer(
            max(10.0, float(self.get_parameter("ap_fallback_sec").value)),
            self._ap_fallback_once)
        self._pose_line = None
        # 1초마다 지금 자리를 남긴다. 값이 안 바뀌면 쓰지 않는다.
        # ⚠ 5 -> 1초. 달리는 중에 노드를 다시 띄웠더니 마지막 저장 뒤 약 1m 를
        #   더 간 상태라, 위치추정이 1m 뒤에서 시작해 스스로 못 고쳤다(2026-10-02,
        #   0.35 m/s 로 5초면 1.75m). 꺼질 때도 한 번 남긴다(main).
        self.create_timer(1.0, self._remember_pose)
        # 위치 감시 기록 (logs/loc_monitor.csv). 2026-11-13 주행 중 위치 틀어짐 원인 잡기용.
        threading.Thread(target=self._locmon_loop, daemon=True).start()
        # 라이다 감시. 스캔이 끊기면 로봇은 자기 위치를 잃은 채 계속 간다.
        # collision_monitor 도 스캔이 죽으면 앞을 못 보므로 못 막는다.
        # 실제로 오늘 라이다가 두 번 죽었다(2026-09-11).
        self._scan_t = 0.0
        self._scan_n = 0
        self._scan_dead = False
        self._watch_t = 0.0
        self.create_timer(0.2, self._scan_watch)

    def _restore_once(self):
        self._restore_timer.cancel()
        try:
            self.restore_active_map()
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("메인 맵 복구 실패: %s" % e)

    # ---------------- 구독 ----------------

    def _on_odom(self, m):
        with self.lock:
            self.odom = m
            self._sess_travel(m)
        # 마지막으로 방향을 바꾼 뒤 한 방향으로 간 거리 (캐스터 예측 보정용)
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - getattr(self, "_run_t", now)))
        self._run_t = now
        v = m.twist.twist.linear.x
        if abs(v) > 0.005:
            d = 1 if v > 0 else -1
            if d != getattr(self, "_run_dir", 0):
                self._run_dir, self._run_m = d, 0.0
            self._run_m += abs(v) * dt
        # 마지막 제자리 회전 방향 (+1 왼쪽 / -1 오른쪽 / 0 없음). 한 번에 2도 넘게 돈 것만 치고,
        # 그 뒤 0.10m 움직이면 지운다. 출발 비틀림 미리 받기(narrow_poi SP_KICK_W)가 쓴다.
        w = m.twist.twist.angular.z
        if abs(v) < 0.02 and abs(w) > 0.05:
            sg = 1 if w > 0 else -1
            if sg != getattr(self, "_spin_cur", 0):
                self._spin_cur, self._spin_ang = sg, 0.0
            self._spin_ang += abs(w) * dt
            if self._spin_ang >= math.radians(2.0):
                self._spin_sign, self._spin_move = sg, 0.0
        else:
            self._spin_cur = 0
            if abs(v) > 0.005:
                self._spin_move = getattr(self, "_spin_move", 0.0) + abs(v) * dt
                if self._spin_move > 0.10:
                    self._spin_sign = 0

    def _on_carts(self, m):
        try:
            with self.lock:
                self.carts = json.loads(m.data)
                self.carts_t = time.time()
        except (ValueError, TypeError):
            pass

    # -------- 새 대차 규격 재기 --------
    #
    # carts.yaml 에 넣는 값은 줄자로 잰 외곽 치수가 아니라 '라이다가 보는
    # 다리 중심 간 거리' 다. 도면값을 그대로 넣었다가 95mm 어긋나 인식률이
    # 11% 로 깜빡인 적이 있다(2026-08-28 S300). 그래서 실측해서 넣는다.

    def cart_unregister(self, name):
        """등록된 대차를 목록에서 뺀다. 잘못 잰 것을 지울 데가 있어야 한다."""
        safe = "".join(ch for ch in (name or "") if ch.isalnum() or ch in "-_")
        if not safe:
            return False, "이름이 없습니다"
        paths = self._carts_yaml_paths()
        if not paths:
            return False, "carts.yaml 을 찾지 못했습니다"
        hit = False
        for path in paths:
            try:
                with open(path, encoding="utf-8") as f:
                    txt = f.read()
                new, gone = self._yaml_drop_cart(txt, safe)
                if gone:
                    hit = True
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(new)
            except OSError as e:                             # noqa: BLE001
                return False, "%s 를 고치지 못했습니다: %s" % (path, e)
        if not hit:
            return False, "'%s' 는 등록돼 있지 않습니다" % safe
        try:
            subprocess.run(["pkill", "-f", "lib/und_amr_bringup/cart_detector"],
                           capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            pass
        self.get_logger().info("대차 '%s' 등록 해제" % safe)
        return True, "'%s' 를 목록에서 뺐습니다 — 검출기를 다시 띄웁니다" % safe

    @staticmethod
    def _yaml_drop_cart(text, name):
        """주석을 살린 채 대차 한 종을 지운다."""
        lines = text.split("\n")
        gone = False
        for i, ln in enumerate(lines):
            st = ln.strip()
            if st.startswith("carts:") and "[" in st:
                inner = st[st.index("[") + 1:st.rindex("]")]
                names = [t.strip().strip('"\'') for t in inner.split(",")
                         if t.strip()]
                if name in names:
                    names.remove(name)
                    gone = True
                pad = ln[:len(ln) - len(ln.lstrip())]
                lines[i] = "%scarts: [%s]" % (
                    pad, ", ".join('"%s"' % n for n in names))
                break
        head = "    %s:" % name
        st = next((i for i, ln in enumerate(lines) if ln.rstrip() == head), None)
        if st is not None:
            gone = True
            en = st + 1
            while en < len(lines) and (lines[en].startswith("      ")
                                       or not lines[en].strip()):
                en += 1
            # 블록 바로 위에 붙은 설명 주석도 같이 지운다
            while st > 0 and lines[st - 1].strip().startswith("#"):
                st -= 1
            lines = lines[:st] + lines[en:]
        return "\n".join(lines), gone

    # ---------------- 네트워크 설정 ----------------
    #
    # 현장에 나가면 공유기가 바뀐다. 그때마다 로봇에 모니터를 붙여
    # nmcli 를 치는 대신 관리자 화면에서 바꾸게 한다.
    #
    # ⚠ 네트워크를 바꾸면 지금 이 화면을 보고 있는 연결이 끊긴다. 그래서
    #   적용은 '해보고 안 되면 되돌리기' 로 한다. 오타 하나로 로봇에
    #   아무도 못 들어가는 일이 없어야 한다.

    WIFI_DEV = "wlp3s0"

    @staticmethod
    def _nm(args, timeout=25):
        """nmcli 를 부른다. (성공?, 출력, 오류)"""
        try:
            r = subprocess.run(["nmcli"] + args, capture_output=True,
                               text=True, timeout=timeout)
        except (OSError, subprocess.SubprocessError) as e:
            return False, "", str(e)
        err = (r.stderr or "").strip()
        if r.returncode != 0:
            # 권한이 없으면 무엇을 해야 하는지 그대로 알려 준다.
            if "not authorized" in err.lower() or "privileges" in err.lower():
                err = ApiNode.NEED_AUTH
            return False, (r.stdout or "").strip(), err
        return True, (r.stdout or "").strip(), err

    @staticmethod
    def _tsplit(line):
        """nmcli -t 출력은 ':' 를 구분자로 쓰고 값 안의 ':' 는 백슬래시로
        피해 둔다. 그 규칙대로 자른다."""
        out, cur, esc = [], "", False
        for ch in line:
            if esc:
                cur += ch
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == ":":
                out.append(cur)
                cur = ""
            else:
                cur += ch
        out.append(cur)
        return out

    def net_status(self):
        """지금 어디에 어떻게 붙어 있는가."""
        out = {"wifi_dev": self.WIFI_DEV, "devices": [], "hint": None,
               "ap": self.ap_info()}
        ok, txt, err = self._nm(["-t", "-f",
                                 "DEVICE,TYPE,STATE,CONNECTION", "dev"], 10)
        if not ok:
            out["hint"] = err or "nmcli 를 부를 수 없습니다"
            return out
        for ln in txt.splitlines():
            f = self._tsplit(ln)
            # loopback 과 wifi-p2p 는 가상 장치라 화면에 낼 것이 없다
            if len(f) < 4 or f[1] in ("loopback", "wifi-p2p"):
                continue
            d = {"device": f[0], "type": f[1], "state": f[2],
                 "connection": f[3] or None, "ip": None, "gateway": None,
                 "method": None, "ssid": None}
            ok2, t2, _ = self._nm(["-t", "-f",
                                   "IP4.ADDRESS,IP4.GATEWAY,GENERAL.CONNECTION",
                                   "dev", "show", f[0]], 10)
            if ok2:
                for l2 in t2.splitlines():
                    k, _, v = l2.partition(":")
                    if k.startswith("IP4.ADDRESS") and v:
                        d["ip"] = v
                    elif k == "IP4.GATEWAY" and v and v != "--":
                        d["gateway"] = v
            if d["connection"]:
                ok3, t3, _ = self._nm(["-t", "-f",
                                       "ipv4.method,802-11-wireless.ssid",
                                       "con", "show", d["connection"]], 10)
                if ok3:
                    for l3 in t3.splitlines():
                        k, _, v = l3.partition(":")
                        if k == "ipv4.method":
                            d["method"] = v
                        elif k.endswith(".ssid") and v and v != "--":
                            d["ssid"] = v
            out["devices"].append(d)
        return out

    NEED_AUTH = ("네트워크를 다루려면 권한을 한 번 열어야 합니다. "
                 "로봇에서 실행하세요:  ~/projects/UND_AMR/tools/"
                 "allow_network_config.sh")

    def wifi_scan(self, rescan=True):
        """주변 WiFi 목록. 신호 센 순으로.

        주변을 다시 훑는 것(rescan)도 권한이 필요하다. 권한이 없으면
        지금 붙어 있는 AP 하나만 보이는데, 그걸 '주변에 이것뿐' 으로
        읽으면 안 되므로 사유를 분명히 돌려준다.
        """
        need_auth = False
        if rescan:
            ok, _, e = self._nm(["dev", "wifi", "rescan"], 20)
            if not ok and ("not authorized" in (e or "").lower()
                           or "권한" in (e or "")):
                need_auth = True
            else:
                time.sleep(3.0)
        ok, txt, err = self._nm(["-t", "-f",
                                 "IN-USE,SSID,SIGNAL,SECURITY,FREQ",
                                 "dev", "wifi", "list"], 20)
        if not ok:
            return False, err or "WiFi 목록을 못 읽었습니다", []
        if need_auth:
            return False, self.NEED_AUTH, []
        seen, out = set(), []
        for ln in txt.splitlines():
            f = self._tsplit(ln)
            if len(f) < 5 or not f[1]:
                continue
            if f[1] in seen:                 # 같은 SSID 는 가장 센 것만
                continue
            seen.add(f[1])
            try:
                sig = int(f[2])
            except ValueError:
                sig = 0
            out.append({"ssid": f[1], "signal": sig,
                        "security": f[3] or "열림",
                        "freq": f[4], "active": f[0].strip() == "*"})
        out.sort(key=lambda d: -d["signal"])
        return True, "%d 개" % len(out), out

    def _ip_args(self, ipcfg):
        """고정 IP 설정을 nmcli 인자로. 잘못된 값은 여기서 막는다."""
        if not ipcfg or ipcfg.get("mode") != "manual":
            return ["ipv4.method", "auto", "ipv4.addresses", "",
                    "ipv4.gateway", "", "ipv4.dns", ""], None
        addr = str(ipcfg.get("address", "")).strip()
        try:
            prefix = int(ipcfg.get("prefix", 24))
        except (TypeError, ValueError):
            return None, "넷마스크 길이가 숫자가 아닙니다"
        gw = str(ipcfg.get("gateway", "")).strip()
        dns = str(ipcfg.get("dns", "")).strip()

        def ok_ip(v):
            p4 = v.split(".")
            return (len(p4) == 4 and all(q.isdigit() and 0 <= int(q) <= 255
                                         for q in p4))
        if not ok_ip(addr):
            return None, "IP 주소 형식이 잘못됐습니다 (예: 192.168.30.50)"
        if not (1 <= prefix <= 32):
            return None, "넷마스크 길이는 1~32 입니다"
        if gw and not ok_ip(gw):
            return None, "게이트웨이 형식이 잘못됐습니다"
        for d in [x.strip() for x in dns.split(",") if x.strip()]:
            if not ok_ip(d):
                return None, "DNS 형식이 잘못됐습니다: %s" % d
        args = ["ipv4.method", "manual",
                "ipv4.addresses", "%s/%d" % (addr, prefix)]
        args += ["ipv4.gateway", gw]
        args += ["ipv4.dns", dns.replace(",", " ").strip()]
        return args, None

    def _wait_online(self, dev, want_ip=None, timeout=45.0):
        """장치가 붙고 주소를 받을 때까지 기다린다."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            ok, txt, _ = self._nm(["-t", "-f", "GENERAL.STATE,IP4.ADDRESS",
                                   "dev", "show", dev], 8)
            if ok:
                state = ip = ""
                for ln in txt.splitlines():
                    k, _, v = ln.partition(":")
                    if k == "GENERAL.STATE":
                        state = v
                    elif k.startswith("IP4.ADDRESS") and not ip:
                        ip = v
                if "100" in state and ip:
                    if want_ip is None or ip.split("/")[0] == want_ip:
                        return True, ip
            time.sleep(1.5)
        return False, None

    def wifi_connect(self, ssid, password=None, ipcfg=None, revert=True):
        """WiFi 에 붙는다. 안 되면 원래대로 되돌린다.

        되돌리기가 이 기능의 핵심이다. 현장에서 암호 오타 하나로 로봇에
        아무도 못 들어가는 일이 없어야 한다.
        """
        ssid = (ssid or "").strip()
        if not ssid:
            return False, "SSID 가 비었습니다", None
        ipargs, err = self._ip_args(ipcfg)
        if err:
            return False, err, None

        # 지금 붙어 있는 연결. 실패하면 여기로 돌아온다.
        before = None
        ok, txt, _ = self._nm(["-t", "-f", "NAME,DEVICE", "con", "show",
                               "--active"], 10)
        if ok:
            for ln in txt.splitlines():
                f = self._tsplit(ln)
                if len(f) >= 2 and f[1] == self.WIFI_DEV:
                    before = f[0]
                    break

        # 같은 이름의 프로필이 있으면 고쳐 쓰고, 없으면 만든다
        exists = False
        ok, txt, _ = self._nm(["-t", "-f", "NAME", "con", "show"], 10)
        if ok:
            exists = ssid in [self._tsplit(l)[0] for l in txt.splitlines()]
        if not exists:
            ok, _, e = self._nm(["con", "add", "type", "wifi",
                                 "con-name", ssid, "ifname", self.WIFI_DEV,
                                 "ssid", ssid], 20)
            if not ok:
                return False, "연결 프로필을 만들지 못했습니다: %s" % e, None
        mod = ["con", "modify", ssid,
               "connection.autoconnect", "yes"] + ipargs
        if password:
            mod += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password]
        ok, _, e = self._nm(mod, 20)
        if not ok:
            return False, "설정을 저장하지 못했습니다: %s" % e, None

        want = None
        if ipcfg and ipcfg.get("mode") == "manual":
            want = str(ipcfg.get("address", "")).strip()
        self._nm(["con", "up", ssid], 45)
        online, ip = self._wait_online(self.WIFI_DEV, want, 45.0)
        if online:
            self.get_logger().info("WiFi '%s' 접속 — %s" % (ssid, ip))
            return True, ("'%s' 에 접속했습니다. 새 주소는 %s 입니다 — "
                          "이 주소로 다시 들어오세요" % (ssid, ip)), {"ip": ip}
        # 실패 - 되돌린다
        msg = "'%s' 에 붙지 못했습니다 (암호나 신호를 확인하세요)" % ssid
        if revert and before and before != ssid:
            self._nm(["con", "up", before], 45)
            back, bip = self._wait_online(self.WIFI_DEV, None, 30.0)
            msg += " — '%s' 로 되돌렸습니다%s" % (
                before, " (%s)" % bip if back else " (복구 실패, 로봇 확인 필요)")
        self.get_logger().warn(msg)
        return False, msg, None

    def net_set_ip(self, name, ipcfg):
        """이미 있는 연결의 주소 방식만 바꾼다 (DHCP <-> 고정)."""
        name = (name or "").strip()
        if not name:
            return False, "연결 이름이 비었습니다", None
        ipargs, err = self._ip_args(ipcfg)
        if err:
            return False, err, None
        ok, _, e = self._nm(["con", "modify", name] + ipargs, 20)
        if not ok:
            return False, "설정을 저장하지 못했습니다: %s" % e, None
        self._nm(["con", "up", name], 45)
        want = (str(ipcfg.get("address", "")).strip()
                if ipcfg and ipcfg.get("mode") == "manual" else None)
        online, ip = self._wait_online(self.WIFI_DEV, want, 45.0)
        if online:
            self.get_logger().info("'%s' 주소 변경 — %s" % (name, ip))
            return True, ("적용했습니다. 새 주소는 %s 입니다 — 이 주소로 "
                          "다시 들어오세요" % ip), {"ip": ip}
        # 되돌린다 - 고정 IP 를 잘못 넣으면 그대로 고립된다
        self._nm(["con", "modify", name, "ipv4.method", "auto",
                  "ipv4.addresses", "", "ipv4.gateway", "", "ipv4.dns", ""], 20)
        self._nm(["con", "up", name], 45)
        back, bip = self._wait_online(self.WIFI_DEV, None, 30.0)
        return False, ("적용했지만 주소를 받지 못해 DHCP 로 되돌렸습니다%s"
                       % (" (%s)" % bip if back else " (복구 실패, 로봇 확인 필요)")), None

    # ---------------- 핫스팟(AP) ----------------
    #
    # 공유기가 없는 현장에서 로봇에 직접 붙기 위한 모드. WiFi 와 '동시에'
    # 돌릴 수도 있지만(mt7921 은 가능) 그때는 AP 가 클라이언트와 같은
    # 채널이어야만 뜬다 - 2026-08-28 실측에서 다른 채널로 요청하면
    # supplicant-timeout 으로 죽었다. 붙는 공유기가 바뀌면 채널도 바뀌므로
    # 그 방식은 깨지기 쉽다.
    #
    # 그래서 둘 중 하나만 쓴다. WiFi 에 붙으면 핫스팟은 꺼지고, 붙을 데가
    # 없으면 핫스팟이 뜬다. 채널 제약이 사라지고 구성이 단순해진다.

    AP_CON = "und-ap"

    def ap_fallback_check(self):
        """기동 뒤 한 번, 붙을 WiFi 가 없으면 핫스팟을 켠다.

        공유기가 없는 현장에서 로봇에 모니터를 붙이지 않고도 들어올 수 있어야
        한다. 다만 이미 붙어 있으면 절대 건드리지 않는다 -
        멀쩡한 연결을 끊는 것이 가장 나쁘다.

        기동 직후 한 번만 본다. 운영 중에 잠깐 끊겼다고 핫스팟으로 바꿔 버리면
        공유기가 재부팅되는 사이 로봇이 사라진다. 그때는 NetworkManager 가
        알아서 다시 붙는 편이 낫다.
        """
        if not self.ap_fallback:
            return
        if self.ap_mode() != "none":
            self.get_logger().info("기동 점검 — 이미 연결돼 있어 그대로 둡니다")
            return
        ok, txt, _ = self._nm(["-t", "-f", "NAME,TYPE", "con", "show"], 10)
        if not ok or self.AP_CON not in [self._tsplit(l)[0]
                                         for l in txt.splitlines()]:
            self.get_logger().warn("붙을 WiFi 가 없는데 핫스팟 프로필도 없습니다")
            return
        self.get_logger().info("붙을 WiFi 가 없어 핫스팟으로 전환합니다")
        ok, msg, _ = self.net_set_mode("ap")
        self.ap_fallback_done = msg if ok else ("실패: %s" % msg)
        self.get_logger().info("기동 점검 — %s" % self.ap_fallback_done)

    def _ap_fallback_once(self):
        """기동 뒤 한 번만 돈다."""
        self._ap_timer.cancel()
        try:
            self.ap_fallback_check()
        except Exception as e:                                # noqa: BLE001
            self.get_logger().warn("기동 점검 실패: %s" % e)

    def ap_mode(self):
        """지금 무선이 어느 모드인가."""
        ok, txt, _ = self._nm(["-t", "-f", "NAME,DEVICE", "con", "show",
                               "--active"], 10)
        if ok:
            for ln in txt.splitlines():
                f = self._tsplit(ln)
                if len(f) >= 2 and f[1] == self.WIFI_DEV:
                    return "ap" if f[0] == self.AP_CON else "wifi"
        return "none"

    def ap_info(self):
        """핫스팟 설정값. 암호는 있는지만 알린다."""
        d = {"connection": self.AP_CON, "ssid": None, "band": None,
             "has_password": False, "address": None,
             "mode": self.ap_mode(), "clients": 0,
             "fallback": self.ap_fallback, "fallback_done": self.ap_fallback_done}
        ok, txt, _ = self._nm(["-t", "-f",
                               "802-11-wireless.ssid,802-11-wireless.band,"
                               "802-11-wireless-security.key-mgmt",
                               "con", "show", self.AP_CON], 10)
        if not ok:
            d["missing"] = True
            return d
        for ln in txt.splitlines():
            k, _, v = ln.partition(":")
            if k.endswith(".ssid"):
                d["ssid"] = v or None
            elif k.endswith(".band"):
                d["band"] = v or None
            elif k.endswith(".key-mgmt"):
                d["has_password"] = bool(v and v != "--")
        if d["mode"] == "ap":
            ok2, t2, _ = self._nm(["-t", "-f", "IP4.ADDRESS", "dev", "show",
                                   self.WIFI_DEV], 8)
            if ok2 and t2:
                d["address"] = t2.split(":", 1)[-1].split("/")[0]
            try:
                with open("/var/lib/NetworkManager/dnsmasq-%s.leases"
                          % self.WIFI_DEV) as f:
                    d["clients"] = sum(1 for _ in f)
            except OSError:
                pass
        return d

    def ap_set(self, ssid=None, password=None, band=None):
        """핫스팟 이름·암호를 바꾼다. 켜져 있으면 다시 올려 반영한다."""
        args = []
        if ssid is not None:
            ssid = str(ssid).strip()
            if not (1 <= len(ssid.encode("utf-8")) <= 32):
                return False, "SSID 는 1~32 바이트여야 합니다"
            args += ["802-11-wireless.ssid", ssid]
        if password is not None:
            password = str(password)
            if not (8 <= len(password) <= 63):
                return False, "암호는 8~63 자여야 합니다 (WPA2 규격)"
            args += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password]
        if band in ("bg", "a"):
            args += ["802-11-wireless.band", band]
        if not args:
            return False, "바꿀 값이 없습니다"
        ok, _, e = self._nm(["con", "modify", self.AP_CON] + args, 20)
        if not ok:
            return False, "설정을 저장하지 못했습니다: %s" % e
        msg = "핫스팟 설정을 저장했습니다"
        if self.ap_mode() == "ap":
            self._nm(["con", "up", self.AP_CON], 45)
            msg += " (다시 올렸습니다 — 붙어 있던 기기는 다시 접속하세요)"
        self.get_logger().info(msg)
        return True, msg

    def net_set_mode(self, mode, ssid=None):
        """핫스팟 <-> WiFi 전환. 무선 하나로 둘 중 하나만 쓴다."""
        cur = self.ap_mode()
        if mode == "ap":
            if cur == "ap":
                return False, "이미 핫스팟 모드입니다", None
            ok, _, e = self._nm(["con", "up", self.AP_CON], 60)
            if not ok:
                return False, "핫스팟을 켜지 못했습니다: %s" % e, None
            online, ip = self._wait_online(self.WIFI_DEV, None, 30.0)
            addr = (ip or "10.42.0.1/24").split("/")[0]
            self.get_logger().info("핫스팟 모드로 전환 — %s" % addr)
            return True, ("핫스팟을 켰습니다. WiFi 는 끊겼습니다. "
                          "'%s' 에 붙어 http://%s:8090 으로 들어오세요"
                          % (self.ap_info().get("ssid") or "UND_AMR", addr)), \
                {"address": addr}
        if mode == "wifi":
            target = (ssid or "").strip()
            if not target:
                # 아는 무선 프로필 중 핫스팟이 아닌 것을 고른다
                ok, txt, _ = self._nm(["-t", "-f", "NAME,TYPE", "con",
                                       "show"], 10)
                if ok:
                    for ln in txt.splitlines():
                        f = self._tsplit(ln)
                        if (len(f) >= 2 and f[1] == "802-11-wireless"
                                and f[0] != self.AP_CON):
                            target = f[0]
                            break
            if not target:
                return False, ("붙을 WiFi 가 없습니다. 먼저 주변 WiFi 를 "
                               "골라 접속하세요"), None
            ok, _, e = self._nm(["con", "up", target], 60)
            if not ok:
                return False, "'%s' 에 붙지 못했습니다: %s" % (target, e), None
            online, ip = self._wait_online(self.WIFI_DEV, None, 45.0)
            if not online:
                # 실패하면 핫스팟으로 돌아간다. 로봇이 고립되면 안 된다.
                self._nm(["con", "up", self.AP_CON], 60)
                return False, ("'%s' 에 붙지 못해 핫스팟으로 되돌렸습니다"
                               % target), None
            addr = ip.split("/")[0]
            self.get_logger().info("WiFi 모드로 전환 — %s (%s)" % (target, addr))
            return True, ("'%s' 에 붙었습니다. 핫스팟은 꺼졌습니다. "
                          "http://%s:8090 으로 들어오세요" % (target, addr)), \
                {"address": addr}
        return False, "mode 는 ap 또는 wifi 여야 합니다", None

    def cart_specs_json(self):
        """등록된 대차 목록. 실측 페이지에서 이미 있는 것과 견주려고 쓴다."""
        paths = self._carts_yaml_paths()
        if not paths:
            return {"available": False, "carts": []}
        try:
            with open(paths[0], encoding="utf-8") as f:
                d = yaml.safe_load(f) or {}
            pr = (d.get("cart_detector") or {}).get("ros__parameters") or {}
        except (OSError, yaml.YAMLError):
            return {"available": False, "carts": []}
        out = []
        for nm in pr.get("carts") or []:
            sp = pr.get(nm) or {}
            if not isinstance(sp, dict):
                continue
            out.append({"name": nm, "width": sp.get("width"),
                        "depth": sp.get("depth"), "tol": sp.get("tol", 0.06),
                        "cam_legs": sp.get("cam_legs", self.JOB_CAM_LEGS),
                        "standoff": sp.get("standoff", self.POI_STANDOFF),
                        "note": sp.get("note", "")})
            for k in ("clearance_width", "center_offset"):
                if k in sp:
                    out[-1][k] = sp[k]
        return {"available": True, "file": paths[0], "carts": out,
                "leg_radius": self.leg_radius}

    def cart_measure(self, seconds=6.0, front=True, name="NEW"):
        """대차를 앞에 두고 몇 초 지켜보며 가로 x 세로를 잰다.

        등록표와 대조하지 않고 '가장 직사각형에 가까운 조합' 을 찾으므로
        처음 보는 대차도 잴 수 있다. 여러 프레임의 중앙값을 쓰고, 흔들림이
        크면 그대로 알려 준다 - 다리 하나가 끊겼다 붙었다 하는 상태에서
        잰 값을 등록하면 그 대차는 계속 깜빡인다.
        """
        if not HAS_IMAGING:
            return False, "numpy 가 없어 잴 수 없습니다", None
        seconds = max(1.0, min(20.0, float(seconds)))
        rows, cand_n, seen = [], [], 0
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < seconds:
            with self.lock:
                m = self.scan
            if m is None or m is last:
                time.sleep(0.02)
                continue
            last = m
            seen += 1
            x, y, groups = cluster_scan(m.ranges, m.angle_min,
                                        m.angle_increment, rmax=3.0, gap=0.05)
            cands = leg_candidates(x, y, groups, radius=self.leg_radius)
            cand_n.append(len(cands))
            if len(cands) < 4:
                continue
            rects = best_rectangles([(c["x"], c["y"]) for c in cands],
                                    front_only=bool(front))
            if rects:
                rows.append(rects[0])
        if seen == 0:
            return False, "라이다 스캔이 들어오지 않습니다", None
        if len(rows) < 3:
            return False, ("사각형을 %d 프레임에서만 찾았습니다 (%d 프레임 관찰). "
                           "대차를 로봇 앞 1~2m 에 홀로 두고 다시 해 보세요"
                           % (len(rows), seen)), {
                "frames": seen, "valid": len(rows),
                "candidates": max(cand_n) if cand_n else 0}
        A = sorted(r["a"] for r in rows)
        B = sorted(r["b"] for r in rows)
        ma, mb = statistics.median(A), statistics.median(B)
        sa = statistics.pstdev(A) if len(A) > 1 else 0.0
        sb = statistics.pstdev(B) if len(B) > 1 else 0.0
        # 흔들리는 만큼은 허용 오차로 흡수하되, 최소 40mm 는 준다.
        tol = max(0.040, round(4 * max(sa, sb), 3))
        safe = "".join(ch for ch in (name or "NEW")
                       if ch.isalnum() or ch in "-_") or "NEW"
        out = {
            "name": safe, "frames": seen, "valid": len(rows),
            "width": round(ma, 4), "depth": round(mb, 4), "tol": tol,
            "spread_w": round(sa, 4), "spread_d": round(sb, 4),
            "range_w": [round(A[0], 4), round(A[-1], 4)],
            "range_d": [round(B[0], 4), round(B[-1], 4)],
            "dist": round(statistics.median([r["dist"] for r in rows]), 3),
            "yaw_deg": round(math.degrees(
                statistics.median([r["yaw"] for r in rows])), 1),
            "candidates": int(statistics.median(cand_n)) if cand_n else 0,
            "shaky": bool(max(sa, sb) > 0.020),
            "yaml": ("    %s:\n      width: %.3f\n      depth: %.3f\n"
                     "      tol: %.3f\n      note: \"실측 %s\""
                     % (safe, ma, mb, tol,
                        datetime.date.today().isoformat())),
        }
        msg = ("%.0f x %.0f mm (흔들림 ±%.0f/±%.0f mm, %d/%d 프레임)"
               % (ma * 1000, mb * 1000, sa * 1000, sb * 1000, len(rows), seen))
        if out["shaky"]:
            msg += " — 흔들림이 큽니다. 대차를 조금 옮겨 다시 재 보세요"
        return True, msg, out

    def _carts_yaml_paths(self):
        """고칠 carts.yaml 들. 설치본과 소스본 둘 다 고친다.

        cart_detector 는 설치본을 읽으므로 그것만 고쳐도 당장은 되지만,
        다시 빌드하면 소스본이 덮어써 등록이 사라진다. 둘 다 쓴다.
        """
        out = []
        try:
            share = get_package_share_directory("und_amr_bringup")
        except Exception:                                    # noqa: BLE001
            return out
        inst = os.path.join(share, "config", "carts.yaml")
        if os.path.exists(inst):
            out.append(inst)
        # <ws>/install/und_amr_bringup/share/und_amr_bringup -> <ws>/src/und_amr_bringup
        src = share.replace(os.path.join("install", "und_amr_bringup", "share",
                                         "und_amr_bringup"),
                            os.path.join("src", "und_amr_bringup"))
        src = os.path.join(src, "config", "carts.yaml")
        if src != inst and os.path.exists(src):
            out.append(src)
        return out

    @staticmethod
    def _yaml_put_cart(text, name, width, depth, tol, note):
        """주석을 살린 채 대차 한 종을 넣거나 고친다.

        yaml 로 읽고 다시 쓰면 이 파일의 설명 주석이 전부 날아간다.
        새 대차를 넣는 사람이 읽어야 할 내용이라 글로 다룬다.
        """
        lines = text.split("\n")
        # ⚠ 사람이 손으로 넣은 설정은 살린다.
        #   등록을 다시 하면 그 항목을 통째로 갈아 끼우는데, 그때 cam_legs
        #   같은 값이 함께 지워져 기본값으로 돌아갔다(2026-09-21: 카메라
        #   확인을 생략해 두었던 렉이 등록 한 번에 다시 3개를 요구했다).
        #   standoff 도 같다 — 지워지면 안 잡히는 거리에서 보게 된다.
        #   측정으로 정해지는 값(width/depth)만 갈고 나머지는 옮겨 붙인다.
        keep = []
        head0 = "    %s:" % name
        st0 = next((i for i, ln in enumerate(lines)
                    if ln.rstrip() == head0), None)
        if st0 is not None:
            j = st0 + 1
            while j < len(lines) and (lines[j].startswith("      ")
                                      or not lines[j].strip()):
                k = lines[j].strip()
                # 재등록해도 실측·보정값은 남긴다
                if k.startswith(("cam_legs:", "standoff:", "clearance_width:",
                                 "center_offset:")):
                    keep.append("      " + k)
                j += 1
        blk = ('    %s:\n      width: %.3f\n      depth: %.3f\n'
               '      tol: %.3f\n' % (name, width, depth, tol))
        for k in keep:
            blk += k + "\n"
        blk += '      note: "%s"\n' % note.replace('"', "'")
        # 1) carts 목록에 이름 넣기
        for i, ln in enumerate(lines):
            st = ln.strip()
            if st.startswith("carts:") and "[" in st:
                inner = st[st.index("[") + 1:st.rindex("]")]
                names = [t.strip().strip('"\'') for t in inner.split(",")
                         if t.strip()]
                if name not in names:
                    names.append(name)
                pad = ln[:len(ln) - len(ln.lstrip())]
                lines[i] = '%scarts: [%s]' % (
                    pad, ", ".join('"%s"' % n for n in names))
                break
        # 2) 이미 있으면 그 블록만 갈아 끼우고, 없으면 끝에 붙인다
        head = "    %s:" % name
        st = next((i for i, ln in enumerate(lines)
                   if ln.rstrip() == head), None)
        if st is None:
            body = "\n".join(lines).rstrip("\n")
            return body + "\n" + blk
        en = st + 1
        while en < len(lines) and (lines[en].startswith("      ")
                                   or not lines[en].strip()):
            en += 1
        return "\n".join(lines[:st]) + "\n" + blk + "\n".join(lines[en:])

    def cart_register(self, name, width, depth, tol=0.06, note=""):
        """잰 규격을 carts.yaml 에 넣고 검출기를 다시 띄운다."""
        safe = "".join(ch for ch in (name or "") if ch.isalnum() or ch in "-_")
        if not safe:
            return False, "이름에 영문/숫자/-/_ 만 쓸 수 있습니다"
        try:
            w, d = float(width), float(depth)
            t = float(tol)
        except (TypeError, ValueError):
            return False, "규격 값이 숫자가 아닙니다"
        if not (0.1 <= w <= 3.0 and 0.1 <= d <= 3.0 and 0.01 <= t <= 0.5):
            return False, "규격 값이 범위를 벗어났습니다 (0.1~3.0m)"
        paths = self._carts_yaml_paths()
        if not paths:
            return False, "carts.yaml 을 찾지 못했습니다"
        note = note or ("실측 %s" % datetime.date.today().isoformat())
        done = []
        for path in paths:
            try:
                with open(path, encoding="utf-8") as f:
                    txt = f.read()
                new = self._yaml_put_cart(txt, safe, w, d, t, note)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(new)
                done.append(os.path.basename(os.path.dirname(
                    os.path.dirname(path))))
            except OSError as e:                             # noqa: BLE001
                return False, "%s 를 고치지 못했습니다: %s" % (path, e)
        # 검출기는 뜰 때 설정을 읽는다. respawn 이 걸려 있어 죽이면 되살아나며
        # 새 규격을 집어 든다.
        killed = False
        try:
            killed = subprocess.run(
                ["pkill", "-f", "lib/und_amr_bringup/cart_detector"],
                capture_output=True, timeout=5).returncode == 0
        except (OSError, subprocess.SubprocessError):
            pass
        self.get_logger().info("대차 '%s' 등록 %.3f x %.3f (tol %.3f)"
                               % (safe, w, d, t))
        return True, ("'%s' 를 %.0f x %.0f mm 로 등록했습니다%s"
                      % (safe, w * 1000, d * 1000,
                         " — 검출기를 다시 띄웁니다" if killed
                         else " (검출기 재시작 실패, 수동으로 다시 띄우세요)"))

    def carts_json(self):
        """대차 인식 결과. laser 좌표에 map 좌표를 얹어서 돌려준다."""
        with self.lock:
            d = dict(self.carts) if self.carts else None
            age = time.time() - self.carts_t if self.carts_t else None
        if d is None:
            return {"available": False,
                    "hint": "cart_detector 노드가 실행 중이어야 합니다"}
        d["available"] = True
        d["age_sec"] = round(age, 2) if age is not None else None
        d["stale"] = bool(age is not None and age > 2.0)
        # TF 조회는 이따금 실패한다(0.05초 타임아웃). 그때마다 map 필드가
        # 빠지면 화면에서 대차가 깜빡이며 사라진다. 직전 값을 재사용한다.
        base = self.tf_pose("map", "base_footprint") or self._carts_tf[0]
        laser = (self.tf_pose("base_footprint", d.get("frame") or "laser")
                 or self._carts_tf[1])
        self._carts_tf = (base, laser)
        if base and laser:
            c, sn = math.cos(base["yaw"]), math.sin(base["yaw"])
            lx = base["x"] + laser["x"] * c - laser["y"] * sn
            ly = base["y"] + laser["x"] * sn + laser["y"] * c
            lyaw = base["yaw"] + laser["yaw"]
            lc, ls = math.cos(lyaw), math.sin(lyaw)

            def to_map(px, py):
                return [round(lx + px * lc - py * ls, 4),
                        round(ly + px * ls + py * lc, 4)]

            for c2 in d.get("carts", []):
                c2["map"] = {"xy": to_map(c2["x"], c2["y"]),
                             "yaw": round(c2["yaw"] + lyaw, 4),
                             "legs": [to_map(p[0], p[1]) for p in c2["legs"]]}
        return d

    def _on_scan(self, m):
        self._scan_t = time.monotonic()
        self._scan_n += 1
        with self.lock:
            self.scan = m
            if self.sess is not None:
                self.sess["scans"] += 1

    SCAN_DEAD_S = 1.0        # 스캔 15Hz 이므로 1초면 15프레임을 놓친 것

    def _scan_watch(self):
        """스캔이 끊기면 이동을 취소하고 정지시킨다.

        판단은 '멈추는 쪽으로' 한다. 끊긴 것이 아니라 잠깐 늦은 것이었다면
        다시 목표를 주면 그만이지만, 반대로 놓치면 로봇이 눈을 감고 달린다.
        """
        now = time.monotonic()
        last, self._watch_t = self._watch_t, now
        # 이 타이머 자신이 늦게 돌았다면, 센서가 끊긴 것이 아니라 이 노드가
        # 멈춰 있던 것이다. 기동 중 지도를 읽고 SLAM 을 띄우는 동안 실제로
        # 20초 넘게 콜백이 밀린다(2026-09-11). 그 구간은 판단할 근거가 없으니
        # 기준만 다시 잡고 넘어간다.
        if last and (now - last) > 1.0:
            self._scan_t = now
            return
        if self._scan_n < 30:             # 15Hz 기준 약 2초치. 흐름이 잡힌 뒤에
            return
        gap = now - self._scan_t
        if gap > self.SCAN_DEAD_S:
            if not self._scan_dead:
                self._scan_dead = True
                self.get_logger().error(
                    "스캔이 %.1f초 끊겼다 — 이동을 멈춘다" % gap)
                # 출발 기어가기·도착 후 동작도 api_node 가 직접 몬다. 같이 끊는다.
                moving = (self.nav_goal is not None or self.nav_state in ("sending", "active")
                          or getattr(self, "_turning", False)
                          or getattr(self, "_depart_timer", None) is not None)
                if moving:
                    self.nav_cancel()
                    self.nav_state = "failed"
                    self.nav_msg = "라이다 신호가 끊겨 정지했습니다"
            # 취소가 먹을 때까지 계속 0 을 덮어쓴다. nav2 가 마지막 지령을
            # 물고 있어도 이쪽이 이긴다. 다만 세워 둔 로봇에까지 쏘면
            # 수동 주행을 막으므로, 이동 중일 때만 한다.
            if self.nav_goal is not None or self.nav_state == "active":
                self._pub_zero()
        elif self._scan_dead:
            self._scan_dead = False
            self.get_logger().info("스캔 복구")
            if self.nav_state == "failed" and "라이다" in self.nav_msg:
                self.nav_state, self.nav_msg = "idle", ""

    def _sess_travel(self, m):
        """오도메트리 델타를 누적해 이 세션의 이동거리/회전량을 잰다.

        지도 품질을 나중에 판단하는 근거가 된다 — 거의 안 움직이고 만든
        지도인지, 충분히 돌아다니며 만든 지도인지."""
        if self.sess is None:
            return
        q = m.pose.pose.orientation
        cur = (m.pose.pose.position.x, m.pose.pose.position.y,
               math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        last = self.sess.get("_last")
        if last is not None:
            self.sess["dist_m"] += math.hypot(cur[0] - last[0], cur[1] - last[1])
            d = (cur[2] - last[2] + math.pi) % (2.0 * math.pi) - math.pi
            self.sess["rot_rad"] += abs(d)
        self.sess["_last"] = cur

    def _on_diag(self, m):
        with self.lock:
            for st in m.status:
                if st.name.startswith("motor/"):
                    d = {kv.key: kv.value for kv in st.values}
                    # rclpy 는 DiagnosticStatus.level(byte) 을 bytes 로 준다
                    lv = st.level
                    if isinstance(lv, (bytes, bytearray)):
                        lv = int.from_bytes(lv, "big")
                    d["level"] = int(lv)
                    d["message"] = st.message
                    self.motors[st.name.split("/")[-1]] = d

    def _on_map(self, m):
        """점유격자를 PNG 로 변환해 들고 있는다.

        ROS 격자는 좌하단이 원점이고 행이 +y 방향으로 쌓인다.
        이미지는 좌상단이 원점이므로 세로로 뒤집어 저장하고,
        UI 는 origin 과 resolution 으로 월드↔픽셀을 환산한다."""
        if not HAS_IMAGING:
            return
        try:
            w, h = m.info.width, m.info.height
            if w == 0 or h == 0:
                return
            # 저장된 지도를 올려 둔 동안에는 화면을 그대로 고정한다.
            # slam_toolbox 는 localization 모드에서도 /map 을 다시 그려서
            # 내보내는데, 그걸 받아 쓰면 지도가 조금씩 달라져 "지도가 또
            # 그려진다" 로 보인다. 로봇 위치는 TF 에서 따로 받으므로
            # 지도만 고정해도 화면은 정상이다.
            #
            # 조건이 slam_mode == "localization" 이었을 때 구멍이 있었다.
            # slam_kill() 이 slam_mode 를 None 으로 만드는데, 새 노드가 뜨는
            # 동안 /map 이 들어오면 조건이 안 맞아 저장된 지도를 갈아치우고
            # 편집까지 날렸다. '매핑 중일 때만' 갈아치우도록 뒤집는다.
            if self.map_source == "saved" and self.slam_mode != "mapping":
                return
            g = np.array(m.data, dtype=np.int8).reshape(h, w)
            img = np.full((h, w), 205, dtype=np.uint8)        # 미지 = 회색
            img[(g >= 0) & (g <= 25)] = 255                    # 자유 = 흰색
            img[g >= 65] = 0                                   # 점유 = 검정
            mid = (g > 25) & (g < 65)
            img[mid] = 150
            img = np.flipud(img)                               # 이미지 좌표계로
            buf = io.BytesIO()
            PilImage.fromarray(img, mode="L").save(buf, format="PNG")
            q = m.info.origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self.lock:
                self._map_seq += 1
                self.map_source = "live"
                # 매핑 중인 지도는 이름이 없어 편집을 붙일 수 없다.
                # 원본을 비워 두어 합성이 옛 지도를 되살리지 못하게 한다.
                self.map_base_png = None
                self.map_edits = []
                self.pois = []
                self.map_png = buf.getvalue()
                self.map_loc_png = self.map_png
                self.map_meta = {
                    "available": True, "seq": self._map_seq,
                    "frame": m.header.frame_id or "map",
                    "width": w, "height": h,
                    "resolution": m.info.resolution,
                    "origin": {"x": m.info.origin.position.x,
                               "y": m.info.origin.position.y, "yaw": yaw},
                }
        except (ValueError, TypeError):
            pass

    def tf_pose_at(self, stamp, target="map", source="base_footprint"):
        """그 시각의 자세. 없으면 None.

        스캔을 화면에 얹을 때 '지금 자세' 를 쓰면, 스캔을 찍은 순간과
        시점이 달라 회전 중에 점들이 통째로 돌아 보인다. 0.4 rad/s 에서
        100ms 만 어긋나도 2.3도, 10m 앞에서 40cm 다(2026-09-14).
        """
        try:
            t = self.tf_buffer.lookup_transform(
                target, source, RclTime.from_msg(stamp),
                timeout=Duration(seconds=0.05))
        except _tf2.TransformException:
            return None
        q = t.transform.rotation
        return {"x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "yaw": math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))}

    TF_MAX_AGE_S = 0.5

    def tf_pose(self, target="map", source="base_footprint"):
        """TF 로 로봇 위치를 구한다. SLAM 이 없으면 map 프레임이 없어 None."""
        try:
            t = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except _tf2.TransformException as e:
            self._tf_err = "%s: %s" % (type(e).__name__, str(e)[:120])
            return None
        # ⚠ 오래된 변환은 '모름' 으로 본다. SLAM 이 죽어도 TF 버퍼는 마지막
        #   map->odom 을 10초 동안 계속 내준다. 그동안 자세가 멈춰 보여 도착 후
        #   회전이 끝을 못 알아채고 계속 돌 수 있었다(2026-09-30 검토).
        #   평소 나이는 최대 0.04초(같은 날 실측). 고정 변환(시각 0)은 뺀다.
        st = t.header.stamp
        if st.sec or st.nanosec:
            age = (self.get_clock().now()
                   - rclpy.time.Time.from_msg(st)).nanoseconds * 1e-9
            if age > self.TF_MAX_AGE_S:
                self._tf_err = "TF 가 %.1f초 전 것 (%s<-%s)" % (age, target, source)
                return None
        self._tf_err = None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"frame": target,
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "yaw": yaw, "yaw_deg": math.degrees(yaw)}

    LOG_LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

    def _on_log(self, m):
        with self.lock:
            self.logs.append({
                "t": m.stamp.sec + m.stamp.nanosec * 1e-9,
                "level": self.LOG_LEVEL.get(m.level, str(m.level)),
                "name": m.name,
                "msg": m.msg,
            })

    def _on_image(self, m, key="front"):
        """ROS Image 를 JPEG 로. 파이썬 인코딩이 무거워 10fps 로 제한한다.
        key: 'front' (정면 D435i) / 'down' (아래 D435F, 2026-11-13)."""
        now = time.monotonic()
        lastd = self.__dict__.setdefault("_cam_last_k", {})
        if now - lastd.get(key, 0.0) < 0.1:
            return
        lastd[key] = now
        if key == "front":
            self._cam_last = now               # camera_reset 이 영상 복귀를 이걸로 본다
        try:
            arr = np.frombuffer(m.data, dtype=np.uint8)
            arr = arr.reshape(m.height, m.width, -1)
            if m.encoding == "bgr8":
                arr = arr[:, :, ::-1]
            elif m.encoding not in ("rgb8",):
                return
            buf = io.BytesIO()
            PilImage.fromarray(arr).save(buf, format="JPEG", quality=70)
            with self.lock:
                if key == "down":
                    self.cam_jpeg_down = buf.getvalue()
                    return
                self.cam_jpeg = buf.getvalue()
                self.cam_info = {"width": m.width, "height": m.height,
                                 "encoding": m.encoding}
        except (ValueError, TypeError):
            pass

    # ---------------- 조회 ----------------

    def device_info(self):
        return {
            "api_version": API_VERSION,
            "rosdistro": os.environ.get("ROS_DISTRO", "unknown"),
            "device": {
                "model": "und_amr_v1",
                "name": os.uname().nodename,
                "kinematics": "differential drive, 2 wheels",
                "hardware": {
                    "controller": {
                        "mcu": "STM32H743VIT6",
                        "link": "USB CDC / USART1 115200",
                    },
                    "motor_driver": {
                        "model": "L2DB4830-CAFC-ASL3",
                        "bus": "CAN 500kbit/s, CANopen SDO",
                        "axes": 2,
                        "encoder_counts_per_rev": 4096,
                        "gear_ratio": 1.0,
                    },
                    "lidars": {
                        "lidar_node": {
                            "model": "LDS-50C-E-12",
                            "vendor": "Pacecat",
                            "interface": "Ethernet UDP 192.168.158.98",
                        },
                    },
                    "cameras": {
                        "camera": {"model": "RealSense D435i", "imu": True},
                    },
                },
            },
        }

    def chassis_status(self):
        with self.lock:
            motors = dict(self.motors)
            mode, estop = self.control_mode, self.estop
            lift = self.lift if time.time() - self.lift_t < 5.0 else None
        any_err = any(int(m.get("level", 0)) >= 2 for m in motors.values())
        return {
            "control_mode": mode,
            "emergency_stop_pressed": estop,
            "motors_ok": not any_err and bool(motors),
            "motors": motors,
            "lift": lift,
            "help": "PATCH to change control mode",
            "sample command": {"control_mode": "auto/manual"},
        }

    def chassis_pose(self):
        with self.lock:
            o = self.odom

        # 맵 좌표(TF map→base_footprint)는 오도메트리와 무관하게 조회한다.
        # SLAM 만 돌고 base_node 가 없어도 로봇 위치는 알 수 있어야 한다.
        out = {"map": self.tf_pose(), "map_error": self._tf_err}
        if o is None:
            out["error"] = "no odometry yet"
            return out

        q = o.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        out.update({
            "frame": o.header.frame_id,
            "x": o.pose.pose.position.x,
            "y": o.pose.pose.position.y,
            "yaw": yaw,
            "yaw_deg": math.degrees(yaw),
            "linear_velocity": o.twist.twist.linear.x,
            "angular_velocity": o.twist.twist.angular.z,
        })
        return out

    # ---------------------------------------------------------- 배터리
    #
    # 타보스 LV-25V25AH-DH255COM. STM32 가 CAN2(500k) 로 1초마다 묻고,
    # base_node 가 /battery_detail(JSON) 로 낸다(2026-09-29). BMS 답이 끊기면
    # 예전처럼 모터 드라이버의 모선 전압으로 물러선다.
    BAT_FRESH_S  = 5.0        # 이보다 오래된 값은 쓰지 않는다
    BAT_HIST_S   = 10.0       # 이력 표본 간격
    BAT_HIST_MAX = 3 * 24 * 360   # 3일치
    BAT_SAVE_S   = 300.0
    BAT_PLOT_MAX = 720        # 그래프에 보내는 최대 점 수
    # 타보스 통신규약 6.2.2 (7S/25V) — 화면에 기준으로 보여 준다
    BAT_LIMITS = {"over_v": 29.47, "under_v": 21.70, "chg_a": 100.0,
                  "dis_a": 70.0, "hot_c": 50.0, "cold_c": 2.0}

    def _on_bat(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.bat = d
            self.bat_t = time.time()

    def _on_lift(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            was = self.lift in self.LIFT_LOADED
            self.lift = d.get("state")
            self.lift_t = time.time()
            now = self.lift in self.LIFT_LOADED
        if was != now and self.nav_cli.server_is_ready():
            self._speed_sent = self._speed_push(self._speed_eff())

    LIFT_LOADED = ("top", "stopped")    # 렉을 들고 있을 수 있는 상태

    def _lift_moving(self):
        return getattr(self, "lift", None) in ("up", "down")

    def lift_busy_error(self):
        """리프트 올림·내림은 로봇이 서 있을 때만 받는다. 정지는 언제든 된다."""
        if self.estop:
            return "비상정지 상태입니다"
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            return "렉 작업 중입니다"
        if getattr(self, "route", {}).get("state") in self.ROUTE_BUSY:
            return "왕복 중입니다. 왕복을 멈춘 뒤 하세요"
        if self.nav_goal is not None or self.nav_state in ("sending", "active"):
            return "이동 중입니다. 도착한 뒤 하세요"
        o = self.odom
        if o is not None and (abs(o.twist.twist.linear.x) > 0.01
                              or abs(o.twist.twist.angular.z) > 0.03):
            return "로봇이 움직이고 있습니다. 멈춘 뒤 하세요"
        return None

    def _speed_eff(self):
        """nav2 에 줄 속도.
        ⚠ 2026-11-12 렉을 든 상태(top/stopped)에서 LIFT_NAV_SPEED(0.20)로 낮추던 것을 뺐다 —
          사용자: "리프트해도 속도는 같아야해". 리프트가 움직이는 동안 서는 것은 그대로."""
        return self.nav_speed

    def lift_stop_async(self):
        """정지·비상정지 때 리프팅도 세운다. 응답은 기다리지 않는다."""
        cli = self.srv.get("lift_stop")
        if cli is not None and cli.service_is_ready():
            cli.call_async(Trigger.Request())

    def _bat_fresh(self):
        with self.lock:
            d, t = self.bat, self.bat_t
        if d and d.get("ok") and time.time() - t < self.BAT_FRESH_S:
            return d
        return None

    def _bat_hist_path(self):
        return os.path.join(os.path.dirname(self.map_dir.rstrip("/")),
                            "battery_history.json")

    def _bat_hist_load(self):
        try:
            with open(self._bat_hist_path()) as f:
                rows = json.load(f).get("rows", [])
            cut = time.time() - self.BAT_HIST_MAX * self.BAT_HIST_S
            self.bat_hist.extend(r for r in rows if r and r[0] >= cut)
        except (OSError, ValueError, AttributeError):
            pass

    def _bat_hist_save(self, wait=False):
        """3일치(약 0.9MB)를 쓰는 데 50ms 넘게 걸려 20Hz 제어 타이머를 막았다.
        주기 저장은 따로 스레드에서 한다(2026-09-29 검토). 종료 때는 기다린다."""
        with self.lock:
            rows = list(self.bat_hist)
        if not wait:
            threading.Thread(target=self._bat_hist_write, args=(rows,),
                             daemon=True).start()
            return
        self._bat_hist_write(rows)

    def _bat_hist_write(self, rows):
        path = self._bat_hist_path()
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"rows": rows}, f, separators=(",", ":"))
            os.replace(tmp, path)
        except OSError as e:
            self.get_logger().warn("배터리 이력 저장 실패: %s" % e)

    def _bat_sample(self):
        d = self._bat_fresh()
        if d is None:
            return
        with self.lock:
            self.bat_hist.append([int(time.time()), d["soc"],
                                  round(d["voltage"], 2), round(d["current"], 2),
                                  round(d["temperature"], 1)])

    def battery_history(self, hours):
        """[t, soc, V, A, ℃] 표본을 BAT_PLOT_MAX 개 이하로 묶어 돌려준다."""
        hours = max(0.1, min(72.0, float(hours)))
        cut = time.time() - hours * 3600.0
        with self.lock:
            rows = [r for r in self.bat_hist if r[0] >= cut]
        k = max(1, math.ceil(len(rows) / self.BAT_PLOT_MAX))
        out = {"t": [], "soc": [], "v": [], "i": [], "temp": []}
        for j in range(0, len(rows), k):
            grp = rows[j:j + k]
            n = float(len(grp))
            out["t"].append(grp[-1][0])
            out["soc"].append(round(sum(r[1] for r in grp) / n, 1))
            out["v"].append(round(sum(r[2] for r in grp) / n, 2))
            out["i"].append(round(sum(r[3] for r in grp) / n, 2))
            out["temp"].append(round(sum(r[4] for r in grp) / n, 1))
        out.update(hours=hours, step_s=k * self.BAT_HIST_S, now=int(time.time()))
        return out

    def battery_state(self):
        """BMS 값. 끊기면 모터 드라이버가 보고하는 모선 전압으로 물러선다."""
        with self.lock:
            motors = dict(self.motors)
            raw, raw_t = self.bat, self.bat_t
        volts = [float(m["voltage"]) for m in motors.values() if "voltage" in m]
        bus = (sum(volts) / len(volts)) if volts else None
        per_axis = {k: float(m.get("voltage", 0)) for k, m in motors.items()}
        d = self._bat_fresh()
        if d is not None:
            out = dict(d)
            out.pop("t", None)
            out.update(source="bms", power_w=round(d["voltage"] * d["current"], 1),
                       state=("charging" if d["current"] > 0.2 else
                              "discharging" if d["current"] < -0.2 else "idle"),
                       age_s=round(time.time() - raw_t + d.get("age_ms", 0) / 1000.0, 1),
                       bus_voltage=bus, per_axis=per_axis, limits=self.BAT_LIMITS,
                       model="타보스 LV-25V25AH-DH255COM")
            return out
        why = ((raw or {}).get("error") if raw and time.time() - raw_t < self.BAT_FRESH_S
               else "BMS 값이 들어오지 않습니다 (base_node /battery_detail)")
        if bus is None:
            return {"ok": False, "error": "no motor feedback", "bms_error": why}
        return {"ok": False, "source": "motor driver DC bus", "voltage": bus,
                "per_axis": per_axis, "bms_error": why, "limits": self.BAT_LIMITS}

    def scan_json(self, max_points=720):
        with self.lock:
            s = self.scan
        if s is None:
            return {"error": "no scan yet"}
        step = max(1, len(s.ranges) // max_points)
        pts = []
        for i in range(0, len(s.ranges), step):
            r = s.ranges[i]
            if math.isfinite(r) and r > 0.0:
                pts.append([round(math.degrees(
                    s.angle_min + i * s.angle_increment), 2), round(r, 3)])
        # 라이다 장착 위치/각도. 화면은 스캔을 로봇 자세에 얹어 그리는데,
        # 스캔 각도는 라이다 프레임 기준이라 장착 오차를 더해 주지 않으면
        # 점들이 통째로 돌아간다. 실제로 2.1도 틀어져 있어 10m 에서 37cm
        # 어긋났다. URDF 가 바뀌어도 따라가도록 TF 에서 읽어 함께 보낸다.
        mount = self.tf_pose("base_footprint", s.header.frame_id or "laser")
        at = self.tf_pose_at(s.header.stamp)
        return {
            "frame": s.header.frame_id,
            # 이 스캔을 찍던 순간의 로봇 자세. 화면은 지금 자세가 아니라
            # 이것에 얹어야 회전 중에도 점이 지도와 붙어 있다.
            "pose": at,
            "mount": ({"x": mount["x"], "y": mount["y"], "yaw": mount["yaw"]}
                      if mount else None),
            "angle_min_deg": math.degrees(s.angle_min),
            "angle_max_deg": math.degrees(s.angle_max),
            "count_raw": len(s.ranges),
            "count": len(pts),
            "range_min": s.range_min,
            "range_max": s.range_max,
            "points": pts,          # [[각도(도), 거리(m)], ...]
            "charger": self._scan_charger(s, at),
        }

    # 화면에 V 대신 충전기 모양을 그리려고 (2026-11-13, 사용자: "v로 인식되면 v대신 충전기
    # 이미지가 떠있게 가능해? 그위치에 맞게"). 충전소 POI 3.5m 안일 때만, 같은 스캔은 한 번만.
    CHARGER_SHOW_NEAR = 3.5

    def _scan_charger(self, s, at):
        if at is None:
            return None
        key = (s.header.stamp.sec, s.header.stamp.nanosec)
        cache = self.__dict__.get("_scan_chg_cache")
        if cache and cache[0] == key:
            return cache[1]
        out = None
        near = any(p.get("kind") == "charger"
                   and math.hypot(p["x"] - at["x"], p["y"] - at["y"]) < self.CHARGER_SHOW_NEAR
                   for p in getattr(self, "pois", []))
        if near:
            from und_amr_bringup.charger_detect import detect_scan
            las = self._poi_laser(s.header.frame_id) or (0.0, 0.0, 0.0)
            c = detect_scan(s.ranges, s.angle_min, s.angle_increment, las)
            if c is not None:
                cs, sn = math.cos(at["yaw"]), math.sin(at["yaw"])
                out = {"x": round(at["x"] + cs * c["x"] - sn * c["y"], 4),
                       "y": round(at["y"] + sn * c["x"] + cs * c["y"], 4),
                       "yaw": round(at["yaw"] + c["yaw"], 4)}
        self._scan_chg_cache = (key, out)
        return out

    def robot_log(self, n=100, level=None):
        with self.lock:
            items = list(self.logs)
        if level:
            want = level.upper()
            items = [x for x in items if x["level"] == want]
        return {"count": len(items), "logs": items[-n:]}

    def maps_list(self):
        out = []
        if os.path.isdir(self.map_dir):
            for f in sorted(os.listdir(self.map_dir)):
                # 지도 yaml 만. <name>.slam.yaml 같은 부속 파일은 제외한다
                # (이름에 점이 들어가면 지도 본체가 아니다)
                if f.endswith(".yaml") and "." not in f[:-5]:
                    name = f[:-5]
                    p = os.path.join(self.map_dir, f)
                    out.append({
                        "name": name,
                        "yaml": p,
                        "image": os.path.join(self.map_dir, name + ".pgm"),
                        "create_time": int(os.path.getmtime(p)),
                    })
        return out

    # ---------------- 제어 ----------------

    def call_motor_service(self, name, timeout=2.0):
        cli = self.srv.get(name)
        if cli is None:
            return False, "알 수 없는 서비스: %s" % name
        if not cli.wait_for_service(timeout_sec=0.5):
            return False, "base_node 가 실행 중이 아닙니다"
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            return False, "응답 시간 초과"
        r = fut.result()
        return bool(r.success), r.message

    # ---------------- 매핑 ----------------

    def _call(self, cli, req, timeout=8.0):
        if cli is None:
            return None, "지원되지 않는 기능"
        if not cli.wait_for_service(timeout_sec=0.5):
            return None, "SLAM 이 실행 중이 아닙니다"
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            return None, "응답 시간 초과"
        return fut.result(), None

    def mapping_status(self):
        running = self.slam_is_running()
        owned = self.slam_proc is not None and self.slam_proc.poll() is None

        # SLAM 이 내려갔는데 마지막 지도를 계속 들고 있으면, 매핑 화면에
        # 들어왔을 때 이미 매핑이 진행 중인 것처럼 보인다. 매핑은 '새 매핑
        # 시작' 을 눌렀을 때만 시작되어야 하므로, 노드가 사라지면 지도도 잊는다.
        # (저장된 지도는 별개다. 그건 '맵' 화면에서 불러온다.)
        if not running:
            with self.lock:
                stale = self.map_source == "live"
            if stale:
                # 저장해 둔 메인 맵이 있으면 그걸로 되돌리고, 없으면 비운다.
                if not (self.active_map and self.adopt_saved_map(self.active_map)):
                    self._forget_map()
                    self.active_map = None
                    self._remember_active(None)
                    self.get_logger().info(
                        "SLAM 이 내려가 작성 중이던 지도를 비웠습니다")

        with self.lock:
            meta = dict(self.map_meta)
        mapping_on = running and self.slam_mode != "localization"
        can_res, res_why = self.can_resume()
        return {
            # slam_running 은 '지도를 그리는 중' 을 뜻한다. 위치추정만 도는
            # 경우는 localizing 으로 따로 알린다. 화면에서 매핑 버튼 상태를
            # 이 값으로 정하기 때문에 섞으면 안 된다.
            "slam_running": mapping_on,
            "localizing": bool(running and self.slam_mode == "localization"),
            "slam_mode": self.slam_mode,
            "owned_by_api": owned,
            "paused": self.mapping_paused,
            "map": meta or {"available": False},
            "saved_maps": self.maps_list(),
            "active_map": self.active_map,
            # 이어서 매핑 중이면 어느 지도에 덧그리는 중인지. 종료할 때
            # 그 이름으로 덮어쓸 수 있게 화면에 알려 준다.
            "resumed_from": self.resumed_from if mapping_on else None,
            "can_resume": can_res,
            "resume_hint": res_why,
            "session": self.session_info(),
            "hint": (None if mapping_on else
                     ("메인 맵 위에서 위치추정 중입니다. 지도는 늘어나지 않습니다"
                      if self.slam_mode == "localization"
                      else "매핑을 시작하면 SLAM 이 기동합니다")),
        }

    def mapping_pause(self):
        """새 스캔 반영을 일시정지/재개 (토글).

        slam_toolbox 의 응답 status 는 '호출 성공' 을 뜻할 뿐 현재 상태가
        아니다. 실제 상태는 서비스가 토글식이므로 여기서 직접 추적한다."""
        r, err = self._call(self.slam.get("pause"), Pause.Request())
        if err:
            return False, err
        self.mapping_paused = not self.mapping_paused
        return True, "일시정지됨" if self.mapping_paused else "재개됨"

    # -------- 매핑 세션 기록 (rosbag + 통계) --------
    #
    # 지도(.pgm/.posegraph)만 남기면 "그 지도가 어떻게 만들어졌는지" 가
    # 사라진다. 파라미터를 바꿔 다시 돌려보거나 문제를 되짚으려면 원본
    # 센서 데이터가 있어야 한다. 매핑 중 주행을 통째로 녹화해 두고
    # 저장할 때 지도와 같은 이름으로 묶어 둔다.

    BAG_TOPICS = ["/scan", "/tf", "/tf_static", "/odom", "/joint_states",
                  "/imu/data", "/map", "/diagnostics"]

    @staticmethod
    def _human(nbytes):
        if nbytes >= 1048576:
            return "%.1fMB" % (nbytes / 1048576.0)
        return "%.0fKB" % (nbytes / 1024.0)

    @staticmethod
    def _dir_size(path):
        total = 0
        for root, _, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
        return total

    def _rm(self, path):
        """파일이든 디렉터리든 지운다."""
        try:
            if os.path.isdir(path) and not os.path.islink(path):
                shutil.rmtree(path)
            else:
                os.remove(path)
            return True
        except OSError:
            return False

    def _bag_rename(self, bag_dir, name):
        """녹화 중에는 이름을 알 수 없어 임시 이름으로 파일이 만들어진다.
        저장할 때 지도 이름으로 바꾸고 metadata.yaml 의 경로도 함께 고친다.
        실패해도 재생에는 지장이 없으므로 조용히 넘어간다."""
        meta = os.path.join(bag_dir, "metadata.yaml")
        stem = os.path.basename(self.bag_tmp)
        try:
            with open(meta) as f:
                text = f.read()
            for fn in sorted(os.listdir(bag_dir)):
                if fn == "metadata.yaml" or not fn.startswith(stem):
                    continue
                new = name + fn[len(stem):]
                os.rename(os.path.join(bag_dir, fn),
                          os.path.join(bag_dir, new))
                text = text.replace(fn, new)
            with open(meta, "w") as f:
                f.write(text)
        except (OSError, ValueError) as e:                   # noqa: BLE001
            self.get_logger().warn("bag 이름 정리 생략: %s" % e)

    def bag_start(self):
        """임시 경로에 녹화를 시작한다. 이름은 저장할 때 정해진다."""
        self.bag_stop()
        self._rm(self.bag_tmp)
        if not self.bag_enabled:
            return False
        os.makedirs(self.map_dir, exist_ok=True)
        cmd = (["ros2", "bag", "record", "-o", self.bag_tmp,
                "--compression-mode", "file", "--compression-format", "zstd"]
               + list(self.BAG_TOPICS))
        try:
            self.bag_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
        except OSError as e:                                 # noqa: BLE001
            self.get_logger().warn("bag 기록 시작 실패: %s" % e)
            self.bag_proc = None
            return False
        return True

    def bag_stop(self, timeout=15.0):
        """SIGINT 로 정상 종료시켜야 인덱스와 압축이 마무리된다."""
        if self.bag_proc is None or self.bag_proc.poll() is not None:
            self.bag_proc = None
            return False
        try:
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
        except OSError:
            pass
        t0 = time.monotonic()
        while self.bag_proc.poll() is None and time.monotonic() - t0 < timeout:
            time.sleep(0.1)
        if self.bag_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGKILL)
            except OSError:
                pass
        self.bag_proc = None
        return True

    def bag_running(self):
        return self.bag_proc is not None and self.bag_proc.poll() is None

    def _sess_new(self):
        self.sess = {"t0": time.time(), "scans": 0,
                     "dist_m": 0.0, "rot_rad": 0.0, "_last": None}

    def session_info(self):
        """진행 중인 매핑 세션의 기록 상태."""
        with self.lock:
            se = dict(self.sess) if self.sess else None
        out = {"bag_enabled": self.bag_enabled,
               "recording": self.bag_running(),
               "bag_bytes": self._dir_size(self.bag_tmp)
                            if os.path.isdir(self.bag_tmp) else 0}
        if se:
            out.update({"elapsed_sec": round(time.time() - se["t0"], 1),
                        "scans": se["scans"],
                        "dist_m": round(se["dist_m"], 2),
                        "rot_deg": round(math.degrees(se["rot_rad"]), 1)})
        return out

    def _robot_params(self):
        """지도를 만들 때 쓰인 차체 설정. 나중에 치수를 바꾸면 예전 지도의
        오도메트리 축척이 달라지므로 함께 남긴다."""
        out = {}
        try:
            names = ["wheel_separation", "wheel_diameter", "gear_ratio",
                     "counts_per_wheel_rev"]
            for n in names:
                r = subprocess.run(
                    ["ros2", "param", "get", "/base_node", n, "--hide-type"],
                    capture_output=True, text=True, timeout=2)
                if r.returncode == 0 and r.stdout.strip():
                    out[n] = r.stdout.strip()
        except Exception:                                    # noqa: BLE001
            pass
        return out

    def _lidar_info(self):
        with self.lock:
            sc = self.scan
        if sc is None:
            return {}
        return {"frame_id": sc.header.frame_id,
                "points": len(sc.ranges),
                "range_min": round(float(sc.range_min), 3),
                "range_max": round(float(sc.range_max), 3),
                "angle_increment_deg": round(math.degrees(sc.angle_increment), 4)}

    def info_path(self, name):
        return os.path.join(self.map_dir, name + ".info.json")

    def map_info(self, name):
        """지도에 딸린 부가 정보. 없으면 빈 dict."""
        base = self.map_path(name)
        if not base:
            return {}
        try:
            with open(self.info_path(os.path.basename(base))) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def map_note(self, name, note):
        base = self.map_path(name)
        if not base or not os.path.exists(base + ".yaml"):
            return False, "해당 지도를 찾지 못했습니다"
        stem = os.path.basename(base)
        info = self.map_info(stem)
        info["note"] = str(note)[:500]
        try:
            with open(self.info_path(stem), "w") as f:
                json.dump(info, f, ensure_ascii=False, indent=1)
        except OSError as e:                                 # noqa: BLE001
            return False, "메모 저장 실패: %s" % e
        return True, "메모 저장됨"

    def mapping_save(self, name):
        """지도와 함께 이 매핑 세션의 부속 자료를 한 이름으로 묶어 저장한다.

            <name>.pgm / .yaml       map_server 용 격자지도
            <name>.posegraph / .data slam_toolbox 포즈그래프 (이어 매핑용)
            <name>.bag/              매핑 중 센서·TF 녹화 (재현/재처리용)
            <name>.slam.yaml         그때 쓴 SLAM 파라미터 사본
            <name>.info.json         시간·이동량·차체설정·라이다 제원·메모

        저장은 기록 구간을 마감한다. 이어서 매핑하면 새 구간이 시작되고
        다음 저장에는 그 이후 구간만 담긴다.
        """
        name = "".join(ch for ch in name if ch.isalnum() or ch in "-_") or "map"
        os.makedirs(self.map_dir, exist_ok=True)
        path = os.path.join(self.map_dir, name)

        req = SaveMap.Request()
        req.name = StringMsg(data=path)
        r, err = self._call(self.slam.get("save"), req)
        if err:
            return False, err
        if r.result != 0:
            return False, "지도 저장 실패 (코드 %d) - 스캔이 아직 없을 수 있음" % r.result

        parts = ["지도"]
        warn = []

        req2 = SerializePoseGraph.Request()
        req2.filename = path
        r2, err2 = self._call(self.slam.get("serialize"), req2)
        if not err2 and r2 and r2.result == 0:
            parts.append("포즈그래프")
        else:
            warn.append("포즈그래프")

        # 녹화 마감 → 지도 이름으로 이동 → 새 구간 시작
        bag_info = {}
        was_recording = self.bag_running()
        self.bag_stop()
        if os.path.isdir(self.bag_tmp):
            dest = path + ".bag"
            self._rm(dest)
            try:
                shutil.move(self.bag_tmp, dest)
                bag_info = {"dir": os.path.basename(dest),
                            "bytes": self._dir_size(dest)}
                self._bag_rename(dest, name)
                bag_info["bytes"] = self._dir_size(dest)
                parts.append("기록 " + self._human(bag_info["bytes"]))
            except OSError as e:                             # noqa: BLE001
                warn.append("기록(%s)" % e)

        # SLAM 파라미터 사본
        if self.slam_cfg and os.path.exists(self.slam_cfg):
            try:
                shutil.copyfile(self.slam_cfg, path + ".slam.yaml")
            except OSError:
                warn.append("SLAM 설정")

        # 메타데이터
        with self.lock:
            se = dict(self.sess) if self.sess else {}
            meta = dict(self.map_meta)
        prev = self.map_info(name)
        info = {
            "name": name,
            "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "started_at": (datetime.datetime.fromtimestamp(se["t0"])
                           .isoformat(timespec="seconds")) if se else None,
            "duration_sec": round(time.time() - se["t0"], 1) if se else None,
            "scans": se.get("scans"),
            "distance_m": round(se["dist_m"], 2) if se else None,
            "rotation_deg": round(math.degrees(se["rot_rad"]), 1) if se else None,
            "map": {k: meta.get(k) for k in
                    ("width", "height", "resolution", "origin")} if meta else {},
            "robot": self._robot_params(),
            "lidar": self._lidar_info(),
            "bag": bag_info or None,
            "slam_config": os.path.basename(path + ".slam.yaml")
                           if os.path.exists(path + ".slam.yaml") else None,
            "note": prev.get("note", ""),
        }
        try:
            with open(self.info_path(name), "w") as f:
                json.dump(info, f, ensure_ascii=False, indent=1)
            parts.append("정보")
        except OSError:
            warn.append("정보파일")

        if was_recording:
            self.bag_start()
        with self.lock:
            self._sess_new()

        msg = "저장 완료: %s (%s)" % (name, " · ".join(parts))
        if warn:
            msg += " — 실패: %s" % ", ".join(warn)
        # 매핑을 끝낼 때 이 이름을 메인 맵으로 채택한다.
        with self.lock:
            self.last_saved = name
        return True, msg

    # -------- SLAM 노드 수명 관리 --------
    #
    # slam_toolbox 에는 '지도 비우기' 서비스가 없다. clear_changes 는 수동
    # 보정만 지운다. 백지에서 다시 그리려면 노드를 재시작하는 수밖에 없어
    # api_node 가 SLAM 노드를 자식 프로세스로 직접 띄우고 내린다.

    def slam_is_running(self):
        if self.slam_proc is not None and self.slam_proc.poll() is None:
            return True
        return bool(self.slam) and self.slam["save"].service_is_ready()

    # ---------------- SLAM 감시 ----------------
    #
    # ⚠ 위치추정 노드가 혼자 죽어도 아무도 몰랐다. 화면은 계속 '위치추정 중',
    #   nav2 와 도착 후 동작은 멈춘 자세로 움직였다. 2026-09-30 15:4x 에 실제로
    #   죽었다(EKF 발산 -> slam_toolbox 세그폴트). 죽으면 세우고, 위치추정이었으면
    #   마지막으로 남긴 자리에서 다시 띄운다. 계속 죽으면 멈춘다.
    SLAM_RESTART_MAX = 3
    SLAM_RESTART_WIN = 600.0

    # ⚠ 2026-11-16 주행 S자: SLAM 이 0.4m 마다 map->odom 을 옆으로 5~15mm(최대 65mm)
    #   계단처럼 고쳐 nav2 가 그때마다 꺾었다. 위치추정 SLAM 은 TF 를 안 내고
    #   map_odom_smoother 가 그 보정을 0.x 초에 걸쳐 펴서 낸다. 매핑은 그대로.
    MAP_ODOM_SMOOTH = True
    SMOOTHER_EXE = "map_odom_smoother"

    def _smoother_ensure(self):
        """map_odom_smoother 를 하나 띄워 둔다. api_node 가 재시작해도 살아 있다."""
        if not self.MAP_ODOM_SMOOTH:
            return
        lk = getattr(self, "_smoother_lock", None)
        if lk is None or not lk.acquire(blocking=False):
            return
        try:
            now = time.monotonic()
            if now - getattr(self, "_smoother_t", -1e9) < 10.0:
                return
            self._smoother_t = now
            if self._pids_matching(self.SMOOTHER_EXE):
                return
            self._smoother_spawn()
        finally:
            lk.release()

    def _smoother_spawn(self):
        log_path = os.path.expanduser("~/projects/UND_AMR/logs/smoother.log")
        try:
            lf = open(log_path, "ab")
        except OSError:
            lf = subprocess.DEVNULL
        try:
            subprocess.Popen(["ros2", "run", "und_amr_bringup", self.SMOOTHER_EXE],
                             stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
            self.get_logger().info("map->odom 부드럽게 내기 노드를 띄움")
        except OSError as e:
            self.get_logger().error("map_odom_smoother 를 못 띄움: %s" % e)
        finally:
            if lf is not subprocess.DEVNULL:
                lf.close()

    def _slam_busy_add(self, n):
        with self._slam_busy_lock:
            self._slam_busy += n

    # ⚠ 2026-11-17 재부팅 뒤 nav2 가 안 떴다 (사용자: "nav2 가 떠 있지 않습니다"). 위치추정 SLAM 은 TF 를
    #   안 내고(map_odom_smoother) /pose 는 스캔을 처리할 때만 내는데, 첫 /pose 가 smoother 가 켜지기
    #   전에 와 버려졌고 로봇이 서 있어 다시 안 왔다 -> map 좌표 없음 -> nav2 는 위치추정을 기다림.
    #   위치추정인데 map 이 MAP_KICK_S 넘게 없으면 마지막 자리로 /initialpose 를 보내 다시 내게 한다.
    MAP_KICK_S = 5.0
    MAP_KICK_EVERY_S = 15.0

    def _map_kick_check(self):
        if not self.MAP_ODOM_SMOOTH or getattr(self, "slam_mode", None) != "localization":
            self._map_missing_t = None
            return
        if self._slam_busy or self.tf_pose() is not None:
            self._map_missing_t = None
            return
        now = time.monotonic()
        if getattr(self, "_map_missing_t", None) is None:
            self._map_missing_t = now
            return
        if now - self._map_missing_t < self.MAP_KICK_S:
            return
        if now - getattr(self, "_map_kick_t", -1e9) < self.MAP_KICK_EVERY_S:
            return
        start = self._load_pose(self.active_map) if self.active_map else None
        if not start:
            return
        self._map_kick_t = now
        self.get_logger().warn("위치추정 중인데 지도 좌표(map)가 %.0f초째 없음 — 마지막 자리 (%.2f, %.2f, %.0f도)"
                               "로 위치를 다시 알림" % (now - self._map_missing_t, start[0], start[1],
                                                       math.degrees(start[2])))
        self._publish_initialpose(start[0], start[1], start[2])

    def _motor_fault_check(self):
        """모터 알람이면 하던 이동을 다 끊는다 (2026-11-17 한쪽 바퀴만 돌아 제자리에서 돈 일)."""
        with self.lock:
            bad = [(k, d.get("message")) for k, d in getattr(self, "motors", {}).items()
                   if int(d.get("level", 0)) >= 2]
        if not bad:
            self._motor_fault_on = False
            return
        if getattr(self, "_motor_fault_on", False):
            return
        self._motor_fault_on = True
        busy = (self.nav_state in ("sending", "active")
                or getattr(self, "route", {}).get("state") in self.ROUTE_BUSY
                or getattr(self, "job", {}).get("state") in self.JOB_BUSY
                or getattr(self, "cdock", {}).get("state") in self.CD_BUSY)
        why = ", ".join("%s %s" % (k, m) for k, m in bad)
        self.get_logger().error("모터 이상 (%s) — %s" % (why, "하던 이동을 멈춤" if busy else "서 있음"))
        if busy:
            self.stop_now()
            self.nav_state, self.nav_msg = "failed", "모터 이상으로 정지했습니다 (%s)" % why

    def _slam_watch(self):
        try:
            self._motor_fault_check()
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("모터 감시 오류: %s" % e)
        try:
            self._map_kick_check()
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("지도 좌표 감시 오류: %s" % e)
        p = self.slam_proc
        if p is None or self._slam_busy or p.poll() is None:
            return
        mode = self.slam_mode
        self.slam_proc = None
        self.slam_mode = None
        if mode is None:
            return                       # 띄우다 실패한 것 — slam_spawn 이 이미 알렸다
        self.get_logger().error("SLAM(%s) 이 혼자 내려갔다 (종료 코드 %s) — 로봇을 세운다. "
                                "logs/slam.log 참고" % (mode, getattr(p, "returncode", None)))
        self.stop_now()
        if self.nav_state in ("idle", "canceled", "succeeded", "failed"):
            self.nav_state, self.nav_msg = "failed", "위치추정(SLAM)이 꺼져 정지했습니다"
        name = self.active_map
        if mode != "localization" or not name:
            return
        now = time.monotonic()
        hist = [t for t in self._slam_restarts if now - t < self.SLAM_RESTART_WIN]
        if len(hist) >= self.SLAM_RESTART_MAX:
            self._slam_restarts = hist
            self.get_logger().error("SLAM 이 %.0f분 안에 %d번 죽어 더 띄우지 않는다. "
                                    "지도를 다시 적용해야 한다"
                                    % (self.SLAM_RESTART_WIN / 60, len(hist)))
            return
        hist.append(now)
        self._slam_restarts = hist
        self._slam_busy_add(1)           # 띄우는 동안 감시가 끼어들지 않게
        threading.Thread(target=self._slam_respawn, args=(name,), daemon=True).start()

    def _slam_respawn(self, name):
        try:
            base = self.map_path(name)
            if self.slam_proc is not None or not base:
                return
            start = self._load_pose(name)
            ok = self.slam_spawn(mode="localization", map_base=base, start_pose=start)
            if ok:
                self.get_logger().warn("위치추정 다시 띄움 — 마지막 자리 %s 에서. "
                                       "자리가 맞는지 화면에서 확인할 것"
                                       % ("(%.2f, %.2f)" % start[:2] if start else "없음(지도 출발점)"))
                if self.nav_state == "failed" and "SLAM" in self.nav_msg:
                    self.nav_state, self.nav_msg = "idle", ""
            else:
                self.get_logger().error("위치추정 다시 띄우기 실패")
        finally:
            self._slam_busy_add(-1)

    # ---- api_node 재시작에도 위치추정을 이어 쓴다 ----
    #
    # ⚠ 2026-11-13 "재시작할때마다 라이다 위치가 조금씩 틀어짐": api_node 가 꺼질 때
    #   SLAM 을 같이 내리고, 켜질 때 1초 주기로 저장한 '마지막 자리' 에서 새로 띄워
    #   스캔을 다시 맞췄다. 맞춤 결과가 매번 몇 cm 씩 달랐고, 손으로 고정한 자리
    #   (slam pause)도 풀렸다. 이제 위치추정 모드면 꺼질 때 SLAM 을 살려 두고,
    #   다시 켜지면 같은 지도로 돌고 있는 그 SLAM 을 그대로 이어받는다.
    #   단, odom(EKF/base_node)이 SLAM 보다 나중에 다시 켜졌으면 odom 이 0 으로
    #   돌아가 SLAM 이 그 점프를 받으므로 이어받지 않고 예전처럼 새로 띄운다.

    class _AdoptedProc:
        """이전 api_node 가 띄운 SLAM. Popen 처럼 poll()/pid 만 흉내 낸다."""

        def __init__(self, pid):
            self.pid = pid

        def poll(self):
            try:
                with open("/proc/%d/stat" % self.pid) as f:
                    st = f.read().rsplit(")", 1)[1].split()[0]
                return 0 if st in ("Z", "X") else None
            except (OSError, IndexError):
                return 0

    def _slam_pid_file(self):
        return os.path.join(self.map_dir, ".slam_proc")

    def _slam_pid_write(self, pid, mode=None, map_base=None):
        try:
            if pid is None:
                if os.path.exists(self._slam_pid_file()):
                    os.remove(self._slam_pid_file())
                return
            with open(self._slam_pid_file(), "w", encoding="utf-8") as f:
                f.write("%d %s %s" % (pid, mode, map_base or ""))
        except OSError:
            pass

    @staticmethod
    def _proc_start(pid):
        """프로세스 시작 시각 (부팅 후 jiffies). 없으면 None."""
        try:
            with open("/proc/%d/stat" % pid) as f:
                return int(f.read().rsplit(")", 1)[1].split()[19])
        except (OSError, IndexError, ValueError):
            return None

    @staticmethod
    def _pids_matching(word):
        """실행 파일(인자 0·1번)이 word 로 끝나는 프로세스. 셸 명령 문자열은 안 걸린다."""
        out = []
        for d in os.listdir("/proc"):
            if not d.isdigit():
                continue
            try:
                with open("/proc/%s/cmdline" % d, "rb") as f:
                    av = f.read().decode(errors="ignore").split("\0")
            except OSError:
                continue
            if any(a.endswith(word) for a in av[:2]):
                out.append(int(d))
        return out

    def _slam_adopt(self, map_base):
        """같은 지도로 도는 위치추정 SLAM 이 있으면 이어받는다. 이어받았으면 True."""
        try:
            with open(self._slam_pid_file(), encoding="utf-8") as f:
                parts = f.read().split()
        except OSError:
            return False
        if len(parts) != 3:
            return False
        try:
            pid = int(parts[0])
        except ValueError:
            return False
        proc = self._AdoptedProc(pid)
        if proc.poll() is not None:
            self._slam_pid_write(None)
            return False
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                cl = f.read().replace(b"\0", b" ").decode(errors="ignore")
        except OSError:
            cl = ""
        same = (parts[1] == "localization" and parts[2] == map_base
                and "localization_slam_toolbox_node" in cl
                and ("map_file_name:=" + map_base) in cl)
        s0 = self._proc_start(pid)
        odom = [self._proc_start(q) for q in
                self._pids_matching("/ekf_node") + self._pids_matching("/und_amr_bringup/base_node")]
        odom_newer = any(t is not None and s0 is not None and t > s0 for t in odom)
        if not same or odom_newer:
            why = "지도가 다름" if not same else "odom 이 SLAM 뒤에 다시 켜짐"
            self.get_logger().info("남아 있던 SLAM(pid %d) 을 내리고 새로 띄운다 — %s" % (pid, why))
            self.slam_proc = proc
            self._slam_kill()
            return False
        self.slam_proc = proc
        self.slam_mode = "localization"
        self.get_logger().info("돌고 있던 위치추정 SLAM(pid %d) 을 그대로 이어받음 — 위치 다시 안 맞춤" % pid)
        return True

    def slam_detach_or_kill(self):
        """꺼질 때: 위치추정이면 SLAM 을 살려 둔다(다음 기동이 이어받음). 매핑이면 내린다."""
        if (self.slam_mode == "localization" and self.slam_proc is not None
                and self.slam_proc.poll() is None):
            self.get_logger().info("위치추정 SLAM 은 살려 둔다 (다음 기동이 이어받음)")
            return
        self.slam_kill()

    def slam_kill(self, timeout=6.0):
        self._slam_busy_add(1)
        try:
            return self._slam_kill(timeout)
        finally:
            self._slam_busy_add(-1)

    def slam_spawn(self, *a, **kw):
        self._slam_busy_add(1)
        try:
            return self._slam_spawn(*a, **kw)
        finally:
            self._slam_busy_add(-1)

    def _slam_kill(self, timeout=6.0):
        """우리가 띄운 노드만 내린다. 외부(launch)에서 띄운 것은 손대지 않는다."""
        if self.slam_proc is None or self.slam_proc.poll() is not None:
            self.slam_proc = None
            return False
        try:
            os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGINT)
        except OSError:
            pass
        t0 = time.monotonic()
        while self.slam_proc.poll() is None and time.monotonic() - t0 < timeout:
            time.sleep(0.1)
        if self.slam_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGKILL)
            except OSError:
                pass
        self.slam_proc = None
        self.slam_mode = None
        self._slam_pid_write(None)
        return True

    def _slam_spawn(self, wait=20.0, mode="mapping", map_base=None,
                   start_pose=None):
        """SLAM 노드를 띄운다.

        mode="mapping"      : 백지에서 지도를 그린다.
        mode="localization" : map_base 의 포즈그래프를 올려 위치만 추정한다.
                              스캔이 들어와도 지도가 늘어나지 않는다.
        """
        if mode == "localization":
            if not map_base or not os.path.exists(map_base + ".posegraph"):
                return False
            exe = "localization_slam_toolbox_node"
        else:
            exe = "async_slam_toolbox_node"
            # map_base 를 주면 '이어서 매핑' 이다. 저장된 포즈그래프를 올린 뒤
            # 그 위에 계속 그린다. 안 주면 여느 때처럼 백지에서 시작한다.
            if map_base and not os.path.exists(map_base + ".posegraph"):
                return False
        cmd = ["ros2", "run", "slam_toolbox", exe]
        if self.slam_cfg:
            cmd += ["--ros-args", "--params-file", self.slam_cfg]
        else:
            cmd += ["--ros-args"]
        if mode == "localization":
            cmd += ["-p", "mode:=localization"]
            if self.MAP_ODOM_SMOOTH:
                # TF 는 map_odom_smoother 가 낸다 (SLAM 은 /pose 만)
                self._smoother_t = -1e9
                self._smoother_ensure()
                cmd += ["-p", "transform_publish_period:=0.0"]
        if map_base:
            cmd += ["-p", "map_file_name:=" + map_base]
            # 시작 자세를 안 주면 slam_toolbox 는 지도의 '출발점' 에 로봇이
            # 있다고 보고 시작한다(map_start_at_dock). 실제 위치가 다르면
            # 그 차이만큼 스캔이 안 맞고, 엉뚱한 곳에 새 영역을 그려 지도가
            # 대각선으로 늘어난다(2026-08-26 에 겪은 문제). 실제 위치를 넘긴다.
            if start_pose is not None:
                cmd += ["-p", "map_start_pose:=[%.4f, %.4f, %.4f]" % start_pose]
            else:
                cmd += ["-p", "map_start_at_dock:=true"]
        # 출력은 logs/slam.log 에 남긴다. DEVNULL 로 버렸더니 2026-09-30 오후에
        # SLAM 이 혼자 내려갔을 때 원인을 알 길이 없었다.
        # 20MB 를 넘으면 .1 로 한 번 돌린다. 띄울 때마다 구분 줄을 남긴다.
        log_path = os.path.expanduser("~/projects/UND_AMR/logs/slam.log")
        try:
            if os.path.getsize(log_path) > 20 * 1024 * 1024:
                os.replace(log_path, log_path + ".1")
        except OSError:
            pass
        try:
            slam_log = open(log_path, "ab")
            slam_log.write(("\n===== %s %s %s\n" % (
                time.strftime("%Y-%m-%d %H:%M:%S"), mode,
                os.path.basename(map_base or ""))).encode())
            slam_log.flush()
        except OSError:
            slam_log = subprocess.DEVNULL
        try:
            self.slam_proc = subprocess.Popen(
                cmd, stdout=slam_log, stderr=subprocess.STDOUT,
                start_new_session=True)
            self._slam_pid_write(self.slam_proc.pid, mode, map_base)
        finally:
            if slam_log is not subprocess.DEVNULL:
                slam_log.close()     # 자식이 따로 들고 있다. 부모 쪽은 닫는다
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            # 위치추정 노드는 save_map 서비스를 내지 않으므로, 그때는
            # map->odom TF 가 나오는지로 기동을 판단한다.
            if mode == "mapping":
                cli = self.slam.get("save")
                if cli is not None and cli.wait_for_service(timeout_sec=0.5):
                    self.slam_mode = mode
                    return True
            else:
                time.sleep(0.5)
                if self.tf_pose("map", "base_footprint") is not None:
                    self.slam_mode = mode
                    return True
            if self.slam_proc.poll() is not None:
                self.slam_mode = None
                return False
        # 위치추정은 TF 가 늦게 나올 수 있다. 프로세스가 살아 있으면 성공으로 본다.
        if mode == "localization" and self.slam_proc.poll() is None:
            self.slam_mode = mode
            return True
        self.slam_mode = None
        return False

    def _forget_map(self):
        with self.lock:
            self.map_png = None
            self.map_base_png = None
            self.map_loc_png = None
            self.map_edits = []
            self.pois = []
            self.map_meta = {}
            self.map_source = None

    def mapping_start(self):
        """백지에서 새로 시작. 노드를 내렸다 다시 띄운다."""
        external = self.slam_proc is None and self.slam_is_running()
        if external:
            return (False, "다른 곳에서 띄운 SLAM 이 실행 중입니다. "
                           "admin.launch.py 를 slam:=false 로 띄우세요")
        self.slam_kill()
        self.bag_stop()
        self._rm(self.bag_tmp)
        self._forget_map()
        self.mapping_paused = False
        self.active_map = None
        self.resumed_from = None
        if not self.slam_spawn():
            return False, "SLAM 노드 기동 실패"
        with self.lock:
            self._sess_new()
        rec = self.bag_start()
        return True, ("새 매핑을 시작했습니다" +
                      (" (주행 기록 중)" if rec else ""))

    def can_resume(self):
        """이어서 매핑이 가능한가. 안 되면 왜 안 되는지 한 줄로 돌려준다.

        화면에서 버튼을 회색으로만 두면 고장으로 읽힌다. 못 누르는 이유를
        그 자리에 적어 주려고 사유를 함께 낸다.
        """
        if self.slam_mode == "mapping":
            return False, "이미 매핑 중입니다"
        if self.slam_mode != "localization":
            if self.active_map:
                return False, ("'%s' 위에서 위치추정 중이 아닙니다 — "
                               "맵 화면에서 다시 적용하세요" % self.active_map)
            return False, "맵 화면에서 지도를 먼저 적용하세요"
        name = self.active_map
        base = self.map_path(name) if name else None
        if not base or not os.path.exists(base + ".posegraph"):
            return False, "이 지도에는 포즈그래프가 없습니다 (.posegraph 필요)"
        if self.tf_pose("map", "base_footprint") is None:
            return False, "로봇 위치를 먼저 잡아 주세요 (지도에서 로봇을 누름)"
        return True, None

    def mapping_resume(self):
        """지금 보고 있는 지도 위에 이어서 매핑한다.

        위치추정 중일 때만 된다. 로봇이 지도 어디에 있는지 알아야 그 자리에서
        이어 그릴 수 있기 때문이다. 제약이 아니라 순서다 - 위치를 모르는 채로
        이어 그리면 엉뚱한 곳에 새 영역이 생겨 지도가 망가진다.
        """
        ok, why = self.can_resume()
        if not ok:
            return False, why
        name = self.active_map
        base = self.map_path(name)
        pose = self.tf_pose("map", "base_footprint")
        if pose is None:
            return False, "로봇 위치를 잃었습니다. 다시 시도해 주세요"
        start = (pose["x"], pose["y"], pose["yaw"])
        self.slam_kill()
        if not self.slam_spawn(mode="mapping", map_base=base,
                               start_pose=start):
            # 매핑 기동에 실패한 채로 두면 map 프레임이 없어 로봇이 지도를
            # 통째로 잃는다. 있던 자리로 되돌린다.
            self.slam_spawn(mode="localization", map_base=base,
                            start_pose=start)
            return False, "매핑 노드 기동 실패 (위치추정으로 되돌렸습니다)"
        self.mapping_paused = False
        # 저장 없이 끝냈을 때 돌아갈 자리. 백지에서 시작한 것이 아니므로
        # 버릴 것은 이번에 덧그린 부분뿐이고 원래 지도는 남아야 한다.
        self.resumed_from = name
        with self.lock:
            self._sess_new()
        rec = self.bag_start()
        self.get_logger().info(
            "'%s' 에 이어서 매핑 시작 — 시작 자세 (%.2f, %.2f, %.1f°)"
            % (name, start[0], start[1], math.degrees(start[2])))
        return True, ("'%s' 에 이어서 매핑합니다%s"
                      % (name, " (주행 기록 중)" if rec else ""))

    def mapping_stop(self):
        # 매핑 중이 아닐 때 종료가 불리면 위치추정 SLAM 을 내리고 메인 맵
        # 설정까지 지워 버린다. 화면 버튼이 잘못 활성화된 적이 있어 서버에서도 막는다.
        if self.slam_mode != "mapping":
            if self.slam_mode == "localization":
                return False, "매핑 중이 아닙니다 (메인 맵 위에서 위치추정 중)"
            return False, "매핑 중이 아닙니다"

        # SLAM 을 내리면 map 프레임이 사라진다. 위치추정으로 이어가려면
        # 그 전에 지금 로봇이 지도 어디에 있는지 받아 두어야 한다.
        pose = self.tf_pose("map", "base_footprint")
        start = ((pose["x"], pose["y"], pose["yaw"]) if pose else None)
        if not self.slam_kill():
            return False, "api_node 가 띄운 SLAM 이 없습니다"
        # 저장하지 않고 끝낸 세션의 녹화는 버린다 (남겨도 지도가 없어 쓸모없다)
        self.bag_stop()
        dropped = self._dir_size(self.bag_tmp) if os.path.isdir(self.bag_tmp) else 0
        self._rm(self.bag_tmp)
        self.mapping_paused = False
        with self.lock:
            saved = self.last_saved
            self.sess = None
        # 저장하고 끝냈으면 그 지도가 곧 메인 맵이 된다. 저장 없이 끝냈으면
        # 남길 것이 없으므로 화면을 비운다.
        adopted = self.adopt_saved_map(saved) if saved else False
        # 이어서 매핑을 저장 없이 끝낸 경우. 백지에서 시작한 것이 아니므로
        # 버릴 것은 이번에 덧그린 부분뿐이다. 멀쩡한 원래 지도까지 없애면
        # 로봇이 지도를 통째로 잃는다.
        back = None
        if not adopted and self.resumed_from:
            back = self.resumed_from
            adopted = self.adopt_saved_map(back)
        if not adopted:
            self._forget_map()
            self.active_map = None
            self._remember_active(None)
        with self.lock:
            self.last_saved = None
        self.resumed_from = None
        if adopted:
            # 지도만 띄우면 로봇이 그 위 어디에 있는지 알 수 없다.
            # 위치추정 모드로 다시 띄워 map->odom TF 를 살린다. 이 모드에서는
            # 새 스캔이 들어와도 지도가 늘어나지 않는다.
            name = back or saved
            loc = self.slam_spawn(mode="localization",
                                  map_base=self.map_path(name),
                                  start_pose=start)
            tail = "" if loc else " (위치추정 기동 실패)"
            if back:
                return True, ("매핑 종료 — 저장하지 않아 '%s' 로 되돌렸습니다%s"
                              % (back, tail))
            return True, ("매핑 종료 — '%s' 를 메인 맵으로 적용했습니다%s"
                          % (name, tail))
        msg = "매핑 종료"
        if dropped > 65536:
            msg += " (저장 안 한 기록 %s 삭제)" % self._human(dropped)
        return True, msg

    def map_path(self, name):
        """경로 조작 문자를 걸러 map_dir 밖으로 못 나가게 한다."""
        safe = "".join(c for c in name if c.isalnum() or c in "-_")
        if not safe:
            return None
        return os.path.join(self.map_dir, safe)

    def maps_detail(self):
        out = []
        for m in self.maps_list():
            info = dict(m)
            try:
                with open(m["yaml"]) as f:
                    for line in f:
                        k, _, v = line.partition(":")
                        if k.strip() == "resolution":
                            info["resolution"] = float(v)
                        elif k.strip() == "origin":
                            info["origin"] = v.strip()
            except OSError:
                pass
            pgm = m["image"]
            info["has_image"] = os.path.exists(pgm)
            info["has_posegraph"] = os.path.exists(
                os.path.join(self.map_dir, m["name"] + ".posegraph"))
            if info["has_image"] and HAS_IMAGING:
                try:
                    with PilImage.open(pgm) as im:
                        info["width"], info["height"] = im.size
                        r = info.get("resolution", 0.05)
                        info["area"] = "%.1f x %.1f m" % (im.size[0] * r,
                                                          im.size[1] * r)
                except OSError:
                    pass
            info["active"] = (m["name"] == self.active_map)
            bag = os.path.join(self.map_dir, m["name"] + ".bag")
            info["has_bag"] = os.path.isdir(bag)
            info["bag_mb"] = (round(self._dir_size(bag) / 1048576.0, 1)
                              if info["has_bag"] else 0)
            extra = self.map_info(m["name"])
            for k in ("duration_sec", "distance_m", "rotation_deg",
                      "scans", "note", "saved_at"):
                if extra.get(k) is not None:
                    info[k] = extra[k]
            info["has_info"] = bool(extra)
            total = 0
            for f in os.listdir(self.map_dir):
                if not f.startswith(m["name"] + "."):
                    continue
                fp = os.path.join(self.map_dir, f)
                total += (self._dir_size(fp) if os.path.isdir(fp)
                          else os.path.getsize(fp))
            info["size_kb"] = round(total / 1024)
            out.append(info)
        return out

    def map_preview(self, name, max_px=520):
        base = self.map_path(name)
        if not base or not HAS_IMAGING:
            return None
        pgm = base + ".pgm"
        if not os.path.exists(pgm):
            return None
        try:
            with PilImage.open(pgm) as im:
                im = im.convert("L")
                # 지도는 보통 수십~수백 px 로 작다. 브라우저가 늘리면 뭉개지므로
                # 정수배로만 키워 격자를 또렷하게 유지한다(축소는 그대로).
                big = max(im.size)
                if big > max_px:
                    k = max_px / big
                    im = im.resize((max(1, int(im.width * k)),
                                    max(1, int(im.height * k))),
                                   PilImage.NEAREST)
                elif big > 0:
                    z = max(1, min(8, int(max_px // big)))
                    if z > 1:
                        im = im.resize((im.width * z, im.height * z),
                                       PilImage.NEAREST)
                buf = io.BytesIO()
                im.save(buf, format="PNG")
                return buf.getvalue()
        except OSError:
            return None

    def _active_file(self):
        return os.path.join(self.map_dir, ".active_map")

    def _pose_file(self):
        return os.path.join(self.map_dir, ".last_pose")

    def _remember_pose(self):
        """지금 로봇이 지도 어디에 있는지 파일에 남긴다.

        노드를 다시 띄우면 slam_toolbox 는 지도의 '출발점' 에서 시작한다.
        실제 위치와 다르면 매번 사람이 위치를 다시 잡아 줘야 하고, 전역
        탐색은 1분 넘게 걸린다. 마지막으로 알던 자리를 남겨 두면 그 자리에서
        시작하므로, 그 사이 로봇을 밀지 않았다면 그대로 맞는다.
        밀렸더라도 지도 출발점보다는 가까워 자동 재조정이 훨씬 쉬워진다.
        """
        if self.slam_mode != "localization":
            return                       # 매핑 중 자세는 기준이 계속 바뀐다
        pose = self.tf_pose("map", "base_footprint")
        if pose is None:
            return
        line = "%s %.4f %.4f %.4f" % (self.active_map or "", pose["x"],
                                      pose["y"], pose["yaw"])
        if line == getattr(self, "_pose_line", None):
            return                       # 안 움직였으면 쓰지 않는다
        if not self._pose_sane(pose["x"], pose["y"], pose["yaw"]):
            return
        # 임시 파일에 쓰고 바꿔 끼운다. 쓰는 중 전원이 나가도 반쪽 줄이 남지 않는다.
        try:
            os.makedirs(self.map_dir, exist_ok=True)
            tmp = self._pose_file() + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(line)
            os.replace(tmp, self._pose_file())
            self._pose_line = line
        except OSError:
            pass

    def _pose_sane(self, x, y, yaw):
        """지도 안(1m 여유)의 유한한 값인가.

        ⚠ 2026-09-30 EKF 가 수백만 m 로 튀었을 때 그 값이 그대로 저장돼, 다음
          기동의 위치추정이 거기서 시작하다 SLAM 이 죽었다."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False
        m = getattr(self, "map_meta", None) or {}
        try:
            res = float(m["resolution"])
            ox, oy = float(m["origin"]["x"]), float(m["origin"]["y"])
            w, h = float(m["width"]) * res, float(m["height"]) * res
        except (KeyError, TypeError, ValueError):
            return abs(x) < 500.0 and abs(y) < 500.0   # 지도 정보가 없을 때
        return ox - 1.0 <= x <= ox + w + 1.0 and oy - 1.0 <= y <= oy + h + 1.0

    def _load_pose(self, name):
        """저장해 둔 자세를 읽는다. 다른 지도의 것이면 쓰지 않는다."""
        try:
            with open(self._pose_file(), encoding="utf-8") as f:
                parts = f.read().split()
        except OSError:
            return None
        if len(parts) != 4 or parts[0] != name:
            return None
        try:
            pose = (float(parts[1]), float(parts[2]), float(parts[3]))
        except ValueError:
            return None
        if not self._pose_sane(*pose):
            self.get_logger().warn("저장된 자리 %s 가 지도 밖이라 쓰지 않는다" % " ".join(parts[1:]))
            return None
        return pose

    def _remember_active(self, name):
        """메인 맵 이름을 파일에 남긴다.

        메모리에만 두면 api_node 를 다시 띄울 때마다 메인 맵이 사라진다.
        운영 중 노드가 재시작되는 일은 얼마든지 있으므로 디스크에 남긴다."""
        try:
            if name:
                os.makedirs(self.map_dir, exist_ok=True)
                with open(self._active_file(), "w", encoding="utf-8") as f:
                    f.write(name)
            elif os.path.exists(self._active_file()):
                os.remove(self._active_file())
        except OSError as e:
            self.get_logger().warn("메인 맵 기록 실패: %s" % e)

    def restore_active_map(self):
        """기동 시 지난 메인 맵을 되살린다. 지도 표시와 위치추정까지 복구한다."""
        try:
            with open(self._active_file(), encoding="utf-8") as f:
                name = f.read().strip()
        except OSError:
            return False
        if not name or not self.adopt_saved_map(name):
            return False
        base = self.map_path(name)
        start = self._load_pose(name)
        if base and os.path.exists(base + ".posegraph"):
            if self._slam_adopt(base):
                self.get_logger().info("지난 메인 맵 '%s' 복구 — 위치추정은 이어받음" % name)
                return True
            # 마지막으로 알던 자리에서 시작한다. 없으면 지도 출발점.
            self.slam_spawn(mode="localization", map_base=base,
                            start_pose=start)
        if start:
            self.get_logger().info(
                "지난 메인 맵 '%s' 복구 — 마지막 자리 (%.2f, %.2f, %.0f°) 에서 시작"
                % (name, start[0], start[1], math.degrees(start[2])))
        else:
            self.get_logger().info(
                "지난 메인 맵 '%s' 복구 — 저장된 자리가 없어 지도 출발점에서 시작"
                % name)
        return True

    def adopt_saved_map(self, name):
        """저장된 지도를 '메인 맵' 으로 채택한다.

        매핑을 끝내면 SLAM 이 내려가 /map 이 끊긴다. 그때 화면을 비워 버리면
        방금 만든 지도를 어디서도 볼 수 없다. 저장된 .pgm/.yaml 을 직접 읽어
        대시보드·매핑·맵 화면이 그대로 쓰게 한다. SLAM 없이 표시만 하는 것이라
        이어서 매핑하려면 map_load 로 포즈그래프를 다시 올려야 한다.
        """
        base = self.map_path(name)
        if not base or not HAS_IMAGING:
            return False
        pgm, yml = base + ".pgm", base + ".yaml"
        if not (os.path.exists(pgm) and os.path.exists(yml)):
            return False
        try:
            with open(yml, encoding="utf-8") as f:
                info = yaml.safe_load(f) or {}
            res = float(info.get("resolution", 0.05))
            org = info.get("origin") or [0.0, 0.0, 0.0]
            ox, oy = float(org[0]), float(org[1])
            oyaw = float(org[2]) if len(org) > 2 else 0.0
            # 저장된 pgm 은 이미 이미지 좌표계라 live 쪽과 달리 뒤집지 않는다.
            with PilImage.open(pgm) as im:
                im = im.convert("L")
                w, h = im.size
                buf = io.BytesIO()
                im.save(buf, format="PNG")
                png = buf.getvalue()
        except (OSError, ValueError, TypeError, IndexError, KeyError,
                yaml.YAMLError):
            return False
        edits = self.load_edits(os.path.basename(base))
        pois = self.load_pois(os.path.basename(base))
        with self.lock:
            self.map_source = "saved"
            self.map_base_png = png
            self.map_edits = edits
            # POI 는 지도 좌표라, 지도가 바뀌면 그 지도의 것을 써야 한다.
            self.pois = pois
            self.map_meta = {
                "available": True, "frame": "map",
                "width": w, "height": h, "resolution": res,
                "origin": {"x": ox, "y": oy, "yaw": oyaw},
            }
            self._compose_map()
        self.active_map = os.path.basename(base)
        self._remember_active(self.active_map)
        self.get_logger().info("'%s' 를 메인 맵으로 적용" % self.active_map)
        return True

    # -------- 지도 편집 --------
    #
    # 매핑할 때는 있었지만 지금은 치워진 물건이 지도에 검게 남는다. 그 자리를
    # 지워 다시 '갈 수 있는 곳' 으로 되돌린다.
    #
    #   * 픽셀이 아니라 미터로 저장한다. 이어서 매핑하면 지도가 넓어지며
    #     원점이 옮겨 가는데, 픽셀로 저장한 편집은 그때 통째로 어긋난다.
    #   * 원본 .pgm 은 고치지 않는다. 지운 자리에 사실 벽이 있었다면 목록에서
    #     빼기만 하면 되돌아온다.
    #   * op 를 두는 이유는 나중에 금지구역(keepout)·가상벽(wall)이 같은
    #     파일에 들어오기 때문이다. 지금 쓰는 것은 erase 뿐이다.
    #
    # 편집은 slam_toolbox 에 반영되지 않는다. 그쪽은 포즈그래프의 원본 스캔으로
    # 위치를 맞추므로 지도를 지워도 그대로다. 이것이 맞는 동작이다 - 치워진
    # 물건을 지웠다고 그때 찍힌 스캔까지 버리면 위치추정만 나빠진다.

    def edits_path(self, name):
        return os.path.join(self.map_dir, name + ".edits.json")

    # ------------------------------------------------------------- 작업(진입)
    #
    # POI 로 가서 대차를 찾아 밑으로 들어간다. 단계는 다음과 같다.
    #
    #   moving    POI 앞 standoff 까지 nav2 로 간다. 도착 후 방향 맞추기 포함
    #   looking   그 자리에서 대차를 찾는다. 카메라가 다리를 3개 이상 확인해야
    #             넘어간다 — 라이다만으로는 사각형이 우연히 맞는 유령이 있다
    #   staging   제자리에서 한 번에 부드럽게 180도 돌아선다
    #   entering  후진으로 들어간다. 대차 중앙에 로봇 중심이 오면 선다
    #
    # ⚠ 단계를 줄이는 것이 곧 부드러움이다. 예전에는 도착해서 자세를 맞추고,
    #   찾고, 물러났다 오고, 축을 맞추느라 멈춤이 여섯 번이었다. 지금은
    #   '가면서 인식 → 한 번 돌기 → 후진' 셋뿐이다(2026-09-16).
    #
    # ⚠ 왜 뒤로 들어가는가 — 라이다와 카메라가 앞에 달려 있다. 앞으로 들어가면
    #   대차 밑으로 센서가 먼저 들어가 바깥을 못 본다. 뒤로 들어가면 센서가
    #   계속 바깥을 향한다. 나중에 리프팅을 붙일 때도 이 방향이 맞다.
    #
    # ⚠ 진입 구간은 nav2 를 거치지 않고 /cmd_vel 로 직접 몬다. 충돌감시의
    #   정지 구역이 앞 0.70m 라 대차 다리가 그 안에 들어와 로봇이 서 버리기
    #   때문이다. 다각형은 실행 중에 못 바꾼다(2026-09-14 확인). 그래서 이
    #   구간의 안전은 여기서 직접 진다 — 진입 거리 상한, 정렬이 무너지면
    #   중단, 시간 초과, 그리고 스캔이 끊기면 멈추는 기존 감시.
    #
    # ⚠ 단계 전환은 이 타이머 하나로만 한다. 단계마다 타이머를 따로 만들면
    #   버려진 타이머가 남아 상태를 덮어쓴다(2026-09-14 에 겪음).

    JOB_HZ          = 20.0
    # ⚠ 찾으러 더 가는 시간(450mm / 0.06 m/s = 7.5초)이 이 안에 들어야
    #   한다. 10초면 인식할 여유가 2.5초뿐이다.
    JOB_LOOK_S      = 20.0    # 대차를 찾는 데 주는 시간
    # 도착해서 안 보이면 렉 쪽으로 곧장 조금 다가가며 찾는다.
    #
    # ⚠ ls_origin 은 1.45m 에선 늘 보이고 1.54m 에선 안 보였다(2026-09-28).
    #   대기 거리 1.5m 에 nav2 도착 오차 ±40mm 가 얹히면 경계를 넘나든다.
    #   되돌아오지 않고 한 방향으로만 기어가므로 9/21 에 뺀 '왔다갔다' 와
    #   다르다. 보이면 그 자리에서 멈추고 평소대로 이어 간다.
    JOB_FIND_WAIT_S = 3.0     # 이만큼 안 보이면 다가가기 시작 [s]
    JOB_FIND_CREEP_M = 0.40   # 최대로 다가갈 거리 [m]
    # ⚠ 2026-11-13 1.00 -> 1.40: 카메라가 넥 위(1.0m)로 올라가 렉에 너무 다가가면 앞 기둥이
    #   컬러 화면(위아래 ±21.6도) 밖이 된다. 렉 중심 1.40m 에서 카메라 확인이 통과했다
    #   (사용자: "원래는 너무 가까워서 카메라가 못찾음 이정도 위치는되어야 카메라가 인식을 함").
    JOB_FIND_NEAR_M = 1.40    # 렉 중심에 이보다 가까워지면 더 안 간다 [m]
    JOB_FIND_V      = 0.05    # 다가가는 속도 [m/s]
    JOB_FIND_YAW    = 0.35    # 렉(POI)을 이 각(20도) 안으로 보고 있어야 한다
    # 최근 JOB_LOOK_W 번 중 JOB_LOOK_N 번 잡히면 진짜로 본 것으로 친다.
    #
    # ⚠ '연속 3번' 이었다. 그런데 이 렉은 크롬 기둥 하나가 간헐적으로
    #   라이다에 안 잡혀 후보가 4개 -> 3개로 떨어진다. 실측에서 인식 6번,
    #   못 찾음 5번이었다(2026-09-21). 한 번만 끊겨도 처음부터 다시 세니
    #   좀처럼 통과하지 못했다. 연속을 요구하지 말고 최근 몇 번을 본다.
    JOB_LOOK_N      = 3
    JOB_LOOK_W      = 6
    JOB_FRESH_S     = 1.0     # 인식 결과가 이보다 오래되면 없는 것으로
    JOB_YAW_TOL     = 0.020   # 약 1.1도. 이 안이면 나란하다고 본다
    # 좌우 어긋남 문턱.
    #
    # ⚠ 처음에 60mm 로 두었다가 멀쩡한 시도를 거절했다(2026-09-16). POI 를
    #   그 자리에서 저장해도, 접근 자리로 다시 가는 동안 오차가 쌓인다 —
    #   도착 위치 오차 28mm + 방향 오차 1도(1.4m 에서 25mm) + 인식 잡음.
    #   63mm 는 그 합이지 잘못된 자리가 아니었다.
    #
    #   그리고 그 정도는 막을 일도 아니다. 다리 사이가 574mm, 로봇 폭이
    #   270mm 라 한쪽에 152mm 씩 남는다. 1.4m 들어가며 2.6도만 꺾으면 된다.
    #   그래서 문턱은 'POI 가 명백히 틀렸다' 수준으로만 두고, 어긋남은
    #   진입하면서 잡는다.
    # 진입을 시작해도 되는 좌우 어긋남. 넘으면 자리를 다시 잡는다.
    #
    # ⚠ 예전 값 250mm 는 들어갈 수 없는 값이었다. 차체 반폭 135mm,
    #   다리 사이 반폭 287mm(lg_custom) 라 한쪽 여유가 152mm 다.
    #   250mm 어긋난 채로 들어가면 반드시 닿는다(2026-09-21).
    JOB_LAT_MAX     = 0.10
    # 차체 반폭·반길이. nav2.yaml 의 footprint 와 같아야 한다.
    # ⚠ 2026-11-13 반폭 0.135 -> 0.1644: 10/26 차체 변경(폭 270 -> 328.8mm)이 여기 안 들어가
    #   있었다(nav2 footprint 는 0.165). ls_origin(통과 폭 344mm)이면 한쪽 여유가 37mm 가 아니라
    #   7.6mm 다 — 진입 중 좌우 보정 한도(_job_free x JOB_LAT_SAFE)가 그만큼 작아진다.
    JOB_HALF_W      = 0.1644
    # ⚠ 2026-11-13 오른쪽 뒤 라이다가 차체 옆면 밖으로 튀어나와(약 18mm 로 가정 — 실측 아님) 렉 판
    #   높이(~200mm, 라이다 스캔면 201.5mm)에서 차체가 오른쪽 뒤만 넓다. 정렬 +3mm 로 잘 들어가고도
    #   '뒤에서 걸림' — 라이다 쪽(사용자 확인). 사용자 결정 "왼쪽으로 치우쳐 들어가기": 진입 줄을
    #   왼쪽으로 돌출의 절반 옮기고, 판정 반폭도 그만큼 키운다(양쪽 여유 같게, 358mm 렉에서 약 5.6mm).
    # 2026-11-13 사용자: 라이다는 날개 밖으로 안 나온다 -> 0. 오른쪽이 걸린 것은 렉 쪽 비대칭
    #   (carts.yaml center_offset 7mm, 9/28 기록) 으로 본다.
    JOB_REAR_R_PROTRUDE = 0.0
    JOB_HALF_L      = 0.43675
    # 진입 조향이 지울 수 있다고 계산된 양 중 이 비율까지만 허용한다.
    # 다 쓰면 조금만 모자라도 다리에 닿는다. 나머지는 인식 떨림과
    # 캐스터에 밀리는 몫이다.
    JOB_LAT_SAFE    = 0.75
    JOB_LAT_NEAR    = 0.12    # 다 들어간 시점에 이보다 어긋나 있으면 중단 [m]
    JOB_NEAR_M      = 0.50    # '다 들어간' 으로 보는 남은 거리 [m]
    JOB_FIND_M      = 0.45
    # 가는 동안 라이다로 본 대차에 맞춰 목표를 다시 잡는다.
    #
    # ⚠ 서는 자리를 POI 로만 계산했었다. POI 는 '대차를 등록했을 때의 자리'
    #   라, 대차가 밀렸거나 돌아가 있으면 로봇이 대차 축에서 벗어난 자리에
    #   선다. 후진 조향으로 지울 수 있는 것은 100mm 쯤이 한계인데(회전 반경
    #   4m) 다리까지 여유는 한쪽에 152mm 뿐이다 — 그대로 다리에 닿았다
    #   (2026-09-21 지적). 실제로 본 대차로 다시 잡는다.
    JOB_AIM_M       = 0.04    # 목표가 이만큼 어긋나면 다시 건다
    # 남은 거리가 이보다 짧으면 안 건드린다.
    #
    # ⚠ 0.60 은 이 렉에서 한 번도 못 넘는 값이었다. 렉이 보이기 시작하는
    #   거리가 1.73m 인데 설 자리가 1.40m 라 남은 거리가 0.33m 뿐이다.
    #   그래서 멈추는 자리가 늘 POI 로만 정해졌다 — POI 와 렉이 어긋나
    #   있으면 그만큼 축 밖에 섰다(2026-09-21 지시: "라이다 박스 인식되면
    #   그걸 기준 중심점으로").
    #
    #   목표만 바꾸는 것이라 갔다가 되돌아오는 동작은 생기지 않는다.
    #   다만 남은 경로가 너무 짧으면 RPP 가 앞점을 못 잡고 끊어 돌므로
    #   0.25m 아래로는 여전히 손대지 않는다.
    #
    # ⚠ 이게 성립하는 이유: 렉이 2.7m 에서도 잡힌다(실측 1508 표본,
    #   2.5~3.0m 구간에서 350회). 예전에 '1.73m 에서 끊긴다' 고 적었던 것은
    #   렉 규격을 틀리게(대각선 709mm 를 변으로) 넣어 두었을 때의 값이다.
    #   규격을 바로잡으면서 사라졌다. 그래서 도착하기 한참 전에 이미 보이고,
    #   목표를 바꿀 여유가 1m 넘게 남는다.
    JOB_AIM_KEEP    = 0.25
    JOB_AIM_MAX     = 3       # 다시 거는 횟수 상한
    # 도착해서 '축에서 너무 벗어났다' 고 나왔을 때 설 자리를 고쳐 다시 갈
    # 횟수. 한 번이면 대개 충분하다 — 재는 값이 정확하기 때문이다.
    JOB_RESTAND_MAX = 2
    # 도착해서 이보다 벗어나 있으면 (한계 안이라도) 다시 선다.
    #
    # ⚠ 한계(_job_lat_max, 약 58mm) 로만 가르면 한계 바로 아래(+58mm)가
    #   그대로 통과하고, 180도 회전이 늘 10mm 안팎을 더해 돌아선 뒤 검사
    #   (63mm)에 걸려 실패한다(2026-09-28: +58 -> +66mm). 회전 몫을 남겨
    #   두려면 도착 자세는 더 좁게 맞춰야 한다. nav2 도착 정밀도가 ±30mm
    #   라 그보다 좁히면 다시 서기를 되풀이하기만 한다.
    JOB_RESTAND_LAT = 0.030
    # 렉 밖에서 '진입 여유 부족' 등으로 멈추면 대기 자리로 돌아가 이만큼
    # 다시 해 본다(2026-09-28: 5회 중 1회, 캐스터에 차여 방향 -11도로 입구
    # 앞 정지). 다리 사이에 들어간 뒤에는 다시 하지 않고 그 자리에서 멈춘다.
    # ⚠ 2026-11-13 1 -> 3: 차체 폭을 실제값(328.8)으로 바로잡아 ls_origin 여유가 14.6mm 가 된 뒤,
    #   입구 0.9m 앞에서 캐스터에 차여 0.5초에 1.5도 틀어지면(라이다·IMU 일치) 안전 정지가 걸렸다.
    #   멈추는 판정은 그대로 두고 물러났다 다시 들어가는 횟수만 늘린다.
    JOB_RETRY_MAX   = 3
    # 다시 할 때 렉을 등지고 입구 앞에 서 있으면 nav2 대신 렉 축을 따라 곧게
    # 대기 거리 + 이만큼까지 나온 뒤, 돌지 않고 바로 다시 후진 진입한다 [m].
    #
    # ⚠ 입구 바로 앞(렉 중심 0.8m)에서는 nav2 정지 구역(0.6m)에 렉 다리가 들어
    #   nav2 가 출발하지 못했다 — 33초 멈춰 있다가 취소, 재시도에 124초
    #   (2026-09-28). 이 자리는 이미 축에 맞춰 렉을 등진 자세라 회전도 필요 없다.
    JOB_BACKOFF_EXTRA = 0.30
    # ⚠ 2026-11-13 다시 하기에서 '대기 거리 + 0.30'(렉 중심 1.70m)까지 나오면 입구 0.88m 에서 멈췄을 때
    #   82cm 를 나갔다 왔다(사용자: "재시도할때 앞으로가는게 너무 멀어"). 이미 축에 맞춰 렉을 등진
    #   자세라 다시 들어가며 좌우·방향을 지울 거리만 있으면 된다 — 입구 줄(렉 반깊이 + 차체 반길이)
    #   보다 JOB_BACKOFF_RUN 앞까지, 적어도 JOB_BACKOFF_MIN 은 나오고, 위 값보다 멀리는 안 간다.
    JOB_BACKOFF_RUN = 0.45
    JOB_BACKOFF_SHORT = False
    JOB_BACKOFF_MIN = 0.25
    # 라이다로 잡은 설 자리가 POI 기준에서 이보다 멀면 안 쓴다.
    # 대차는 POI 자리 45cm 안에서만 찾으므로 그 이상 벌어질 수 없다.
    JOB_STAND_MAX   = 0.55
    # 마지막 자리 맞춤 — nav2 가 아니라 직접 몬다.
    #
    # ⚠ 위 '목표 다시 걸기' 는 이 렉에서 한 번도 돌지 않는다. 렉이 보이기
    #   시작하는 거리가 1.73m 인데 멈출 자리가 1.40m 라, 보이고 나서 남은
    #   거리가 0.33m 뿐이다. 0.60m 문턱을 절대 못 넘는다 — 죽은 코드였다.
    #   그래서 멈추는 자리가 늘 POI 로만 정해졌다(2026-09-21 지적:
    #   "앞에 멈추는 것도 라이다 박스 중심 기준이 아닌 것 같다").
    #
    #   짧은 거리에 nav2 목표를 다시 걸면 RPP 가 앞점을 못 잡고 45도씩
    #   끊어 돈다. 그 마지막 30cm 는 우리가 직접 몬다.
    # 조향 상한 [rad/s]. 0.35 = 0.10 m/s 에서 반경 0.29m.
    # 아직 렉 밖이라 크게 꺾어도 닿지 않는다.
    # 직선 추종 이득. z = k_yaw / (2*sqrt(v*k_lat)) = 1.0 (임계감쇠)
    # 넘겨받을 때 허용할 방향 차이. 이보다 틀어져 있으면 nav2 에 맡긴다.
    # 자리 맞춤 안에서 제자리 회전으로 방향을 맞출 문턱과 상한
    # 카메라가 이만큼은 다리로 확인해야 들어간다. 렉마다 carts.yaml 의
    # cam_legs 로 따로 정할 수 있고, 0 이면 카메라 확인을 건너뛴다.
    #
    # ⚠ 0 으로 두는 것은 유령 대차 방어를 하나 잃는 것이다. 다만 작업은
    #   POI 자리에서 45cm 안의 대차만 고르므로, 그 문은 그대로 남는다.
    JOB_CAM_LEGS    = 3
    # 중간 단계(POI 앞·자리 고쳐 서기·진입 자리)의 방향 허용오차. 5도면
    # 카메라가 대차를 보는 데 지장이 없고, 최종 방향은 진입 직전 축 정렬이
    # 정한다. 여기를 조이면 돌고 멈추기가 단계마다 되풀이돼 회전만 늘어난다.
    JOB_MID_YAW     = 0.09
    # 인식한 자리보다 이만큼 더 물러나 등을 돌린다. 회전에서 밀린 만큼을
    # nav2 가 이 거리를 주행하며 잡는다. 진입 거리는 그만큼 길어진다.
    JOB_SPIN_W      = 0.45    # 돌아설 때 최고 각속도 [rad/s]
    JOB_SPIN_ACC    = 0.50    # 각가속도 [rad/s^2]. 올리고 내리는 완만함
    JOB_SPIN_TOL    = 0.030   # 약 1.7도
    JOB_SPIN_LAG    = 0.30    # 지령을 끊고도 더 도는 시간. 그만큼 미리 끊는다
    JOB_SPIN_SETTLE = 0.8     # 돌고 나서 가라앉히는 시간 [s]
    # 목표가 이보다 크게 등 뒤에 있으면 먼저 돌아선 뒤 nav2 에 넘긴다.
    JOB_FACE_MIN    = 1.05    # 약 60도
    # 도착 자세가 대차 축에서 이만큼 넘게 틀어져 있으면 정면으로 돌아선다.
    JOB_FACE_TOL    = 0.07    # 약 4도
    # 옆으로 이만큼 넘게 어긋났을 때만 다시 서 본다. 진입 제어가 100mm
    # 쯤은 지우지만, 다리 사이 여유가 한쪽에 152mm 뿐이라 여유를 남긴다.
    JOB_REPOSE_M    = 0.10
    JOB_REPOSE_R    = 0.09    # 약 5도
    JOB_REPOSE_MAX  = 1       # 되풀이 한도. 맴돌지 않게
    # 받아들인 인식을 기준에 섞는 정도.
    #
    # ⚠ 시간이 아니라 '주행 거리' 기준이다. 시정수를 0.6초로 고정해 뒀더니
    #   속도를 올릴수록 거리로는 더 뒤처져 방향 오차가 커졌다(0.40 m/s 에서
    #   1.89도). 6cm 주행 기준으로 바꾸니 0.86도로 돌아왔다(2026-09-16).
    JOB_ASSOC_M     = 0.30    # 인식 결과가 대차 자리에서 이만큼 안이어야 받는다
    # ⚠ 진입 중에는 더 좁게 본다. 0.30 으로 두면 300mm 옆의 엉뚱한 사각형도
    #   받아들여, 조금씩 섞이며 기준이 그쪽으로 걸어간다. 대차는 움직이지
    #   않으므로 120mm 면 넉넉하다.
    JOB_ASSOC_ENTER = 0.12
    # 진입하는 동안 기준이 처음 자리에서 벗어날 수 있는 한계
    # 믿을 만한 검출의 조건. 이 조건을 갖춘 검출은 빨리 따라가고, 못 갖춘
    # 것만 예전처럼 천천히 섞는다.
    JOB_TRUST_SCORE = 0.85
    JOB_TRUST_ERR   = 15.0    # 규격과의 오차 [mm]
    JOB_BLEND_GOOD  = 0.15    # 믿을 때 섞는 거리 [m] (시정수 1.25초)
    # ⚠ 0.08 -> 0.15. 80mm 는 '유령 대차' 방어로 넣은 값인데, 2026-09-22
    #   실측에서 진짜 어긋남 77mm 를 여기서 막아 버렸다. 기준은 +5mm 로
    #   '맞았다' 고 하는데 실제는 오른쪽 72mm 였고, 오른쪽 틈 38mm 를 넘어
    #   다리에 부딪혔다. 유령 방어는 사실상 JOB_ASSOC_ENTER(120mm) 가
    #   하고 있다 — 기준에서 120mm 밖은 애초에 고르지 않는다. 그 안에서는
    #   따라갈 수 있어야 한다.
    JOB_ANCHOR_MAX  = 0.15
    JOB_ANCHOR_YAW  = 0.06    # 약 3.4도
    # 진입 처음 이 거리는 기어가듯 천천히 간다. 캐스터가 방향을 트는 힘을
    # 줄이려는 것이고, 무조향 구간(JOB_PREALIGN_M)과 함께 쓴다.
    JOB_CREEP_V     = 0.05
    # 기어가기에서 제 속도까지 올리는 데 쓰는 거리. 계단으로 올리면 그
    # 순간 확 나가면서 캐스터를 걷어찬다.
    #
    # ⚠ 예전에는 200mm 에서 0.05 -> 0.20 m/s 로 한 번에 올렸다. 그 턱이
    #   조향이 꺼진 구간(JOB_PREALIGN_M=400mm) 안에 있어서, 걷어차인 것을
    #   400mm 까지 아무도 잡지 않았다. 틀어진 채로 들어갔다(2026-09-21).
    #   무조향 구간이 끝날 때 속도도 막 다 올라오도록 같은 거리를 쓴다.
    JOB_RAMP_M      = 0.40
    # 멈추기 전 이 거리부터 매끈하게 줄인다.
    JOB_SLOW_M      = 0.40

    # 진입 처음 이 거리는 조향 없이 곧게만 간다(캐스터 정렬).
    #
    # ⚠ 0.20 으로 줄였다가 되돌렸다. '걷어차임이 1.1초에 끝나니 그때부터
    #   잡자' 고 본 것인데, 실제로는 그 와중에 제어가 끼어들어 오히려 키웠다
    #   (2026-09-21 실측, 같은 조건):
    #       0.40m — 걷어차임 최대 +124mm, 최종 좌우 -22mm / 방향 +1.19도
    #       0.20m — 걷어차임 최대 +153mm, 최종 좌우 -71mm / 방향 -9.17도
    #   캐스터가 자리를 잡는 동안은 건드리지 않는 편이 낫다.
    # ⚠ 0.40 -> 0.30. 예전에 0.20 으로 줄였다가 더 나빠졌는데, 그때는
    #   200mm 에서 속도가 0.05 -> 0.20 으로 계단처럼 튀어 그 자리에서
    #   캐스터를 걷어차고 있었다. 지금은 400mm 에 걸쳐 매끈하게 올린다.
    #   교정할 거리가 282mm 뿐이라 조금이라도 넓혀야 한다(2026-09-21).
    # ⚠ 250mm 를 손 놓고 가는 동안 좌우가 -12mm -> +78mm 로 90mm 밀렸다
    #   (2026-09-21 진입 추적). 그 뒤 1.3m 내내 그걸 되돌리느라 좌우로
    #   꺾었고, 그 흔들림이 다리에 닿았다. 원인이 캐스터든 아니든, 90mm 를
    #   깔고 시작하면 나머지가 다 어려워진다. 일찍 잡는다.
    #
    #   예전에 0.20 으로 줄였다가 더 나빠진 적이 있는데(2026-09-21 오전),
    #   그때는 200mm 지점에서 속도가 0.05 -> 0.20 으로 계단처럼 튀어 그
    #   자리에서 캐스터를 걷어차고 있었고 이득도 포화 상태였다. 조건이 다르다.
    JOB_PREALIGN_M  = 0.25
    # 그 구간(캐스터가 뒤집히는 동안)의 속도 [m/s].
    #
    # ⚠ 0.10m 만 붙잡고 평소처럼 가속했더니, 캐스터가 뒤집히는 180~250mm
    #   지점(0.07 m/s)에서 방향이 +1.4 -> -3.7도로 1초 만에 흔들렸고 좌우가
    #   +72 -> -68mm 로 쓸렸다(2026-09-29 12:10, 앞으로 물러났다 바로 후진).
    #   직진 시험에서도 튐은 출발 뒤 10~20cm 에서 났다. 후진 출발 튐은 방향이
    #   들쭉날쭉해서 예측 보정이 안 된다(전진은 된다 — JOB_FF_FWD_DEG).
    #   캐스터 미리 뒤집기처럼 0.03 m/s 로 붙잡으면 0.5도였다(09-28).
    #   튐 크기는 거리로 정해지지만, 천천히 가면 방향 붙잡기가 따라간다.
    #   0.03 에서 진입 렉 밖 방향폭 평균 3.2도(이전 약 6도), 나오기 튐 최대
    #   1.0~1.7도(이전 2.3~3.7도). 시간이 나오기 약 7초 늘어 0.04 로 올려
    #   본다(2026-09-29 사용자 요청).
    JOB_FLIP_V      = 0.04
    # 진입 출발 구간의 캐스터 튐 받아치기.
    #
    # ⚠ 출발 직후 0.3m 에서 방향이 평균 4.8도(최대 9.3도) 흔들렸고, 이를
    #   되돌리는 것이 렉 밖의 S자였다(2026-09-28 반복 10회, 영상). 저속이라
    #   회전 반경 제한(w <= v/1.5)이 조향을 0.03 rad/s 로 묶어 튄 뒤에 늦게
    #   크게 되돌렸다. 같은 날 캐스터 미리 뒤집기에서 반경 제한 없이 방향을
    #   붙잡자 흔들림이 0.5도였다. 9/21 '초반에 끼어들면 키운다' 는 정수 rpm
    #   디더링·반경 제한 시절의 결과다.
    #   - JOB_PREALIGN_M 안 : 고치지 않고 출발 방향만 유지 (튐만 받아침)
    #   - JOB_START_FREE_M 안 : 평소 조향, 반경 제한 없이 상한 JOB_START_W
    JOB_START_FREE_M = 0.45
    JOB_START_W      = 0.15
    # 진입·나오기 전체를 반경 제한 없이 조향하고 자이로로 받아친다.
    #
    # ⚠ 드라이브 이득(적분 8->64, 비례 35->60)·구동륜 쪽 무게로는 직진 중
    #   원치 않는 회전이 0.9 도/s 안팎에서 줄지 않았다(2026-09-29 기록 도구).
    #   튈 때 전류는 1~6A 로 여유가 많았다. 반면 캐스터 미리 뒤집기처럼 반경
    #   제한 없이 붙잡은 동작은 흔들림이 0.5도였다. 저속이라 반경 제한
    #   (w <= v/1.5) 이 조향을 0.03~0.08 rad/s 로 묶어 캐스터(약 0.06 rad/s)를
    #   못 이겼다. 방향이 '틀어진 뒤' 가 아니라 '돌기 시작할 때' 받아치도록
    #   자이로 항을 더한다.
    JOB_FREE_STEER   = True
    JOB_RATE_KD      = 0.6     # 0.8·1.0 은 스스로 흔들렸다(주행 중 회전 평균 0.80 -> 5.35·4.70 도/s, 2026-09-29)
    JOB_START_HOLD_K = 1.2
    # 전진 출발 캐스터 튐 예측 보정. 후진 뒤 처음 JOB_FF_M 을 가는 동안
    # 캐스터가 돌릴 만큼(JOB_FF_FWD_DEG)을 거리에 고르게 나눠 미리 반대로 돈다.
    #
    # ⚠ 조향 없는 1.5m 직진 시험(2026-09-29): 후진 뒤 전진 출발 0.3m 에서
    #   차체가 매번 시계 방향으로 −3.1/−3.4/−3.4도 돌았다(주로 10~20cm 지점).
    #   −3.3 보정을 넣자 −0.6/−0.8/+0.9/+0.8도. 튀고 나서 받아치는 자이로 항은
    #   늦어서 이만큼 못 줄였다. 후진 출발은 +1.6/+1.1/−1.2, +2.7/+2.8/−0.6/0.0
    #   으로 튀었다 안 튀었다 해서 고정 보정을 넣으면 오히려 넘친다(+3.2 로
    #   −3.3~−4.7도). 그래서 전진만. 캐스터 미리 뒤집기(40mm)는 이 구간의
    #   앞부분일 뿐이라 거리는 상태가 아니라 odom 에서 이어서 센다.
    #
    # ⚠ 렉에서는 거꾸로 됐다. 보정 없이 나올 때 출발 직후 −2.6~−3.7도(시계)로
    #   튀던 것이, 보정을 넣자 +1.7~+3.5도(반시계)로 튀었다(2026-09-29 11:41~
    #   12:20, 9회). 캐스터가 180도 뒤집힐 때 어느 쪽으로 돌지는 처음 옆으로
    #   밀리는 방향이 정한다 — 미리 돌리면 반대쪽으로 돌며 찬다. 0 으로 끄고
    #   후진 출발과 같이 기어가며 붙잡는다(JOB_PREALIGN_M, JOB_FLIP_V).
    JOB_FF_FWD_DEG   = 0.0     # 전진 출발에서 캐스터가 돌리는 각 [도, +=반시계]
    JOB_FF_M         = 0.30
    # ⚠ 0.06 은 너무 빨랐다. 인식이 ±50mm 로 떨리는데 0.3초 만에 따라가니,
    #   제어가 떨리는 목표를 쫓아 로봇을 실제로 좌우로 몰았다 — 진입 추적에서
    #   0.5초 만에 좌우가 92mm 움직였는데, 그 시간에 로봇이 그만큼 옆으로
    #   갈 방법이 없다(2026-09-16).
    #
    #   대차는 진입하는 동안 움직이지 않는다. 따라갈 이유가 없다. 1.0m 를
    #   가는 동안 섞으면 주기 3초짜리 떨림은 1/10 로 줄고, 진짜로 어긋난
    #   것만 천천히 반영된다.
    #
    # ⚠ 1.00 -> 0.50. 떨림은 잘 막았지만 너무 느려서 '진짜 어긋난 것' 마저
    #   진입이 끝날 때까지 못 따라잡는다. 진입 거리가 1.5m 이므로 1.0m
    #   시정수면 초기 오차의 e^-1.5 = 22% 가 남는다. 2026-09-22 실측에서
    #   마지막 500mm 내내 기준과 실제가 13mm 벌어진 채 고정이었고, 로봇은
    #   그 틀린 기준에 정확히 맞춰 섰다 — 다리 양쪽 여유가 64mm 대 90mm 가
    #   된다.
    #
    #   0.50m 면 잔류가 e^-3 = 5%(약 4mm)로 떨어진다. 위에서 막으려던 주기
    #   3초짜리 떨림에 대한 감쇠는 1/17 에서 1/8.7 로 줄지만, 그 떨림은
    #   ANCHOR_MAX(80mm)와 ANCHOR_YAW(3.4도) 가 한 번 더 막는다.
    JOB_BLEND_M     = 0.50
    JOB_TURN_MAX    = 3
    JOB_SETTLE_S    = 1.0
    JOB_W_MAX       = 0.30
    JOB_W_MIN       = 0.10    # 1 rpm. 그 아래는 정수 rpm 에 뭉개진다
    JOB_KP          = 1.8
    JOB_LAG_S       = 0.25
    # 진입 속도는 화면의 주행속도를 그대로 따른다. 다만 위아래로 막는다 —
    # 충돌감시를 우회하는 구간이라 너무 빠르면 기댈 것이 없고, 너무 느리면
    # 바퀴 지령이 정수 rpm 에 뭉개진다.
    # ⚠ 상한은 0.20 이다. 0.40 까지 열어 뒀다가 좌우 흔들림이 심해지고
    #   중심도 안 맞았다(2026-09-16).
    #
    #   두 가지가 겹친다. 하나는 지연이 '시간' 으로 고정돼 있다는 것 —
    #   지령이 바퀴에 닿고 자이로에 잡히기까지 0.3 초쯤인데, 빨리 갈수록
    #   그동안 더 먼 거리를 지나가므로 제어가 그만큼 뒤늦게 반응한다.
    #   이득까지 속도에 비례해 키우면 고리가 더 헐거워진다.
    #   다른 하나는 고칠 시간이 준다는 것 — 2.2m 를 0.20 에서는 11초,
    #   0.40 에서는 5.5초에 지나가니 좌우를 다 못 지운다.
    #
    #   0.20 은 실측으로 확인된 값이다(지령 0.50 도/s, 상한 4.58 의 1/9).
    JOB_V_MIN       = 0.08
    # ⚠ 진입 속도 상한. 조향값이 속도에 비례해 같이 오르므로(반경을 일정하게
    #   두려고), 0.20 에서는 조향 상한이 2배가 되어 0.6 rad/s = 34도/s 까지
    #   나온다 — 눈에 너무 빠르다(2026-09-21 지적). 0.12 로 묶으면 같은
    #   반경에서 14도/s 다. 진입 1.4m 에 12초 걸린다.
    #   2026-09-29: 캐스터 뒤집히는 출발 0.25m 를 0.04 로 따로 기어가게 한
    #   뒤 0.16 으로 다시 올려 본다(사용자 요청, 도킹 시간 단축).
    JOB_V_MAX       = 0.16
    JOB_V_REF       = 0.10    # 아래 이득·상한을 정한 기준 속도
    # ⚠ 2026-11-13 렉 진입·나오기는 설정 주행 속도와 상관없이 이 속도로 간다(사용자: "렉들어갈때는
    #   설정된속도랑 상관없이 천천히"). 전에는 설정 속도를 0.08~0.16 으로 잘라 써서 0.7 m/s 설정이면
    #   0.16 이었고, 입구에서 캐스터에 차여 1.5도 틀어졌다. 이득을 맞춘 기준 속도(0.10)로 묶는다.
    JOB_V_FIXED     = 0.10
    JOB_V_SLOW_R    = 0.6     # 마지막 20cm 는 이 비율로 줄인다
    # ⚠ 감속 제한(base_node 0.9 m/s^2)을 넣은 뒤 다시 잰 값이다. 제한이
    #   없을 때는 0.42 였는데, 완만하게 세우는 만큼 더 굴러가 37mm 깊어졌다
    #   (앞뒤 -5.7 -> +31.0 mm, 2026-09-16).
    JOB_STOP_LAG_S  = 0.72
    # 정지 거리 = 속도 x 순수 지연 + 속도^2 / (2 x base_node 감속 제한).
    #
    # ⚠ 예전에는 '속도 x 0.72초' 하나였다. 그 0.72 는 0.072 m/s 에서 잰 값이라
    #   감속 몫(속도 제곱에 비례)이 섞여 있어, 속도나 감속 제한이 바뀌면
    #   틀린다(2026-09-28 감속 0.9 -> 0.6). 둘로 나눠 base_node 의 실제 감속
    #   제한을 읽어 쓴다. 0.68 = (0.072 x 0.72 - 0.072^2 / (2 x 0.9)) / 0.072.
    JOB_STOP_DELAY_S = 0.68
    # 진입 조향.
    #
    # ⚠ 세게 잡으면 안 된다. 눈에 보이는 것은 '차체가 얼마나 기우느냐' 다.
    #   상한 0.25 rad/s 는 0.10 m/s 에서 회전 반경 0.4m 라 확 꺾이고, 0.12
    #   에서도 0.83m 라 비틀거리는 것이 보였다(2026-09-16). 0.04 면 2.5m 다.
    #
    # ⚠ 그리고 세게 잡을 필요가 없다. 진입 시작 좌우가 48mm 였는데, 이것은
    #   2.4m 를 가는 동안 평균 1.2도만 틀어도 지워지는 양이다. 상한을
    #   0.12 -> 0.04 로 낮춰도 최종 좌우 1.6 -> 2.1mm, 각도 0.42 -> 0.39도로
    #   정확도는 그대로면서 차체 최대 기울기가 1.63 -> 1.24도로 줄었다.
    #
    # ⚠ 그리고 이득을 세게 잡을 근거가 애초에 약했다. 눌러야 할 차체 흔들림
    #   3.5 도/s 는 0.45 m/s 에서 잰 값인데, 캐스터가 미는 힘은 속도에 비례해
    #   진입 속도 0.10 m/s 에서는 0.8 도/s 수준이다. 작은 외란에 센 이득을
    #   걸어 두니 측정 잡음만 쫓아다녔다.
    #
    # 지금 값에서 좌우 5.4mm, 방향 0.46도, 지령 출렁임 1.2 도/s.
    # (6.0 짜리는 방향 0.17도로 조금 낫지만 출렁임이 2.2 도/s 였다)
    # 아래 값들은 JOB_V_REF 에서의 값이고, 실제로는 속도에 비례해 같이
    # 키운다. 그래야 속도를 바꿔도 '거리 기준으로 같은 궤적' 이 나온다.
    # 속도만 올리고 조향을 그대로 두면 회전 반경이 그만큼 커져 좌우 오차를
    # 못 지운다(0.40 m/s 에서 각도 1.94도 대 0.86도).
    # ⚠ 이 값은 모사가 아니라 실측에 맞춰 낮춘 것이다.
    #   2026-09-16 진입 중 실측: 지령 출렁임 4.56 도/s 인데 그때 조향 상한이
    #   4.58 도/s 였다 — 내내 상한에 붙어 좌우로 튕기고 있었다(포화 한계진동).
    #   차체가 캐스터 때문에 저절로 흔들리는 것을 제어가 매번 상한까지
    #   꺾어 쫓아다닌 것이다. 모사로는 상한에 0% 붙는다고 나왔으니,
    #   모사의 외란 모형이 실제보다 무르다. 모사로 이 값을 다시 고르지 말 것.
    # ⚠ 들어가면서 꺾는 것을 최대한 줄이는 쪽으로 잡았다. 진입 시작 좌우가
    #   20~30mm 면 그대로 곧게 들어가도 그만큼으로 끝난다 — 실측 산포가
    #   24mm 라 어차피 같은 자릿수다. 꺾어서 얻는 것보다 곧게 가서 잃는
    #   것이 적다(2026-09-16, 사용자 요청).
    #   대신 돌아선 뒤 남은 어긋남을 들어가면서 지운다.
    # ⚠ 이득을 올려도 상한(JOB_W_ENTER)이 낮으면 눈에 띄는 흔들림은 안 생긴다.
    #   상한이 보이는 흔들림의 크기를 정하고, 이득은 거기 닿는 속도만 정한다.
    #   0.18 로 낮췄더니 180도 회전 뒤 남은 25mm 를 못 지워 좌우가 한쪽으로
    #   21mm 치우쳤다(2026-09-16, 5회 모두 같은 쪽). 이득만 올린다.
    # ⚠ 방향 이득은 좌우 이득에 매여 있다. 직선 추종의 감쇠비는
    #     z = K_YAW / (2 * sqrt(v_ref * K_LAT))
    #   이고 둘 다 속도에 비례하므로 속도와 무관하다. 0.40/0.15 는 z=0.37
    #   로 크게 과소감쇠라, 좌우를 잡느라 꺾은 각도를 못 되돌린다.
    #   실측(2026-09-21): 좌우는 -93 -> -12mm 로 잡았는데 방향이 +1.2 ->
    #   +6.1도로 계속 쌓이고, 그 각도 때문에 좌우가 다시 -39mm 로 벌어졌다.
    #   870mm 차체가 7도 기울면 훑는 폭이 270 -> 374mm 라 다리에 닿는다.
    #   0.40 이면 z=1.00 (임계감쇠).
    # ⚠ 0.40 은 너무 느렸다. 고유진동수 sqrt(v*k_lat) = 0.20 rad/s 라
    #   가라앉는 데 15초, 0.2 m/s 로 3m 를 간다 — 진입 거리가 1.4m 이므로
    #   사실상 아무것도 못 고쳤다(실측 13mm). 76mm 를 430mm 안에 지우려면
    #   0.6 rad/s 는 되어야 한다(2026-09-21 지시: "들어갈때 조향으로").
    #   그리고 상한만 올려 봐야 소용없다 — 지령 자체가 0.03 rad/s 였다.
    #
    # ⚠ 3.00 은 넘어갔다. 좌우가 +62 <-> -13mm 를 4초 주기로 왕복했다
    #   (2026-09-21 진입 추적). 좌우 이득을 낮추고 방향 이득을 올려
    #
    # ⚠ 2.00 도 앞 구간에서 포화됐다. 좌우 78mm / 방향 4.2도면 지령이
    #   0.327 인데 상한이 0.240 이다 — 다리 사이만 고치고 앞 구간은
    #   그대로 둬서, 거기서 ±40mm 로 튕기다가 그 봉우리가 다리에
    #   닿았다(2026-09-21 진입 추적: 남은 696mm 에서 모서리 53mm,
    #   여유 56mm). 흔한 오차에서 포화되지 않는 값으로 내린다.
    #
    # ⚠ 1.20 은 되돌린다. 위 계산의 전제가 틀렸다 — 아래 임계감쇠식은
    #   ωn = sqrt(v*k_lat) = 0.38 rad/s, 즉 주기 16.5초를 전제하는데,
    #   실측 진동 주기는 2.6초였다(2026-09-22: 남은 1549->1083mm 466mm
    #   구간에 1.5주기, 0.12 m/s). 6배 빠르다. 이 루프를 정하는 것은
    #   이득이 아니라 지연이다(검출 5Hz + 구동륜 미끄러짐). 지연이
    #   지배하는 루프에서 이득을 3배 올리면 반드시 진동한다.
    #
    #   실제로 좌우가 -13 -> +65 -> +14 -> -32 -> +20 -> +66mm 로 ±65mm
    #   진동했고, 다리 사이에 들어가기도 전에 여유 56mm 를 세 번 넘겼다.
    #   그 상태로 들어가 렉을 밀었다.
    #
    #   주기로 역산한 필요 이득 여유가 약 3배이므로 0.40 이 이론값이고,
    #   0.40 은 어제 실측에서 모서리 36mm(여유 56mm 안쪽)로 이 조합만이
    #   유일하게 측정된 값이다. "0.40 은 사실상 무보정" 이라던 판단도
    #   같은 틀린 ωn 에서 나온 것이다.
    #
    #   상한(W_ENTER)은 건드리지 않는다. 진동 구간의 지령을 계산해 보면
    #   +0.094 / -0.082 로 상한 0.200 에 닿지도 않았다 — 포화가 아니라
    #   이득 문제이므로 상한을 내려도 진동에는 아무 효과가 없다.
    #
    # ⚠ 0.40 -> 1.90. 오차의 정의를 바꿨다(2026-09-28). 예전 입력은 '로봇이
    #   본 렉 중심의 옆 거리' 로, 렉 축 좌우에 '남은거리 x 방향' 이 섞여
    #   있었다. 그래서 방향에 걸리는 실제 이득이 0.40*남은거리 + 0.50 으로
    #   1.5m 앞에서 두 배가 되어 방향이 ±3~4도로 흔들렸고(S자), 정작 렉
    #   축에서의 좌우는 두 항이 상쇄되는 자리에서 +13mm 가 남은 채
    #   들어갔다(ls_origin, 여유 44mm 에서 접촉).
    #   지금은 렉 축 좌우와 방향을 따로 먹인다(선 추종). 이득이 거리와
    #   무관하므로 여기 값이 곧 실제 이득이다.
    #     e' = v*방향, 방향' = -w  ->  s^2 + k_yaw*s + v*k_lat
    #   0.12 m/s 에서 ωn 0.52, ζ 0.81. 지연 0.72초·5Hz 검출·캐스터 외란을
    #   넣은 모의(40회)에서 입구 이후 최대 모서리 중앙값 38 -> 20mm,
    #   조향 부호 반전 5.5 -> 1.9회였다. 실기 검증 전이다.
    JOB_K_LAT       = 1.90
    # 임계감쇠는 2*sqrt(v*k_lat) = 2*sqrt(0.12*1.20*1.2) = 0.83 이지만,
    # 캐스터가 계속 차체를 밀어 실제로는 그보다 덜 잠긴다. 넉넉히 둔다.
    # z = 1.00*1.2 / 0.83 = 1.44 (과감쇠)
    #
    # ⚠ 1.00 -> 0.50. 위 임계감쇠 계산은 지연이 없다고 보는 것이다. 실제로는
    #   지령을 끊어도 0.72초어치를 더 가는 지연이 있고(JOB_STOP_LAG_S,
    #   2026-09-16 실측), 방향 루프는 적분기(yaw' = w)라 지연과 만나면
    #   그것만으로 자기발진한다.
    #
    #     적분기+지연의 임계점  wT = pi/2
    #     w = pi/(2*0.72) = 2.18 rad/s  ->  주기 2.88초, 임계 이득 2.18
    #
    #   2026-09-22 실측 진동 주기가 2.6초다 — 예측 2.88초와 맞는다. 즉 이
    #   진동은 이득 대 감쇠가 아니라 이득 대 지연의 문제이고, 도는 것은
    #   안쪽의 빠른 루프인 방향 항이다. 1.00 은 임계 2.18 대비 여유가
    #   2.2배뿐인데, 검출 지연까지 더하면 실질 여유가 없다.
    #
    #   0.50 이면 여유 4.4배. 바깥 좌우 루프의 감쇠는
    #   trace = k_lat*L + k_yaw = 0.40*1.4 + 0.50 = 1.06,
    #   wn = sqrt(v*k_lat) = 0.219 -> z = 2.4 로 여전히 과감쇠다.
    #   수렴이 느려지지 않으면서 진동만 걷힌다.
    #
    # ⚠ 0.50 -> 0.70 (2026-09-28, 선 추종으로 바꾸며). 예전에는 좌우 항이
    #   방향 이득을 0.40*남은거리 만큼 더 얹어 줬으므로 실제 방향 이득은
    #   1.1 안팎이었다. 이제 그 몫이 없어 이 값이 전부다. 지연 임계 2.18
    #   대비 여유 2.6배(0.12 m/s 에서 0.84).
    JOB_K_YAW       = 0.70
    # 감쇠를 제대로 걸면 방향 항이 커지므로 상한도 조금 올린다. 0.035 는
    # 0.15 m/s 에서 회전 반경 2.9m — 여전히 눈으로는 직선이다.
    # 진입 중 조향 상한 [rad/s]. 기준 속도 0.10 m/s 에서의 값이고
    # 속도에 비례해 같이 오른다(= 회전 반경이 일정하다).
    #
    # ⚠ 0.035 는 반경 2.86m 다. 그 정도로는 좌우를 거의 못 줄인다 —
    #   다리 사이에 들어가기 전 남은 거리가 282mm 뿐이라 7mm 다.
    #   "들어가면서 치우치면 회전되게" 하려면 풀어야 한다(2026-09-21 지시).
    #   0.200 = 반경 0.50m. 이 구간은 아직 다리 밖이라 몸을 돌려도
#   안 닿는다. 최대 기울기는 16도인데, 그건 한계치를 다 쓸 때이고
#   어긋남이 작으면 기울기도 그만큼 작다(기울기 = 2*어긋남/거리).
    JOB_W_ENTER     = 0.200
    # ⚠ 다리 사이에 들어간 뒤는 반대로 조여야 한다. 870mm 차체가 1도만
    #   돌아도 끝이 7.6mm 움직이는데 한쪽 여유가 56mm 인 렉이 있다.
    # ⚠ 0.028 은 너무 조였다. 다리 사이에서 방향 오차 4도가 그대로
    #   남았는데, 870mm 차체라 그것만으로 모서리가 33mm 나간다 — 여유
    #   56mm 중 대부분이다(2026-09-21: "앞부분에서 살짝 대차를 밀었음").
    #   1.9도/s 로는 4도를 펴는 데 2초, 그 사이 240mm 를 간다.
    #   0.060 = 3.4도/s. 모서리는 초당 26mm 만 움직이므로 여전히 얌전하다.
    # ⚠ 2026-09-28 반복 시험: 입구에서 렉 축 -3mm/모서리 5mm 로 거의
    #   맞았는데, 다리 사이에서 조향 없이(사각지대 15mm) 가는 동안 차체가
    #   저절로 약 1도/s 로 쏠려 +13mm / +1.9도 로 끝났다. 쏠림을 붙잡도록
    #   다리 사이 사각지대를 좁히고 권한을 올린다. 모의(쏠림 0.015 rad/s,
    #   60회)에서 다리 사이 최대 모서리 95% 27 -> 16mm, 끝 좌우 14 -> 9mm.
    JOB_W_NEAR      = 0.100
    # 다리 사이에서 쓸 이득 비율. 좌우는 느슨하게, 방향은 거의 그대로.
    JOB_NEAR_KL     = 0.60    # 0.30 -> 0.60 (2026-09-28, 위 참조)
    JOB_NEAR_KY     = 1.00    # 0.80 -> 1.00 (2026-09-28)
    # 적분 이득. 비례만으로는 일정한 외력 앞에서 오차를 남긴 채 평형을
    # 잡는다.
    #
    # ⚠ 실측(2026-09-21): 좌우 +30mm / 방향 -2.5도 에서 두 항이 상쇄돼
    #   (+0.043 - 0.052 ≈ 0) 1m 넘게 그대로 갔다. 그 상태로 다리 사이에
    #   들어가면 모서리 42mm, 여유 56mm — 14mm 밖에 안 남는다.
    #   시간에 따라 쌓아서 남는 오차를 지운다.
    #
    # ⚠ 0.60 -> 0 (2026-09-28). 위의 '두 항이 상쇄돼 오차가 남는' 것은
    #   좌우 입력에 방향이 섞여 있던 예전 오차 정의 때문이었다. 렉 축
    #   좌우를 직접 먹이면 비례항만으로 0 으로 간다. 모의에서 적분을 켜면
    #   모서리 중앙값이 20 -> 50mm 로 오히려 나빠졌다. 구조는 남겨 둔다.
    JOB_K_INT       = 0.0
    # ⚠ 2026-11-13 후진 진입 중 로봇을 한쪽으로 돌리는 일정한 힘이 있다: 회전 지령은 -0.005~-0.027
    #   인데 실제 방향은 +0.5 -> +1.4도로 갔다(라이다·IMU 일치) — 약 +0.02 rad/s. 비례 제어(K_LAT 1.9,
    #   K_YAW 0.7)는 그 앞에서 축 좌우 ~10mm · 방향 ~1.6도로 평형을 잡아(실측 +10~+17mm, +1~2도)
    #   여유 14.6mm 렉에서 입구마다 섰다. 위치 오차를 쌓는 적분(K_INT) 대신 '자이로 실제 회전 -
    #   JOB_DIST_DELAY 전 지령' 을 JOB_DIST_TAU 로 걸러 쏠림을 추정하고 그만큼 지령에서 뺀다.
    JOB_DIST_TAU    = 1.0     # [s]
    JOB_DIST_DELAY  = 0.40    # 지령이 자이로에 잡히기까지 [s]
    JOB_DIST_MAX    = 0.05    # 보정 상한 [rad/s]
    JOB_INT_MAX     = 0.40    # 적분이 낼 수 있는 몫 (상한 대비 비율)
    JOB_INT_BLEED   = 0.90    # 다리 사이에서 적분을 흘려보내는 비율/주기
    # 다리에 닿은 것으로 볼 모서리. 여유의 이 배수를 넘으면 멈춘다.
    #
    # ⚠ 닿은 뒤에도 500mm 를 더 밀고 들어가 렉을 밀었다(모서리 293mm).
    #   걸린 것은 제어로 못 푼다. 멈추는 편이 낫다.
    # ⚠ 2.0 은 '여유의 두 배까지 벗어나도 봐준다' 는 뜻이었다. free 는 이미
    #   다리 반지름과 차체 절반을 뺀 순수 여유이므로, 모서리가 free 에
    #   닿으면 그 순간 다리에 닿는다. 2.0 이면 닿고 나서 한참 뒤에야 걸린다.
    #   실제로 그래서 못 멈췄다 — 모서리 60mm(여유 56mm)로 돌파한 뒤
    #   72mm 까지 벌어졌는데 문턱 112mm 에 못 미쳐 렉을 밀고 들어갔다
    #   (2026-09-22). 닿는 지점 그대로 1.0 으로 둔다.
    JOB_JAM_R       = 1.0
    # 이 안에 들면 아예 안 꺾는다. 잡음을 쫓아 미세하게 떠는 것을 막는다.
    # 이 안에 들면 안 꺾는다. 다리 사이 여유가 한쪽에 140mm 쯤이라
    # 25mm 는 넉넉히 안전한 범위다.
    # 안 꺾는 범위를 '여유의 몇 %' 로 정한다. 렉 밖에서는 넉넉히(안 떨게),
    # 다리 사이에 들어가면 좁게(안 닿게).
    #
    # ⚠ 25mm 로 못박아 뒀더니 좁은 렉에서 모자랐다. ls_origin 은 한쪽
    #   여유가 56mm 뿐이라, 시작 오차 28mm + 사각지대 25mm 면 거의 다
    #   쓴다(2026-09-21: "옆에 살짝 부딪힘"). 렉마다 달라야 한다.
    #
    # ⚠ 0.45 -> 0.15 (2026-09-28). 0.45 는 예전 조향이 S자로 흔들리던 때
    #   '안 떨게' 넉넉히 둔 값이다. 선 추종으로 바꾼 뒤 첫 실기에서 방향은
    #   ±0.5도로 조용해졌는데, 렉 축 +14mm 가 문턱 20mm 안이라 1394 ->
    #   560mm 구간을 조향 계산(P -0.03)을 버린 채 직진했고, 다리 사이의
    #   약한 권한으로는 못 지워 +19mm 로 끝났다(ls_origin, 여유 44mm).
    #   렉 밖에서 지워야 할 크기다. 0.15 = 여유 44mm 에서 켜는 7mm/끄는 3mm.
    JOB_DEAD_FAR    = 0.15
    # ⚠ 0.18(=10mm)은 너무 좁았다. 바퀴 지령이 정수 rpm 이라 회전 분해능이
    #   2.85도/s 다 — 그보다 잘게는 못 돈다. 10mm 를 쫓으면 분해능 아래를
    #   쫓는 셈이라 영원히 떨린다. 0.35(=20mm)면 여유 56mm 중 36mm 가 남고,
    #   그 안에 들면 손을 뗀다.
    #
    # ⚠ 0.35 -> 0.15 (2026-09-28). 위 0.18 에서 떨었던 것은 S자로 흔들리던
    #   예전 조향 때다. 선 추종으로 바꾼 뒤에는 15mm 안에서 1도/s 쏠림을
    #   못 잡는 것이 더 컸다(JOB_W_NEAR 참조). 떨면 0.25 로 되돌릴 것.
    JOB_DEAD_IN     = 0.15
    # 남은 거리가 이보다 짧으면 차체가 이미 다리 사이에 있다.
    # 렉 깊이 절반(283mm) + 차체 반길이(435mm).
    JOB_IN_M        = 0.72
    JOB_DEAD_LAT    = 0.025
    JOB_DEAD_YAW    = 0.035   # 약 2.0 도
    # 이 거리부터 조향 상한을 낮춘다. 0.75m 는 로봇 뒤끝(435mm)이 가까운
    # 다리(중심에서 287mm)에 닿는 거리다 — 그 안쪽은 이미 대차 사이다.
    JOB_HOLD_M      = 0.75
    JOB_ENTER_EXTRA = 0.25    # 예상 거리보다 이만큼 더 가면 중단 [m]
    JOB_ENTER_S     = 75.0    # 기어가는 구간만큼 여유를 둔다
    # 진입 중 중단 문턱. 들어갈 때의 문턱(JOB_LAT_MAX)보다 커야 한다.
    # 작으면 가까스로 통과한 시도가 진입하자마자 중단된다.
    JOB_BAD_LAT     = 0.30
    # 렉에서 앞으로 빠져나오기. 차체 뒤끝이 렉 끝을 이만큼 지나면 멈춘다.
    #
    # ⚠ 손으로 몰아 나오면 많이 틀어졌다(2026-09-28 지적). 진입과 같은 렉 축
    #   선 추종으로 곧게 뺀다. 방향만 반대(전진)다.
    JOB_EXIT_CLEAR  = 0.30
    JOB_BUSY        = ("moving", "looking", "staging", "entering", "preflip",
                       "exiting", "exit_fix")
    # 나오다 튀어 여유를 넘으면 멈추고 제자리에서 렉 축에 맞춘 뒤 이어 간다.
    #
    # ⚠ 캐스터를 미리 뒤집어도 나오기 출발에서 3~4도 튀는 일이 남았다
    #   (2026-09-28, 지령 이상 없음). 예전에는 그 자리에서 실패로 끝나 렉에
    #   걸친 채 섰다. 렉 축 쪽으로 도는 것이라 다리 옆 외곽은 줄어든다.
    JOB_EXIT_FIX_MAX = 2
    JOB_EXIT_FIX_W   = 0.10    # 제자리 회전 상한 [rad/s]
    JOB_EXIT_FIX_K   = 1.0
    JOB_EXIT_FIX_TOL = 0.005   # 약 0.3도
    JOB_EXIT_FIX_S   = 6.0
    # 도킹 끝에 캐스터를 미리 앞쪽으로 뒤집어 둔다 [m], [m/s]. 0 이면 끔.
    #
    # ⚠ 후진으로 들어오면 캐스터가 후진 방향으로 선다. 나올 때 전진을 시작하는
    #   순간 180도 뒤집히며 차체를 홱 돌려, 나온 지 150~230mm(뒤끝이 뒤 다리
    #   옆)에서 여유를 넘어 멈췄다. 0.05 m/s 로 늦춰도 남았다(2026-09-28,
    #   사용자 지적 "출발할 때 캐스터 영향으로 급회전"). 렉 한가운데에서 기어가듯
    #   조금 앞으로 가서 미리 뒤집는다 — 그 자리가 앞뒤 다리에서 가장 멀다.
    #   뒤집히려면 캐스터 오프셋의 두 배쯤 움직여야 한다.
    JOB_PREFLIP_M   = 0.04
    # ⚠ 2026-11-13 로봇 앞끝에서 뒤로 190mm 에 포스트(넥)가 새로 섰다(사용자). 끝까지(로봇 중심 =
    #   렉 중심) 들어가면 포스트 뒤 면이 렉 중심에서 247mm — ls_origin 입구 다리 줄(283mm)보다 안쪽이라
    #   맨 아래 선반에 부딪힌다. 포스트 뒤 면이 '입구 다리 중심 + 선반 내민 몫 + 여유' 밖에 남도록
    #   그만큼 덜 들어간다(ls_origin 86mm). 리프트는 렉 중심에서 그만큼 입구 쪽을 든다.
    JOB_POST_FROM_FRONT = 0.19   # 앞끝 -> 포스트 뒤 면 [m]
    JOB_SHELF_PAST_LEG = 0.02    # 선반 끝이 다리 중심보다 내민 몫 (추정) [m]
    JOB_POST_CLEAR = 0.03        # 여유 [m]
    JOB_PREFLIP_V   = 0.03
    JOB_PREFLIP_S   = 8.0     # 시간 한도 [s]
    JOB_PREFLIP_SETTLE_S = 0.6  # 후진이 끝나고 완전히 설 때까지 [s]
    # 캐스터 뒤집기 중 방향 붙잡기. w = K x 방향오차, 상한 W.
    #
    # ⚠ 조향 없이 기어가면 캐스터가 뒤집히며 차체를 5도 돌렸다(2026-09-28,
    #   다리 옆 외곽 40mm > 여유 37mm). 튐은 초당 약 4도(0.07 rad/s)다.
    #   지연 0.72초에서 K 1.2 면 위상 여유 약 40도. base_node 반경 제한을
    #   건너뛰는 /cmd_vel_noradius 로 보낸다.
    JOB_PREFLIP_K   = 1.2
    JOB_PREFLIP_W   = 0.15
    JOB_EXIT_S      = 40.0    # 빠져나오기 시간 한도 [s]
    JOB_EXIT_NEAR   = 0.35    # 도킹 자세(POI)에서 이 안에 있어야 시작한다 [m]
    JOB_EXIT_YAW    = 0.52    # 렉 밖을 이 각(30도) 안으로 보고 있어야 한다
    # 앞 장애물. 차체 앞끝(구동축에서 0.435m)부터 이 거리 안, 폭 안쪽에 점이
    # 있으면 멈춘다. 폭은 렉 캐스터 판(축에서 179mm)을 안 잡도록 좁게 둔다.
    JOB_EXIT_FRONT  = 0.30
    # 2026-11-13 0.15 -> 0.165 (새 차체 반폭). 캐스터 판(축에서 179mm)보다는 여전히 안쪽.
    JOB_EXIT_HALF_W = 0.165
    # 캐스터 판의 반폭. 다리 중심에서 축 방향으로도 이만큼 차지한다고 본다
    # (2026-09-23 실측: 봉 중심 기준 전체 65mm).
    JOB_PLATE_HALF  = 0.0325
    # 빠져나올 때 여유를 넘은 채 이 주기 수(20Hz)만큼 이어져야 멈춘다.
    #
    # ⚠ 2026-09-28 반복 시험에서 두 번, 나온 지 150~230mm 지점에서 0.34초
    #   만에 외곽이 15 -> 44mm 로 뛰었다(초당 11도 회전에 해당, 지령은 거의
    #   0). 속도가 오르며 캐스터가 뒤집혀 차체를 차는 것으로 보고, 닿지는
    #   않았다(사용자 확인). 순간 튐은 봐주되 크게 넘으면 바로 멈춘다.
    JOB_EXIT_JAM_N  = 3
    # 나오기 시작 후 이 거리는 기어가는 속도 그대로 간다 [m].
    #
    # ⚠ 도킹하면 캐스터가 후진 방향으로 서 있다. 전진하면 180도 뒤집히는데,
    #   그동안 속도를 올리면 뒤집히는 힘이 커져 차체가 돈다(2026-09-28 지적:
    #   "빨라지면서 캐스터가 회전하니 로봇도 회전"). 튐 두 번이 모두 나온 지
    #   150~230mm, 속도 0.068~0.085 m/s 로 오르던 때였다. 다 뒤집힌 뒤에 올린다.
    JOB_EXIT_CREEP_M = 0.30
    # 나오기 최고 속도와 가속 거리. 진입(주행속도, 0.40m)보다 느리고 완만하게.
    # ⚠ 캐스터가 방향을 트는 순간 차체가 돌고, 빠를수록 그 힘이 크다
    #   (2026-09-28 지적: "나올 때 속도가 빨라지면 캐스터 영향으로 회전").
    #   2026-09-29: 뒤집히는 구간을 따로 기어가게 한 뒤 0.12 로 올려 본다.
    JOB_EXIT_V_MAX  = 0.12
    # 나오기 조향 : 방향만, 사각지대(켜는 문턱) 밖일 때만, 약하게.
    JOB_EXIT_YAW_DEAD = 0.0105   # 약 0.6도 (끄는 문턱은 40%, 약 0.25도)
    JOB_EXIT_K_YAW    = 0.5
    JOB_EXIT_RAMP_M = 0.60
    JOB_EXIT_JAM_HARD = 0.015   # 여유를 이만큼 넘으면 즉시 멈춘다 [m]
    # ⚠ 2026-11-13 진입 라이다 안전 판정은 최근 이 장수의 중앙값(좌우·방향)으로 본다. 한 장씩은
    #   방향이 ±0.5도(뒤 모서리 4mm) 흔들려, 작업 기준 모서리 8mm 로 잘 들어가는데도 한 장짜리
    #   20/24mm 두 번에 '여유 부족(14.6mm)' 으로 섰다(사용자: "잘들어가는데 왜계속 멈추는거야").
    #   한 장이 여유를 JOB_EXIT_JAM_HARD 넘게 넘으면 그 장 그대로 즉시 멈추는 것은 유지.
    JOB_LIDAR_MED_N = 5
    # 라이다 기준은 작업 기준보다 이만큼 더 봐준다 [m]. 같은 진입에서 라이다 생값이 작업 기준보다
    #   방향 0.6도 · 좌우 5mm 쯤 일관되게 더 나와(중앙값 모서리 20mm, 작업 기준 8mm) 여유 14.6mm 로는
    #   잘 들어가는 진입을 세웠다. 작업 기준 검사는 그대로 여유에서 멈춘다.
    JOB_LIDAR_NOISE = 0.008

    @staticmethod
    def _smooth(u):
        """0 에서 1 로 매끈하게 건너간다. 양 끝 기울기가 0 이라 턱이 없다."""
        u = max(0.0, min(1.0, u))
        return u * u * (3.0 - 2.0 * u)

    def _job_speed(self, gone, remain, gn):
        """후진 진입 속도. 출발은 올리고 도착은 내린다, 둘 다 매끈하게.

        ⚠ 계단으로 올리지 말 것. 200mm 에서 0.05 -> 0.20 m/s 로 한 번에
          올렸더니 그 자리에서 확 나가며 캐스터를 걷어차 틀어졌다
          (2026-09-21 지적: "천천히 가다가 갑자기 훅 감").
        """
        # 도착 쪽 — 남은 거리가 줄면 v 에서 v_slow 로 내려간다
        v_top = gn["v_slow"] + (gn["v"] - gn["v_slow"]) * self._smooth(
            remain / self.JOB_SLOW_M)
        # 출발 쪽 — 기어가기에서 v_top 으로 올라간다
        return self.JOB_CREEP_V + (v_top - self.JOB_CREEP_V) * self._smooth(
            gone / self.JOB_RAMP_M)

    def _job_gains(self):
        """지금 주행속도에 맞춘 진입 속도와 조향값.

        모두 속도에 비례한다. 그래야 속도를 바꿔도 거리 기준으로 같은
        궤적이 나온다 — 빠르면 그만큼 더 꺾어야 같은 반경이 된다.
        """
        v = self.JOB_V_FIXED
        limits = self._job_base_limits()
        if limits is not None:
            v = min(v, limits["max_linear_mps"])
        # 이득은 기준 속도의 2배까지만 키운다. 그 위로는 지연 때문에
        # 키울수록 오히려 흔들린다.
        k = min(2.0, v / self.JOB_V_REF)
        return {"v": v, "v_slow": v * self.JOB_V_SLOW_R,
                "k_lat": self.JOB_K_LAT * k, "k_yaw": self.JOB_K_YAW * k,
                "w_far": self.JOB_W_ENTER * k, "w_near": self.JOB_W_NEAR * k,
                # 6cm 가는 동안 섞는다. 20Hz 에서 한 주기에 v/20 m 를 간다.
                "blend": min(0.5, (v / self.JOB_HZ) / self.JOB_BLEND_M)}

    def _job_refresh_base_limits(self):
        """ROS 실행기를 막지 않고 실행 중 base_node의 제한을 읽는다."""
        pending = self._base_limits_future
        if pending is not None and not pending.done():
            if time.monotonic() - self._base_limits_request_t < 4.0:
                return
            pending.cancel()
        if not self._base_limits_cli.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["min_turn_radius_m", "max_angular_rps",
                         "max_linear_mps", "max_decel_mps2"]
        future = self._base_limits_cli.call_async(request)
        self._base_limits_future = future
        self._base_limits_request_t = time.monotonic()

        def received(f):
            if f is not self._base_limits_future or f.cancelled():
                return
            # 응답은 왔지만 값이 유효하지 않으면 이전 제한을 계속 쓰지 않는다.
            self._base_limits = None
            try:
                values = f.result().values
                if len(values) != len(request.names):
                    return
                limits = {}
                for name, value in zip(request.names, values):
                    if value.type != ParameterType.PARAMETER_DOUBLE:
                        return
                    number = value.double_value
                    if not math.isfinite(number) or number < 0.0:
                        return
                    limits[name] = number
                if (limits["max_angular_rps"] <= 0.0
                        or limits["max_linear_mps"] < self.JOB_CREEP_V):
                    return
                self._base_limits = limits
                self._base_limits_t = time.monotonic()
            except Exception as exc:                         # noqa: BLE001
                self.get_logger().warn("모터 제한 조회 실패: %s" % exc)

        future.add_done_callback(received)

    def _job_base_limits(self):
        if time.monotonic() - self._base_limits_t > 6.0:
            return None
        return self._base_limits

    def _job_stop_dist(self, speed):
        """지령을 끊은 뒤 더 가는 거리 [m]. 감속 제한을 모르면 예전 식."""
        v = abs(speed)
        limits = self._job_base_limits() or {}
        dec = limits.get("max_decel_mps2") or 0.0
        if dec <= 0.0:
            return v * self.JOB_STOP_LAG_S
        return v * self.JOB_STOP_DELAY_S + v * v / (2.0 * dec)

    def _on_imu(self, m):
        self._gyro_z = m.angular_velocity.z
        self._gyro_t = time.monotonic()

    def _job_dist_comp(self, w, gone, v):
        """후진 진입 중 일정한 회전 쏠림을 자이로로 추정해 지령에서 뺀다 (JOB_DIST_* 참조)."""
        now = time.monotonic()
        hist = self.__dict__.get("_job_wcmd")
        if hist is None:
            hist = self._job_wcmd = deque(maxlen=40)
        d = getattr(self, "_job_dist", 0.0)
        w_old = None
        for tt, ww in hist:
            if now - tt >= self.JOB_DIST_DELAY:
                w_old = ww
        gz_fresh = now - getattr(self, "_gyro_t", 0.0) < 0.3
        if (w_old is not None and gz_fresh and gone >= self.JOB_PREALIGN_M
                and v >= 0.03):
            a = min(1.0, (1.0 / self.JOB_HZ) / self.JOB_DIST_TAU)
            d += a * ((self._gyro_z - w_old) - d)
            d = max(-self.JOB_DIST_MAX, min(self.JOB_DIST_MAX, d))
            self._job_dist = d
        out = w - (d if gone >= self.JOB_PREALIGN_M else 0.0)
        hist.append((now, out))
        return out

    def _job_ff(self, v):
        """전진 출발 캐스터 튐 예측 보정 각속도 [rad/s]. v 는 지령 전진 속도."""
        if v <= 0.0 or not self.JOB_FF_FWD_DEG:
            return 0.0
        gone = (getattr(self, "_run_m", 0.0)
                if getattr(self, "_run_dir", 0) == 1 else 0.0)
        if gone >= self.JOB_FF_M:
            return 0.0
        return -math.radians(self.JOB_FF_FWD_DEG) * v / self.JOB_FF_M

    def _job_w_free(self, requested):
        """반경 제한 없이 모터 최대 각속도만 지키는 조향 상한."""
        limits = self._job_base_limits()
        if limits is None:
            return 0.0
        return min(requested, limits["max_angular_rps"])

    def _job_damp(self, w, cap):
        """캐스터가 돌리는 몫을 자이로로 받아친다. 자이로가 끊겼으면 그대로.

        실제 회전(자이로)에서 직전에 명령한 회전을 뺀 것이 외력이 만든
        회전이다. 그 JOB_RATE_KD 배를 반대로 더한다.
        """
        if time.monotonic() - getattr(self, "_gyro_t", 0.0) > 0.2:
            out = w
        else:
            dist = self._gyro_z - getattr(self, "_job_w_last", 0.0)
            out = w - self.JOB_RATE_KD * dist
        out = max(-cap, min(cap, out))
        self._job_w_last = out
        return out

    def _job_w_limit(self, speed, requested):
        limits = self._job_base_limits()
        if limits is None:
            return 0.0
        cap = min(requested, limits["max_angular_rps"])
        radius = limits["min_turn_radius_m"]
        if radius > 0.0:
            cap = min(cap, abs(speed) / radius)
        return cap

    def _job_reset(self, state="idle", msg=""):
        t = getattr(self, "_job_timer", None)
        self._job_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass
            self._pub_zero()
        self._job_cart = None
        self._job_cart0 = None
        self._job_cart_stamp = None
        self._job_live_hist = deque(maxlen=self.JOB_LIDAR_MED_N)
        self._job_dist = 0.0
        self._job_wcmd = deque(maxlen=40)
        self._job_trace_t = -9.0
        self._job_reanchor = False
        self._job_steer = False
        self._job_int = 0.0
        self._job_anchor = None
        self._job_entry_yaw = None
        self._job_restand = 0
        self._job_retry = 0
        self._job_find_xy = None
        self._job_creeping = False
        self._lidar_jam = (0, None)
        self._exit_then_enter = False
        self._exit_fix_n = 0
        self._job_faced = False
        self._job_repose = 0
        self._job_stage_fix = 0
        self._job_stand = 0.0
        self._job_spin_target = 0.0
        self._job_spin_next = "enter"
        self._job_spin_cut = 0.0
        self._job_spin_trace = -9.0
        self._job_steer = False
        self._job_int = 0.0
        self._job_faced = False
        self._job_goal = None
        self._job_aim_n = 0
        self._job_cart0 = None
        self._job_w = 0.0
        self._job_settle = 0.0
        self._job_bad = 0
        self.job = {"state": state, "message": msg, "poi": None,
                    "cart": None, "step": "", "remain": None}

    def job_status(self):
        j = dict(getattr(self, "job", None)
                 or {"state": "idle", "message": "", "poi": None,
                     "cart": None, "step": "", "remain": None})
        j["align"] = self._job_align()
        return j

    def _job_align(self):
        """지금 보이는 대차 정렬값. 오래됐거나 없으면 None."""
        with self.lock:
            c, t = self.carts, self.carts_t
        if not c or (time.time() - t) > self.JOB_FRESH_S:
            return None
        al = c.get("align")
        return al if isinstance(al, dict) and al.get("ready") else None

    def job_start(self, poi_name):
        if self.estop:
            return False, "비상정지 상태입니다"
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        if self.control_mode != "auto":
            return False, "자동 모드에서만 됩니다"
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            return False, "이미 작업 중입니다"
        if getattr(self, "cdock", {}).get("state") in self.CD_BUSY:
            return False, "충전소 도킹 중입니다"
        poi = self.poi_find(poi_name)
        if poi is None:
            return False, "'%s' 가 없습니다" % poi_name
        if poi["kind"] != "cart":
            return False, "'%s' 는 대차 지점이 아닙니다" % poi_name
        clearance_error = self._job_clearance_error(poi.get("cart"))
        if clearance_error:
            return False, clearance_error
        if self._job_base_limits() is None:
            return False, "모터 주행 제한을 확인하지 못했습니다. 잠시 후 다시 시도해 주세요"
        # 이미 입구 앞에서 렉을 등지고 축에 맞춰 서 있으면(앞선 도킹이 렉 밖에서
        # 멈춘 자리 등) nav2 를 쓰지 않는다 — 정지 구역에 렉 다리가 들어 출발을
        # 못 했다(2026-09-28). 축을 따라 물러났다가 돌지 않고 바로 들어간다.
        rp0 = self._job_pose()
        # ⚠ 2026-11-13 충전소에 붙은 채 렉 작업을 걸었더니 '먼저 돌아섬' 이 충전소 나오기보다
        #   먼저 돌아, 뒤가 스테이션에 걸린 채 제자리 104도를 돌았다 — 바퀴가 미끄러져
        #   위치가 바로 틀어졌다(사용자: "도킹 렉1로 보낼려는데 바로 위치정보가 틀어짐").
        #   붙어 있으면 제자리 회전·축 따라 물러나기를 건너뛰고 nav_goto 로 보낸다.
        #   nav_goto 가 0.5m 곧게 나간 뒤 출발 정렬을 한다.
        docked = self._cdock_leave_needed()
        if docked:
            self.get_logger().info("작업 시작: 충전소에 붙어 있음 — 먼저 곧게 나간 뒤 렉으로")
        if not docked and rp0 is not None and math.hypot(poi["x"] - rp0[0],
                                                         poi["y"] - rp0[1]) < 1.2:
            q0 = self._job_pick_near((poi["x"], poi["y"]), self.JOB_FIND_M,
                                     want=poi.get("cart"))
            cart0 = ((q0["x"], q0["y"], q0["yaw"]) if q0
                     else (poi["x"], poi["y"], poi["yaw"]))
            self._job_reset()
            self._job_poi = dict(poi)
            self.job = {"state": "idle", "message": "", "poi": poi["name"],
                        "cart": poi.get("cart"), "step": "", "remain": None}
            if self._job_backoff_start(poi, cart0, rp0, "렉 입구 앞에서 시작", -1):
                self._job_retry = 0
                self._job_timer = self.create_timer(1.0 / self.JOB_HZ,
                                                    self._job_tick)
                self.get_logger().info("작업 시작: %s (입구 앞 — 축 따라 물러났다 진입)"
                                       % poi["name"])
                return True, "'%s' 작업을 시작합니다" % poi["name"]
        # POI 는 도킹을 마쳤을 때의 자세다. 로봇은 거기서 바라보는 쪽으로
        # standoff 만큼 나간 자리에, 렉을 돌아보는 방향으로 선다.
        so = float(poi.get("standoff") or self.POI_STANDOFF)
        ax = poi["x"] + so * math.cos(poi["yaw"])
        ay = poi["y"] + so * math.sin(poi["yaw"])
        rp = self._job_pose()
        self._job_reset()
        self._job_goal = (ax, ay, poi["yaw"] + math.pi)
        # ⚠ 목표가 크게 등 뒤에 있으면 nav2 에 그냥 넘기지 않는다. 경로가
        #   짧은데 방향이 크게 다르면 RPP 는 앞점이 자꾸 등 뒤로 넘어가
        #   '돌고 조금 가고 또 돌고' 를 되풀이한다 — 45도씩 끊어 도는 것이
        #   그것이다(2026-09-16). 먼저 한 번에 부드럽게 돌아서고 넘긴다.
        if rp is not None and not docked:
            bear = math.atan2(ay - rp[1], ax - rp[0])
            if abs(wrap_pi(bear - rp[2])) > self.JOB_FACE_MIN:
                self.job = {"state": "staging", "message": "목표 쪽으로 도는 중",
                            "poi": poi["name"], "cart": poi.get("cart"),
                            "step": "", "remain": None}
                self._job_poi = dict(poi)
                self._job_t = 0.0
                self._job_seen = 0
                self._job_hits = []
                self._job_seen_t = 0.0
                self._job_spin_start(bear, poi.get("cart"), nxt="approach",
                                     msg="목표 쪽으로 도는 중")
                self._job_timer = self.create_timer(1.0 / self.JOB_HZ,
                                                    self._job_tick)
                self.get_logger().info("작업 시작: %s (먼저 돌아섬)" % poi["name"])
                return True, "'%s' 작업을 시작합니다" % poi["name"]
        # 충전소에서 나온 뒤에는 목표가 크게 옆·뒤일 수 있어, 출발 정렬(먼저 제자리로
        # 돌고 저속 출발)을 쓴다. 정밀 도착은 렉 작업이 도착 뒤 따로 맞추므로 끈다.
        ok, msg = self.nav_goto(ax, ay, poi["yaw"] + math.pi,
                                yaw_ok=self.JOB_MID_YAW, turn_max=1, depart=docked)
        if docked:
            self._nav_precise = False
        if not ok:
            return False, "이동을 걸지 못했습니다: %s" % msg
        self.job = {"state": "moving",
                    "message": "대차 앞 %.0fmm 까지 이동 중" % (so * 1000),
                    "poi": poi["name"], "cart": poi.get("cart"),
                    "step": "", "remain": None}
        self._job_poi = dict(poi)
        self._job_t = 0.0
        self._job_seen = 0
        self._job_hits = []
        self._job_timer = self.create_timer(1.0 / self.JOB_HZ, self._job_tick)
        self.get_logger().info("작업 시작: %s" % poi["name"])
        return True, "'%s' 작업을 시작합니다" % poi["name"]

    def job_exit(self):
        """도킹한 렉에서 렉 축을 따라 앞으로 곧게 빠져나온다.

        도킹을 마치면 로봇은 렉 밖을 보고 있다. 가까운 대차 POI 를 찾아
        그 렉 축(라이다로 보이면 실제 렉, 아니면 POI)을 기준으로 삼고,
        진입과 같은 선 추종으로 전진한다. 차체 뒤끝이 렉을 벗어나면 멈춘다.
        """
        if self.estop:
            return False, "비상정지 상태입니다"
        if self.control_mode != "auto":
            return False, "자동 모드에서만 됩니다"
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            return False, "이미 작업 중입니다"
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        if self._job_base_limits() is None:
            return False, "모터 주행 제한을 확인하지 못했습니다. 잠시 후 다시 시도해 주세요"
        rp = self._job_pose()
        rpo = self._job_pose("odom")
        if rp is None or rpo is None:
            return False, "로봇 위치를 모릅니다"
        # 기준 축 : (1) 방금 도킹할 때 쓴 렉 기준(오돔) (2) 로봇 근처에서 라이다로
        # 본 렉. POI 로는 대신하지 않는다.
        #
        # ⚠ 예전에는 'POI 근처 45cm 에서 보이는 렉, 없으면 POI' 였다. 지도
        #   위치가 틀어지면(0.5~0.6m, 2026-09-28~29) 렉이 보여도 POI 근처가
        #   아니라 못 찾고 POI 를 기준으로 삼았다 — 렉 한가운데 서 있는데
        #   '축 좌우 -35mm' 로 계산해 다리 사이에서 옆으로 틀다 외곽 64mm
        #   로 멈췄다(2026-09-29). '렉 밑에 있지 않습니다' 거부도 같은 원인.
        with self.lock:
            pois = [dict(p) for p in self.pois if p.get("kind") == "cart"]
        r = None
        src = None
        poi = None
        docked = getattr(self, "_docked", None)
        if docked and docked.get("c") is not None:
            r0 = self._job_to_robot(rpo, docked["c"])
            if math.hypot(r0["forward"], r0["lateral"]) <= self.JOB_EXIT_NEAR:
                r, src = r0, "도킹 기준"
                poi = next((p for p in pois if p["name"] == docked.get("poi")),
                           None) or {"name": docked.get("poi") or "렉",
                                     "cart": docked.get("cart")}
        if r is None:
            q = self._job_pick_near((rp[0], rp[1]), self.JOB_EXIT_NEAR)
            if q is not None:
                r, src = self._job_to_robot(rp, (q["x"], q["y"], q["yaw"])), "라이다"
                same = [p for p in pois if p.get("cart") == q["cart"]]
                poi = (min(same, key=lambda p: math.hypot(p["x"] - q["x"],
                                                          p["y"] - q["y"]))
                       if same else {"name": q["cart"], "cart": q["cart"]})
        if r is None:
            return False, ("렉 밑에서 렉을 확인하지 못해 나오기를 시작하지 않습니다"
                           " (라이다로 로봇 %.0fcm 안에 렉이 보여야 합니다)"
                           % (self.JOB_EXIT_NEAR * 100))
        # 90도 주기로 접힌 축 중 로봇 머리에 가장 가까운 것이 나가는 쪽이다.
        if abs(r["yaw"]) > self.JOB_EXIT_YAW:
            return False, "렉 밖을 보고 있지 않습니다 (축과 %.0f도)" % (
                math.degrees(r["yaw"]))
        c = self._job_to_map(rpo, r)      # 오돔 기준 렉 중심과 나가는 축
        depth = float(self._cart_spec(poi.get("cart")).get("depth") or 0.0)
        self._job_reset()
        self._exit_c = c
        self._exit_jam = 0
        self._exit_fix_n = 0
        self._exit_steer = False
        self._exit_rack_end = depth / 2.0 + self.JOB_HALF_L
        self._exit_target = self._exit_rack_end + self.JOB_EXIT_CLEAR
        self._job_start_xy = (rpo[0], rpo[1])
        self._job_t = 0.0
        self.job = {"state": "exiting", "message": "렉에서 빠져나오는 중",
                    "poi": poi["name"], "cart": poi.get("cart"),
                    "step": "", "remain": None}
        self._job_timer = self.create_timer(1.0 / self.JOB_HZ, self._job_tick)
        s0, e0, y0 = self._exit_axis(rpo)
        self.get_logger().info(
            "빠져나오기 시작: %s (%s) 축 좌우 %+.0fmm 방향 %+.1f도, "
            "나갈 거리 %.0fmm" % (poi["name"], src,
                                  e0 * 1000, math.degrees(y0),
                                  (self._exit_target - s0) * 1000))
        return True, "'%s' 에서 빠져나옵니다" % poi["name"]

    def _job_center_offset(self):
        """실제 통로 중심이 라이다 렉 축에서 도킹 자세 기준 왼쪽으로 떨어진 거리 [m].

        carts.yaml 의 center_offset. 캐스터 판이 좌우 다르게 튀어나와 있으면
        라이다가 보는 기둥 축과 실제로 지나갈 수 있는 통로의 중심이 다르다.
        """
        try:
            v = float(self._cart_spec(self.job.get("cart"))
                      .get("center_offset") or 0.0)
        except (TypeError, ValueError, AttributeError):
            return 0.0
        v = v if math.isfinite(v) and abs(v) < 0.05 else 0.0
        return v + self.JOB_REAR_R_PROTRUDE / 2.0

    def _exit_axis(self, rpo):
        """오돔 자세를 나가는 축 기준 (나온 거리, 좌우, 방향) 으로.

        좌우 + 는 축의 왼쪽(나가는 쪽을 보고). 방향 + 는 왼쪽으로 틀어짐.
        """
        cx, cy, a = self._exit_c
        dx, dy = rpo[0] - cx, rpo[1] - cy
        ca, sa = math.cos(a), math.sin(a)
        # 좌우는 실제 통로 중심 기준(center_offset 만큼 왼쪽). 나올 때도
        # 로봇은 도킹 자세와 같은 쪽을 보므로 왼쪽의 뜻이 같다.
        return (dx * ca + dy * sa,
                -dx * sa + dy * ca - self._job_center_offset(),
                wrap_pi(rpo[2] - a))

    def _exit_front_blocked(self):
        """차체 앞에 장애물이 있나. 진입과 달리 nav2 를 거치지 않으므로 따로 본다."""
        with self.lock:
            m = self.scan
        if m is None:
            return False
        x0 = self.JOB_HALF_L
        a = m.angle_min
        for rng in m.ranges:
            if m.range_min < rng < m.range_max:
                # 라이다는 구동축 근처(x -0.015)에 있다. 몇 mm 는 무시한다.
                x, y = rng * math.cos(a), rng * math.sin(a)
                if (x0 < x < x0 + self.JOB_EXIT_FRONT
                        and abs(y) < self.JOB_EXIT_HALF_W):
                    return True
            a += m.angle_increment
        return False

    def _exit_envelope(self, s_out, e, yaw, depth):
        """다리 자리에서 차체 가장자리가 축에서 벗어난 거리 [m].

        ⚠ 모서리(|좌우| + 반길이*sin방향)는 '차체 끝이 다리 옆에 있다' 고
          본다. 빠져나오는 도중에는 뒤끝이 이미 뒤 다리를 지나 앞 다리만
          옆에 걸쳐 있고, 그 다리까지의 지렛대는 50~80mm 뿐이다. 그래서
          닿지도 않았는데 '모서리 50mm' 로 멈췄다(2026-09-28 반복 3회차).
          실제로 다리 옆에 걸친 차체 구간만 본다.
        """
        worst = 0.0
        tn = math.tan(yaw)
        for xc in (-depth / 2.0, depth / 2.0):
            lo = max(xc - self.JOB_PLATE_HALF, s_out - self.JOB_HALF_L)
            hi = min(xc + self.JOB_PLATE_HALF, s_out + self.JOB_HALF_L)
            if lo > hi:
                continue
            for x in (lo, hi):
                worst = max(worst, abs(e + (x - s_out) * tn))
        return worst

    def _job_exit_fix(self):
        """나오다 멈춘 자리에서 천천히 제자리 회전해 렉 축에 맞춘다."""
        rpo = self._job_pose("odom")
        if rpo is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        s_out, e, yaw = self._exit_axis(rpo)
        depth = float(self._cart_spec(self.job.get("cart")).get("depth")
                      or 0.0)
        corner = self._exit_envelope(s_out, e, yaw, depth)
        free = self._job_free(self.job.get("cart"))
        self._exit_fix_t += 1.0 / self.JOB_HZ
        if free is not None and corner > free + self.JOB_EXIT_JAM_HARD:
            self._job_fail("방향을 맞추다 여유를 크게 넘어 멈췄습니다 — 외곽 %.0fmm"
                           % (corner * 1000))
            return
        if abs(yaw) <= self.JOB_EXIT_FIX_TOL or self._exit_fix_t > self.JOB_EXIT_FIX_S:
            self._pub_zero()
            self.get_logger().info(
                "방향 맞춤 끝: 방향 %+.2f도 외곽 %.0fmm — 나오기 계속"
                % (math.degrees(yaw), corner * 1000))
            self._exit_jam = 0
            self.job.update(state="exiting", message="렉에서 빠져나오는 중")
            return
        t = Twist()
        # 나오는 축 기준 방향' = w 이므로 w = -K x 방향
        t.angular.z = float(max(-self.JOB_EXIT_FIX_W, min(
            self.JOB_EXIT_FIX_W, -self.JOB_EXIT_FIX_K * yaw)))
        self.pub_twist.publish(t)

    def _job_exiting(self):
        """렉 축을 따라 앞으로 나온다. 진입과 같은 선 추종, 방향만 반대.

        전진이므로 e' = v*방향, 방향' = w. w = -(k_lat*e + k_yaw*방향).
        """
        rpo = self._job_pose("odom")
        if rpo is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        s_out, e, yaw = self._exit_axis(rpo)
        remain = self._exit_target - s_out
        depth = float(self._cart_spec(self.job.get("cart")).get("depth")
                      or 0.0)
        corner = self._exit_envelope(s_out, e, yaw, depth)
        in_legs = s_out < self._exit_rack_end
        free = self._job_free(self.job.get("cart"))
        gone = math.hypot(rpo[0] - self._job_start_xy[0],
                          rpo[1] - self._job_start_xy[1])
        # --- 안전 ---
        if abs(e) > self.JOB_BAD_LAT:
            self._job_fail("빠져나오다 렉 축에서 좌우로 %.0fmm 벌어져 멈췄습니다"
                           % (e * 1000))
            return
        if in_legs and free is not None and corner > 0.6 * free:
            # 튐을 들여다보려고 매 주기 남긴다
            self.get_logger().info(
                "  나오기 근접 %.2fs 나온 %.0fmm 축 좌우 %+.1fmm 방향 %+.2f도"
                " 외곽 %.1fmm / 여유 %.0fmm" % (
                    self._job_t, s_out * 1000, e * 1000, math.degrees(yaw),
                    corner * 1000, free * 1000))
        over = in_legs and free is not None and corner > free * self.JOB_JAM_R
        self._exit_jam = (getattr(self, "_exit_jam", 0) + 1) if over else 0
        if (over and self._exit_jam >= self.JOB_EXIT_JAM_N
                and corner <= free + self.JOB_EXIT_JAM_HARD
                and getattr(self, "_exit_fix_n", 0) < self.JOB_EXIT_FIX_MAX):
            self._pub_zero()
            self._exit_fix_n = getattr(self, "_exit_fix_n", 0) + 1
            self._exit_fix_t = 0.0
            self._exit_jam = 0
            self.job.update(state="exit_fix",
                            message="나오다 틀어져 방향을 맞추는 중 (%d/%d)"
                            % (self._exit_fix_n, self.JOB_EXIT_FIX_MAX))
            self.get_logger().warn(
                "나오기 튐 — 멈추고 방향 맞춤 %d/%d: 외곽 %.0fmm 방향 %+.1f도"
                % (self._exit_fix_n, self.JOB_EXIT_FIX_MAX, corner * 1000,
                   math.degrees(yaw)))
            return
        if over and (self._exit_jam >= self.JOB_EXIT_JAM_N
                     or corner > free + self.JOB_EXIT_JAM_HARD):
            self._job_fail("빠져나오다 다리 여유를 넘어 멈췄습니다 — 외곽 %.0fmm"
                           " (여유 %.0fmm)" % (corner * 1000, free * 1000))
            return
        if gone > self._exit_target + self.JOB_ENTER_EXTRA:
            self._job_fail("예상 거리를 넘겨 멈췄습니다")
            return
        if self._job_t > self.JOB_EXIT_S:
            self._job_fail("빠져나오기 시간이 초과되었습니다")
            return
        if self._exit_front_blocked():
            self._job_fail("앞에 장애물이 있어 멈췄습니다")
            return
        gn = self._job_gains()
        if remain <= self._job_stop_dist(gn["v_slow"]):
            self._pub_zero()
            if getattr(self, "_exit_then_enter", False):
                self._exit_then_enter = False
                self._job_reenter()
                return
            poi, cart = self.job.get("poi"), self.job.get("cart")
            self._job_reset("idle", "")
            self.job.update(state="done", message="렉에서 빠져나왔습니다",
                            poi=poi, cart=cart)
            self.get_logger().info(
                "빠져나오기 끝: 축 좌우 %+.0fmm 방향 %+.2f도" % (
                    e * 1000, math.degrees(yaw)))
            return
        ge = dict(gn)
        ge["v"] = min(gn["v"], self.JOB_EXIT_V_MAX)
        ge["v_slow"] = min(gn["v_slow"], ge["v"])
        # 가속 거리만 늘린다: 기어가기 뒤 남은 거리를 비율로 줄여 넘긴다
        v_now = self._job_speed(
            max(0.0, gone - self.JOB_EXIT_CREEP_M)
            * self.JOB_RAMP_M / self.JOB_EXIT_RAMP_M, remain, ge)
        wm = (self._job_w_free(gn["w_near"] if in_legs else gn["w_far"])
              if self.JOB_FREE_STEER else
              self._job_w_limit(v_now, gn["w_near"] if in_legs else gn["w_far"]))
        # 방향만 렉 축에 맞추고 곧게 간다. 좌우는 고치지 않는다.
        #
        # ⚠ 진입과 같은 선 추종으로 다리 사이의 좌우 5~12mm 까지 되돌리려다
        #   지연에 넘어가며 방향이 ±2~3도로 4~5번 오갔다 — 나올 때도 S자
        #   (2026-09-29 지적). 출발 방향이 0도였던 회차만 곧게 나왔다.
        #   나오는 데는 좌우를 고칠 필요가 없다. 틀어진 방향을 한 번 펴고
        #   (사각지대 밖일 때만) 그다음은 조향 없이 직진한다. 다리 옆 외곽은
        #   위에서 계속 본다.
        if abs(yaw) > self.JOB_EXIT_YAW_DEAD:
            self._exit_steer = True
        elif abs(yaw) < self.JOB_EXIT_YAW_DEAD * 0.4:
            self._exit_steer = False
        raw = -self.JOB_EXIT_K_YAW * yaw
        w = (max(-wm, min(wm, raw))
             if getattr(self, "_exit_steer", False) else 0.0)
        w += self._job_ff(v_now)
        # 후진 뒤 처음 JOB_PREALIGN_M 은 캐스터가 뒤집히는 구간이다. 진입
        # 출발과 같이 기어가며 방향을 붙잡는다(사각지대 없이). 캐스터 미리
        # 뒤집기(40mm)도 이 구간에 들어가므로 거리는 odom 에서 이어서 센다.
        run_m = (getattr(self, "_run_m", 0.0)
                 if getattr(self, "_run_dir", 0) == 1 else 0.0)
        if run_m < self.JOB_PREALIGN_M:
            v_now = min(v_now, self.JOB_FLIP_V)
            w = max(-self.JOB_PREFLIP_W, min(
                self.JOB_PREFLIP_W, -self.JOB_PREFLIP_K * yaw))
        if self.JOB_FREE_STEER:
            # 곧게 가는 동안(조향 끔)에도 캐스터가 돌리는 것은 받아친다
            w = self._job_damp(w, wm)
        t = Twist()
        t.linear.x = v_now
        t.angular.z = float(w)
        (self.pub_twist_free if self.JOB_FREE_STEER else self.pub_twist).publish(t)
        self.job["remain"] = round(remain, 3)
        self.job["step"] = "남은 %.0fmm 좌우 %.0fmm" % (remain * 1000, e * 1000)
        now = time.time()
        if now - getattr(self, "_job_ent_t", 0.0) > 0.4:
            self._job_ent_t = now
            self.get_logger().info(
                "  나오기 남은 %4.0fmm  축 좌우 %+4.0fmm  방향 %+5.1f도  "
                "모서리 %4.0f%s  지령 %+.3f 상한 %.3f 속도 %.3f"
                % (remain * 1000, e * 1000, math.degrees(yaw), corner * 1000,
                   " (다리 사이)" if in_legs else "", w, wm, v_now))

    def job_cancel(self):
        st = getattr(self, "job", {}).get("state")
        if st not in self.JOB_BUSY:
            self._pub_zero()
            return True, "진행 중인 작업이 없습니다"
        if st == "moving":
            self.nav_cancel()
        self._job_reset("canceled", "작업을 취소했습니다")
        self._pub_zero()
        self.get_logger().info("작업 취소")
        return True, "작업을 취소했습니다"

    def _job_fail(self, why):
        self._pub_zero()
        if hasattr(self, "pub_twist_free"):
            self.pub_twist_free.publish(Twist())
        poi = self.job.get("poi")
        self._job_reset("failed", why)
        self.job["poi"] = poi
        self.get_logger().warn("작업 실패: %s" % why)

    def _job_backoff_start(self, poi, cart, rp, why, n):
        """렉을 등지고 입구 앞에 있으면 축을 따라 곧게 물러난 뒤 다시 진입한다.

        나오기(exiting)와 같은 동작이고, 끝나면 돌지 않고 바로 후진 진입한다.
        조건이 안 맞으면 False (그때는 nav2 로 다시 선다).
        """
        rpo = self._job_pose("odom")
        if rpo is None:
            return False
        r = self._job_to_robot(rp, cart)
        # 렉이 로봇 뒤에 있고(등지고), 축과 나란해야 한다
        if r["forward"] > -0.30 or abs(r["yaw"]) > self.JOB_EXIT_YAW:
            return False
        so = float(poi.get("standoff") or self.POI_STANDOFF)
        depth = float(self._cart_spec(poi.get("cart")).get("depth") or 0.0)
        c = self._job_to_map(rpo, r)      # 오돔 기준 렉 중심과 나가는 축
        self._exit_c = c
        self._exit_jam = 0
        self._exit_rack_end = depth / 2.0 + self.JOB_HALF_L
        self._exit_target = so + self.JOB_BACKOFF_EXTRA
        self._exit_then_enter = True
        self._exit_fix_n = 0
        self._job_start_xy = (rpo[0], rpo[1])
        self._job_retry = n + 1
        self._job_t = 0.0
        self._job_steer = False
        self._job_int = 0.0
        s0, e0, y0 = self._exit_axis(rpo)
        # ⚠ 2026-11-13 짧게 물러나기(입구+0.45m)는 되돌림: 다시 후진을 시작하면 캐스터가 뒤집히며
        #   첫 10~20cm 에 방향을 2~2.5도 차는데, 1.17m 에서 시작하면 판정 구간(0.80m)까지 그걸 지울
        #   거리가 없어 재시도가 매번 실패했다. JOB_BACKOFF_SHORT 가 True 일 때만 짧게 간다.
        if self.JOB_BACKOFF_SHORT:
            self._exit_target = min(self._exit_target,
                                    max(s0 + self.JOB_BACKOFF_MIN,
                                        self._exit_rack_end + self.JOB_BACKOFF_RUN))
        self.job.update(state="exiting", step="",
                        message="진입 자세가 나빠 물러났다 다시 들어가는 중 (%d/%d)"
                        % (self._job_retry, self.JOB_RETRY_MAX))
        self.get_logger().warn(
            "다시 시도 %d/%d (축 따라 %.0fmm 물러남): %s" % (
                self._job_retry, self.JOB_RETRY_MAX,
                max(0.0, self._exit_target - s0) * 1000, why))
        return True

    def _job_reenter(self):
        """물러난 자리(렉을 등지고 축 위)에서 돌지 않고 바로 후진 진입한다."""
        rpo = self._job_pose("odom")
        if rpo is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        self._job_cart = self._exit_c       # 오돔 기준 렉 중심 (축은 90도 주기)
        self._job_cart0 = self._job_cart
        self._job_cart_stamp = None
        self._job_bad = 0
        self._job_steer = False
        self._job_int = 0.0
        self._lidar_jam = (0, None)
        self._job_t = 0.0
        self._job_start_xy = (rpo[0], rpo[1])
        self._job_start_yaw = None
        r = self._job_to_robot(rpo, self._job_cart)
        self._job_budget = -r["forward"] + self.JOB_ENTER_EXTRA
        self.job.update(state="entering", message="후진 진입 중 (다시)")
        self.get_logger().info("다시 후진 진입: 남은 %.0fmm 좌우 %.0fmm" % (
            -r["forward"] * 1000, -r["lateral"] * 1000))

    def _job_retry_or_fail(self, why, inside=False):
        """렉 밖에서 멈춘 것이면 대기 자리로 돌아가 한 번 더 한다.

        다리 사이에 들어간 뒤(inside)는 nav2 로 빼면 안 된다 — 그대로 멈춘다.
        """
        self._pub_zero()
        n = getattr(self, "_job_retry", 0)
        poi = getattr(self, "_job_poi", None)
        rp = self._job_pose()
        if (inside or n >= self.JOB_RETRY_MAX or rp is None
                or not poi or not all(k in poi for k in ("x", "y", "yaw"))):
            self._job_fail(why)
            return
        q = self._job_pick((poi["x"], poi["y"]), self.JOB_FIND_M)
        cart = ((q["x"], q["y"], q["yaw"]) if q is not None
                else (poi["x"], poi["y"], poi["yaw"]))
        if self._job_backoff_start(poi, cart, rp, why, n):
            return
        sp = self._job_stand_point(rp, cart)
        if not self._job_stand_ok(sp):
            self._job_fail(why)
            return
        ok, _ = self.nav_goto(sp[0], sp[1], sp[2],
                              yaw_ok=self.JOB_MID_YAW, turn_max=1)
        if not ok:
            self._job_fail(why)
            return
        self._job_retry = n + 1
        self._job_goal = sp
        self._job_restand = 0
        self._job_faced = False
        self._job_hits = []
        self._job_seen = 0
        self._job_seen_t = 0.0
        self._job_t = 0.0
        self._job_steer = False
        self._job_int = 0.0
        self.job.update(state="moving", step="",
                        message="진입 자세가 나빠 다시 시도하는 중 (%d/%d)"
                        % (self._job_retry, self.JOB_RETRY_MAX))
        self.get_logger().warn("다시 시도 %d/%d: %s" % (
            self._job_retry, self.JOB_RETRY_MAX, why))

    def _job_leg_envelope(self, remain, lat, yaw):
        """다 들어간 뒤, 다리 옆에 걸친 차체 구간이 축에서 벗어난 거리 [m].

        remain 은 렉 중심까지 남은 거리(렉 축 방향), lat 은 렉 축 좌우.
        |좌우| + 다리까지 지렛대 x |tan 방향| 로, 부호를 따지지 않아 늘 나쁜 쪽이다.

        ⚠ 모서리 식(|좌우| + 반길이 x sin방향)은 '차체 끝이 다리 옆' 이라 본다.
          렉 한가운데에서는 다리까지 지렛대가 반길이(435mm)가 아니라 최대
          316mm(283 + 판 32.5)다. 캐스터 미리 뒤집기 중 4도 튄 것을 38mm 로
          읽어 닿지 않았는데 멈췄다(2026-09-28). 실제 다리 자리로 잰다.
        """
        depth = float(self._cart_spec(self.job.get("cart")).get("depth")
                      or 0.0)
        s_c = remain                   # 렉 중심에서 로봇 중심까지 (입구 쪽 +)
        tn = abs(math.tan(yaw))
        worst = abs(lat)
        for xc in (-depth / 2.0, depth / 2.0):
            lo = max(xc - self.JOB_PLATE_HALF, s_c - self.JOB_HALF_L)
            hi = min(xc + self.JOB_PLATE_HALF, s_c + self.JOB_HALF_L)
            if lo > hi:
                continue
            lever = max(abs(lo - s_c), abs(hi - s_c))
            worst = max(worst, abs(lat) + lever * tn)
        return worst

    def _job_post_stop(self, cart=None):
        """포스트가 렉 선반 아래로 안 들어가도록 렉 중심 앞에서 멈출 거리 [m] (0 이상)."""
        try:
            depth = float(self._cart_spec(cart or self.job.get("cart")).get("depth") or 0.0)
        except (TypeError, ValueError, AttributeError):
            depth = 0.0
        if depth <= 0.0:
            depth = 0.70                      # 모르면 넉넉히 (가장 깊은 등록 렉 수준)
        need = (depth / 2.0 + self.JOB_SHELF_PAST_LEG + self.JOB_POST_CLEAR
                - (self.JOB_HALF_L - self.JOB_POST_FROM_FRONT))
        return max(0.0, need)

    def _job_preflip_start(self):
        """다 들어갔다. 캐스터를 앞쪽으로 뒤집어 두는 단계로 넘어간다."""
        self._pub_zero()
        rpo = self._job_pose("odom")
        if self.JOB_PREFLIP_M <= 0.0 or rpo is None:
            self._job_done()
            return
        self._preflip_xy = (rpo[0], rpo[1])
        # ⚠ 후진이 끝나자마자 전진을 시키면 관성으로 먼저 17mm 더 들어갔다가
        #   나왔고, 거리를 출발점에서의 직선거리로 재서 그것까지 셌다 — 실제로는
        #   20mm 만 나와 캐스터가 덜 뒤집혔다(2026-09-28). 먼저 완전히 세우고,
        #   렉 축을 따라 앞으로 나온 거리(부호 있음)로 잰다.
        self._preflip_settle = self.JOB_PREFLIP_SETTLE_S
        self._preflip_rem0 = None
        self._job_t = 0.0
        self.job.update(state="preflip", message="캐스터 정렬 중 (조금 앞으로)")
        self.get_logger().info("캐스터 미리 뒤집기: %.0fmm 앞으로" % (
            self.JOB_PREFLIP_M * 1000))

    def _job_preflip(self):
        """렉 한가운데에서 조금 앞으로 기어간다. 조향 없이, 외곽은 계속 본다."""
        rpo = self._job_pose("odom")
        if rpo is None or self._job_cart is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        r = self._job_to_robot(rpo, self._job_cart)
        rem, lat, _ = self._job_entry_geometry(r)
        corner = self._job_leg_envelope(rem, lat, r["yaw"])
        if getattr(self, "_preflip_settle", 0.0) > 0.0:
            self._preflip_settle -= 1.0 / self.JOB_HZ
            self._pub_zero()
            if self._preflip_settle <= 0.0:
                self._preflip_rem0 = rem
            return
        free = self._job_free(self.job.get("cart"))
        if free is not None and corner > free * self.JOB_JAM_R:
            self._job_fail("캐스터 정렬 중 여유를 넘어 멈췄습니다 — 다리 옆 외곽 %.0fmm"
                           " (여유 %.0fmm)" % (corner * 1000, free * 1000))
            return
        # 렉 축 방향으로 앞으로(입구 쪽으로) 나온 거리. 남은 거리가 늘어난 만큼.
        rem0 = getattr(self, "_preflip_rem0", None)
        gone = (rem - rem0) if rem0 is not None else 0.0
        # 0.03 m/s 에서는 지령을 끊은 뒤 몇 mm 만 더 간다(감속 몫). 지연 몫까지
        # 빼면 절반밖에 못 나온다.
        if (gone >= self.JOB_PREFLIP_M - 0.005
                or self._job_t > self.JOB_PREFLIP_S):
            self.pub_twist_free.publish(Twist())
            self._pub_zero()
            self.get_logger().info(
                "캐스터 미리 뒤집기 끝: %.0fmm 나옴, 렉 축 좌우 %+.0fmm 방향 %+.2f도"
                % (gone * 1000, lat * 1000, math.degrees(r["yaw"])))
            self._job_done()
            return
        t = Twist()
        t.linear.x = self.JOB_PREFLIP_V
        # 렉 축에 나란하도록 붙잡는다 (방향' = -w, 진입 조향과 같은 부호)
        t.angular.z = float(max(-self.JOB_PREFLIP_W, min(
            self.JOB_PREFLIP_W, self.JOB_PREFLIP_K * r["yaw"]
            + self._job_ff(self.JOB_PREFLIP_V))))
        self.pub_twist_free.publish(t)
        now = time.time()
        if now - getattr(self, "_preflip_log_t", 0.0) >= 0.1:
            self._preflip_log_t = now
            self.get_logger().info(
                "  뒤집기 %.0fmm 방향 %+.2f도 좌우 %+.1fmm 외곽 %.1fmm 지령 %+.3f"
                % (gone * 1000, math.degrees(r["yaw"]), lat * 1000,
                   corner * 1000, t.angular.z))

    def _job_done(self):
        self._pub_zero()
        clearance_error = self._job_clearance_error(self.job.get("cart"))
        if clearance_error:
            self._job_fail(clearance_error)
            return
        # 작업이 스스로 보는 최종 어긋남. 밖에서 잰 값(dock_accuracy)과
        # 다르면, 제어가 아니라 '보는 값' 이 틀린 것이다.
        #
        # ⚠ 여기서 모서리를 보지 않아, 좌우 -22mm / 방향 -6.37도 로 렉을
        #   밀고 들어간 것이 '대차 중앙까지 후진 진입했습니다' 로 끝났다
        #   (2026-09-22). 모서리로 환산하면 117mm 로 여유 56mm 의 두 배다.
        #   진입 중 관문이 좌우만 보고 있었으니(방향은 어느 관문에도 걸리지
        #   않는다) 마지막 판정이 유일하게 남은 그물이다.
        rp = self._job_pose("odom")
        corner = None
        if rp is not None and self._job_cart is not None:
            r = self._job_to_robot(rp, self._job_cart)
            rem0, lat, _ = self._job_entry_geometry(r)
            corner = self._job_leg_envelope(rem0, lat, r["yaw"])
            self.get_logger().info(
                "진입 끝(작업 기준): 렉 축 좌우 %+.0fmm 앞뒤 %+.0fmm 방향 %+.2f도"
                "  모서리 %.0fmm"
                % (lat * 1000, r["forward"] * 1000,
                   math.degrees(r["yaw"]), corner * 1000))
        # ⚠ 못박은 기준으로만 재면 그 기준이 틀렸을 때 그대로 통과한다.
        #   2026-09-22: 기준 +5mm 로 '모서리 12mm 완료' 였는데 실제는
        #   오른쪽 72mm 였다. 지금 라이다가 보는 렉으로도 재서 나쁜 쪽을
        #   쓴다. 둘이 크게 다르면 그 자체가 기준이 틀렸다는 뜻이다.
        if rp is not None and self._job_cart is not None:
            ql = self._job_pick(self._job_cart, self.JOB_ASSOC_ENTER, "odom")
            if ql is not None:
                rl = self._job_to_robot(rp, (ql["x"], ql["y"], ql["yaw"]))
                rl0, ll, _ = self._job_entry_geometry(rl)
                cl = self._job_leg_envelope(rl0, ll, rl["yaw"])
                if corner is None or cl > corner:
                    self.get_logger().info(
                        "진입 끝(라이다 기준): 렉 축 좌우 %+.0fmm 방향 %+.2f도"
                        "  모서리 %.0fmm"
                        % (ll * 1000,
                           math.degrees(rl["yaw"]), cl * 1000))
                    corner = cl
                    lat, r = ll, rl
        if corner is None:
            self._job_fail("최종 도킹 자세를 확인하지 못했습니다")
            return
        free = self._job_free(self.job.get("cart"))
        if corner is not None and free is not None and corner > free:
            self._job_fail(
                "다 들어갔지만 차체가 축에서 벗어났습니다 — 모서리 %.0fmm"
                " (여유 %.0fmm). 좌우 %+.0fmm 방향 %+.2f도"
                % (corner * 1000, free * 1000, lat * 1000,
                   math.degrees(r["yaw"])))
            return
        poi = self.job.get("poi")
        cart = self.job.get("cart")
        # 나오기가 이 기준을 그대로 쓴다(오돔). 지도 위치가 틀어져도 정확하다.
        self._docked = {"poi": poi, "cart": cart, "c": self._job_cart,
                        "t": time.time()}
        self._job_reset("done", "대차 중앙까지 후진 진입했습니다")
        self.job["poi"], self.job["cart"] = poi, cart
        self.get_logger().info("작업 완료: 진입")

    def _job_tick(self):
        dt = 1.0 / self.JOB_HZ
        self._job_t += dt
        if self.estop:
            self._job_fail("비상정지가 걸렸습니다")
            return
        if self._scan_dead:
            self._job_fail("라이다가 끊겼습니다")
            return
        # 수동으로 바꾸면 조이스틱과 지령을 다투지 않게 작업을 멈춘다.
        if self.control_mode != "auto":
            self._job_fail("수동 모드로 바뀌어 멈췄습니다")
            return
        st = self.job["state"]
        if st == "moving":
            self._job_moving()
        elif st == "looking":
            self._job_looking()
        elif st == "staging":
            self._job_staging()
        elif st == "entering":
            self._job_entering()
        elif st == "preflip":
            self._job_preflip()
        elif st == "exiting":
            self._job_exiting()
        elif st == "exit_fix":
            self._job_exit_fix()

    def _cart_spec(self, cart):
        """carts.yaml 에 적힌 이 대차의 설정. 없으면 빈 칸."""
        now = time.time()
        if now - getattr(self, "_spec_t", 0.0) > 10.0:
            self._spec_t = now
            try:
                self._spec = {c["name"]: c
                              for c in self.cart_specs_json().get("carts", [])}
            except Exception:                                # noqa: BLE001
                self._spec = {}
        return getattr(self, "_spec", {}).get(cart) or {}

    def _job_cam_need(self, cart):
        """이 대차에 요구할 카메라 확인 다리 수. carts.yaml 의 cam_legs."""
        try:
            return int(self._cart_spec(cart).get("cam_legs",
                                                 self.JOB_CAM_LEGS))
        except (TypeError, ValueError):
            return self.JOB_CAM_LEGS

    def cart_standoff(self, cart):
        """이 대차를 알아볼 수 있는 거리. carts.yaml 의 standoff.

        ⚠ 렉마다 다르다. 크롬 와이어 선반은 1.73m 에서 100%, 1.83m 에서
          0% 로 끊긴다(2026-09-21 실측). 기본값 1.8m 로 POI 를 찍으면
          '너무 멀리서 보다가' 못 찾는다. 그래서 POI 를 새로 찍을 때
          이 값을 기본으로 가져다 쓴다.
        """
        try:
            v = float(self._cart_spec(cart).get("standoff",
                                                self.POI_STANDOFF))
        except (TypeError, ValueError):
            return self.POI_STANDOFF
        return v if math.isfinite(v) and 0.5 <= v <= 3.0 else self.POI_STANDOFF

    def _job_lat_max(self, cart, distance=None):
        """이 렉에 들어갈 때 허용할 좌우 어긋남 [m].

        입구 전 남은 거리에서 가능한 S자 보정의 근사값을 쓴다.
        distance가 있으면 캐스터 정렬 뒤 실제 남은 조향 거리다.
        다리 사이 여유와는 별개이며 외곽 검사는 매 주기 수행한다.
        """
        free = self._job_free(cart)
        if free is None:
            return self.JOB_LAT_MAX
        # ⚠ '다리까지 여유의 절반' 으로 잡으면 안 된다. 그것은 다리 사이에
        #   들어갈 때 지켜야 할 값이지 출발할 때 지켜야 할 값이 아니다.
        #   출발할 때 틀어져 있어도 다리에 닿기 전에 지우면 된다. 그러니
        #   문턱은 '진입 조향이 실제로 지울 수 있는 양' 이어야 한다.
        #
        #   조향이 켜진 뒤 다리 사이로 들어가기 전까지 달리는 거리 L 동안,
        #   최소 반경 R 로 S 자를 그리면 옆으로 L^2/(4R) 만큼 옮긴다.
        #   (다리 사이에 들어간 뒤 지켜야 할 값은 모서리 사각지대가 맡는다.)
        so = float((getattr(self, "_job_poi", None) or {})
                   .get("standoff") or self.POI_STANDOFF)
        if distance is None:
            distance = so - self.JOB_PREALIGN_M - self.JOB_IN_M
        L = max(0.0, distance)
        gn = self._job_gains()
        cap = self._job_w_limit(gn["v"], gn["w_far"])
        fix = L * L * cap / (4.0 * gn["v"])
        # 계산값을 다 쓰지 않는다. 다 쓰면 조금만 모자라도 다리에 닿는다.
        return min(self.JOB_LAT_MAX, fix * self.JOB_LAT_SAFE)

    def _job_free(self, cart):
        """렉 축 중심의 통과 폭에서 차체 반폭을 뺀 여유 [m].

        clearance_width는 캐스터를 포함해 실측한 축 중심의 보장 통과 폭.
        검출용 width와 표면 보정용 leg_radius를 변경하지 않는다.
        명시한 실측값이 무효면 기존 기둥 폭으로 돌아가지 않는다.
        """
        spec = self._cart_spec(cart)
        if "clearance_width" in spec:
            try:
                value = spec["clearance_width"]
                w = float(value)
            except (TypeError, ValueError):
                return None
            if isinstance(value, bool) or not math.isfinite(w) or w <= 0:
                return None
            return max(0.0, w / 2.0 - self.JOB_HALF_W - self.JOB_REAR_R_PROTRUDE / 2.0)
        try:
            w = float(spec.get("width") or 0.0)
        except (TypeError, ValueError):
            w = 0.0
        if w <= 0.0:
            return None
        return max(0.005, w / 2.0 - self.leg_radius - self.JOB_HALF_W)

    def _job_clearance_error(self, cart):
        """실측 통과 폭을 요구한 렉은 미측정·통과 불가 상태에서 시작하지 않는다."""
        if "clearance_width" not in self._cart_spec(cart):
            return None
        free = self._job_free(cart)
        if free is None:
            return ("'%s'의 캐스터 포함 통과 폭이 미측정이거나 잘못 설정돼 "
                    "도킹할 수 없습니다" % cart)
        if free <= 0.0:
            return ("'%s'의 캐스터 포함 통과 폭이 차체 폭 %.0fmm 이하라 "
                    "도킹할 수 없습니다" % (cart, self.JOB_HALF_W * 2000))
        return None

    def _job_stand_point(self, rp, cart, so=None):
        """라이다로 본 대차 기준으로 로봇이 서야 할 자리. (x, y, 방향)

        대차 축 위, 대차에서 standoff 만큼 떨어진 곳에, 대차를 바라보고 선다.

        ⚠ 검출기가 주는 대차 방향은 90도 주기로 접혀 있다(다리 넷만으로는
          앞뒤·좌우를 못 가린다). 네 갈래 중 하나를 골라야 한다.

        ⚠ 로봇의 머리 방향으로 고르면 안 된다. 접근하는 동안 로봇은 비스듬히
          설 수 있고, 45도를 넘어가는 순간 옆 축을 골라 목표가 90도 튄다 —
          엉뚱한 데로 간다(2026-09-21: "또 이상한데로 감"). POI 가 진입
          방향을 이미 담고 있으므로 그것으로 고른다. POI 는 사람이 렉 중심에
          로봇을 세워 찍은 것이라 어느 쪽이 입구인지가 분명하다.
        """
        poi = getattr(self, "_job_poi", None)
        if so is None:
            so = float((poi or {}).get("standoff") or self.POI_STANDOFF)
        if poi is not None:
            # POI 가 바라보는 쪽(= 렉에서 나오는 쪽)에 가장 가까운 축으로 푼다
            out = cart[2]
            best = None
            for k in range(4):
                c = cart[2] + k * math.pi / 2
                d = abs(wrap_pi(c - poi["yaw"]))
                if best is None or d < best:
                    best, out = d, c
        else:
            # POI 가 없으면 어쩔 수 없이 로봇 기준으로 푼다
            r = self._job_to_robot(rp, cart)
            out = rp[2] + r["yaw"] + math.pi
        return (cart[0] + so * math.cos(out),
                cart[1] + so * math.sin(out),
                wrap_pi(out + math.pi))

    def _job_axis_off(self, rp, cart):
        """설 자리 기준 어긋남. (앞뒤, 좌우, 방향) — 모두 렉 축 기준이다.

        ⚠ 로봇 정면 기준(_job_to_robot 의 lateral)으로 재면 안 된다. 그것은
          '로봇이 보는 방향에서 렉이 얼마나 옆에 있나' 이지 '로봇이 렉 축에서
          얼마나 벗어나 있나' 가 아니다. 로봇이 3.8도 틀어져 있으면 1.4m
          거리에서 94mm 가 벌어진다 — 실제로 같은 순간을 한쪽은 65mm,
          한쪽은 159mm 로 읽고 있었다(2026-09-21). 진입은 축을 따라 하므로
          축 기준이 맞다.

            앞뒤  + 면 아직 더 가야 한다
            좌우  + 면 축의 왼쪽으로 벗어나 있다
        """
        sp = self._job_stand_point(rp, cart)
        dx, dy = rp[0] - sp[0], rp[1] - sp[1]
        c, sn = math.cos(sp[2]), math.sin(sp[2])
        return (-(dx * c + dy * sn),
                -dx * sn + dy * c,
                wrap_pi(sp[2] - rp[2]), sp)

    def _job_stand_ok(self, sp):
        """POI 로 계산한 자리에서 너무 멀면 믿지 않는다."""
        poi = getattr(self, "_job_poi", None)
        if poi is None:
            return True
        so = float(poi.get("standoff") or self.POI_STANDOFF)
        px = poi["x"] + so * math.cos(poi["yaw"])
        py = poi["y"] + so * math.sin(poi["yaw"])
        d = math.hypot(sp[0] - px, sp[1] - py)
        if d > self.JOB_STAND_MAX:
            self.get_logger().warn(
                "설 자리가 POI 기준에서 %.0fmm 나 떨어져 있어 쓰지 않는다"
                " (한계 %.0fmm)" % (d * 1000, self.JOB_STAND_MAX * 1000))
            return False
        return True

    def _job_aim(self):
        """가는 동안 라이다로 본 대차에 맞춰 목표를 다시 건다.

        건너뛸 때마다 그 이유를 _job_aim_why 에 남긴다(접근 추적 로그용).
        """
        self._job_aim_why = None
        if getattr(self, "_job_aim_n", 0) >= self.JOB_AIM_MAX:
            self._job_aim_why = "횟수 상한"
            return
        if self._job_seen < self.JOB_LOOK_N:    # 확실히 본 것만 믿는다
            self._job_aim_why = "인식 %d/%d" % (self._job_seen, self.JOB_LOOK_N)
            return
        q = getattr(self, "_job_last", None)
        rp = self._job_pose()
        if q is None or rp is None or self._job_goal is None:
            self._job_aim_why = "자세·목표 없음"
            return
        sp = self._job_stand_point(rp, (q["x"], q["y"], q["yaw"]))
        if not self._job_stand_ok(sp):
            self._job_aim_why = "POI 에서 너무 멂"
            return
        # 코앞에서는 건드리지 않는다. 짧은 경로에 목표를 다시 걸면 RPP 가
        # 앞점을 못 잡고 '돌고 조금 가고' 를 되풀이한다(2026-09-16).
        near = math.hypot(sp[0] - rp[0], sp[1] - rp[1])
        if near < self.JOB_AIM_KEEP:
            self._job_aim_why = "코앞 %.0fmm" % (near * 1000)
            return
        off = math.hypot(sp[0] - self._job_goal[0], sp[1] - self._job_goal[1])
        if off < self.JOB_AIM_M:
            self._job_aim_why = "차이 %.0fmm 작음" % (off * 1000)
            return
        ok, _ = self.nav_goto(sp[0], sp[1], sp[2],
                              yaw_ok=self.JOB_MID_YAW, turn_max=1)
        if not ok:
            self._job_aim_why = "nav2 거부"
            return
        self._job_aim_n = getattr(self, "_job_aim_n", 0) + 1
        self._job_goal = sp
        self.get_logger().info(
            "라이다 박스 기준으로 설 자리를 %.0fmm 옮김 — 남은 %.0fmm"
            " (%d/%d회)"
            % (off * 1000, math.hypot(sp[0] - rp[0], sp[1] - rp[1]) * 1000,
               self._job_aim_n, self.JOB_AIM_MAX))

    def _job_look_tick(self, cam=True):
        """대차가 보이는지 한 번 본다. 조건을 채우면 True.

        cam=False 면 라이다만 본다. 가는 동안에는 아직 멀어서 카메라에
        다리가 다 안 들어온다 — 라이다로 먼저 잡아 그 자리로 가고,
        카메라 확인은 도착해서 한다(2026-09-21 지시).

        ⚠ 가는 동안에도 부른다. 로봇은 접근하는 내내 렉을 마주보고 있으므로
          도착해서 따로 볼 이유가 없다. 도착 뒤에 보려면 그 자리에서 자세를
          맞춰야 하고, 그 제자리 회전이 눈에 거슬렸다(2026-09-16).
        """
        poi = getattr(self, "_job_poi", None)
        if poi is None:
            return False
        hits = getattr(self, "_job_hits", None)
        if hits is None:
            hits = self._job_hits = []
        with self.lock:
            t = self.carts_t
        q = self._job_pick((poi["x"], poi["y"]), self.JOB_FIND_M)
        if t <= getattr(self, "_job_seen_t", 0.0):
            return q is not None and sum(hits) >= self.JOB_LOOK_N
        self._job_seen_t = t
        need = (self._job_cam_need(q["cart"]) if (q and cam) else 0)
        ok = not (q is None or (need > 0 and (q["cam_ok"] is False
                                              or q["cam_n"] < need)))
        # ⚠ 떨어진 이유를 남긴다. 예전에는 실패 문구가 _job_last(이동 중에
        #   봐 둔 것)만 보고 늘 카메라를 탓했다. cam_legs 0 인 렉은 카메라가
        #   판정에 아예 끼지 않으므로 '1개만 확인했습니다 (0개 필요)' 처럼
        #   앞뒤가 안 맞는 문구가 나왔다(2026-09-22). 실제 원인은 라이다가
        #   POI 자리 안에서 못 찾은 것이었다.
        self._job_why = None if ok else ("lidar" if q is None else "cam")
        hits.append(1 if ok else 0)
        if len(hits) > self.JOB_LOOK_W:
            del hits[0]
        if q is not None:
            self._job_last = q          # 마지막으로 본 것은 남겨 둔다
        self._job_seen = sum(hits)
        return ok and self._job_seen >= self.JOB_LOOK_N

    def _job_live_off(self, rp, q):
        """못박아 둔 기준과 '지금 라이다가 보는 렉' 의 차이.

        진입 중 좌우는 _job_cart(시정수 8.3초로 무겁게 건 기준)를 상대로
        재므로, 그 기준 자체가 실제와 벌어져 있으면 로그는 0 에 맞았다고
        나오면서 몸은 치우쳐 들어간다. 둘을 나란히 찍어 두면 다음에
        '제어가 못 따라간 것' 인지 '기준이 틀린 것' 인지 한눈에 갈린다
        (2026-09-22: 로그는 -13mm 인데 실제로는 오른쪽으로 치우쳐 닿았다).
        """
        if q is None:
            return ""
        try:
            rl = -self._job_to_robot(rp, (q["x"], q["y"], q["yaw"]))["lateral"]
            return "  | 생 %+.0f (차 %+.0f)" % (
                rl * 1000, (rl - (-self._job_to_robot(
                    rp, self._job_cart)["lateral"])) * 1000)
        except Exception:                                    # noqa: BLE001
            return ""

    def _job_moving(self):
        # 가는 동안 라이다로만 본다. 카메라 확인은 도착해서 한다.
        self._job_look_tick(cam=False)
        # 멀리 있으면 nav2 목표를 본 자리로 다시 건다.
        self._job_aim()
        self._job_approach_trace()
        if self.nav_state == "done":
            self._job_find_xy = None      # 새 자리에 섰다. 다가가기 거리 새로
            self.job.update(state="looking", message="대차를 찾는 중",
                            step="")
            self._job_t = 0.0
        elif self.nav_state in ("failed", "canceled"):
            self._job_fail("POI 로 가지 못했습니다 (%s)" % self.nav_msg)

    def _job_approach_trace(self):
        """다가가는 동안 무엇을 봤고 왜 목표를 안 옮겼는지 0.5초마다 남긴다.

        ⚠ 2026-09-28: 라이다가 2.3m 에서 렉을 잡았는데도 목표를 한 번도
          옮기지 않아 POI 기준(렉과 74mm 어긋남)으로 섰다. 먼 거리에서
          검출이 끊기는지, POI 반경 밖으로 잡히는지, 조건에 막히는지를
          가리려고 둔다. 반경과 무관하게 같은 이름의 대차를 전부 찍는다.
        """
        now = time.time()
        if now - getattr(self, "_job_mv_trace_t", 0.0) < 0.5:
            return
        self._job_mv_trace_t = now
        poi = getattr(self, "_job_poi", None)
        rp = self._job_pose()
        if poi is None or rp is None:
            return
        d_poi = math.hypot(poi["x"] - rp[0], poi["y"] - rp[1])
        goal = self._job_goal
        d_goal = (math.hypot(goal[0] - rp[0], goal[1] - rp[1])
                  if goal is not None else float("nan"))
        q = self._job_pick_near((poi["x"], poi["y"]), 99.0,
                                want=self.job.get("cart"))
        if q is None:
            seen = "안 보임"
        else:
            bearing = math.degrees(wrap_pi(
                math.atan2(q["y"] - rp[1], q["x"] - rp[0]) - rp[2]))
            seen = ("보임 POI차 %.0fmm 거리 %.0fmm 방위 %+.0f도 점수 %.2f"
                    " 다리 %d held=%s" % (
                        math.hypot(q["x"] - poi["x"], q["y"] - poi["y"]) * 1000,
                        math.hypot(q["x"] - rp[0], q["y"] - rp[1]) * 1000,
                        bearing, q["score"], len(q.get("legs") or []),
                        q.get("held", False)))
        self.get_logger().info(
            "  접근 렉까지 %.0fmm 목표까지 %.0fmm | %s | 인식 %d/%d | 다시걸기 %s"
            % (d_poi * 1000, d_goal * 1000, seen,
               getattr(self, "_job_seen", 0), self.JOB_LOOK_W,
               getattr(self, "_job_aim_why", None) or "함"))

    def _job_creep_find(self):
        """렉 쪽으로 곧장 조금 기어간다. 더 갈 수 없으면 False."""
        poi = getattr(self, "_job_poi", None)
        rp = self._job_pose()
        if poi is None or rp is None:
            return False
        start = getattr(self, "_job_find_xy", None)
        if start is None:
            start = self._job_find_xy = (rp[0], rp[1])
        gone = math.hypot(rp[0] - start[0], rp[1] - start[1])
        self._job_find_gone = gone
        d = math.hypot(poi["x"] - rp[0], poi["y"] - rp[1])
        bearing = wrap_pi(math.atan2(poi["y"] - rp[1], poi["x"] - rp[0])
                          - rp[2])
        if (gone >= self.JOB_FIND_CREEP_M or d <= self.JOB_FIND_NEAR_M
                or abs(bearing) > self.JOB_FIND_YAW
                or self._exit_front_blocked()):
            return False
        if not getattr(self, "_job_creeping", False):
            self._job_creeping = True
            self.get_logger().info(
                "렉이 안 보인다 — 렉 쪽으로 다가가며 찾는다 (렉까지 %.0fmm)"
                % (d * 1000))
        self.job["step"] = "렉을 찾으며 다가가는 중 %.0fmm" % (gone * 1000)
        t = Twist()
        t.linear.x = self.JOB_FIND_V
        self.pub_twist.publish(t)
        return True

    def _job_looking(self):
        """대차 확인. 가는 동안 이미 봤으면 그대로 넘어간다.

        ⚠ 여기서 자리를 고치려고 움직이지 않는다. 기어가서 찾기, 자리 맞춤,
          nav2 다시 걸기를 넣어 봤는데 전부 왔다갔다로 보였다
          (2026-09-21 지시: "왓다갓다하는거 다 빼고 원래처럼 중심에서 바로
          들어가는거"). 보고, 정면으로 한 번 돌고, 180도 돌아 바로 들어간다.
          남은 어긋남은 진입 조향이 지운다 — 반경 0.67m 로 54mm 까지 된다.
        """
        if self._job_base_limits() is None:
            self._job_fail("모터 주행 제한을 확인하지 못해 도킹을 중단합니다")
            return
        if not self._job_look_tick():
            if self._job_t > self.JOB_FIND_WAIT_S and self._job_creep_find():
                return
            if getattr(self, "_job_creeping", False):
                self._pub_zero()
                self._job_creeping = False
            if self._job_t > self.JOB_LOOK_S:
                q = getattr(self, "_job_last", None)
                if q is not None and getattr(self, "_job_why", None) == "cam":
                    self._job_fail(
                        "대차는 보이는데 카메라가 다리를 %d개만 확인했습니다"
                        " (%d개 필요)" % (q["cam_n"],
                                        self._job_cam_need(q["cart"])))
                else:
                    self._job_fail("라이다가 대차를 찾지 못했습니다"
                                   " (POI 자리 %.0fcm 안에서 찾습니다)"
                                   % (self.JOB_FIND_M * 100))
            return
        if getattr(self, "_job_creeping", False):
            self._pub_zero()
            self._job_creeping = False
            self.get_logger().info("다가가며 찾음: %.0fmm 다가가서 보였다" % (
                getattr(self, "_job_find_gone", 0.0) * 1000))
        q = self._job_last
        clearance_error = self._job_clearance_error(q["cart"])
        if clearance_error:
            self._job_fail(clearance_error)
            return
        rp = self._job_pose()
        if rp is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        self._job_cart = (q["x"], q["y"], q["yaw"])
        r = self._job_to_robot(rp, self._job_cart)
        self._job_entry_yaw = rp[2] + r["yaw"]
        # 렉을 정면으로 보고 한 번 다시 잰다. 정면에서 봐야 대차 자세가
        # 정확히 잡힌다. 회전은 각가속도를 걸어 한 번에 끝낸다.
        if (not getattr(self, "_job_faced", False)
                and abs(wrap_pi(self._job_entry_yaw - rp[2]))
                > self.JOB_FACE_TOL):
            self.get_logger().info(
                "렉을 정면으로 보려고 %.1f도 돈다"
                % math.degrees(wrap_pi(self._job_entry_yaw - rp[2])))
            self._job_spin_start(self._job_entry_yaw, q["cart"],
                                 nxt="face", msg="렉을 정면으로 보는 중")
            return
        self._job_stand = float((getattr(self, "_job_poi", None) or {})
                                .get("standoff") or self.POI_STANDOFF)
        fwd, lat, yerr, _sp = self._job_axis_off(rp, self._job_cart)
        lat_max = self._job_lat_max(q["cart"])
        self.get_logger().info(
            "대차 확인: %s 점수 %.2f — 렉 축 기준 앞뒤 %+.0fmm 좌우 %+.0fmm"
            " 방향 %+.1f도 (좌우 한계 %.0fmm)"
            % (q["cart"], q["score"], fwd * 1000, lat * 1000,
               math.degrees(yerr), lat_max * 1000))
        tries = getattr(self, "_job_restand", 0)
        if abs(lat) > min(lat_max, self.JOB_RESTAND_LAT) and (
                abs(lat) > lat_max or tries < self.JOB_RESTAND_MAX):
            # ⚠ 예전에는 여기서 그냥 실패했다. 그런데 이 값이 커지는 원인은
            #   대부분 '렉이 POI 에서 벗어난 것' 이다 — nav2 목표는 POI 로
            #   잡는데 로봇이 서야 할 자리는 실제 렉 축 위이기 때문이다.
            #
            #   가는 동안 고치라고 _job_aim 을 뒀지만 거의 안 걸린다.
            #   JOB_AIM_KEEP(250mm) 안에서는 목표를 안 건드리는데, 이 렉은
            #   1.5m 안쪽에서만 제대로 보이고 대기 거리가 바로 1.5m 라
            #   '보이기 시작하는 시점 = 도착' 이다. 고칠 거리가 없다.
            #   (그 방어의 주석에 적힌 '2.5~3.0m 에서도 보인다' 는 전제는
            #    2026-09-21 에 이미 철회됐다 — 프레임 수를 인식률로 오독.)
            #
            #   도착해서 재는 이 값은 정확하다. 그러니 여기서 한 번 고쳐
            #   다시 간다. 찾아 헤매는 것이 아니라 '실제 렉 축 위로' 한 번
            #   옮기는 것이라 왔다갔다로 보이지 않는다(2026-09-22 지적:
            #   "POI 에 렉이 정확히 있으면 잘 들어가는데 벗어나 있으면
            #   많이 부딪힌다").
            tries = getattr(self, "_job_restand", 0)
            if tries < self.JOB_RESTAND_MAX:
                sp = self._job_stand_point(rp, self._job_cart)
                if self._job_stand_ok(sp):
                    ok, _ = self.nav_goto(sp[0], sp[1], sp[2],
                                          yaw_ok=self.JOB_MID_YAW, turn_max=1)
                    if ok:
                        self._job_restand = tries + 1
                        self._job_goal = sp
                        self._job_faced = False
                        self._job_hits = []
                        self._job_seen = 0
                        self._job_t = 0.0
                        self.get_logger().info(
                            "렉이 POI 에서 벗어나 있다 — 실제 축 위로 설 자리를"
                            " 옮긴다 (좌우 %+.0fmm, %d/%d회)"
                            % (lat * 1000, self._job_restand,
                               self.JOB_RESTAND_MAX))
                        self.job.update(state="moving",
                                        message="렉 축에 맞춰 다시 서는 중",
                                        step="")
                        return
            if abs(lat) <= lat_max:
                # 좁혀 보려던 것뿐이다. 한계 안이면 그대로 들어간다.
                self._job_spin_start(self._job_entry_yaw + math.pi, q["cart"])
                return
            self._job_fail(
                "렉 축에서 좌우로 %.0fmm 벗어나 있습니다 (진입 조향이 지울 수"
                " 있는 한계 %.0fmm). 설 자리를 %d회 고쳐도 안 됩니다 —"
                " POI 자리를 다시 잡아 주세요"
                % (lat * 1000, lat_max * 1000,
                   getattr(self, "_job_restand", 0)))
            return
        # 곧장 돌아선다. 도착 자세를 따로 맞추지 않는다 — 어차피 돌 것이다.
        self._job_spin_start(self._job_entry_yaw + math.pi, q["cart"])

    def _job_spin_start(self, target, cart=None, nxt="enter", msg="돌아서는 중"):
        """제자리 회전을 건다. 한 번에 이어서 돌고, 끝나면 nxt 로 넘어간다."""
        self.job.update(state="staging", message=msg)
        if cart:
            self.job["cart"] = cart
        self._job_spin_next = nxt
        self._job_spin_target = target
        self._job_w = 0.0
        self._job_t = 0.0
        self._job_settle = 0.0

    def _job_staging(self):
        """제자리에서 한 번에 부드럽게 돌아선다.

        ⚠ 예전에는 nav2 에 '물러난 자리에 등 돌리고 서라' 고 시켰다. 회전에서
          밀린 것을 주행으로 잡는 방법이었는데, 앞으로 나갔다 오는 동작이
          군더더기로 보이고 멈춤이 두 번 더 늘었다(2026-09-16 지적).
          지금은 그냥 돌고, 밀린 것은 진입하면서 지운다 — 진입 조향은 상한이
          낮아(회전 반경 4m) 눈에는 직선인데도 100mm 쯤은 지운다.

        ⚠ 돌고-멈추고-재고를 되풀이하지 않는다. 그 멈춤이 '비틀비틀' 로
          보였다. 각가속도를 걸어 매끄럽게 올렸다 내리고, 한 번에 끝낸다.
        """
        rp = self._job_pose()
        if rp is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        dt = 1.0 / self.JOB_HZ
        err = wrap_pi(self._job_spin_target - rp[2])

        if self._job_settle > 0.0:
            # 다 돌았다. 잠깐 가라앉힌 뒤 진입으로 넘어간다.
            self._job_settle -= dt
            self._pub_zero()
            if self._job_settle > 0.0:
                return
            nxt = getattr(self, "_job_spin_next", "enter")
            if nxt == "face":
                # 정면으로 섰다. 그 자리에서 다시 재고 이어 간다.
                self._job_faced = True
                self.job.update(state="looking", message="대차를 다시 보는 중")
                self._job_t = 0.0
                self._job_seen = 0
                self._job_hits = []
                self._job_seen_t = 0.0
                return
            if nxt == "approach":
                # 목표 쪽을 보고 섰다. 이제 nav2 에 넘긴다.
                ok, msg = self.nav_goto(*self._job_goal,
                                        yaw_ok=self.JOB_MID_YAW, turn_max=1)
                if not ok:
                    self._job_fail("이동을 걸지 못했습니다: %s" % msg)
                    return
                self.job.update(state="moving", message="렉 앞으로 가는 중")
                self._job_t = 0.0
                return
            q = self._job_pick(self._job_cart, self.JOB_ASSOC_M)
            if q is not None:
                self._job_cart = (q["x"], q["y"], q["yaw"])
            r = self._job_to_robot(rp, self._job_cart)
            rpo = self._job_pose("odom")
            if rpo is None:
                self._job_fail("오돔 자세를 못 읽었습니다")
                return
            self._job_cart = self._job_to_map(rpo, r)   # 이제부터 오돔 기준
            # ⚠ 들어가는 동안 대차는 움직이지 않는다. 처음 잡은 이 자리에서
            #   벗어나지 못하게 묶어 둔다. '한 번에 받아들일 거리' 만 좁혀서는
            #   기준이 걸어가는 것을 못 막는다 — 120mm 씩 옮겨가면 다음
            #   판정은 옮겨간 자리를 중심으로 또 120mm 를 허용해서, 1초면
            #   390mm 를 걸어간다(2026-09-16: 진입 0.95초 만에 -34 -> 358mm).
            self._job_cart0 = self._job_cart
            self.get_logger().info(
                "돌아섬: 좌우 %.0fmm 방향 %.1f도 (지령 끊은 뒤 %.1f도 더 돔)"
                % (-r["lateral"] * 1000, math.degrees(err),
                   math.degrees(getattr(self, "_job_spin_cut", 0.0) - err)))
            self._job_bad = 0
            self.job.update(state="entering", message="후진 진입 중")
            self._job_t = 0.0
            self._job_start_xy = (rpo[0], rpo[1])
            self._job_start_yaw = None
            self._job_budget = -r["forward"] + self.JOB_ENTER_EXTRA
            self.get_logger().info("후진 진입 시작: 남은 %.0fmm 좌우 %.0fmm "
                                   "속도 %.2f m/s"
                                   % (-r["forward"] * 1000,
                                      -r["lateral"] * 1000,
                                      self._job_gains()["v"]))
            return

        if self._job_t > 30.0:
            self._job_fail("돌아서지 못했습니다")
            return

        # 남은 각에서 감속 곡선을 만든다. 끝에서 매끄럽게 0 이 된다.
        a = self.JOB_SPIN_ACC
        w_want = math.copysign(
            min(self.JOB_SPIN_W, math.sqrt(2.0 * a * max(0.0, abs(err)))), err)
        # 각가속도 제한 — 톡 튀지 않게 올리고 내린다
        dw = max(-a * dt, min(a * dt, w_want - self._job_w))
        self._job_w += dw
        # ⚠ 끝나는 조건에 속도를 같이 걸면 안 된다. 감속 곡선이
        #   w = sqrt(2a|err|) 라 err 가 0.03 rad 일 때도 0.17 rad/s 이고,
        #   0.05 아래로 내려가려면 0.0025 rad(0.14도)까지 가야 한다.
        #   그래서 영영 안 끝나고 시간 초과로 실패했다(2026-09-16).
        #   지령을 끊어도 더 도는 만큼 미리 끊는다 — 진입과 같은 방식이다.
        if abs(err) <= max(self.JOB_SPIN_TOL,
                           abs(self._job_w) * self.JOB_SPIN_LAG):
            self._pub_zero()
            self._job_spin_cut = err      # 지령을 끊은 시점의 남은 각
            self.get_logger().info(
                "  회전 지령 끊음: 남은 %.1f도, 그때 속도 %.2f rad/s"
                % (math.degrees(err), self._job_w))
            self._job_w = 0.0
            self._job_settle = self.JOB_SPIN_SETTLE
            return
        t = Twist()
        t.angular.z = float(self._job_w)
        self.pub_twist.publish(t)
        self.job["step"] = "남은 %.0f도" % math.degrees(err)
        # 회전 흐름을 남긴다. 지나쳤다 돌아오는지, 옆으로만 밀리는지 가른다.
        if self._job_t - getattr(self, "_job_spin_trace", -9.0) >= 0.3:
            self._job_spin_trace = self._job_t
            rr = (self._job_to_robot(rp, self._job_cart)
                  if self._job_cart else None)
            self.get_logger().info(
                "  회전 %4.1fs 남은 %+6.1f도 지령 %+5.2f rad/s%s"
                % (self._job_t, math.degrees(err), self._job_w,
                   ("  좌우 %+.0fmm" % (-rr["lateral"] * 1000)) if rr else ""))

    def _job_xy(self):
        q = self.tf_pose()
        return (q["x"], q["y"]) if q else None

    def _job_pose(self, frame="map"):
        q = self.tf_pose(frame, "base_footprint")
        return (q["x"], q["y"], q["yaw"]) if q else None

    @staticmethod
    def _job_to_map(rp, al):
        """로봇 기준 정렬값을 지도 좌표의 대차 자세로 바꾼다."""
        rx, ry, ryaw = rp
        c, sn = math.cos(ryaw), math.sin(ryaw)
        return (rx + al["forward"] * c - al["lateral"] * sn,
                ry + al["forward"] * sn + al["lateral"] * c,
                ryaw + al["yaw"])

    def _job_carts_map(self, frame="map"):
        """지금 보이는 대차를 전부 지도 좌표로. 카메라 판정은 보지 않는다.

        ⚠ 진입 중에는 검출기가 고른 목표(target)를 쓰면 안 된다. 검출기는
          카메라 검증에 실패한 대차를 목표에서 빼는데, 들어가기 시작하면
          카메라가 대차 밑을 보게 되어 진짜 대차가 바로 그렇게 떨어진다.
          그 자리를 엉뚱한 후보가 차지해 '좌우로 1976mm' 같은 값이 나왔다
          (2026-09-16). 들어갈 때는 라이다만 믿는다.
        """
        with self.lock:
            d, t = self.carts, self.carts_t
        if not d or (time.time() - t) > self.JOB_FRESH_S:
            return []
        # 검출은 스캔을 찍은 순간의 laser 좌표다. 최신 TF로 매 주기
        # 변환하면 정지한 렉이 로봇 회전을 따라 움직이는 것으로 보인다.
        # 시각이 없는 구버전 검출은 도킹에 쓰지 않는다.
        stamp = d.get("stamp")
        if not isinstance(stamp, dict):
            return []
        sec, nsec = stamp.get("sec"), stamp.get("nanosec")
        if (type(sec) is not int or type(nsec) is not int
                or sec < 0 or not 0 <= nsec < 1000000000):
            return []
        stamp_ns = sec * 1000000000 + nsec
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if stamp_ns <= 0 or not 0.0 <= age <= self.JOB_FRESH_S:
            return []
        source = d.get("frame") or "laser"
        key = (stamp_ns, source)
        cached = self._job_cart_tf_cache.get(frame)
        if cached is not None and cached[0] == key:
            laser = cached[1]
        else:
            laser = self.tf_pose_at(TimeMsg(sec=sec, nanosec=nsec),
                                    target=frame, source=source)
            if laser is None:
                return []
            self._job_cart_tf_cache[frame] = (key, laser)
        lx, ly, lyaw = laser["x"], laser["y"], laser["yaw"]
        lc, ls = math.cos(lyaw), math.sin(lyaw)
        out = []
        for q in d.get("carts", []):
            try:
                out.append({
                    "x": lx + q["x"] * lc - q["y"] * ls,
                    "y": ly + q["x"] * ls + q["y"] * lc,
                    "yaw": q["yaw"] + lyaw,
                    "cart": q.get("cart"),
                    "score": float(q.get("score") or 0.0),
                    "legs": [[lx + a * lc - b * ls, ly + a * ls + b * lc]
                             for a, b in (q.get("legs") or [])],
                    "cam_ok": q.get("cam_ok"),
                    # 규격과의 오차와 '점수 미달인데 직전 자리로 살린 것'
                    # 인지. 진입 중 이 검출을 얼마나 믿을지 정하는 데 쓴다.
                    "err_mm": float(q.get("err_mm") or 0.0),
                    "held": bool(q.get("held")),
                    # 2026-11-13 다리 셋으로 잡고 넷째를 계산한 것. 진입 중에는 천천히 섞는다.
                    "legs3": bool(q.get("legs3")),
                    "stamp_ns": stamp_ns,
                    # 카메라가 기둥으로 확인해 준 다리 수. "3/3" 꼴로 온다.
                    "cam_n": self._job_cam_n(q.get("cam_legs"))})
            except (KeyError, TypeError):
                continue
        return out

    def _job_pick(self, near, radius, frame="map"):
        """지도 좌표 near 에서 radius 안에 있는 대차 중 가장 가까운 것.

        ⚠ 검출기가 고른 목표(target)를 쓰지 않는 이유가 있다. 방 안의 다른
          물건 넷이 우연히 등록 규격의 사각형을 이루면 유령 대차가 잡히는데,
          검출기는 '카메라로 확인된 것' 을 먼저 고른다. 진짜 대차가 가까워져
          카메라 검증에 실패하면(대차 밑을 보게 된다) 유령이 목표가 된다.
          실제로 진짜가 (4.02, 4.06) 에 점수 0.94 로 있는데도 2m 옆의 유령이
          목표로 뽑혔다(2026-09-16). 우리는 렉이 어디 있는지 POI 로 알고
          있으므로 자리로 고른다.
        """
        best = None
        want = self.job.get("cart")
        # ⚠ 2026-11-13 렉에 들어가는 중·안에서는 다리 셋 검출(넷째 계산)을 쓰지 않는다. 다리 사이로
        #   들어가면 가까운 다리가 가려져 계산한 넷째가 틀리고(좌우 +14 -> +26mm, 실제 +4mm), 그
        #   값이 작업 기준에 섞여 기준이 끌려가 '진입 여유 부족' 으로 섰다(반 넘게 들어간 뒤).
        #   그동안은 예전처럼 넷이 안 보이면 바퀴·IMU 로 이어 간다. 찾기·다가가기에서는 쓴다.
        no3 = self.job.get("state") in ("entering", "preflip", "exiting", "exit_fix")
        for q in self._job_carts_map(frame):
            if want and q["cart"] != want:
                continue
            if no3 and q.get("legs3"):
                continue
            off = math.hypot(q["x"] - near[0], q["y"] - near[1])
            if off > radius:
                continue
            if best is None or off < best[0]:
                best = (off, q)
        return best[1] if best else None

    def _job_pick_near(self, xy, radius, want=None, frame="map"):
        """지도 좌표 xy 근처에서 가장 가까운 대차. 카메라 판정은 보지 않는다."""
        best = None
        for q in self._job_carts_map(frame):
            if want and q["cart"] != want:
                continue
            off = math.hypot(q["x"] - xy[0], q["y"] - xy[1])
            if off > radius:
                continue
            if best is None or off < best[0]:
                best = (off, q)
        return best[1] if best else None

    @staticmethod
    def _rect_sides(legs, yaw):
        """네 다리가 이루는 사각형에서 (진입축 길이, 가로 폭) 을 낸다.

        규격표의 width/depth 중 어느 쪽이 진입축인지는 다리 넷만으로는 알 수
        없다(90도 주기). 실제로 잰 사각형의 두 변 중 로봇이 바라보는 방향에
        가까운 쪽을 진입축으로 본다.
        """
        if not legs or len(legs) < 4:
            return None
        (x0, y0), (x1, y1), (x2, y2) = legs[0], legs[1], legs[2]
        s0 = (x1 - x0, y1 - y0)
        s1 = (x2 - x1, y2 - y1)
        l0 = math.hypot(*s0)
        l1 = math.hypot(*s1)
        if l0 < 0.05 or l1 < 0.05:
            return None
        # 진입축(yaw)과 더 나란한 변이 '앞뒤' 다
        a0 = abs(math.cos(math.atan2(s0[1], s0[0]) - yaw))
        a1 = abs(math.cos(math.atan2(s1[1], s1[0]) - yaw))
        return (round(l0, 4), round(l1, 4)) if a0 >= a1 else (round(l1, 4),
                                                              round(l0, 4))

    @staticmethod
    def _job_cam_n(legs):
        """'확인된수/검사한수' 문자열에서 확인된 다리 수만 뽑는다."""
        try:
            return int(str(legs).split("/")[0])
        except (ValueError, AttributeError, IndexError):
            return 0

    @staticmethod
    def _job_fold(d):
        """대차 방향은 90도 주기다. 차이를 -45~45도로 접는다."""
        return (d + math.pi / 4) % (math.pi / 2) - math.pi / 4

    @staticmethod
    def _job_to_robot(rp, cart):
        """지도 좌표의 대차 자세를 로봇 기준 앞뒤·좌우·회전으로 바꾼다."""
        rx, ry, ryaw = rp
        cx, cy, cyaw = cart
        dx, dy = cx - rx, cy - ry
        c, sn = math.cos(ryaw), math.sin(ryaw)
        d = cyaw - ryaw
        d = (d + math.pi / 4) % (math.pi / 2) - math.pi / 4
        return {"forward": dx * c + dy * sn,
                "lateral": -dx * sn + dy * c,
                "yaw": d}

    def _job_entry_geometry(self, r):
        """후진 자세를 렉 축으로 투영한다. axis_lat는 로봇의 축 기준 좌우.

        조향 입력인 -lateral에는 거리×방향 오차가 들어 있다. 이를
        횡이동이나 차체 외곽으로 해석하면 안 된다. 반폭은 축에 나란할 때의
        값을 유지하여 모서리 여유를 보수적으로 계산한다.
        """
        fwd, lat, yaw = -r["forward"], -r["lateral"], r["yaw"]
        c, sn = math.cos(yaw), math.sin(yaw)
        # axis_lat + 는 로봇이 축의 오른쪽(도킹 자세 기준)에 있다는 뜻이다.
        # 실제 통로 중심이 라이다 축에서 왼쪽으로 center_offset 만큼 있으면
        # 그 중심을 기준으로 잰다. 조향·외곽 검사가 모두 이 값을 쓴다.
        axis_lat = fwd * sn - lat * c + self._job_center_offset()
        axis_remain = fwd * c + lat * sn
        corner = abs(axis_lat) + self.JOB_HALF_L * abs(sn)
        return axis_remain, axis_lat, corner

    # 라이다 기준 검사는 서로 다른 스캔 이만큼이 연달아 넘어야 멈춘다.
    #
    # ⚠ 다리 사이에서는 다리를 일부만 봐서 라이다 값이 흔들린다. 한 장이
    #   2mm 넘은 것으로(모서리 39 / 여유 37mm) 다리 사이에 선 채 끝났다
    #   (2026-09-28 반복 6회차, 접촉 없음). 작업 기준 검사는 그대로 즉시
    #   멈추고, 크게(JOB_EXIT_JAM_HARD) 넘으면 라이다도 즉시 멈춘다.
    #   검출은 5Hz, 이 검사는 20Hz 라 같은 스캔을 여러 번 보므로 시각으로 센다.
    JOB_LIDAR_JAM_N = 2

    def _job_entry_clear(self, r, speed, source="작업 기준", scan=None):
        """매 주기 명령 발행 전에 렉 축 기준 외곽과 정지 여유를 검사한다.

        scan 을 주면(라이다 기준) 서로 다른 스캔이 연달아 넘을 때만 멈춘다.
        """
        clearance_error = self._job_clearance_error(self.job.get("cart"))
        if clearance_error:
            self._job_fail(clearance_error)
            return False
        remain, lat, corner = self._job_entry_geometry(r)
        free = self._job_free(self.job.get("cart"))
        spec = self._cart_spec(self.job.get("cart"))
        depth = float(spec.get("depth") or 0.0)
        # 차체의 렉 축 방향 투영 + 가까운 다리가 차지하는 반폭.
        #
        # ⚠ leg_radius(20mm, 검출 보정값)를 썼었다. 캐스터 판은 기둥 중심에서
        #   32.5mm 나와 있어(2026-09-23 실측) 검사가 12.5mm 늦게 켜졌다.
        #   2026-09-28 입구 앞 정지에서 멈춘 뒤 밀려간 거리까지 치면 판 끝에
        #   거의 닿는 자리였다. 둘 중 큰 쪽을 쓴다.
        yaw = r["yaw"]
        entry = max(self.JOB_IN_M, depth / 2.0
                    + max(self.leg_radius, self.JOB_PLATE_HALF)
                    + self.JOB_HALF_L * abs(math.cos(yaw))
                    + self.JOB_HALF_W * abs(math.sin(yaw)))
        lim = (None if free is None else
               free * self.JOB_JAM_R + (self.JOB_LIDAR_NOISE if scan is not None else 0.0))
        over = (free is not None
                and remain <= entry + self._job_stop_dist(speed)
                and corner > lim)
        if scan is not None:
            if not over:
                self._lidar_jam = (0, None)
            elif corner <= free + max(self.JOB_EXIT_JAM_HARD, 2 * self.JOB_LIDAR_NOISE):
                n, last = getattr(self, "_lidar_jam", (0, None))
                if scan != last:
                    n, last = n + 1, scan
                    self._lidar_jam = (n, last)
                    self.get_logger().info(
                        "  라이다 기준 여유 초과 %d/%d — 모서리 %.0fmm / 여유 %.0fmm"
                        % (n, self.JOB_LIDAR_JAM_N, corner * 1000, free * 1000))
                if n < self.JOB_LIDAR_JAM_N:
                    return True
        if over:
            self._job_retry_or_fail(
                "진입 여유 부족 (%s) — 렉 축 좌우 %+.0fmm 방향 %+.1f도, "
                "모서리 %.0fmm / 여유 %.0fmm, 축 거리 %.0fmm"
                % (source, lat * 1000, math.degrees(yaw), corner * 1000,
                   free * 1000, remain * 1000),
                inside=remain < entry)
            return False
        return True

    def _job_steering(self, lat, yaw, remain, gn, wm, s_lat, s_yaw):
        """lat 은 렉 축 기준 좌우 오차(-axis_lat). 출력 제한에 맞춰 적분한다."""
        p = s_lat * gn["k_lat"] * lat + s_yaw * gn["k_yaw"] * yaw
        imax = self.JOB_INT_MAX * wm / max(self.JOB_K_INT, 1e-6)
        old = max(-imax, min(imax, self._job_int))
        if not self._job_steer:
            self._job_int = 0.0
            return 0.0, p, 0.0
        if remain <= self.JOB_IN_M:
            updated = old * self.JOB_INT_BLEED
        else:
            updated = max(-imax, min(imax, old + lat / self.JOB_HZ))
            candidate = p + self.JOB_K_INT * updated
            # 포화를 더 키우는 누적만 막고, 풀어 주는 방향은 허용한다.
            if ((candidate > wm and updated > old)
                    or (candidate < -wm and updated < old)):
                updated = old
        self._job_int = updated
        raw = p + self.JOB_K_INT * updated
        return max(-wm, min(wm, raw)), p, raw

    def _job_entering(self):
        """대차 밑으로 들어간다.

        기준은 지도에 못박아 둔 대차 자리다. 로봇의 지도 자세만 있으면 남은
        거리와 어긋남이 나온다. 인식이 살아 있고 그 자리 근처를 가리키면
        조금씩 보정하고, 엉뚱한 데를 가리키면 버린다.

        ⚠ 인식을 그대로 믿으면 안 된다. 들어갈수록 다리 넷을 온전히 못 봐서
          검출기가 다른 덩어리 넷으로 사각형을 만든다. 처음에는 그걸 믿고
          '좌우로 1976mm 벌어졌다' 며 멈췄다(2026-09-16).
        """
        if self._job_base_limits() is None:
            self._job_fail("모터 주행 제한을 확인하지 못해 진입을 중단합니다")
            return
        rp = self._job_pose("odom")
        if rp is None:
            self._job_fail("로봇 위치를 모릅니다")
            return
        if self._job_cart is None:
            self._job_fail("대차 자리를 잃었습니다")
            return

        # 못박아 둔 자리에서 가장 가까운 대차. 카메라 판정은 보지 않는다.
        q = self._job_pick(self._job_cart, self.JOB_ASSOC_ENTER, "odom")
        seen = q is not None
        stamp = q["stamp_ns"] if seen else None
        previous = getattr(self, "_job_cart_stamp", None)
        if seen and (previous is None or stamp > previous):
            # 조금씩만 당긴다 — 한 프레임이 튀어도 기준이 통째로 끌려가지 않게
            #
            # ⚠ 다만 '느리게' 하나로만 막으면 진짜 어긋남도 같이 막힌다.
            #   2026-09-22: 기준이 실제를 43~77mm 뒤처진 채 진입 내내
            #   따라잡지 못했고, 로봇은 그 틀린 기준에 정확히 맞춰 서다가
            #   다리에 부딪혔다. 생값은 실측과 일치함을 두 번 확인했으니,
            #   버린 쪽이 맞는 값이었다.
            #
            #   막아야 할 것은 '단발 오검출' 이지 '지속되는 차이' 가 아니다.
            #   둘은 검출 품질로 갈린다 — 엉뚱한 덩어리 넷으로 만든 사각형은
            #   규격 오차가 크고 점수가 낮다. 품질을 갖춘 검출은 1.25초
            #   시정수로 따라가고, 못 갖춘 것만 예전처럼 천천히 섞는다.
            trust = (q.get("score", 0.0) >= self.JOB_TRUST_SCORE
                     and q.get("err_mm", 999.0) <= self.JOB_TRUST_ERR
                     and not q.get("held")
                     and not q.get("legs3")
                     and len(q.get("legs") or []) == 4)
            # 같은 5Hz 검출을 20Hz로 여러 번 섞지 않는다. 새 스캔 사이
            # 경과시간으로 기존 필터 시정수를 유지한다. 장시간 단절은 제한한다.
            dt = (1.0 / self.JOB_HZ if previous is None else
                  min(self.JOB_FRESH_S, (stamp - previous) * 1e-9))
            blend = self.JOB_BLEND_GOOD if trust else self.JOB_BLEND_M
            k = min(0.5, -math.expm1(-self._job_gains()["v"] * dt / blend))
            self._job_cart_stamp = stamp
            dy = self._job_fold(q["yaw"] - self._job_cart[2])
            nx = self._job_cart[0] + k * (q["x"] - self._job_cart[0])
            ny = self._job_cart[1] + k * (q["y"] - self._job_cart[1])
            nyaw = self._job_cart[2] + k * dy
            # 처음 자리에서 이 거리 밖으로는 못 나간다
            c0 = self._job_cart0
            if c0 is not None:
                off = math.hypot(nx - c0[0], ny - c0[1])
                if off > self.JOB_ANCHOR_MAX:
                    f = self.JOB_ANCHOR_MAX / off
                    nx = c0[0] + (nx - c0[0]) * f
                    ny = c0[1] + (ny - c0[1]) * f
                dy0 = self._job_fold(nyaw - c0[2])
                if abs(dy0) > self.JOB_ANCHOR_YAW:
                    nyaw = c0[2] + math.copysign(self.JOB_ANCHOR_YAW, dy0)
            self._job_cart = (nx, ny, nyaw)
            self._job_bad = 0
        elif not seen:
            self._job_bad += 1

        r = self._job_to_robot(rp, self._job_cart)
        # 뒤로 들어간다. 로봇을 180도 돌린 좌표계에서 보면 앞으로 가는 것과
        # 같다 — 앞뒤와 좌우의 부호만 뒤집으면 된다. 회전은 그대로다
        # (대차 방향은 90도 주기라 180도 돌려도 접힌 값이 변하지 않는다).
        fwd = -r["forward"]
        lat = -r["lateral"]
        remain = fwd
        axis_remain, axis_lat, corner = self._job_entry_geometry(r)
        self.job["remain"] = round(remain, 3)
        self.job["step"] = "남은 %.0fmm 렉 축 좌우 %.0fmm%s" % (
            remain * 1000, axis_lat * 1000, "" if seen else " (인식없음)")
        # 기존 좌우는 조향 입력이다. 횡이동과 구분하도록 렉 축 값도 남긴다.
        if self._job_t - getattr(self, "_job_trace_t", -9.0) >= 0.5:
            self._job_trace_t = self._job_t
            self.get_logger().info(
                "  진입 %4.1fs 남은 %5.0f 좌우 %+5.0f 방향 %+5.1f도 %s%s"
                " | 렉 축 좌우 %+.0fmm"
                % (self._job_t, remain * 1000, lat * 1000,
                   math.degrees(r["yaw"]), "인식" if seen else "추측",
                   self._job_live_off(rp, q), axis_lat * 1000))
            if q is not None:
                live = self._job_to_robot(rp, (q["x"], q["y"], q["yaw"]))
                live_remain, live_lat, live_corner = self._job_entry_geometry(live)
                self.get_logger().info(
                    "  진입 라이다: 축 거리 %.0fmm 좌우 %+.0fmm 방향 %+.2f도"
                    " 모서리 %.0fmm 점수 %.3f held=%s 다리3=%s"
                    % (live_remain * 1000, live_lat * 1000,
                       math.degrees(live["yaw"]), live_corner * 1000,
                       q.get("score", 0.0), q.get("held", False), q.get("legs3", False)))

        gn = self._job_gains()

        # --- 안전 ---
        if abs(axis_lat) > self.JOB_BAD_LAT:
            self._job_fail("진입 중 렉 축에서 좌우로 %.0fmm 벌어져 멈췄습니다"
                           % (axis_lat * 1000))
            return
        if axis_remain < self.JOB_NEAR_M and abs(axis_lat) > self.JOB_LAT_NEAR:
            self._job_fail("다 들어간 시점에 좌우로 %.0fmm 어긋나 멈췄습니다"
                           % (axis_lat * 1000))
            return
        if self._job_start_xy is not None:
            gone = math.hypot(rp[0] - self._job_start_xy[0],
                              rp[1] - self._job_start_xy[1])
            if gone > self._job_budget:
                self._job_fail("예상 거리를 %.0fmm 넘겨 멈췄습니다"
                               % ((gone - self._job_budget) * 1000))
                return
        if self._job_t > self.JOB_ENTER_S:
            self._job_fail("진입 시간이 초과되었습니다")
            return

        # ⚠ 지령을 끊어도 더 굴러간다. 그만큼 미리 끊는다.
        #   실측 2026-09-16: 5회 모두 50mm 씩 더 깊이 들어갔다(산포 6mm).
        #   마지막 속도 0.12 m/s 에서 0.42 초어치다.
        lead = self._job_stop_dist(gn["v_slow"])
        post = self._job_post_stop()
        if remain - post <= lead:
            self.get_logger().info("진입 멈춤: 렉 중심 %.0fmm 앞 (포스트 여유로 %.0fmm 덜 들어감)"
                                   % (remain * 1000, post * 1000))
            self._job_preflip_start()
            return

        # 진입 시작점에서 얼마나 왔나. 아래 속도·무조향 판단이 모두 이 값을
        # 쓰므로 먼저 구해 둔다.
        gone_all = (math.hypot(rp[0] - self._job_start_xy[0],
                               rp[1] - self._job_start_xy[1])
                    if self._job_start_xy else 0.0)

        # 출발은 기어가듯 천천히.
        #
        # ⚠ 180도 제자리 회전 뒤에는 캐스터 넷이 비틀려 있다. 후진을 시작하면
        #   한꺼번에 풀리며 차체를 미는데, 1초 만에 124~153mm 였다. 캐스터가
        #   방향을 트는 힘은 속도에 비례하므로 천천히 출발하면 그 힘이 준다.
        #   앞으로 갔다 오는 것은 소용없다 — 캐스터는 진행 방향을 따라
        #   정렬하므로 후진을 시작하면 다시 뒤집힌다(2026-09-21).
        #   속도는 계단이 아니라 거리에 따라 매끈하게 그린다. 양 끝에서
        #   기울기가 0 인 곡선(smoothstep)이라 밟는 순간도 놓는 순간도
        #   턱이 없다.
        t = Twist()
        # 캐스터가 뒤집히는 구간은 기어가고, 가속은 그 뒤부터 센다
        if gone_all < self.JOB_PREALIGN_M:
            v_now = min(self.JOB_FLIP_V,
                        self._job_speed(0.0, remain - post, gn))
        else:
            v_now = self._job_speed(gone_all - self.JOB_PREALIGN_M, remain - post, gn)
        t.linear.x = -v_now
        if not self._job_entry_clear(r, v_now):
            return
        # 제어 기준을 부드럽게 섞더라도 정지 판단까지 늦추면 안 된다.
        # 완료 때만 생값을 비교하던 검사를 진입 중에도 수행한다.
        # 같은 종류·주변 위치·스캔 시각/TF 검사를 통과한 q만 사용하며,
        # 생값은 위험할 때 멈추는 데만 쓴다. 안전한 생값이 작업 기준의
        # 위험 판정을 덮어쓰거나 조향 기준을 즉시 옮기지는 않는다.
        # ⚠ 2026-11-13 다리 셋으로 넷째를 계산한 검출(legs3)·유지로 살린 검출(held)은 안전 판정에
        #   쓰지 않는다. 렉 입구(축 거리 0.70m)에서 작업 기준 좌우 +1mm 인데 라이다 기준이 0.2초 만에
        #   +15mm 로 튀어 두 번 연속 넘겨 멈췄다 — 다리 사이로 들어가며 가까운 다리가 가려질 때다.
        if q is not None and not q.get("legs3") and not q.get("held"):
            live = self._job_to_robot(rp, (q["x"], q["y"], q["yaw"]))
            hist = self.__dict__.get("_job_live_hist")
            if hist is None:
                hist = self._job_live_hist = deque(maxlen=self.JOB_LIDAR_MED_N)
            st_ns = q.get("stamp_ns")
            if not hist or hist[-1][0] != st_ns:
                hist.append((st_ns, live["lateral"], live["yaw"]))
            use, src = live, "라이다 기준"
            free_now = self._job_free(self.job.get("cart"))
            raw_corner = self._job_entry_geometry(live)[2]
            if len(hist) >= 3 and not (free_now is not None
                                       and raw_corner > free_now + self.JOB_EXIT_JAM_HARD):
                use = dict(live,
                           lateral=float(np.median([h[1] for h in hist])),
                           yaw=float(np.median([h[2] for h in hist])))
                src = "라이다 기준 %d장 중앙값" % len(hist)
            if not self._job_entry_clear(use, v_now, source=src, scan=st_ns):
                return
        # 대차의 진입축을 따라간다. 계산은 180도 돌린 좌표계에서 한다. 그
        # 좌표계에서 구한 회전 지령은 실제 회전과 부호가 같다 — 좌표계가
        # 차체에 붙어 같이 돌기 때문이다.
        #
        # 두 구간으로 나눈다.
        #
        #   멀리서  축에 붙이면서 방향도 같이 맞춘다 (직선 추종)
        #   가까이  좌우는 이미 맞았으므로 방향만 붙든다
        #
        # ⚠ 왜 이렇게 나누는가 — 대차 안에서는 몸을 못 돌린다. 차체 대각
        #   반경이 455mm 인데 다리 사이 반폭이 287mm 라 몇 도도 못 돈다.
        #   다 들어간 뒤에 고칠 수 없으므로 들어가기 전에 끝내야 한다.
        #
        # ⚠ 그리고 끝까지 조향을 놓으면 안 된다. 이 차체는 시키지 않아도
        #   좌우로 3.5 도/s 로 흔들린다(2026-09-14 실측, 캐스터가 미는 힘).
        #   마지막 0.25m 에서 조향을 멈췄더니 그 2.5 초 동안 몸이 밀려
        #   방향 오차가 3.2 도까지 벌어졌다. 붙들고 있어야 0.5 도로 선다.
        #
        # ⚠ 중심을 곧장 겨누는 방식(순수추종)으로는 방향이 안 맞는다. 중심에
        #   닿기는 하지만 비스듬히 선다. 비스듬하면 훑는 폭이 넓어져(870mm
        #   차체가 10도 기울면 270 -> 417mm) 다리에 닿고, 렉을 들면 그만큼
        #   틀어진 채로 들린다.
        # ⚠ 끝까지 좌우도 같이 본다. 안쪽에서 방향만 붙들게 했더니, 각도가
        #   1도만 남아도 마지막 0.75m 에서 13mm 가 밀려 14mm 가 36mm 로
        #   도로 벌어졌다(2026-09-16 계산). 안쪽에서는 상한만 낮춘다.
        # 처음 얼마간은 곧게만 간다.
        #
        # ⚠ 180도 제자리 회전 직후에는 캐스터 넷이 비틀려 있다. 움직이기
        #   시작하면 제자리를 찾느라 차체를 걷어차는데, 실측에서 첫 2초에
        #   방향이 7도까지 흔들렸다(2026-09-16). 조향 상한이 2.9도/s 라
        #   제어가 만든 것일 수 없다 — 물리적으로 밀린 것이다.
        #   그 구간에서 제어가 반응하면 떨림을 쫓아 오히려 키운다.
        #   캐스터가 자리 잡을 때까지 곧게 가고, 그 뒤에 기준을 다시 잡는다.
        if gone_all < self.JOB_PREALIGN_M:
            t = Twist()
            # 위에서 구한 속도를 그대로 쓴다. 여기서 따로 정하면 구간이
            # 바뀌는 자리에서 속도가 튄다.
            t.linear.x = -v_now
            # 출발 방향을 붙잡는다 (고치지는 않는다). 방향' = -w 이므로
            # w = K x (방향 - 출발 방향). 반경 제한 없는 길로 보낸다.
            y0 = getattr(self, "_job_start_yaw", None)
            if y0 is None:
                y0 = self._job_start_yaw = r["yaw"]
            t.angular.z = float(max(-self.JOB_START_W, min(
                self.JOB_START_W, self.JOB_START_HOLD_K * (r["yaw"] - y0))))
            self.pub_twist_free.publish(t)
            self.job["step"] = "캐스터 정렬 중 (%.0f/%.0fmm, %.2f m/s)" % (
                gone_all * 1000, self.JOB_PREALIGN_M * 1000, v_now)
            return
        if not getattr(self, "_job_reanchor", False):
            self._job_reanchor = True
            q2 = self._job_pick(self._job_cart, self.JOB_ASSOC_ENTER, "odom")
            if q2 is not None:
                self._job_cart = (q2["x"], q2["y"], q2["yaw"])
                self._job_cart0 = self._job_cart
                self._job_cart_stamp = q2["stamp_ns"]
                r = self._job_to_robot(rp, self._job_cart)
                lat = -r["lateral"]
                remain = -r["forward"]
            axis_remain, axis_lat, corner = self._job_entry_geometry(r)
            self.get_logger().info(
                "캐스터 정렬 끝 — 렉 축 좌우 %+.0fmm 방향 %+.1f도"
                " (조향 입력 좌우 %+.0fmm)"
                % (axis_lat * 1000, math.degrees(r["yaw"]), lat * 1000))
            # 회전·초기 후진 뒤 실제 렉 축 오차를 다시 검사한다. 이전의
            # '좌우 20~260mm 밀림' 해석은 로봇 기준과 렉 기준을 섞은
            # 것이므로 물리 횡이동량으로 단정하지 않는다.
            lat_max2 = self._job_lat_max(
                self.job.get("cart"), distance=axis_remain - self.JOB_IN_M)
            if abs(axis_lat) > lat_max2:
                self._job_retry_or_fail(
                    "돌아선 뒤 렉 축에서 좌우로 %.0fmm 벗어났습니다"
                    " (진입 보정 한계 %.0fmm). 진입 자세를 다시 맞춰 주세요"
                    % (axis_lat * 1000, lat_max2 * 1000))
                return
            if not self._job_entry_clear(r, v_now):
                return

        # 어긋났을 때만 꺾는다.
        #
        # ⚠ 사각지대를 8mm/1도 로 좁게 뒀더니, 차체가 저절로 흔들리는 폭이
        #   그보다 커서 사실상 쉬지 않고 꺾었다. 그것이 '휘적거림' 이다.
        #   넓게 잡아 곧게 가다가 벗어났을 때만 잡는다(2026-09-21 지적).
        #
        # ⚠ 문턱 하나만 두면 그 경계에서 켜졌다 꺼졌다 한다. 켜는 문턱과
        #   끄는 문턱을 따로 둬서(끄는 쪽은 40%) 한 번 잡기 시작하면
        #   충분히 맞을 때까지 이어 간다.
        # ⚠ 다리에 닿는 것은 중심이 아니라 차체 모서리다. 870mm 차체가
        #   2도만 기울어도 모서리는 435*sin2도 = 15mm 더 나간다. 좌우와
        #   방향을 따로 보면 둘 다 문턱 아래인데 합쳐서 닿는 일이 생긴다.
        #   둘을 합친 '모서리가 축에서 벗어난 거리' 하나로 본다.
        _, axis_lat, corner = self._job_entry_geometry(r)
        free = self._job_free(self.job.get("cart"))
        if free is None:
            dead = self.JOB_DEAD_LAT
        else:
            # 렉 밖에서는 넓게, 다리 사이로 들어가면 좁게. 사이는 매끈하게.
            u = self._smooth((remain - self.JOB_IN_M) / 0.40)
            dead = free * (self.JOB_DEAD_IN
                           + (self.JOB_DEAD_FAR - self.JOB_DEAD_IN) * u)
        off = corner > dead
        back = corner < dead * 0.4
        if off:
            self._job_steer = True
        elif back:
            self._job_steer = False
        # ⚠ 다리 사이에서는 상한만 조이면 안 된다. 이득은 그대로 두고 상한만
        #   낮추면 지령이 늘 상한에 붙어(계산 0.138 vs 상한 0.072) 비례제어가
        #   아니라 온·오프 스위치가 된다. 그러면 ±3도로 튕기며 2.3초 주기로
        #   진동한다(2026-09-21 진입 추적). 상한을 좁힌 만큼 이득도 같이
        #   줄여야 포화되지 않고 비례로 움직인다.
        # ⚠ 다리 사이에서는 좌우보다 '방향' 을 먼저 잡아야 한다. 모서리는
        #   |좌우| + 435*sin(방향) 인데, 좌우 13mm 는 13mm 지만 방향 3도는
        #   23mm 다 — 각도가 훨씬 비싸다. 그런데 좌우 이득이 4배 무거워서,
        #   좌우를 잡으려고 각도를 틀고 있었다. 손해 보는 거래다
        #   (2026-09-21: 끝에서 좌우 13mm / 방향 3.0도 / 모서리 36mm).
        #   안쪽에서는 좌우를 느슨하게, 방향을 단단히 잡는다.
        if remain > self.JOB_IN_M:
            wm, s_lat, s_yaw = gn["w_far"], 1.0, 1.0
        else:
            wm = gn["w_near"]
            s_lat, s_yaw = self.JOB_NEAR_KL, self.JOB_NEAR_KY
        # base_node의 반경 제한도 포함한다. 출발 중에는 v_now가 작아서
        # 도킹 노드 상한보다 먼저 모터 쪽에서 잘리는 경우가 있었다.
        wm = self._job_w_limit(v_now, wm)
        # 남는 오차를 시간으로 지운다. 다리 사이에 들어가기 전에만 쌓는다 —
        # 안에서 쌓으면 걸렸을 때 계속 밀어붙인다.
        # ⚠ 쌓기를 멈추는 것만으로는 모자랐다. 상한을 '먼 구간 권한'
        #   (w_far 0.200) 으로 잡아 두면 적분 몫이 0.40*0.200 = 0.079 까지
        #   차는데, 다리 사이 상한은 w_near 0.060 이다. 포화된 적분 하나가
        #   근거리 권한 전체를 32% 넘어선다. 그것이 값이 그대로 얼어붙어
        #   상수로 얹히니, 비례항은 그것을 상쇄하는 자리에서만 멈춘다 —
        #   0.80*방향 + 0.079 = 0 에서 방향 -5.7도다. 0도가 아니라 -5.7도로
        #   수렴하는 제어기였다(2026-09-22 실측 최종 -6.37도, 로그상
        #   적분이 15초간 +0.079 로 고정).
        #
        #   두 가지를 같이 고친다.
        #   (1) 상한을 '지금 쓸 수 있는 권한'(wm) 으로 잡는다. 적분이
        #       상한을 혼자 넘는 일이 구조적으로 생기지 않는다.
        #   (2) 다리 사이에 들어오면 쌓아 둔 것을 흘려보낸다. 0.90/주기,
        #       20Hz 에서 시정수 0.48초 — 진입 1.4m 를 0.12 m/s 로 가는
        #       12초에 비하면 충분히 빠르다.
        # 렉 축 좌우를 먹인다(선 추종). 부호는 예전 입력(lat)과 같다 —
        # lat 은 -axis_lat + 남은거리*방향 이었다(2026-09-28).
        start_zone = gone_all < self.JOB_START_FREE_M
        free = self.JOB_FREE_STEER or start_zone
        if free:
            # 반경 제한 없이 (다리 사이·밖 상한은 그대로), 출발 구간은 더 넉넉히
            wm = self._job_w_free(gn["w_near"] if remain <= self.JOB_IN_M
                                  else gn["w_far"])
            if start_zone:
                wm = max(wm, self.JOB_START_W)
        w, p_term, raw_w = self._job_steering(
            -axis_lat, r["yaw"], remain, gn, wm, s_lat, s_yaw)
        if self.JOB_FREE_STEER:
            w = self._job_damp(w, wm)
        w = self._job_dist_comp(w, gone_all, v_now)
        t.angular.z = float(w)
        # 외곽 검사는 위에서 완료했다. 위험한 지령을 먼저 내보내지 않는다.
        (self.pub_twist_free if free else self.pub_twist).publish(t)

        # 어디서 얼마나 벗어나는지 남긴다. 다음에 고칠 때 근거가 된다.
        now = time.time()
        if now - getattr(self, "_job_ent_t", 0.0) > 0.4:
            self._job_ent_t = now
            self.get_logger().info(
                "  진입 남은 %4.0fmm  좌우 %+5.0f  방향 %+5.1f도  "
                "모서리 %4.0f / 문턱 %3.0f  적분 %+.3f  %s  여유 %.0fmm"
                " | 렉 축 좌우 %+.0fmm P %+.3f 합 %+.3f 지령 %+.3f"
                " 상한 %.3f 속도 %.3f 쏠림 %+.3f"
                % (remain * 1000, lat * 1000, math.degrees(r["yaw"]),
                   corner * 1000, dead * 1000,
                   self.JOB_K_INT * self._job_int,
                   "꺾음" if self._job_steer else "직진",
                   (free or 0.0) * 1000, axis_lat * 1000, p_term,
                   raw_w, w, wm, v_now, getattr(self, "_job_dist", 0.0)))

    # ---------------------------------------------------------------- POI
    #
    # 이름 붙인 지점. 지도에 딸려 있으므로 지도별 파일로 둔다. 지도가 바뀌면
    # 좌표계가 달라져 예전 POI 는 엉뚱한 곳을 가리킨다. 편집(.edits.json)과
    # 같은 방식이다.
    #
    # kind 는 그 지점에서 무엇을 할지다.
    #   point  그냥 그 자리로 간다
    #   cart   렉 밑으로 들어간다.
    #
    # ⚠ 대차 POI 가 가리키는 것은 '도킹을 마쳤을 때의 로봇 자세' 다. 렉 중심에
    #   로봇을 세워 두고 등록하면 된다. 로봇이 뒤로 들어가므로 도킹을 마치면
    #   나왔던 쪽을 보고 있고, 그래서 '바라보는 쪽으로 standoff 만큼 나간 곳'
    #   이 곧 접근 자리가 된다.
    #
    #   처음에는 대차를 인식해서 그 자리를 저장하게 했는데(2026-09-16), 등록
    #   자체가 인식에 걸려 대차가 안 보이면 등록조차 못 했다. 사람이 로봇을
    #   원하는 자리에 세우는 편이 뜻도 분명하고 걸리는 것도 없다.

    POI_KINDS = ("point", "cart", "charger")
    POI_MAX = 200
    # 충전소 POI (2026-11-11, 사용자: "충전소 POI 따로 구분해서 쓸 수 있도록").
    #   좌표 · 방향은 대차와 같은 약속 = '도킹을 마쳤을 때의 로봇 자세'. 충전 접점이
    #   로봇 뒤에 있고 "접촉은 완전 접촉" 이라, 로봇 뒤끝이 스테이션 앞면에 붙고
    #   스테이션 반대쪽을 본다. 등록은 스테이션이 라이다에 보이는 곳에서 하면
    #   charger_detect 가 앞면 V 홈으로 스테이션 자리를 재서 도킹 자세를 계산한다
    #   (로봇을 직접 붙여 세울 필요 없음). 이동은 '바라보는 쪽으로 standoff 만큼
    #   나간 곳' = 스테이션 앞 대기 자세로 간다. 후진 도킹은 다음 단계.
    CHARGER_STANDOFF = 0.8      # 도킹 자세에서 대기 자세까지 [m]
    CHARGER_SCANS = 8           # 등록할 때 평균 낼 스캔 수
    # 렉에서 이만큼 떨어져 서서 인식한다 [m]. 렉마다 carts.yaml 의
    # standoff 로 따로 정할 수 있고, 안 적힌 렉만 이 값을 쓴다.
    #
    # ⚠ 1.8 -> 1.4 -> 1.7 -> 1.5. 1.7 에서는 인식이 안 됐다(2026-09-21).
    #   진입 중 좌우를 지울 거리는 멀수록 길지만, 진입 문턱 상한이
    #   100mm 라 1.5m 면 이미 다 채운다. 더 멀리 갈 이유가 없다.
    #
    # ⚠ 대신 카메라 확인에는 빠듯하다. 화각이 55.5도라 1.43m 면 가까운
    #   다리까지 여유가 47mm 뿐인데 nav2 도착 오차가 28mm 다(2026-09-16
    #   계산). 카메라 확인을 쓰는 렉(cam_legs > 0)은 다리를 놓칠 수 있다.
    #   ls_origin 은 2026-11-13 부터 cam_legs 2 (넥 위 정면 카메라로 앞 기둥 2개).
    POI_STANDOFF = 1.5

    def poi_path(self, name):
        return os.path.join(self.map_dir, name + ".pois.json")

    def load_pois(self, name):
        try:
            with open(self.poi_path(name), encoding="utf-8") as f:
                d = json.load(f)
        except (OSError, ValueError):
            return []
        raw = d.get("pois")
        if not isinstance(raw, list):
            return []
        out = self._clean_pois(raw)
        if int(d.get("version") or 1) < 2:
            # 1판은 대차 POI 에 '들어가는 방향' 을 넣었다. 2판은 '도킹을
            # 마쳤을 때 바라보는 방향' 이라 정반대다. 말없이 180도 틀린 데로
            # 가면 안 되므로 읽을 때 뒤집는다.
            for q in out:
                if q["kind"] == "cart":
                    q["yaw"] = round(
                        math.atan2(math.sin(q["yaw"] + math.pi),
                                   math.cos(q["yaw"] + math.pi)), 4)
        return out

    def save_pois(self, name, pois):
        path = self.poi_path(name)
        if not pois:
            if os.path.exists(path):
                self._rm(path)
            return True, None
        try:
            os.makedirs(self.map_dir, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"version": 2, "map": name, "pois": pois},
                          f, ensure_ascii=False, indent=1)
        except OSError as e:                                 # noqa: BLE001
            return False, str(e)
        return True, None

    def _clean_pois(self, raw):
        """화면이나 파일에서 온 POI 목록을 걸러 낸다. 잘못된 줄만 버린다."""
        out, seen = [], set()
        for r in raw[:self.POI_MAX]:
            if not isinstance(r, dict):
                continue
            nm = str(r.get("name", "")).strip()[:40]
            if not nm or nm in seen:
                continue
            try:
                x, y = float(r["x"]), float(r["y"])
                yaw = float(r.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                continue
            kind = r.get("kind", "point")
            if kind not in self.POI_KINDS:
                kind = "point"
            try:
                so = float(r.get("standoff", self.POI_STANDOFF))
            except (TypeError, ValueError):
                so = self.POI_STANDOFF
            if not math.isfinite(so):
                so = self.POI_STANDOFF
            so = max(0.5, min(3.0, so))
            seen.add(nm)
            size = None
            raw_sz = r.get("size")
            if isinstance(raw_sz, (list, tuple)) and len(raw_sz) == 2:
                try:
                    sz = [float(raw_sz[0]), float(raw_sz[1])]
                    if all(0.1 < v < 3.0 for v in sz):
                        size = [round(v, 4) for v in sz]
                except (TypeError, ValueError):
                    size = None
            legs = None
            raw_lg = r.get("legs")
            if isinstance(raw_lg, (list, tuple)) and len(raw_lg) == 4:
                try:
                    lg = [[float(p[0]), float(p[1])] for p in raw_lg]
                    if all(abs(v) < 1.5 for p in lg for v in p):
                        legs = [[round(v, 4) for v in p] for p in lg]
                except (TypeError, ValueError, IndexError):
                    legs = None
            out.append({"name": nm, "x": round(x, 4), "y": round(y, 4),
                        "yaw": round(yaw, 4), "kind": kind,
                        "standoff": round(so, 3), "size": size, "legs": legs,
                        "cart": (str(r.get("cart"))[:40]
                                 if r.get("cart") else None),
                        # 2026-11-16 좁은 곳: 앞 자리를 거쳐 POI 축을 따라 곧게 들어간다 (narrow_poi.py)
                        "narrow": bool(r.get("narrow")) if kind == "point" else False,
                        "note": str(r.get("note", ""))[:200]})
        return out

    def poi_json(self):
        with self.lock:
            return {"map": self.active_map,
                    "editable": bool(self.active_map),
                    "pois": list(self.pois)}

    def poi_put(self, body):
        """POI 하나를 넣거나 덮어쓴다. 자리를 안 주면 지금 로봇 자리를 쓴다."""
        if not self.active_map:
            return False, "적용된 지도가 없습니다", None
        nm = str(body.get("name", "")).strip()[:40]
        if not nm:
            return False, "이름이 필요합니다", None
        snapped = False
        if body.get("x") is None or body.get("y") is None:
            cur = self.tf_pose()
            if cur is None:
                return False, "로봇 위치를 모릅니다", None
            yaw = body.get("yaw", cur["yaw"])
            x, y = cur["x"], cur["y"]
            # ⚠ 대차 POI 가 가리키는 것은 '로봇이 서는 자리' 가 아니라
            #   '대차 자리' 다. 로봇 자리를 저장하면 대차를 옮길 때마다
            #   POI 를 다시 찍어야 한다. 대차 자리를 두면 얼마나 떨어져서
            #   설지는 standoff 로 따로 정할 수 있다.
            if body.get("kind") == "cart":
                body = dict(body,
                            standoff=body.get("standoff")
                            or self.cart_standoff(body.get("cart")))
                snapped = True
                # 렉 중심에 서 있으면 라이다가 다리 넷을 다 본다. 보이면
                # 실제 축에 맞춰 방향을 다듬고, 어느 변이 진입축인지도 같이
                # 적어 둔다. 다리 넷만으로는 90도까지만 알 수 있으므로,
                # 로봇이 바라보는 방향을 기준으로 접어서 쓴다.
                q = self._job_pick_near((x, y), 0.35, body.get("cart"))
                if q is not None:
                    yaw = yaw + self._job_fold(q["yaw"] - yaw)
                    sz = self._rect_sides(q.get("legs"), yaw)
                    if sz:
                        body = dict(body, size=list(sz))
                    # ⚠ 크기 두 개로는 이 모양을 못 담는다. 다리 배치가
                    #   직사각형이 아닌 렉이 있고(변 588/696/569/714),
                    #   가로·세로 중 어느 쪽이 진입축인지도 헷갈린다.
                    #   인식된 다리 네 점을 POI 기준 좌표로 그대로 저장해
                    #   화면이 그것을 그리게 한다 — 노란(인식)과 보라(POI)가
                    #   같은 모양·같은 자리가 된다(2026-09-21).
                    lg = q.get("legs") or []
                    if len(lg) >= 4:
                        c, sn = math.cos(-yaw), math.sin(-yaw)
                        body = dict(body, legs=[
                            [round((lx - x) * c - (ly - y) * sn, 4),
                             round((lx - x) * sn + (ly - y) * c, 4)]
                            for lx, ly in lg[:4]])
                    if not body.get("cart"):
                        body = dict(body, cart=q["cart"])
            if body.get("kind") == "charger":
                st = self._charger_measure()
                if st is None:
                    return False, ("충전소가 라이다에 안 보입니다 — 스테이션 앞면이 "
                                   "보이게 0.5~2.5m 안에 세우고 다시 하세요"), None
                x, y, yaw = st["dock"]
                body = dict(body, standoff=body.get("standoff") or self.CHARGER_STANDOFF,
                            note=("스테이션 앞면 (%.3f, %.3f), %d장 평균, 흔들림 %.1fmm"
                                  % (st["face"][0], st["face"][1], st["n"], st["spread"] * 1000)))
                snapped = True
            body = dict(body, x=x, y=y, yaw=yaw)
        one = self._clean_pois([body])
        if not one:
            return False, "값이 잘못되었습니다", None
        with self.lock:
            rest = [p for p in self.pois if p["name"] != nm]
            if len(rest) >= self.POI_MAX:
                return False, "POI 가 너무 많습니다 (%d개)" % self.POI_MAX, None
            self.pois = rest + one
            ok, err = self.save_pois(self.active_map, self.pois)
        if not ok:
            return False, "저장 실패: %s" % err, None
        self.get_logger().info("POI 저장: %s (%.2f, %.2f)%s"
                               % (nm, one[0]["x"], one[0]["y"],
                                  "" if not snapped else
                                  " 충전소 인식 (%s)" % one[0]["note"]
                                  if one[0]["kind"] == "charger" else " 대차에 맞춤"))
        msg = "'%s' 을(를) 저장했습니다" % nm
        if snapped and one[0]["kind"] == "charger":
            msg += (" — 충전소 인식, 도킹 자세 (%.2f, %.2f), 대기 %.1fm 앞"
                    % (one[0]["x"], one[0]["y"], one[0]["standoff"]))
        elif snapped:
            msg += (" — 도킹 자세 (%.2f, %.2f), %.0fmm 앞에서 인식"
                    % (one[0]["x"], one[0]["y"], one[0]["standoff"] * 1000))
            if one[0].get("size"):
                msg += (" · 렉 실측 %.0f x %.0f mm (앞뒤 x 좌우)"
                        % (one[0]["size"][0] * 1000, one[0]["size"][1] * 1000))
        return True, msg, one[0]

    def poi_delete(self, name):
        if not self.active_map:
            return False, "적용된 지도가 없습니다"
        nm = str(name or "").strip()
        with self.lock:
            rest = [p for p in self.pois if p["name"] != nm]
            if len(rest) == len(self.pois):
                return False, "'%s' 가 없습니다" % nm
            self.pois = rest
            ok, err = self.save_pois(self.active_map, self.pois)
        if not ok:
            return False, "저장 실패: %s" % err
        return True, "'%s' 을(를) 지웠습니다" % nm

    def poi_find(self, name):
        nm = str(name or "").strip()
        with self.lock:
            for p in self.pois:
                if p["name"] == nm:
                    return dict(p)
        return None

    def poi_goto(self, name):
        why = self._loc_unsure()
        if why:
            return False, why
        p = self.poi_find(name)
        if p is None:
            return False, "'%s' 가 없습니다" % name
        if p.get("kind") == "charger":
            # POI 는 도킹을 마친 자세(스테이션에 붙은 자리)다. 그대로 가면 들이받는다.
            ax, ay, ayaw = self.charger_approach(p)
            return self.nav_goto(ax, ay, ayaw, depart=True)
        # 2026-11-16 좁은 곳 POI 는 앞 자리까지 간 뒤 축을 따라 곧게 들어간다 (narrow_poi.py)
        if p.get("kind", "point") == "point" and self.np_goto(p):
            return True, "좁은 곳: '%s' 앞 자리까지 간 뒤 곧게 들어갑니다" % p["name"]
        # 2026-11-17 그 밖의 지점 POI: 경로가 들어오는 방향으로 직선 길이를 정한다 (판정 없음)
        if p.get("kind", "point") == "point" and self.SP_ENABLE:
            self.sp_cancel()
            return self.sp_request(p)
        return self.nav_goto(p["x"], p["y"], p["yaw"], depart=True, poi=p)

    def charger_approach(self, p):
        """충전소 POI 의 대기 자세: 도킹 자세에서 바라보는 쪽으로 standoff 만큼."""
        so = float(p.get("standoff") or self.CHARGER_STANDOFF)
        return (p["x"] + so * math.cos(p["yaw"]), p["y"] + so * math.sin(p["yaw"]), p["yaw"])

    def _charger_measure(self, n=None, timeout=2.0):
        """서 있는 로봇에서 충전 스테이션을 라이다로 재 지도 좌표로 낸다.
        {face: (x, y), yaw: 바깥 방향, dock: 도킹 자세 (x, y, yaw), n, spread} 또는 None."""
        from und_amr_bringup.charger_detect import detect_scan
        n = n or self.CHARGER_SCANS
        cur = self.tf_pose()
        if cur is None:
            return None
        got, last, t0 = [], None, time.time()
        while len(got) < n and time.time() - t0 < timeout:
            with self.lock:
                m = self.scan
            st_ = None if m is None else (m.header.stamp.sec, m.header.stamp.nanosec)
            if m is None or st_ == last:
                time.sleep(0.02)
                continue
            last = st_
            las = self._poi_laser(m.header.frame_id) or (0.0, 0.0, 0.0)
            c = detect_scan(m.ranges, m.angle_min, m.angle_increment, las)
            if c is not None:
                got.append((c["x"], c["y"], c["yaw"]))
        if len(got) < max(3, n // 2):
            return None
        G = np.array(got)
        bx, by = float(np.median(G[:, 0])), float(np.median(G[:, 1]))
        byaw = math.atan2(float(np.median(np.sin(G[:, 2]))), float(np.median(np.cos(G[:, 2]))))
        spread = float(np.max(np.hypot(G[:, 0] - bx, G[:, 1] - by)))
        c, s_ = math.cos(cur["yaw"]), math.sin(cur["yaw"])
        fx, fy = cur["x"] + c * bx - s_ * by, cur["y"] + s_ * bx + c * by
        yaw = math.atan2(math.sin(cur["yaw"] + byaw), math.cos(cur["yaw"] + byaw))
        d = self.CHARGER_HALF_LEN
        return {"face": (fx, fy), "yaw": yaw,
                "dock": (fx + d * math.cos(yaw), fy + d * math.sin(yaw), yaw),
                "n": len(got), "spread": spread}

    # ---------------------------------------------------------- 지점 왕복
    #
    # 등록한 '지점' 두 곳(A, B)을 오간다. 1회 = A -> B -> A. 시작하면 먼저
    # A 로 간다. repeat 0 은 무한. 각 지점에 도착하면 dwell 초 기다린다.
    # (2026-09-29 대시보드 요청)
    #
    # 리프팅 (2026-10-30, 사용자: "반복주행할때 자동으로 ... 테스트단계니깐"):
    # 지점에 도착하면 내리고, 다음 지점으로 출발하기 전에 올린다.
    #   A 도착 -> 내림 -> dwell -> 올림 -> B 로 ... B 도착 -> 내림 -> ...
    # 리프트는 끝 센서가 없어 펌웨어가 정해진 시간(l t, 기본 10초) 뒤 끝낸다.
    # 그 끝(top/bottom)을 /lift_state 로 보고 다음으로 넘어간다. 서 있을 때만
    # 명령을 보내고, ROUTE_LIFT_TIMEOUT_S 안에 끝나지 않거나 중간에 멈추면
    # 왕복을 멈춘다. (올린 동안 감속은 2026-11-12 뺐다.)
    #
    # 대차 POI 는 좌표가 렉 한가운데(도킹을 마친 자세)라 그대로 가면 렉에
    # 들이받는다. 그래서 '지점' 만 받는다.
    ROUTE_TICK_S   = 0.5
    ROUTE_RETRY    = 1        # 한 구간이 실패하면 이만큼 다시 보낸다
    ROUTE_RETRY_S  = 3.0
    ROUTE_BUSY     = ("going", "waiting", "retry", "lifting", "docking")
    ROUTE_CHARGER_DWELL_S = 3.0   # 충전소 도킹 완료 뒤 출발까지 (충전 전류 확인 뒤)
    ROUTE_LIFT_TIMEOUT_S = 30.0   # 펌웨어 올림/내림 시간(기본 10초) + 여유
    ROUTE_LIFT_STILL_S   = 5.0    # 도착 뒤 이 안에 서지 않으면 실패
    ROUTE_REPEAT_MAX = 9999
    ROUTE_DWELL_MAX  = 600.0

    def route_start(self, a, b, repeat, dwell=0.0, lift=True):
        why = self._loc_unsure()
        if why:
            return False, why
        if self.estop:
            return False, "비상정지 중입니다"
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        if self.control_mode != "auto":
            return False, "자동 모드에서만 왕복할 수 있습니다"
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            return False, "렉 작업 중입니다. 작업을 끝낸 뒤 시작하세요"
        if getattr(self, "cdock", {}).get("state") in self.CD_BUSY:
            return False, "충전소 도킹 중입니다"
        pa, pb = self.poi_find(a), self.poi_find(b)
        for nm, q in ((a, pa), (b, pb)):
            if q is None:
                return False, "'%s' 가 없습니다" % nm
            # 2026-11-18 충전소도 끝점으로 된다 (사용자: "poi 1 이랑 충전소 왔다갔다 하면서 테스트") - 도킹으로 도착
            if q.get("kind") == "cart":
                return False, ("'%s' 는 대차 POI 입니다. 왕복은 '지점' 이나 '충전소' 로 "
                               "등록한 곳만 됩니다" % nm)
        if pa["name"] == pb["name"]:
            return False, "A 와 B 가 같습니다"
        try:
            repeat = int(repeat)
            dwell = float(dwell or 0.0)
        except (TypeError, ValueError):
            return False, "반복 횟수나 대기 시간이 올바르지 않습니다"
        if not 0 <= repeat <= self.ROUTE_REPEAT_MAX:
            return False, "반복 횟수는 1 ~ %d (0 은 무한)" % self.ROUTE_REPEAT_MAX
        if not 0.0 <= dwell <= self.ROUTE_DWELL_MAX:
            return False, "대기 시간은 0 ~ %.0f초" % self.ROUTE_DWELL_MAX
        self.route = {"state": "idle", "a": pa["name"], "b": pb["name"],
                      "repeat": repeat, "dwell": dwell, "lap": 0,
                      "visited_b": False, "target": "A", "retries": 0,
                      "legs": 0, "started": time.time(), "message": "",
                      # 2026-11-18 lift=False: 리프트 없이 주행 · 도킹만 (리프트 중 축1 알람으로 시험이 끊겨서)
                      "lift_on": lift is not False}
        ok, msg = self._route_send()
        if not ok:
            self.route.update(state="failed", message=msg)
            return False, msg
        self.get_logger().info("왕복 시작: %s <-> %s, %s, 대기 %.0f초%s" % (
            pa["name"], pb["name"],
            "무한" if repeat == 0 else "%d회" % repeat, dwell,
            "" if self.route["lift_on"] else ", 리프트 생략"))
        return True, "왕복을 시작합니다 — 먼저 %s 로 갑니다" % pa["name"]

    def _route_send(self):
        r = self.route
        nm = r["a"] if r["target"] == "A" else r["b"]
        p = self.poi_find(nm)
        if p is not None and p.get("kind") == "charger":
            ok, msg = self.cdock_start(nm, from_route=True)
            if ok:
                r.update(state="docking", seq=getattr(self, "nav_seq", None),
                         message="%s(%s) 충전소 도킹 중" % (r["target"], nm))
            return ok, msg
        ok, msg = self.poi_goto(nm)
        if ok:
            r.update(state="going", seq=self.nav_seq,
                     message="%s(%s) 로 이동 중" % (r["target"], nm))
        return ok, msg

    def _route_arrived(self, r, at_charger):
        """끝점에 닿았다 (이동 도착 또는 도킹 완료). 바퀴 수를 세고 다음으로."""
        r["legs"] += 1
        r["retries"] = 0
        if r["target"] == "B":
            r["visited_b"] = True
        elif r["visited_b"]:
            r["lap"] += 1
            r["visited_b"] = False
        finish = bool(r["target"] == "A" and r["repeat"] and r["lap"] >= r["repeat"])
        nm = r["a"] if r["target"] == "A" else r["b"]
        if not at_charger and r.get("lift_on", True):
            self._route_lift(r, "down", "finish" if finish else "wait",
                             "%s(%s) 도착" % (r["target"], nm))
            return
        # 충전소: 리프트 없이 기다렸다 바로 출발 (나갈 때는 nav_goto 가 충전소에서 곧게 나온다)
        if finish:
            r.update(state="done", message="왕복 %d회를 마쳤습니다" % r["lap"])
            self.get_logger().info("왕복 끝: %d회" % r["lap"])
            return
        r["target"] = "B" if r["target"] == "A" else "A"
        r.update(state="waiting",
                 until=time.time() + (max(r.get("dwell", 0.0), self.ROUTE_CHARGER_DWELL_S) if at_charger
                                      else r.get("dwell", 0.0)),
                 seq=getattr(self, "nav_seq", None), from_charger=True,
                 message=("%s 충전소 도킹 완료 — 잠시 뒤 출발" % nm) if at_charger
                 else "%s 도착 (리프트 생략) — 다음으로" % nm)

    def route_cancel(self, why="왕복을 멈췄습니다", stop=True):
        r = self.route
        if r.get("state") not in self.ROUTE_BUSY:
            return False, "진행 중인 왕복이 없습니다"
        lifting = r.get("state") == "lifting"
        docking = r.get("state") == "docking"
        r.update(state="canceled", message=why)
        if docking and stop and getattr(self, "cdock", {}).get("state") in self.CD_BUSY:
            self.cdock_cancel("왕복을 멈춰 도킹도 멈췄습니다")
        if lifting:
            self.lift_stop_async()
        if stop:
            self.nav_cancel()
        self.get_logger().info("왕복 종료: %s (%d회 완료)" % (why, r.get("lap", 0)))
        return True, why

    def _route_tick(self):
        r = self.route
        st = r.get("state")
        if st not in self.ROUTE_BUSY:
            return
        if self.estop or self.control_mode != "auto":
            self.route_cancel("비상정지 또는 수동 모드로 왕복을 멈췄습니다")
            return
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            self.route_cancel("렉 작업이 시작되어 왕복을 멈췄습니다", stop=False)
            return
        now = time.time()
        if st == "going":
            if self.nav_seq != r.get("seq"):
                # 다른 곳에서 새 이동 목표를 보냈다. 그 목표를 살려 둔다.
                self.route_cancel("다른 이동 명령이 들어와 왕복을 멈췄습니다",
                                  stop=False)
                return
            ns = self.nav_state
            if ns == "done":
                self._route_arrived(r, False)
            elif ns == "canceled":
                self.route_cancel("이동이 취소되어 왕복을 멈췄습니다", stop=False)
            elif ns == "failed":
                if r["retries"] < self.ROUTE_RETRY:
                    r["retries"] += 1
                    r.update(state="retry", until=now + self.ROUTE_RETRY_S,
                             message="이동에 실패해 %.0f초 뒤 다시 시도합니다"
                             % self.ROUTE_RETRY_S)
                else:
                    r.update(state="failed", message="%s 로 가지 못해 왕복을 "
                             "멈췄습니다 (%s)" % (r["target"], self.nav_msg))
                    self.get_logger().warn("왕복 실패: %s" % r["message"])
        elif st == "docking":
            cs = getattr(self, "cdock", {}).get("state")
            if cs == "done":
                self._route_arrived(r, True)
            elif cs in ("failed", "canceled"):
                if cs == "failed" and r["retries"] < self.ROUTE_RETRY:
                    r["retries"] += 1
                    r.update(state="retry", until=now + self.ROUTE_RETRY_S, seq=getattr(self, "nav_seq", None),
                             message="충전소 도킹에 실패해 %.0f초 뒤 다시 시도합니다 (%s)"
                             % (self.ROUTE_RETRY_S, self.cdock.get("message")))
                else:
                    r.update(state="failed" if cs == "failed" else "canceled",
                             message="충전소 도킹을 못 해 왕복을 멈췄습니다 (%s)" % self.cdock.get("message"))
                    self.get_logger().warn("왕복 실패: %s" % r["message"])
        elif st == "lifting":
            self._route_lift_tick(r, now)
        elif st in ("waiting", "retry") and self.nav_seq != r.get("seq"):
            # 기다리는 동안 다른 이동 명령이 들어왔다. 그 목표를 살려 둔다.
            self.route_cancel("다른 이동 명령이 들어와 왕복을 멈췄습니다", stop=False)
        elif st in ("waiting", "retry") and now >= r.get("until", 0.0):
            if st == "waiting" and not r.pop("from_charger", False):
                self._route_lift(r, "up", "go", "출발 전")
                return
            ok, msg = self._route_send()
            if not ok:
                r.update(state="failed", message=msg)

    def _route_lift(self, r, cmd, after, why):
        """왕복 중 리프트 올림/내림을 건다. 명령은 서 있을 때 보낸다 (지금 또는 다음 tick)."""
        now = time.time()
        r.update(state="lifting",
                 lift={"cmd": cmd, "after": after, "t0": now,
                       "sent": None, "seen": False},
                 message="%s — 리프트 %s" % (why, "내림" if cmd == "down" else "올림"))
        self._route_lift_tick(r, now)

    def _route_lift_send(self, cmd):
        """base_node 에 올림/내림을 보낸다. 응답은 기다리지 않는다 (timer 안)."""
        cli = self.srv.get("lift_" + cmd)
        if cli is None or not cli.service_is_ready():
            return False
        cli.call_async(Trigger.Request())
        return True

    def _route_still(self):
        o = self.odom
        return o is None or (abs(o.twist.twist.linear.x) <= 0.01
                             and abs(o.twist.twist.angular.z) <= 0.03)

    def _route_lift_fail(self, r, why):
        r.update(state="failed", message="%s — 왕복을 멈췄습니다" % why)
        self.lift_stop_async()
        self.get_logger().warn("왕복 실패: %s" % r["message"])

    def _route_lift_tick(self, r, now):
        L = r["lift"]
        if self.nav_seq != r.get("seq"):
            self.route_cancel("다른 이동 명령이 들어와 왕복을 멈췄습니다", stop=False)
            return
        if L["sent"] is None:
            if not self._route_still():
                if now - L["t0"] > self.ROUTE_LIFT_STILL_S:
                    self._route_lift_fail(r, "리프트 명령 전 로봇이 멈추지 않았습니다")
                return
            if not self._route_lift_send(L["cmd"]):
                self._route_lift_fail(r, "리프트 명령을 보내지 못했습니다 (base_node)")
                return
            L["sent"] = now
            self.get_logger().info("왕복: 리프트 %s" % L["cmd"])
            return
        # 보낸 뒤 움직임(up/down)을 한 번 본 다음의 끝(top/bottom)만 믿는다.
        # 그 전의 top/bottom 은 명령 전 상태가 1초마다 다시 온 것일 수 있다.
        lift = getattr(self, "lift", None)
        if lift in ("up", "down"):
            L["seen"] = True
        end = "bottom" if L["cmd"] == "down" else "top"
        if L["seen"] and lift == end:
            if L["after"] == "finish":
                r.update(state="done", message="왕복 %d회를 마쳤습니다" % r["lap"])
                self.get_logger().info("왕복 끝: %d회" % r["lap"])
            elif L["after"] == "wait":
                r.update(state="waiting", until=now + r["dwell"],
                         message=r["message"].split(" — ")[0]
                         + (" — %.0f초 대기" % r["dwell"] if r["dwell"] else ""))
            else:
                r["target"] = "B" if r["target"] == "A" else "A"
                ok, msg = self._route_send()
                if not ok:
                    r.update(state="failed", message=msg)
        elif L["seen"] and lift == "stopped":
            self._route_lift_fail(r, "리프트가 중간에 멈췄습니다")
        elif now - L["sent"] > self.ROUTE_LIFT_TIMEOUT_S:
            self._route_lift_fail(r, "리프트가 %.0f초 안에 끝나지 않았습니다"
                                  % self.ROUTE_LIFT_TIMEOUT_S)

    def route_status(self):
        r = dict(self.route)
        r.pop("seq", None)
        if r.get("state") in ("waiting", "retry"):
            r["remain_s"] = round(max(0.0, r.get("until", 0.0) - time.time()), 1)
        r.pop("until", None)
        r["busy"] = r.get("state") in self.ROUTE_BUSY
        return r

    def load_edits(self, name):
        """지도에 딸린 편집 목록을 읽는다. 없거나 깨졌으면 빈 목록."""
        try:
            with open(self.edits_path(name), encoding="utf-8") as f:
                d = json.load(f)
        except (OSError, ValueError):
            return []
        sh = d.get("shapes")
        return self._clean_shapes(sh) if isinstance(sh, list) else []

    def save_edits(self, name, shapes):
        path = self.edits_path(name)
        if not shapes:
            if os.path.exists(path):
                self._rm(path)
            return True, None
        try:
            os.makedirs(self.map_dir, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "map": name, "shapes": shapes},
                          f, ensure_ascii=False, indent=1)
        except OSError as e:                                 # noqa: BLE001
            return False, str(e)
        return True, None

    @staticmethod
    def _clean_shapes(raw, limit=6000):
        """화면에서 온 도형 목록을 걸러 낸다. 길이 단위는 미터.

        브러시를 한 번 끌면 원이 수십 개 쌓이므로 개수에 상한을 둔다.
        범위를 벗어난 값은 그 줄만 버린다 - 잘못된 한 줄 때문에 지도가
        통째로 지워지는 일은 없어야 한다.
        """
        out = []
        for sp in raw:
            if len(out) >= limit:
                break
            # 타입 검사를 먼저 한다. op 를 먼저 꺼내면 dict 가 아닌 값에서
            # 그 자리에서 죽어 isinstance 검사가 무용지물이 된다.
            if not isinstance(sp, dict):
                continue
            op = sp.get("op", "erase")
            if op not in ("erase", "wall"):
                continue
            try:
                kind = sp.get("kind", "circle")
                if kind in ("circle", "rect"):
                    x, y = float(sp["x"]), float(sp["y"])
                    if not (abs(x) < 1e4 and abs(y) < 1e4):
                        continue
                if kind == "circle":
                    r = float(sp["r"])
                    if not (0.01 <= r <= 5.0):
                        continue
                    out.append({"op": op, "kind": "circle",
                                "x": round(x, 4), "y": round(y, 4),
                                "r": round(r, 4)})
                elif kind in ("line", "poly"):
                    # 점을 찍어 만드는 선/영역. 가상벽·금지구역용.
                    # 브러시로 원을 줄줄이 찍으면 선 하나에 수십 개가 들어가고
                    # 가장자리가 울퉁불퉁하다. 점 몇 개로 두는 것이 정확하고
                    # 나중에 고치기도 쉽다.
                    pts = sp.get("points")
                    if not isinstance(pts, list):
                        continue
                    P = []
                    for q in pts[:200]:
                        try:
                            qx, qy = float(q[0]), float(q[1])
                        except (TypeError, ValueError, IndexError):
                            P = []
                            break
                        if not (abs(qx) < 1e4 and abs(qy) < 1e4):
                            P = []
                            break
                        P.append([round(qx, 4), round(qy, 4)])
                    need = 2 if kind == "line" else 3
                    if len(P) < need:
                        continue
                    d = {"op": op, "kind": kind, "points": P}
                    if kind == "line":
                        try:
                            w = float(sp.get("width", 0.15))
                        except (TypeError, ValueError):
                            continue
                        if not (0.02 <= w <= 2.0):
                            continue
                        d["width"] = round(w, 4)
                    out.append(d)
                elif kind == "rect":
                    w, h = float(sp["w"]), float(sp["h"])
                    if not (0.01 <= w <= 20.0 and 0.01 <= h <= 20.0):
                        continue
                    out.append({"op": op, "kind": "rect",
                                "x": round(x, 4), "y": round(y, 4),
                                "w": round(w, 4), "h": round(h, 4),
                                "yaw": round(float(sp.get("yaw", 0.0)), 4)})
            except (KeyError, TypeError, ValueError):
                continue
        return out

    def _compose_map(self):
        """원본 지도에 편집을 얹는다. 두 장을 만든다.

          map_png      화면 표시·(앞으로) nav2 용. 지우개와 가상벽 모두 반영
          map_loc_png  자동 위치 재조정용. 지우개만 반영

        가상벽을 위치추정용에 넣으면 안 된다. 실제로 없는 벽이라 라이다가
        볼 수 없는데, 지도에 넣으면 "여기서 벽이 보여야 한다" 고 기대하게
        되어 맞는 자세의 점수가 깎인다. 반대로 지우개는 넣어야 한다 -
        치워진 물건을 계속 벽으로 치면 그것대로 틀린다.

        지운 자리는 흰색(자유공간), 가상벽은 검정(점유)으로 둔다. 지운 자리를
        회색(미지)으로 두면 자동 재조정이 '모르는 곳' 으로 보고 점수를 주지
        않는다.

        호출하는 쪽이 self.lock 을 잡고 있어야 한다.
        """
        base = self.map_base_png
        if base is None:
            return
        self._map_seq += 1
        self.map_meta["seq"] = self._map_seq
        if not self.map_edits or not HAS_IMAGING:
            # 편집이 없어도 nav2 는 지도가 있어야 한다. 원본 그대로 내보낸다.
            self.map_png = base
            self.map_loc_png = base
            self._publish_map_nav(self._png_to_arr(base))
            return
        try:
            with PilImage.open(io.BytesIO(base)) as im:
                a = np.array(im.convert("L"))
        except Exception:                                    # noqa: BLE001
            self.map_png = base
            self.map_loc_png = base
            self._publish_map_nav(self._png_to_arr(base))
            return
        loc = a.copy()          # 위치추정용 (가상벽 제외)
        has_wall = False

        H, W = a.shape
        res = float(self.map_meta.get("resolution") or 0.05)
        o = self.map_meta.get("origin") or {}
        ox, oy = float(o.get("x", 0.0)), float(o.get("y", 0.0))

        # 픽셀 좌표 규칙은 _score_pose 와 같다.
        #   col = (x - ox)/res ,  row = H-1 - (y - oy)/res
        # 도형마다 그 도형이 닿는 사각형 안만 훑는다. 지도 전체를 훑으면
        # 도형 하나에 지도 크기만큼 계산이 든다.
        def to_px(qx, qy):
            return ((qx - ox) / res, H - 1 - (qy - oy) / res)

        for sp in self.map_edits:
            if sp["kind"] in ("line", "poly"):
                continue                     # 아래에서 PIL 로 한꺼번에 그린다
            if sp["kind"] == "circle":
                r = sp["r"]
                x0, x1 = sp["x"] - r, sp["x"] + r
                y0, y1 = sp["y"] - r, sp["y"] + r
            else:
                # 회전한 사각형이 닿는 범위는 반대각선 길이로 넉넉히 잡는다
                d = math.hypot(sp["w"], sp["h"]) / 2.0
                x0, x1 = sp["x"] - d, sp["x"] + d
                y0, y1 = sp["y"] - d, sp["y"] + d
            c0 = max(0, int(math.floor((x0 - ox) / res)))
            c1 = min(W - 1, int(math.ceil((x1 - ox) / res)))
            r0 = max(0, H - 1 - int(math.ceil((y1 - oy) / res)))
            r1 = min(H - 1, H - 1 - int(math.floor((y0 - oy) / res)))
            if c1 < c0 or r1 < r0:
                continue                                     # 지도 밖
            cx = ox + (np.arange(c0, c1 + 1) + 0.5) * res
            cy = oy + (H - 1 - np.arange(r0, r1 + 1) + 0.5) * res
            dx = cx[None, :] - sp["x"]
            dy = cy[:, None] - sp["y"]
            if sp["kind"] == "circle":
                hit = dx * dx + dy * dy <= sp["r"] * sp["r"]
            else:
                c, sn = math.cos(-sp["yaw"]), math.sin(-sp["yaw"])
                lx = dx * c - dy * sn
                ly = dx * sn + dy * c
                hit = (np.abs(lx) <= sp["w"] / 2.0) & (np.abs(ly) <= sp["h"] / 2.0)
            if sp.get("op") == "wall":
                a[r0:r1 + 1, c0:c1 + 1][hit] = 0        # 가상벽 = 점유
                has_wall = True
            else:
                a[r0:r1 + 1, c0:c1 + 1][hit] = 255      # 지우개 = 자유공간
                loc[r0:r1 + 1, c0:c1 + 1][hit] = 255

        # 선(가상벽)과 영역(금지구역). 픽셀을 직접 훑는 것보다 PIL 이 정확하고
        # 빠르다. 선은 굵기를 주고 모서리를 둥글게 이어 끊기지 않게 한다.
        polys = [sp for sp in self.map_edits if sp["kind"] in ("line", "poly")]
        if polys:
            for arr, only_erase in ((a, False), (loc, True)):
                im2 = PilImage.fromarray(arr, mode="L")
                dr = ImageDraw.Draw(im2)
                drew = False
                for sp in polys:
                    wall = sp.get("op") == "wall"
                    if only_erase and wall:
                        continue             # 위치추정용에는 가상벽을 안 넣는다
                    col = 0 if wall else 255
                    pts = [to_px(q[0], q[1]) for q in sp["points"]]
                    if sp["kind"] == "poly":
                        dr.polygon(pts, fill=col, outline=col)
                    else:
                        wpx = max(1, int(round(sp.get("width", 0.15) / res)))
                        dr.line(pts, fill=col, width=wpx, joint="curve")
                        # 굵은 선은 끝이 각져서 이음매가 벌어진다. 점마다
                        # 원을 얹어 메운다.
                        r = wpx / 2.0
                        for px, py in pts:
                            dr.ellipse([px - r, py - r, px + r, py + r], fill=col)
                    drew = True
                    if wall:
                        has_wall = True
                if drew:
                    out_arr = np.array(im2)
                    if only_erase:
                        loc = out_arr
                    else:
                        a = out_arr

        try:
            buf = io.BytesIO()
            PilImage.fromarray(a, mode="L").save(buf, format="PNG")
            self.map_png = buf.getvalue()
            if has_wall:
                buf2 = io.BytesIO()
                PilImage.fromarray(loc, mode="L").save(buf2, format="PNG")
                self.map_loc_png = buf2.getvalue()
            else:
                self.map_loc_png = self.map_png
            self._publish_map_nav(a)
        except Exception:                                    # noqa: BLE001
            self.map_png = base
            self.map_loc_png = base

    @staticmethod
    def _png_to_arr(png):
        if png is None or not HAS_IMAGING:
            return None
        try:
            with PilImage.open(io.BytesIO(png)) as im:
                return np.array(im.convert("L"))
        except Exception:                                    # noqa: BLE001
            return None

    def _publish_map_nav(self, arr):
        """합성된 지도(가상벽 포함)를 OccupancyGrid 로 내보낸다.

        회색조 -> 점유값 규칙은 _on_map 의 역변환이다.
            255 흰색  = 자유   ->   0
              0 검정  = 점유   -> 100
            그 밖    = 미지   ->  -1
        _on_map 에서 np.flipud 로 이미지 좌표계로 뒤집었으므로 되돌린다.

        호출하는 쪽이 self.lock 을 잡고 있어야 한다.
        """
        meta = self.map_meta
        if arr is None or not meta.get("available"):
            return
        try:
            g = np.full(arr.shape, -1, dtype=np.int8)
            g[arr >= 250] = 0
            g[arr <= 5] = 100
            g = np.flipud(g)                      # 이미지 -> 지도 좌표계

            m = OccupancyGrid()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = meta.get("frame") or "map"
            m.info.resolution = float(meta.get("resolution") or 0.05)
            m.info.width = int(meta["width"])
            m.info.height = int(meta["height"])
            o = meta.get("origin") or {}
            m.info.origin.position.x = float(o.get("x", 0.0))
            m.info.origin.position.y = float(o.get("y", 0.0))
            yaw = float(o.get("yaw", 0.0))
            m.info.origin.orientation.z = math.sin(yaw / 2.0)
            m.info.origin.orientation.w = math.cos(yaw / 2.0)
            m.data = g.reshape(-1).tolist()
            self.pub_map_nav.publish(m)
            self._map_nav_seq = self._map_seq
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("nav 지도 발행 실패: %s" % e)

    # ---------------- 자율주행 ----------------

    def _on_plan(self, m):
        """경로를 화면용으로 솎아 둔다.

        계획은 5cm 간격이라 10m 경로가 200점이 넘는다. 그대로 JSON 으로
        내보내면 새로고침마다 수십 KB 가 오간다. 방향이 바뀌는 곳만 남기면
        모양은 그대로면서 점 수가 10분의 1로 준다.
        """
        try:
            pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        except Exception:                                    # noqa: BLE001
            return
        if len(pts) < 2:
            self.nav_path = [[round(x, 3), round(y, 3)] for x, y in pts]
            return
        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            ax, ay = out[-1]
            bx, by = pts[i]
            cx, cy = pts[i + 1]
            # 직선 위의 점은 버린다. 이전 점에서 다음 점을 잇는 선과
            # 이 점 사이가 2cm 안이면 없어도 모양이 같다.
            ux, uy = cx - ax, cy - ay
            L = math.hypot(ux, uy)
            if L < 1e-6:
                continue
            d = abs((bx - ax) * uy - (by - ay) * ux) / L
            if d > 0.02 or math.hypot(bx - ax, by - ay) > 1.0:
                out.append((bx, by))
        out.append(pts[-1])
        self.nav_path = [[round(x, 3), round(y, 3)] for x, y in out[:400]]

    def nav_path_json(self):
        """화면용 남은 경로.
        ⚠ 2026-11-13 (사용자: "이동경로가 안사라짐 지나가면 그뒷부분은 사라지고 도착하면 다
          사라져야하는데 계속 남아있음"): 도착하면 지우는데 플래너의 늦은 /plan 이 다시
          채워 넣어 남았다. 이동 중이 아니면 빈 경로를 내고, 이동 중이면 로봇에서 가장
          가까운 점 앞을 잘라 로봇 자리부터 남은 부분만 낸다."""
        moving = (self.nav_state in ("sending", "active")
                  or getattr(self, "_turning", False)
                  or getattr(self, "_depart_timer", None) is not None)
        pts = list(self.nav_path) if moving else []
        if len(pts) >= 2:
            cur = self.tf_pose()
            if cur is not None:
                rx, ry = cur["x"], cur["y"]
                # 가장 가까운 '선분' 위의 점에서 자른다 (꼭짓점만 보면 긴 직선에서 늦게 줄어든다)
                best, bi, bp = None, 0, None
                for i in range(len(pts) - 1):
                    (ax, ay), (bx, by) = pts[i], pts[i + 1]
                    dx, dy = bx - ax, by - ay
                    L2 = dx * dx + dy * dy
                    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((rx - ax) * dx + (ry - ay) * dy) / L2))
                    px, py = ax + t * dx, ay + t * dy
                    d = (px - rx) ** 2 + (py - ry) ** 2
                    if best is None or d < best:
                        best, bi, bp = d, i, (px, py)
                pts = [[round(rx, 3), round(ry, 3)]] + pts[bi + 1:]
        return {"points": pts, "n": len(pts), "state": self.nav_state}

    # 멈춰 있다 출발할 때 처음 DEPART_M 은 기어가며 방향을 붙잡는다.
    #
    # ⚠ 도착해서 제자리 180도를 돌면 캐스터 넷이 옆으로 선다. 출발하며 한꺼번에
    #   돌아서며 차체를 밀어, 출발 0.6m 에서 지령 외 회전이 15~28 도/s, 누적
    #   -12도까지 났다. 가속을 0.2 -> 0.1 로 반 줄여도 거의 같았다(2026-09-29
    #   왕복 22회). 튐은 거리로 정해지지만 천천히 가면 붙잡기가 따라간다 —
    #   도킹 출발에서 같은 방법으로 6도 -> 3도였다. 지점 이동·왕복에만 쓴다
    #   (poi_goto). 목표가 대략 앞(±DEPART_BEARING)이고 앞이 비어 있을 때만.
    # 0.25 -> 0.20 (사용자 요청). 캐스터 튐은 출발 후 10~20cm 에서 났다
    # (직진 시험) — 그 아래로는 줄이지 않는다.
    # 2026-10-27: 캐스터 교체 뒤 휘청임이 없어 0.20 -> 0.10 (사용자 요청). 출발 한 번에
    #   20cm 기어가기 5.4초 + 올리기 5~8초 = 구간(약 35초)의 1/3 이었다.
    #   실행 중 POST /nav/depart_params 로 바꿔 같은 왕복에서 비교한다.
    DEPART_M       = 0.10
    DEPART_V       = 0.04
    DEPART_BEARING = math.radians(30)
    DEPART_K       = 1.2      # 방향 붙잡기 (캐스터 미리 뒤집기와 같은 값)
    DEPART_KD      = 0.6      # 자이로 받아치기 (도킹과 같은 값)
    DEPART_W       = 0.15
    DEPART_MAX_S   = 12.0
    DEPART_MIN_GOAL_M = 0.8   # 이보다 가까운 목표는 그냥 nav2 로
    # 기어가는 동안 목표 쪽으로 방향을 서서히 맞춘다(이 거리에 걸쳐) [m].
    #
    # ⚠ 출발 방향 그대로 곧게 갔더니, 넘겨받은 nav2 가 목표 쪽과의 몇 도를
    #   급히 고치며 지령을 -5 -> +14 -> -7 -> +7 도/s 로 흔들었고, 실제는 0.5초
    #   늦게 따라가 ±5도 S자를 그렸다(2026-09-29, 사용자 "천천히 가다 빨라지면
    #   S자"). 넘길 때 이미 목표 쪽을 보고 있으면 nav2 가 고칠 것이 없다.
    DEPART_ALIGN_M = 0.12
    # ⚠ 해 보고 버린 것(2026-10-02, 앞 캐스터 교체 후): 넘긴 뒤에도 nav2 속도가
    #   0.2 m/s 가 될 때까지 조향을 곧게 붙잡았다(저속에서 캐스터가 작은 조향을
    #   무시하다 한꺼번에 도는 것을 피하려고). 붙잡는 2초 동안 무시된 nav2 조향이
    #   넘기는 순간 9~14 도/s 로 한꺼번에 들어가, 넘김 뒤 방향 흔들림이 4.5 -> 10도로
    #   나빠졌다. 같은 날 경로 재계획·SLAM 보정·드라이브 적분 8·앞보기 늘리기·
    #   velocity_smoother scale_velocities 도 효과 없었다. 남은 원인은 뒤 캐스터로 본다.
    # ⚠ 같은 날 하나 더: 회전 뒤 첫 SLAM 보정(제자리 회전 중엔 보정 0회)을 기어가는
    #   동안 받고 넘기도록 최대 40cm 까지 더 기어갔다. 보정은 의도대로 기어가는 중에
    #   들어왔지만 넘김 뒤 흔들림은 4.6 -> 6.4도로 줄지 않았다(코드는
    #   logs/api_node.py.try_slamwait_20261002). SLAM 간격 0.1m 는 더 나빴다.

    # 목표가 DEPART_BEARING 보다 틀어져 있으면 먼저 제자리에서 목표 쪽으로 돈 뒤
    # 기어간다.
    #
    # ⚠ 예전에는 그냥 nav2 에 넘겨, nav2 가 0.6 rad/s 로 돌자마자 가속했다. 크게
    #   돈 직후 곧바로 빨라지는 출발이 S자가 가장 큰 경우였다(2026-10-02, 두 지점
    #   방향이 같아 한쪽 출발마다 180도를 돌던 왕복). 어느 방향 출발이든 '돌고 ->
    #   멈추고 -> 기어가고 -> 넘기기' 로 같게 한다. 차체 대각 반경(약 0.46m)에
    #   여유를 둔 DEPART_SWEEP 안에 라이다 점이 있으면 돌지 않고 nav2 에 맡긴다.
    DEPART_SWEEP     = 0.55
    # ⚠ 점 하나로 막혔다고 보면 라이다 잡음에 정렬이 끊겼다(2026-10-02 16:4x, 같은
    #   순간 따로 본 스캔에는 0.6m 안 점이 0개). 점 DEPART_SWEEP_PTS 개 이상이
    #   DEPART_BLOCK_CONFIRM_S 넘게 이어질 때만 막힘으로 보고, 그때도 바로 넘기지
    #   않고 멈춘 채 DEPART_BLOCK_WAIT_S 까지 비기를 기다린다(사람이 지나가는 경우).
    DEPART_SWEEP_PTS = 3
    DEPART_BLOCK_CONFIRM_S = 0.3
    DEPART_BLOCK_WAIT_S = 5.0
    DEPART_TURN_MAX_S = 15.0
    DEPART_TURN_SETTLE_S = 0.5
    DEPART_SETTLE_MIN_S = 0.15     # 회전 뒤 바퀴 회전이 DEPART_SETTLE_W 아래면 이때부터 판정 (2026-11-18)
    DEPART_SETTLE_W = 0.03
    # 30도(DEPART_BEARING)는 '돌기 시작할' 기준이다. 다 돌았는지는 멈춘 뒤 실제
    # 오차가 이 안인지로 판정한다 — 0.5초 기다렸다는 이유만으로 출발하지 않는다.
    DEPART_ALIGN_OK  = math.radians(3.0)
    # ⚠ 2026-11-20 사용자: POI 1 출발(빠져나오기 + 회전) 8초 이하면 된다 — 11/18 58회 평균 11.7초(5.3 + 6.4).
    #   리프트가 내려가 있으면(렉 없음) 출발 회전을 DEPART_W_NORACK 까지 (base_node 각가속 0.6 rad/s^2 제한이
    #   그대로라 출발은 급하지 않다). 끝 비례 감속 KP 1.8 -> DEPART_KP 2.5 (남은 0.5 rad 를 1.2초 끌던 것).
    #   ⚠ 일정 감속(sqrt) 으로 바꾸면 0.6 rad/s 에서 지령을 끊게 돼 TURN_LAG_S(저속에서 맞춘 값)가 안 맞아
    #   3도 넘게 지나칠 수 있다 - 끊는 시점은 지금처럼 저속(TURN_W_MIN 근처)이어야 한다.
    DEPART_W_NORACK = 0.90    # [rad/s]
    # ⚠ 2026-11-20 2.5 는 1↔충전소1 10회에서 1회차마다 3.5~7.1도 지나쳐 18번 다시 돌았다 — 0.9 rad/s 에서 지령이
    #   줄어드는 빠르기(2.5x0.9=2.25 rad/s^2)가 base_node 감속 한도 1.5 를 넘어 실제가 못 따라왔다.
    #   DEPART_KP x DEPART_W_NORACK <= 1.5 로 둔다.
    DEPART_KP = 1.6
    # ⚠ 2026-11-20 0.05초 기록: 끝에 6도 남기고 지령이 0.16 rad/s 까지 내려가면 차체가 0.6초 가까이 거의 선다
    #   (실제 0.03~0.09, 자이로는 반대로 -0.08) — 너무 느린 회전 지령은 차체가 못 따라 돈다. 렉 없을 때 끝도
    #   DEPART_W_END 아래로 안 내리고, 지령을 끊고 더 도는 양은 '실제 회전 속도 x DEPART_COAST_S' 로 본다
    #   (실측: 실제 0.23 rad/s 에서 끊고 3.6도 더 돎 = 0.27초). 지연은 지령 -> 실제 약 0.2초.
    #   -> 4차 10회: 끝 0.30 지령 중에도 차체가 0.4초 멈췄다 지령을 끊는 순간 0.4 rad/s 로 풀려 6.6~10.7도 지나침
    #   (재회전 9). 멈췄다 풀리는 양이 매번 달라 끊는 시점으로는 못 맞춘다 -> 끔(2차 설정 = KP 1.6 만, 9.1초 · 재회전 0).
    DEPART_END_HOLD = False
    DEPART_W_END = 0.30       # [rad/s]
    # ⚠ 2026-11-20 3차 9회: 실제 속도로 끊으면(0.47~0.58 rad/s, 감속이 늦게 따라오는 중) 9도 더 돈다고 보고 7~8도
    #   남기고 끊었는데 실제로는 3도만 더 돌아 5도 남김 → 재회전 6번. 지령(끝 0.30) 기준으로 끊은 회차는
    #   0.9~2.5도 남기고 끝 → 지령 x 0.20초.
    DEPART_COAST_S = 0.20     # [s]
    DEPART_TRACE = False      # True = 회전 끝 0.05초마다 지령 · 실제 기록 (2026-11-20 지연 실측에 씀)
    DEPART_TURN_PASSES = 3

    # ---- 제자리 회전 공간 (2026-11-16) ----
    #
    # ⚠ 예전에는 '중심 반지름 DEPART_SWEEP(0.55m) 안에 라이다 점이 있으면 돌지 않고 nav2 에
    #   맡긴다' 였다. 좁은 POI 에서는 늘 nav2 로 넘어갔고, nav2 의 제자리 회전은 충돌 감시
    #   (앞 0.6m 정지 상자, 카메라 포함)를 거쳐 돌아도 안 닿는 물건에 섰다. 반대로 우리 회전은
    #   카메라 장애물을 보지 않았다(사용자: "좁은 곳에서 회전할 때 카메라 가까워서 멈출 것 같은데").
    #   이제 직사각형 차체가 실제로 돌 방향 · 각도만큼 쓸고 가는 영역만 라이다 + 카메라로 본다.
    #   짧은 쪽이 막히면 반대쪽, 둘 다 막히면 앞/뒤로 SPIN_PREMOVE_M 빠진 뒤 돈다.
    # ⚠ 2026-11-16 0.03 -> 0.01: 벽이 중심에서 0.485m 인 곳에서 도착 회전이 라이다 12점에 막혔는데
    #   수동으로는 부딪히지 않고 충분히 돈다(사용자). 각진 직사각형 모델(모서리 반경 0.467m)이 실제
    #   차체보다 크다(깎인 모서리 추정, 실측 전). 모든 제자리 회전 검사를 1cm 로.
    SPIN_PAD = 0.01           # 차체 바깥 여유 [m]
    SPIN_PTS = 3              # 이 수 이상 걸리면 막힘 (잡음 점 한두 개 무시)
    SPIN_STEP = math.radians(2.0)
    SPIN_PREMOVE_M = 0.15
    SPIN_PREMOVE_V = 0.05
    SPIN_CAM_FRESH_S = 0.5
    SPIN_RMAX = 1.2           # 이보다 먼 점은 볼 필요 없다 (외접 반경 0.467 + 앞뒤 빠지기)

    SPIN_CACHE_S = 0.15       # 센서 점 모으기는 이 간격으로만 (도는 중 매 주기 하면 제어가 끊겼다)

    def _tf_nowait(self, target, source, stamp=None):
        """기다리지 않는 TF 조회 -> {x, y, yaw} 또는 None."""
        buf = getattr(self, "tf_buffer", None)
        if buf is None:
            return None
        try:
            t = buf.lookup_transform(target, source,
                                     RclTime.from_msg(stamp) if stamp is not None else rclpy.time.Time())
        except Exception:                                    # noqa: BLE001
            return None
        q = t.transform.rotation
        return {"x": t.transform.translation.x, "y": t.transform.translation.y,
                "yaw": math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))}

    def _scan_xy(self, m, cam):
        """스캔 -> 그 스캔 시각의 로봇 기준 점 (x, y)."""
        try:
            ext = self._poi_laser(m.header.frame_id)
        except Exception:                                    # noqa: BLE001
            ext = None
        if ext is None:
            return None
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        # 카메라 스캔은 range_max = 감지거리 + 0.5 (그 위는 '비어 있음' 표시)
        top = (m.range_max - 0.45) if cam else m.range_max
        ok = np.isfinite(r) & (r > m.range_min) & (r < top)
        x = ext[0] + r[ok] * np.cos(a[ok] + ext[2])
        y = ext[1] + r[ok] * np.sin(a[ok] + ext[2])
        k = np.hypot(x, y) < self.SPIN_RMAX + 0.3
        return x[k], y[k]

    def _obstacle_sources(self):
        lk = getattr(self, "lock", None)
        if lk is None:
            scans = [("라이다", getattr(self, "scan", None), False)]
        else:
            with lk:
                scans = [("라이다", getattr(self, "scan", None), False)]
        now = time.monotonic()
        for k, (m, t) in list(getattr(self, "_cam_obst", {}).items()):
            if now - t < self.SPIN_CAM_FRESH_S:
                scans.append(("카메라 " + k, m, True))
        return scans

    def _obstacle_pts(self):
        """로봇(base_footprint) 기준 장애물 점 (N, 2): 라이다 + 신선한 카메라 장애물 스캔.

        ⚠ 2026-11-16 각 스캔은 찍힌 순간의 자세로 odom(고정) 좌표에 놓아 SPIN_CACHE_S 동안 두고,
          부를 때마다 지금 자세로만 옮긴다. 도는 중 카메라 지연(0.1~0.2초)으로 벽이 쓸고 갈 영역에
          들어와 회전이 멈췄고, 매 주기 TF 를 기다리며 모으니 회전이 뚝뚝 끊겼다.
        """
        if not HAS_IMAGING:
            return None
        now_m = time.monotonic()
        cur = self._tf_nowait("odom", "base_footprint")
        cache = getattr(self, "_obst_cache", None)
        if cache is None or now_m - cache[0] >= self.SPIN_CACHE_S or cur is None:
            W = []
            for _name, m, cam in self._obstacle_sources():
                if m is None:
                    continue
                got = self._scan_xy(m, cam)
                if got is None:
                    continue
                x, y = got
                then = self._tf_nowait("odom", "base_footprint", m.header.stamp) or cur
                if then is None:
                    W.append(("robot", np.stack([x, y], 1)))
                    continue
                c, s_ = math.cos(then["yaw"]), math.sin(then["yaw"])
                W.append(("odom", np.stack([then["x"] + x * c - y * s_, then["y"] + x * s_ + y * c], 1)))
            cache = (now_m, W)
            self._obst_cache = cache
        out = []
        for frame, P in cache[1]:
            if frame == "odom":
                if cur is None:
                    continue
                c, s_ = math.cos(-cur["yaw"]), math.sin(-cur["yaw"])
                dx, dy = P[:, 0] - cur["x"], P[:, 1] - cur["y"]
                P = np.stack([dx * c - dy * s_, dx * s_ + dy * c], 1)
            out.append(P[np.hypot(P[:, 0], P[:, 1]) < self.SPIN_RMAX])
        if not out:
            return np.zeros((0, 2))
        return np.vstack(out)

    def _obst_to_now(self, m, x, y):
        """스캔 시각의 로봇 기준 점을 지금 로봇 기준으로 (odom, 기다리지 않음). 못 구하면 그대로."""
        then = self._tf_nowait("odom", "base_footprint", m.header.stamp)
        now = self._tf_nowait("odom", "base_footprint")
        if then is None or now is None:
            return x, y
        c0, s0 = math.cos(then["yaw"]), math.sin(then["yaw"])
        wx, wy = then["x"] + x * c0 - y * s0, then["y"] + x * s0 + y * c0
        c1, s1 = math.cos(-now["yaw"]), math.sin(-now["yaw"])
        dx, dy = wx - now["x"], wy - now["y"]
        return dx * c1 - dy * s1, dx * s1 + dy * c1

    def _sweep_hits(self, pts, ang, dx=0.0, pad=None):
        """차체를 앞으로 dx 옮긴 뒤 ang [rad] (부호 = 방향) 만큼 돌 때 쓸고 가는 영역의 점 수."""
        if pts is None or len(pts) == 0:
            return 0
        pad = self.SPIN_PAD if pad is None else pad
        L, W = self.JOB_HALF_L, self.JOB_HALF_W
        # 지금 자리에서 이미 차체 안인 점은 자기 몸 · 잡음이다
        self_pts = (np.abs(pts[:, 0]) <= L) & (np.abs(pts[:, 1]) <= W)
        p = pts[~self_pts] - np.array([dx, 0.0])
        if len(p) == 0:
            return 0
        n = max(1, int(abs(ang) / self.SPIN_STEP) + 1)
        hit = np.zeros(len(p), dtype=bool)
        for th in np.linspace(0.0, ang, n + 1):
            c, sn = math.cos(th), math.sin(th)
            xr = p[:, 0] * c + p[:, 1] * sn
            yr = -p[:, 0] * sn + p[:, 1] * c
            hit |= (np.abs(xr) <= L + pad) & (np.abs(yr) <= W + pad)
        return int(hit.sum())

    def _path_hits(self, pts, dx):
        """곧게 dx [m] 갈 때 차체 앞(뒤) 띠에 드는 점 수."""
        if pts is None or len(pts) == 0 or dx == 0.0:
            return 0
        L, W, pad = self.JOB_HALF_L, self.JOB_HALF_W, self.SPIN_PAD
        band = np.abs(pts[:, 1]) <= W + pad
        if dx > 0:
            band &= (pts[:, 0] > L) & (pts[:, 0] <= L + dx + pad)
        else:
            band &= (pts[:, 0] < -L) & (pts[:, 0] >= -L + dx - pad)
        return int(band.sum())

    def _spin_plan(self, err):
        """err [rad] 만큼 돌아야 할 때의 계획. {"dir", "ang", "move"} 또는 None(공간 없음)."""
        pts = self._obstacle_pts()
        if pts is None:
            return {"dir": 1.0 if err > 0 else -1.0, "ang": err, "move": 0.0}
        short = err
        long_ = err - math.copysign(2.0 * math.pi, err)
        for dx in (0.0, self.SPIN_PREMOVE_M, -self.SPIN_PREMOVE_M):
            if dx and self._path_hits(pts, dx) >= self.SPIN_PTS:
                continue
            for ang in (short, long_):
                if self._sweep_hits(pts, ang, dx) < self.SPIN_PTS:
                    return {"dir": 1.0 if ang > 0 else -1.0, "ang": ang, "move": dx}
        return None

    def _sweep_sources(self, ang):
        """어느 센서 점이 쓸고 갈 영역에 걸리나 (로그용)."""
        out = []
        with self.lock:
            sc = self.scan
        srcs = [("라이다", sc, False)] + [
            ("카메라 " + k, m, True) for k, (m, t) in getattr(self, "_cam_obst", {}).items()
            if time.monotonic() - t < self.SPIN_CAM_FRESH_S]
        for name, m, cam in srcs:
            if m is None:
                continue
            ext = self._poi_laser(m.header.frame_id)
            if ext is None:
                continue
            r = np.asarray(m.ranges, dtype=float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            top = (m.range_max - 0.45) if cam else m.range_max
            ok = np.isfinite(r) & (r > m.range_min) & (r < top)
            x = ext[0] + r[ok] * np.cos(a[ok] + ext[2])
            y = ext[1] + r[ok] * np.sin(a[ok] + ext[2])
            x, y = self._obst_to_now(m, x, y)
            out.append("%s %d" % (name, self._sweep_hits(np.stack([x, y], 1), ang)))
        return ", ".join(out)

    def _turn_pick_dir(self, err, pref):
        """도착 회전 방향. pref 쪽으로 err 를 채우는 회전이 막혔으면 반대쪽, 둘 다 막히면 None."""
        pts = self._obstacle_pts()
        if pts is None:
            return pref
        _pad = self.NP_TURN_PAD if getattr(self, "_np_turning", False) else None
        for d in (pref, -pref):
            ang = err if d * err > 0 else err - math.copysign(2.0 * math.pi, err)
            if self._sweep_hits(pts, ang, pad=_pad) < self.SPIN_PTS:
                if d != pref:
                    self.get_logger().info("도착 회전: %s 쪽이 막혀 반대로 %.0f도 돎"
                                           % ("왼" if pref > 0 else "오른", abs(math.degrees(ang))))
                return d
        return None

    @staticmethod
    def _plan_note(plan):
        if not plan:
            return ""
        side = "왼쪽" if plan["dir"] > 0 else "오른쪽"
        mv = ("" if not plan.get("move") else
              ", 먼저 %s %.0fcm" % ("앞으로" if plan["move"] > 0 else "뒤로", abs(plan["move"]) * 100))
        return " (%s으로 %.0f도%s)" % (side, abs(math.degrees(plan["ang"])), mv)

    def _spin_clear(self, radius):
        """제자리에서 돌아도 되나 — 라이다 기준 반경 안 점이 DEPART_SWEEP_PTS 개
        미만이면 True (잡음 점 한두 개는 무시)."""
        with self.lock:
            m = self.scan
        if m is None:
            return False
        a = m.angle_min
        n = 0
        for rng in m.ranges:
            if m.range_min < rng < m.range_max:
                x = rng * math.cos(a) - 0.015       # 라이다는 구동축 근처
                y = rng * math.sin(a)
                if math.hypot(x, y) < radius:
                    n += 1
                    if n >= self.DEPART_SWEEP_PTS:
                        return False
            a += m.angle_increment
        return True

    def _depart_ok(self, x, y, turn=False, aim=None):
        """출발 기어가기를 할 수 있으면 지금 자세, 아니면 None.

        turn=True 면 목표가 옆·뒤에 있어도 돌 자리가 비어 있으면 받아 준다
        (그때 cur["turn"] = True)."""
        cur = self.tf_pose()
        with self.lock:
            od = self.odom
        if cur is None or od is None:
            return None
        if abs(od.twist.twist.linear.x) > 0.01 or abs(od.twist.twist.angular.z) > 0.03:
            return None                      # 이미 움직이는 중
        dx, dy = x - cur["x"], y - cur["y"]
        if math.hypot(dx, dy) < self.DEPART_MIN_GOAL_M:
            return None
        if aim is not None:                  # 방향은 경로가 처음 향하는 점으로 (2026-11-17)
            dx, dy = aim[0] - cur["x"], aim[1] - cur["y"]
        b = math.atan2(dy, dx) - cur["yaw"]
        b = math.atan2(math.sin(b), math.cos(b))
        if abs(b) > getattr(self, "_depart_bearing_lim", self.DEPART_BEARING):
            if turn:
                plan = self._spin_plan(b)
                cur = dict(cur)
                if plan is None:
                    cur["noroom"] = True            # 어느 쪽으로도, 앞뒤로 빠져도 못 돈다
                    return cur
                cur["turn"] = True
                cur["plan"] = plan
                return cur
            return None
        if self._exit_front_blocked():
            return None
        return cur

    def _depart_stop(self):
        t = getattr(self, "_depart_timer", None)
        self._depart_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass

    # 출발 'nav2' 방식 — 처음부터 nav2 가 조향하고, DEPART_M 동안 속도만 낮춘다.
    #
    # ⚠ 'creep' 방식은 api_node 가 기어가다 nav2 로 넘겨 조향 주체가 바뀌고, 넘기는
    #   순간 지령이 계단처럼 뛰었다(+1.9 -> +4.9 도/s, 2026-10-02). nav2 속도 제한은
    #   RPP 가 회전을 '속도 x 곡률' 로 내므로 회전도 같은 비율로 줄어 경로 곡선이
    #   유지된다. 실행 중 POST /nav/depart_mode 로 바꿔 같은 왕복에서 비교한다.
    DEPART_SLOW_V = 0.04
    # 저속 구간이 끝나면 제한을 이 거리 동안 설정 속도까지 서서히 올린 뒤 푼다.
    # ⚠ 2026-10-08: 20cm 에서 제한을 한 번에 풀자 0.04 -> 0.40 m/s 로 가속하며
    #   휘청했다(사용자: '0.04 제한을 푸는 순간'). RPP 는 회전을 속도 x 곡률로
    #   내므로 속도가 10배 오르는 동안 작은 방향 오차를 고치는 회전도 같이 커진다.
    # 2026-10-27: 0.80 -> 0.40 (캐스터 교체 뒤, DEPART_M 과 같이).
    DEPART_RAMP_M = 0.40
    # ⚠ 2026-11-17 사용자: "완전 부드럽게 끊기지 않게" / "바로 가속먹이면 휘청거리지 않아?". 0.04 m/s 로
    #   DEPART_M 동안 평평하게 가다가 올리면 2초쯤 서 있다 가는 것처럼 보였다. 처음 속도는 그대로 두고
    #   (회전 직후 밀림을 막는 몫) 평평한 구간 없이 DEPART_M + DEPART_RAMP_M 에 걸쳐 S자 곡선으로 올린다.
    #   10cm 지점 0.04 -> 약 0.09 m/s (설정 0.5 기준), 10cm 지나는 시간 2.5 -> 약 1.6초. False 면 예전 방식.
    DEPART_SMOOTH = True

    def _pub_speed_limit(self, v):
        m = SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = False
        a = getattr(self, "_appr_lim", None)  # POI 앞 자리로 다가오는 중 (narrow_poi._appr_update)
        if a is not None and (v <= 0.0 or v > a):
            v = a
        m.speed_limit = float(v)            # 0 = 제한 없음
        self.pub_speed_limit.publish(m)

    def _slow_lost(self):
        """저속 출발 중 위치(odom)를 잃었다 — 제한만 풀면 정상 속도가 허용되므로
        이동을 취소하고 세운다. 그 뒤(목표가 없을 때) 제한을 푼다."""
        self.get_logger().error("저속 출발 중 위치(odom) %.1f초째 없음 — 이동 취소 (%s)"
                                % (getattr(self, "_slow_miss_t", 0.0), getattr(self, "_tf_err", None) or "?"))
        self.nav_cancel()
        self._pub_zero()
        self.nav_state, self.nav_msg = "failed", "위치 정보가 없어 출발을 멈췄습니다"
        self._slow_release()

    SLOW_LOST_S = 1.0

    def _slow_start(self):
        od = self.tf_pose("odom", "base_footprint")
        self._slow_miss_t = 0.0
        # 시작 순간 못 읽어도 바로 포기하지 않는다 — 제한(0.04)은 걸어 두고 _slow_tick 이 SLOW_LOST_S 동안 본다
        self._slow_xy = (od["x"], od["y"]) if od is not None else None
        # 직진으로 빠져나온 직후 회전 없이 그대로 출발하면 천천히 시작할 까닭이 없다 (2026-11-17)
        self._slow_skip = bool(getattr(self, "_sp_fwd_out", False)) and self.DEPART_SKIP_AFTER_OUT
        self._sp_fwd_out = False
        if self._slow_skip:
            self.get_logger().info("직진으로 빠져나와 회전 없이 출발 — 천천히 시작 생략")
            self._pub_speed_limit(0.0)
            return
        self._pub_speed_limit(self.DEPART_SLOW_V)
        if self._slow_timer is None:
            self._slow_timer = self.create_timer(1.0 / self.NAV_WATCH_HZ, self._slow_tick)

    def _slow_tick(self):
        od = self.tf_pose("odom", "base_footprint")
        if (getattr(self, "_slow_seq", None) != getattr(self, "nav_seq", None)
                or self.nav_state not in ("sending", "active")):
            self._slow_release()                # 목표가 끝났거나 바뀌었다
            return
        if od is None:
            # ⚠ 2026-11-23 api_node 재기동 1~2분 안에 두 번, 한 번 못 읽은 것(0.5초 넘은 TF)만으로 이동을 취소했다.
            #   EKF 는 30Hz 그대로였다 — 잠깐 늦은 것. SLOW_LOST_S 동안 계속 못 읽을 때만 위치를 잃은 것으로 본다.
            self._slow_miss_t = getattr(self, "_slow_miss_t", 0.0) + 1.0 / self.NAV_WATCH_HZ
            if self._slow_miss_t >= self.SLOW_LOST_S:
                self._slow_lost()
            return
        self._slow_miss_t = 0.0
        if getattr(self, "_slow_xy", None) is None:
            self._slow_xy = (od["x"], od["y"])          # 시작할 때 못 읽었으면 지금부터 잰다
        gone = math.hypot(od["x"] - self._slow_xy[0], od["y"] - self._slow_xy[1])
        top = (self._speed_eff() if hasattr(self, "nav_speed")
               else self.NAV_SPEED_DEFAULT)
        if self.DEPART_SMOOTH:
            v, k = self._depart_smooth_v(gone, top)
            if k < 1.0 and v < top:
                if not getattr(self, "_slow_ramping", False):
                    self._slow_ramping = True
                    self.get_logger().info("출발 — %.0fcm 동안 %.2f 에서 곡선으로 속도를 올림"
                                           % ((self.DEPART_M + self.DEPART_RAMP_M) * 100, self.DEPART_SLOW_V))
                self._pub_speed_limit(v)
                return
        if not self.DEPART_SMOOTH and gone < self.DEPART_M:
            self._pub_speed_limit(self.DEPART_SLOW_V)     # 늦게 뜬 controller 도 받게 계속
            return
        k = (gone - self.DEPART_M) / max(self.DEPART_RAMP_M, 1e-6)
        v = self.DEPART_SLOW_V + (top - self.DEPART_SLOW_V) * k
        if k >= 1.0 or v >= top:
            self.get_logger().info("출발 %.0fmm — 설정 속도 도달, nav2 속도 제한 풂"
                                   % (gone * 1000))
            self._slow_release()
            return
        if not getattr(self, "_slow_ramping", False):
            self._slow_ramping = True
            self.get_logger().info("출발 %.0fmm 저속 끝 — %.0fcm 동안 속도를 서서히 올림"
                                   % (gone * 1000, self.DEPART_RAMP_M * 100))
        self._pub_speed_limit(v)

    def _depart_smooth_v(self, gone, top):
        """출발 속도 제한: DEPART_SLOW_V 에서 S자 곡선(smoothstep)으로 top 까지. (v, 진행 비율)."""
        total = max(self.DEPART_M + self.DEPART_RAMP_M, 1e-6)
        x = max(0.0, min(1.0, gone / total))
        f = x * x * (3.0 - 2.0 * x)
        return self.DEPART_SLOW_V + (top - self.DEPART_SLOW_V) * f, x

    def _slow_release(self):
        self._slow_ramping = False
        t = self._slow_timer
        self._slow_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass
            self._pub_speed_limit(0.0)

    # POST /nav/depart_params 로 바꿀 수 있는 값과 범위 (재시작하면 위 기본값으로)
    DEPART_PARAMS = {"slow_m": ("DEPART_M", 0.0, 0.50),
                     "ramp_m": ("DEPART_RAMP_M", 0.0, 1.50),
                     "slow_v": ("DEPART_SLOW_V", 0.02, 0.20)}

    def depart_params(self):
        return {k: getattr(self, a) for k, (a, _, _) in self.DEPART_PARAMS.items()}

    def set_depart_params(self, b):
        """출발 저속 거리 · 속도 올리는 거리 · 저속 속도를 실행 중에 바꾼다.
        다음 출발부터 적용. 하나라도 틀리면 아무것도 안 바꾼다."""
        new = {}
        for k, (a, lo, hi) in self.DEPART_PARAMS.items():
            if b.get(k) is None:
                continue
            try:
                v = float(b[k])
            except (TypeError, ValueError):
                return False, "%s 는 숫자" % k
            if not (lo <= v <= hi) or math.isnan(v):
                return False, "%s 는 %.2f ~ %.2f" % (k, lo, hi)
            new[a] = v
        if not new:
            return False, "slow_m, ramp_m, slow_v 중 하나는 있어야 합니다"
        for a, v in new.items():
            setattr(self, a, v)
        self.get_logger().info("출발 설정: 저속 %.0fcm (%.2f m/s), 올리기 %.0fcm" % (
            self.DEPART_M * 100, self.DEPART_SLOW_V, self.DEPART_RAMP_M * 100))
        return True, "다음 출발부터 적용합니다"

    def set_depart_mode(self, mode):
        if mode not in ("creep", "nav2"):
            return False, "mode 는 creep 또는 nav2"
        self.depart_mode = mode
        self.get_logger().info("출발 방식: %s" % mode)
        return True, "출발 방식을 %s 로 바꿨습니다" % mode

    def _depart_handover(self, why):
        """출발 동작을 그만두고 nav2 에 넘긴다(돌 자리가 막힘 등)."""
        self._depart_stop()
        self._pub_zero()
        self.get_logger().info("출발 회전 그만둠 (%s) — nav2 로 넘김" % why)
        self._nav_send(self._depart_goal, self._depart_seq)
        if self.depart_mode == "nav2":
            self._slow_seq = self._depart_seq
            self._slow_start()

    def _depart_turn_tick(self):
        dt = 1.0 / self.NAV_WATCH_HZ
        self._depart_t += dt
        if self.estop or self.control_mode != "auto" or getattr(self, "_scan_dead", False):
            self._depart_stop()
            self._pub_zero()
            self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
            return
        cur = self.tf_pose()
        if cur is None:
            self._depart_handover("자세 모름")
            return
        g = self._depart_goal_xy
        if self._depart_phase == "premove":
            # 돌 자리를 만들려고 앞(뒤)으로 곧게 조금 간다
            dx = self._depart_premove
            gone = math.hypot(cur["x"] - self._depart_premove_xy[0],
                              cur["y"] - self._depart_premove_xy[1])
            if gone >= abs(dx) - 0.005 or self._depart_t > 8.0:
                self._pub_zero()
                self._depart_phase = "turn"
                self._depart_t = 0.0
                self.get_logger().info("출발 정렬: 돌 자리를 만들려고 %+.0fmm 움직임 — 이제 회전" % (gone * 1000 * (1 if dx > 0 else -1)))
                return
            if self._path_hits(self._obstacle_pts(), math.copysign(abs(dx) - gone + 0.02, dx)) >= self.SPIN_PTS:
                self._pub_zero()
                self._depart_handover("돌 자리 만들기 중 앞(뒤) 막힘")
                return
            t = Twist()
            t.linear.x = float(math.copysign(self.SPIN_PREMOVE_V, dx))
            self.pub_twist_free.publish(t)
            return
        bearing = math.atan2(g[1] - cur["y"], g[0] - cur["x"])
        err = math.atan2(math.sin(bearing - cur["yaw"]), math.cos(bearing - cur["yaw"]))
        if self._depart_phase == "turn_settle":
            # 2026-11-18 늘 0.5초 서 있던 것 -> 바퀴가 실제로 멈췄으면(0.15초 뒤부터) 바로 판정
            od = getattr(self, "odom", None)
            wz = abs(od.twist.twist.angular.z) if od is not None else 1.0
            if self.DEPART_TRACE:
                _d = getattr(self, "_depart_turn_dir", 0.0) or 1.0
                self.get_logger().info("  출발 회전 멈춤 뒤 %.2fs 남은 %+.2f도 실제 %+.3f 자이로 %+.3f"
                                       % (self._depart_t, math.degrees(err * _d),
                                          od.twist.twist.angular.z if od is not None else float("nan"),
                                          getattr(self, "_gyro_z", float("nan"))))
            if self._depart_t < self.DEPART_TURN_SETTLE_S and not (
                    self._depart_t >= self.DEPART_SETTLE_MIN_S and wz < self.DEPART_SETTLE_W):
                return
            if abs(err) > self.DEPART_ALIGN_OK:
                self._depart_passes = getattr(self, "_depart_passes", 0) + 1
                if self._depart_passes >= self.DEPART_TURN_PASSES:
                    self._depart_handover("정렬 %d회 뒤에도 %.1f도" % (
                        self._depart_passes, math.degrees(err)))
                    return
                self.get_logger().info("출발 정렬 %d회차 뒤 %+.1f도 남음 — 다시 돎"
                                       % (self._depart_passes, math.degrees(err)))
                self._depart_phase = "turn"
                self._depart_t = 0.0
                self._depart_turn_dir = 0.0         # 남은 오차 쪽으로 새로 정한다
                return
            if self._exit_front_blocked():
                self._depart_handover("앞 막힘")
                return
            if self.depart_mode == "nav2":
                # 정렬만 하고 전진은 nav2 에 (저속 제한으로 시작)
                self._depart_stop()
                self.get_logger().info("출발 정렬 끝 (목표 쪽과 %+.1f도) — nav2 저속 출발"
                                       % math.degrees(err))
                self._slow_seq = self._depart_seq
                self._nav_send(self._depart_goal, self._depart_seq)
                self._slow_start()
                return
            # 여기서부터 평소 출발 기어가기
            self._depart_phase = "creep"
            self._depart_xy, self._depart_yaw = (cur["x"], cur["y"]), cur["yaw"]
            self._depart_bearing = bearing
            self._depart_t = 0.0
            self._depart_w_last = 0.0
            self.get_logger().info("출발 회전 끝 (목표 쪽과 %+.1f도) — 기어감" % math.degrees(err))
            return
        # --- 도는 중 ---
        if self._depart_t > self.DEPART_TURN_MAX_S:
            self._depart_handover("회전 시간 초과")
            return
        _dir = getattr(self, "_depart_turn_dir", 0.0) or (1.0 if err > 0 else -1.0)
        _rem = err * _dir
        if _rem < -math.pi / 2.0:
            _rem += 2.0 * math.pi
        if self._sweep_hits(self._obstacle_pts(), _dir * max(_rem, 0.05)) >= self.SPIN_PTS:
            self._depart_blocked_t = getattr(self, "_depart_blocked_t", 0.0) + dt
            if self._depart_blocked_t >= self.DEPART_BLOCK_CONFIRM_S:
                self._pub_zero()                    # 확실히 막혔다 — 멈추고 기다린다
                if self._depart_blocked_t >= self.DEPART_BLOCK_WAIT_S:
                    self._depart_handover("돌 자리에 물체 %.0f초" % self._depart_blocked_t)
                    return
                if not getattr(self, "_depart_wait_logged", False):
                    self._depart_wait_logged = True
                    self.get_logger().info("출발 정렬: 돌 자리에 물체 — 비기를 기다림")
                self._depart_t -= dt                # 기다리는 시간은 회전 시간에서 뺀다
                return
        else:
            if getattr(self, "_depart_blocked_t", 0.0) >= self.DEPART_BLOCK_CONFIRM_S:
                self._depart_t = 0.0                # 섰다가 다시 돈다 — 처음처럼 천천히 올린다
            self._depart_blocked_t = 0.0
            self._depart_wait_logged = False
        if getattr(self, "_depart_turn_dir", 0.0) == 0.0:
            self._depart_turn_dir = 1.0 if err > 0 else -1.0
        w = self.TURN_KP * err
        wmax = self.TURN_W_MAX
        if (getattr(self, "cdock", None) or {}).get("state") in ("starting", "moving"):
            wmax = self.TURN_W_MAX_NORACK         # 충전소로 가는 중 - 렉을 싣지 않는다
        kp, wmin, norack = self.TURN_KP, self.TURN_W_MIN, False
        if self._depart_t <= 1.5 / self.NAV_WATCH_HZ or getattr(self, "_depart_err0", None) is None:
            self._depart_err0 = abs(err)         # 이번 회차 시작 각
        if getattr(self, "lift", None) == "bottom":
            wmax = max(wmax, self.DEPART_W_NORACK)   # 렉 없음 (DEPART_W_NORACK 주석)
            kp = self.DEPART_KP
            if self.DEPART_END_HOLD and self._depart_err0 > math.radians(30.0):   # 큰 회전만
                wmin, norack = self.DEPART_W_END, True
        cap = max(self.TURN_W_MIN, wmax * min(1.0, self._depart_t / self.TURN_RAMP_S))
        rem = err * self._depart_turn_dir
        if rem < -math.pi / 2.0:
            rem += 2.0 * math.pi            # ±180도 경계 (도착 회전과 같은 처리)
        w = max(wmin, min(cap, abs(kp * err)))
        _od = getattr(self, "odom", None)
        w_act = abs(_od.twist.twist.angular.z) if _od is not None else w
        coast = (w * self.DEPART_COAST_S) if norack else (w * self.TURN_LAG_S)
        if rem <= max(self.TURN_TOL, coast):
            self._pub_zero()
            self._depart_phase = "turn_settle"
            self._depart_t = 0.0
            return
        t = Twist()
        t.angular.z = float(self._depart_turn_dir * w)
        self.pub_twist_free.publish(t)
        if self.DEPART_TRACE and rem < 0.8:
            od = getattr(self, "odom", None)
            self.get_logger().info("  출발 회전 %.2fs 남은 %+.2f도 지령 %+.3f 실제 %+.3f 자이로 %+.3f"
                                   % (self._depart_t, math.degrees(rem), self._depart_turn_dir * w,
                                      od.twist.twist.angular.z if od is not None else float("nan"),
                                      getattr(self, "_gyro_z", float("nan"))))

    def _depart_tick(self):
        if getattr(self, "_depart_phase", "creep") in ("turn", "turn_settle", "premove"):
            self._depart_turn_tick()
            return
        cur = self.tf_pose()
        self._depart_t += 1.0 / self.NAV_WATCH_HZ
        gone = (math.hypot(cur["x"] - self._depart_xy[0], cur["y"] - self._depart_xy[1])
                if cur is not None else 0.0)
        blocked = self._exit_front_blocked()
        if (cur is None or gone >= self.DEPART_M or self._depart_t > self.DEPART_MAX_S
                or blocked or getattr(self, "_scan_dead", False)
                or self.estop or self.control_mode != "auto"):
            self._depart_stop()
            if blocked:
                self._pub_zero()        # 우선 통로의 마지막 전진 지령을 지운다
            if (self.estop or self.control_mode != "auto"
                    or getattr(self, "_scan_dead", False)):
                self._pub_zero()
                self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
                return
            b = getattr(self, "_depart_bearing", self._depart_yaw)
            self.get_logger().info("출발 %.0fmm 기어감 (목표 쪽과 %+.1f도) — nav2 로 넘김" % (
                gone * 1000, math.degrees(math.atan2(math.sin(cur["yaw"] - b),
                                                     math.cos(cur["yaw"] - b))) if cur else 0.0))
            self._nav_send(self._depart_goal, self._depart_seq)
            return
        # 붙잡을 방향: 출발 방향에서 목표 쪽으로 DEPART_ALIGN_M 에 걸쳐 옮긴다
        b = getattr(self, "_depart_bearing", self._depart_yaw)
        db = math.atan2(math.sin(b - self._depart_yaw), math.cos(b - self._depart_yaw))
        want = self._depart_yaw + db * min(1.0, gone / self.DEPART_ALIGN_M)
        e = math.atan2(math.sin(want - cur["yaw"]), math.cos(want - cur["yaw"]))
        w = self.DEPART_K * e
        if time.monotonic() - getattr(self, "_gyro_t", 0.0) < 0.2:
            w -= self.DEPART_KD * (self._gyro_z - getattr(self, "_depart_w_last", 0.0))
        w = max(-self.DEPART_W, min(self.DEPART_W, w))
        self._depart_w_last = w
        t = Twist()
        t.linear.x = self.DEPART_V
        t.angular.z = float(w)
        self.pub_twist_free.publish(t)

    def _nav_send(self, g, seq):
        if seq != getattr(self, "nav_seq", seq):
            return                              # 그 사이 다른 목표가 들어왔다
        fut = self.nav_cli.send_goal_async(g)
        fut.add_done_callback(lambda f: self._nav_accepted(f, seq))

    def _nav_goal_msg(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.map_meta.get("frame") or "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return g

    def nav_goto(self, x, y, yaw, yaw_ok=None, turn_max=None, depart=False, poi=None):
        """목표로 보낸다.

        yaw_ok / turn_max 로 도착 후 방향 맞추기를 느슨하게 할 수 있다.
        작업 중간 단계는 방향이 정확할 필요가 없다 — 어차피 진입 직전에
        대차 축에 맞춰 다시 돌고, 그 사이 180도 회전에서 또 틀어진다.
        그런데 매번 2도까지 맞추느라 돌고 멈추기를 되풀이하면 회전이
        눈에 띄게 많아진다(2026-09-16 지적).
        """
        """지도 좌표로 목표를 보낸다. 보내기만 하고 결과는 콜백으로 받는다.

        HTTP 요청 스레드에서 future 를 기다리면 그 동안 응답이 막힌다.
        rclpy.spin 은 메인 스레드에서 돌고 있으므로 콜백으로 받는다.
        """
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        if not self.nav_cli.server_is_ready():
            # 한 번은 짧게 기다려 본다. 갓 띄운 직후일 수 있다.
            if not self.nav_cli.wait_for_server(timeout_sec=2.0):
                return False, ("nav2 가 떠 있지 않습니다. "
                               "ros2 launch und_amr_bringup nav2.launch.py")
        if self.slam_mode != "localization":
            return False, "위치추정 중이 아닙니다. 지도를 적용한 뒤 이동하세요"
        try:
            x, y, yaw = float(x), float(y), float(yaw)
        except (TypeError, ValueError):
            return False, "좌표가 올바르지 않습니다"
        # 지점이면 라이다로 잰 보정을 목표에 미리 얹는다 (한 번에 맞는 자리로)
        self._nav_poi = None
        if poi is not None:
            x, y, yaw = self._poi_apply(poi, x, y, yaw)

        g = self._nav_goal_msg(x, y, yaw)

        self._turn_reset()          # 이전 목표의 방향 보정 잔재를 치운다
        self._turn_ok_n = (self.TURN_OK if yaw_ok is None else float(yaw_ok))
        self._turn_max_n = (self.TURN_PASS_MAX if turn_max is None
                            else int(turn_max))
        self.nav_state = "sending"
        self.nav_msg = ""
        self.nav_target = (x, y, yaw)
        # ⚠ 목표를 다시 걸면 nav2 는 이전 목표를 '중단' 으로 끝낸다. 그
        #   결과가 새 목표를 보낸 뒤에 도착해서 nav_state 를 failed 로
        #   덮어썼고, 작업은 출발하자마자 실패했다 — 라이다 기준으로 설
        #   자리를 옮길 때마다 그랬다(2026-09-21: "왜 또 왓다갓다 생긴건데").
        #   목표마다 번호를 붙여, 지난 목표의 결과는 버린다.
        self.nav_seq = getattr(self, "nav_seq", 0) + 1
        seq = self.nav_seq
        self._nav_early_seq = None
        # 자리 맞추기(옆 어긋남 교정)는 지점 이동에서만. 렉 작업의 중간 목표는
        # 도착 뒤 자기가 다시 맞추므로 여기서 더 돌리지 않는다.
        self._nav_precise = bool(depart)
        if getattr(self, "_depart_timer", None) is not None:
            self._depart_stop()
            self._pub_zero()
        # 좁은 POI 곧게 가기 중이었으면 멈춘다 (np_goto 는 이 뒤에 상태를 새로 단다)
        if getattr(self, "_np_timer", None) is not None:
            self.np_cancel()
        else:
            self._np = None
        self._appr_lim = None               # 지난 목표의 POI 접근 감속도 (아래 _slow_release / _slow_start 가 다시 낸다)
        self._slow_release()                # 지난 목표의 출발 속도 제한을 푼다
        self._sp_fwd_out = False            # 직진 빠져나오기 표시는 이 목표 안에서만 (narrow_poi.sp_out)
        self._sp_out_total = 0.0            # 빠져나온 거리 (회전 공간이 생길 때까지 더 나온다)
        self.sp_cancel()                    # 경로 확인 중이던 POI 요청은 버린다
        # 충전소에 붙어 있으면 먼저 앞으로 곧게 나간다 (charger_dock.py, 2026-11-12)
        if self._cdock_leave_needed():
            p0 = self.tf_pose()
            if p0 is not None:              # 나오는 동안 나온 자리부터의 경로를 미리 받아 둔다
                self._out_prefetch(seq, x, y, yaw, p0, +1, self.CD_LEAVE_M)
            self._cdock_leave_start(g, seq, x, y, yaw, depart)
            return True, ("충전소에서 %.1fm 나간 뒤 목표로 갑니다 (%.2f, %.2f)"
                          % (self.CD_LEAVE_M, x, y))
        return self._nav_dispatch(g, seq, x, y, yaw, depart)

    # ⚠ 2026-11-17 사용자: "충전스테이션에서 나갈때도 휘청거리면서 가는 느낌". 충전소에서 0.5m 곧게 나온
    #   직후 아직 감속 중이라 _depart_ok 가 '움직이는 중' 으로 보고 정렬 회전을 건너뛰었다 -> 목표와 36도
    #   틀어진 채 nav2 가 달리며 꺾었다. 곧게 빠져나온 뒤(충전소 · 직선 도착 POI · 좁은 곳 후진)에는
    #   설 때까지 기다리고(최대 OUT_STILL_MAX_S), 목표가 DEPART_BEARING_OUT 넘게 틀어졌으면 먼저 돈다.
    # ⚠ 2026-11-17 사용자: "처음 출발할때 휘청거림이 심한거같음. 배터리 스테이션에서 나올때도". 10도 기준에
    #   6.8 · 8.7도 틀어진 채 회전 없이 nav2 로 넘겼고, 직진으로 나왔다고 천천히 시작까지 생략해 바로 가속하며
    #   꺾었다. 이제 목표 점이 아니라 nav2 경로가 처음 향하는 곳(DEPART_AIM_M 앞 경로 점)을 보고 3도 넘으면
    #   먼저 제자리에서 맞춘다. 천천히 시작 생략은 없앴다(DEPART_SKIP_AFTER_OUT).
    DEPART_BEARING_OUT = math.radians(3)
    DEPART_AIM_M = 0.6
    DEPART_SKIP_AFTER_OUT = False
    OUT_STILL_MAX_S = 1.5
    OUT_PLAN_MAX_S = 2.0
    # ⚠ 2026-11-18 사용자: "나갈때 직선으로 가다 제자리에서 회전하고 출발하는데 부드럽지 않은거 같아".
    #   실측(11:xx 왕복): 빠져나온 뒤 경로 기다리며 0.4~0.8초 정지, 회전 뒤 0.5초 고정 정지, 충전소에서 나와
    #   20도만 틀어져 있어도 멈춰 제자리 회전(약 2초). 이제
    #   - 경로는 빠져나오기 시작할 때 '나올 자리' 에서부터 미리 묻는다(_out_prefetch).
    #   - 경로 방향이 DEPART_CURVE_OUT 안이면 서지 않고 nav2 저속 출발로 곡선으로 붙는다
    #     (같은 날 서 있다가 -21.5도로 nav2 저속 출발한 기록: 회전 속도가 고르게 올라 흔들림 없음).
    #     11/17 6.8 · 8.7도 휘청임은 천천히 시작을 건너뛰었던 때 일이다(지금은 늘 천천히 시작).
    #   - 후진으로 빠져나온 경우(움직이는 방향이 뒤)는 설 때까지 기다린다.
    DEPART_CURVE_OUT = math.radians(25)
    OUT_PRE_FAR_M = 0.25           # 미리 받은 경로 시작이 지금 자리와 이보다 멀면 다시 묻는다

    def _out_prefetch(self, seq, x, y, yaw, p, d, dist):
        """곧게 빠져나오기 시작할 때: 나올 자리(p 에서 몸 방향 d 로 dist)부터 목표까지 경로를 미리 묻는다."""
        sx = p["x"] + d * dist * math.cos(p["yaw"])
        sy = p["y"] + d * dist * math.sin(p["yaw"])
        self._out_pre = {"seq": seq, "pl": self._depart_plan_request(x, y, yaw, start=(sx, sy))}

    def _dispatch_after_out(self, g, seq, x, y, yaw, depart):
        """곧게 빠져나온 뒤: 경로가 DEPART_CURVE_OUT 안이면 바로, 아니면 설 때까지 기다렸다 _nav_dispatch."""
        self._pub_zero()                    # 곧게 가기 지령은 끊는다 (nav2 가 이어받는다)
        old = getattr(self, "_out_wait_timer", None)
        if old is not None:
            self._out_wait_stop()
        self._out_wait = (g, seq, x, y, yaw, depart, time.monotonic())
        pre = getattr(self, "_out_pre", None)
        self._out_pre = None
        self._out_replanned = False
        if pre is not None and pre.get("seq") == seq and pre.get("pl") is not None:
            self._out_plan = pre["pl"]
        else:
            self._out_plan = self._depart_plan_request(x, y, yaw)
        self._out_wait_timer = self.create_timer(1.0 / self.NAV_WATCH_HZ, self._out_wait_tick)
        self._out_wait_tick()               # 미리 받은 경로가 있으면 지금 바로 보낸다

    def _depart_plan_request(self, x, y, yaw, start=None):
        """출발 방향을 잡으려고 목표까지 경로를 묻는다(start 없으면 지금 자리부터). {goal_f, res_f, path} 또는 None."""
        cli = getattr(self, "plan_cli", None)
        if cli is None or not cli.server_is_ready():
            return None
        from nav2_msgs.action import ComputePathToPose
        gp = ComputePathToPose.Goal()
        gp.goal.header.frame_id = "map"
        gp.goal.pose.position.x, gp.goal.pose.position.y = float(x), float(y)
        gp.goal.pose.orientation.z = math.sin(yaw / 2.0)
        gp.goal.pose.orientation.w = math.cos(yaw / 2.0)
        gp.planner_id = "GridBased"
        gp.use_start = start is not None
        if start is not None:
            gp.start.header.frame_id = "map"
            gp.start.pose.position.x, gp.start.pose.position.y = float(start[0]), float(start[1])
            gp.start.pose.orientation.w = 1.0
        try:
            return {"goal_f": cli.send_goal_async(gp), "res_f": None, "path": None, "done": False}
        except Exception:                                    # noqa: BLE001
            return None

    @staticmethod
    def _plan_poll(pl):
        """경로 요청을 한 번 들여다본다. 끝났으면 True (path 는 pl['path'])."""
        if pl is None or pl["done"]:
            return True
        try:
            if pl["res_f"] is None and pl["goal_f"].done():
                gh = pl["goal_f"].result()
                if gh is None or not gh.accepted:
                    pl["done"] = True
                    return True
                pl["res_f"] = gh.get_result_async()
            if pl["res_f"] is not None and pl["res_f"].done():
                r = pl["res_f"].result()
                if r is not None and r.status == 4:
                    pl["path"] = [(q.pose.position.x, q.pose.position.y) for q in r.result.path.poses]
                pl["done"] = True
        except Exception:                                    # noqa: BLE001
            pl["done"] = True
        return pl["done"]

    def _path_aim(self, path):
        """경로에서 지금 자리로부터 DEPART_AIM_M 떨어진 첫 점 (없으면 끝점). 모르면 None."""
        cur = self.tf_pose()
        if not path or cur is None:
            return None
        for px, py in path:
            if math.hypot(px - cur["x"], py - cur["y"]) >= self.DEPART_AIM_M:
                return (px, py)
        return path[-1]

    def _out_wait_stop(self):
        t = getattr(self, "_out_wait_timer", None)
        self._out_wait_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass

    def _out_wait_tick(self):
        g, seq, x, y, yaw, depart, t0 = self._out_wait
        if seq != getattr(self, "nav_seq", None) or self.estop:
            self._out_wait_stop()
            return
        with self.lock:
            od = self.odom
        still = (od is not None and abs(od.twist.twist.linear.x) < 0.01
                 and abs(od.twist.twist.angular.z) < 0.03)
        backing = od is not None and od.twist.twist.linear.x < -0.01
        el = time.monotonic() - t0
        pl = getattr(self, "_out_plan", None)
        planned = self._plan_poll(pl)
        if not planned and el < self.OUT_PLAN_MAX_S:
            self._pub_zero()
            return
        cur = self.tf_pose() if hasattr(self, "tf_buffer") else None
        path = pl.get("path") if pl else None
        if (path and cur is not None and not getattr(self, "_out_replanned", False)
                and math.hypot(path[0][0] - cur["x"], path[0][1] - cur["y"]) > self.OUT_PRE_FAR_M):
            # 미리 받은 경로가 다른 자리(나오기가 덜 됨 등)에서 시작한다 — 지금 자리에서 다시 묻는다
            self._out_replanned = True
            self._out_plan = self._depart_plan_request(x, y, yaw)
            self._out_wait = (g, seq, x, y, yaw, depart, time.monotonic())
            self._pub_zero()
            return
        aim = self._path_aim(path) if path else None
        curve, b = False, None
        if cur is not None:
            ax, ay = aim or (x, y)
            b = math.atan2(ay - cur["y"], ax - cur["x"]) - cur["yaw"]
            b = math.atan2(math.sin(b), math.cos(b))
            curve = abs(b) <= self.DEPART_CURVE_OUT and not backing
        if not curve and not still and el < self.OUT_STILL_MAX_S:
            self._pub_zero()
            return
        self._out_wait_stop()
        self._out_plan = None
        if curve:
            self.get_logger().info("빠져나온 뒤 경로와 %+.1f도 — 서서 돌지 않고 곡선으로 출발 (%.1f초)"
                                   % (math.degrees(b), el))
        self._depart_bearing_lim = self.DEPART_CURVE_OUT if curve else self.DEPART_BEARING_OUT
        self._depart_aim = aim
        self._sp_out_done = True             # 방금 빠져나왔다 - 또 빠져나오지 않는다
        try:
            self._nav_dispatch(g, seq, x, y, yaw, depart)
        finally:
            self._depart_bearing_lim = self.DEPART_BEARING
            self._depart_aim = None
            self._sp_out_done = False

    def _seq_chain(self, old):
        """같은 이동 안에서 nav2 목표를 새로 걸었다(번호가 올랐다) - 왕복이 '다른 명령' 으로 보지 않게.
        2026-11-17 직선 도착(경로 확인 뒤 출발 · 넘겨받기 · 반대편 입구)이 번호를 올렸다."""
        r = getattr(self, "route", None)
        if isinstance(r, dict) and r.get("seq") == old and old is not None:
            r["seq"] = self.nav_seq

    def _nav_dispatch(self, g, seq, x, y, yaw, depart):
        """nav_goto 뒷부분: 출발 기어가기 또는 nav2 로 보낸다 (충전소 나오기 뒤에도 여기로)."""
        # 2026-11-16 직선으로 들어간 POI 에서 출발하면 먼저 직선으로 빠져나온다 (narrow_poi.py SP_*)
        if depart and not getattr(self, "_sp_out_done", False):
            plan = self.sp_out_plan(x, y)
            if plan is not None:
                def _after(g=g, seq=seq, x=x, y=y, yaw=yaw, depart=depart):
                    if seq == getattr(self, "nav_seq", None):
                        self._sp_out_done = True
                        try:
                            self._dispatch_after_out(g, seq, x, y, yaw, depart)
                        finally:
                            self._sp_out_done = False
                self._out_prefetch(seq, x, y, yaw, plan[0], plan[1], plan[2])
                self.sp_out(plan, _after)
                return True, "직선으로 빠져나온 뒤 목표로 갑니다 (%.2f, %.2f)" % (x, y)
        # 출발 방향은 경로가 처음 향하는 점으로 (있으면, _out_wait_tick), 없으면 목표 점
        ax, ay = getattr(self, "_depart_aim", None) or (x, y)
        if depart and self.depart_mode == "nav2":
            # 전진은 처음부터 nav2 가 조향, 속도만 낮춘다. 다만 목표가 옆·뒤면
            # 먼저 제자리에서 정렬한다 — nav2 는 52도 안으로만 돌고 출발해 남은
            # 30도 넘게를 달리면서 꺾었다(2026-10-02 nav2 방식 시험).
            self._slow_seq = seq
            cur = self._depart_ok(x, y, turn=True, aim=(ax, ay))
            if cur is not None and not cur.get("turn") and not cur.get("noroom"):
                cur = None                  # 앞쪽이면 정렬 없이 바로 nav2
        else:
            cur = self._depart_ok(x, y, turn=True, aim=(ax, ay)) if depart else None
        if cur is not None and cur.get("noroom"):
            ext = self.sp_out_extend_plan()
            if ext is not None:
                def _after2(g=g, seq=seq, x=x, y=y, yaw=yaw, depart=depart):
                    if seq == getattr(self, "nav_seq", None):
                        self._dispatch_after_out(g, seq, x, y, yaw, depart)
                self.get_logger().info("빠져나왔는데 아직 돌 공간이 없음 — %.1fm 더 곧게" % ext[2])
                self._out_prefetch(seq, x, y, yaw, ext[0], ext[1], ext[2])
                self.sp_out(ext, _after2)
                return True, "돌 공간이 생길 때까지 곧게 더 나옵니다"
            last = self.np_back_out_needed()
            if last is not None:
                def _after(g=g, seq=seq, x=x, y=y, yaw=yaw, depart=depart):
                    if seq == getattr(self, "nav_seq", None):
                        self._dispatch_after_out(g, seq, x, y, yaw, depart)
                self.np_back_out(last, _after)
                return True, "좁은 곳에서 곧게 후진해 나온 뒤 목표로 갑니다 (%.2f, %.2f)" % (x, y)
            # ⚠ 2026-11-16 돌 자리가 없으면 nav2 에 넘기지 않는다 — nav2 도 제자리에서 돌다 닿는다.
            self.nav_state, self.nav_msg = "failed", "회전할 공간이 없습니다 — 주변을 비우거나 로봇을 옮겨 주세요"
            self.get_logger().warn("출발 실패: 목표가 옆·뒤인데 제자리 회전 공간이 없음 (양방향 · 앞뒤 %.0fcm 빠져도 막힘)"
                                   % (self.SPIN_PREMOVE_M * 100))
            return False, self.nav_msg
        if cur is not None:
            self._sp_fwd_out = False        # 제자리 회전·기어가기를 한다 — 회전 뒤에는 천천히 시작해야 한다
        if depart:
            # 출발 때 목표 쪽과 몇 도 틀어져 있었나 (20~30도 앞쪽 출발을 따로 보려고)
            p0 = self.tf_pose()
            if p0 is not None:
                b0 = math.atan2(ay - p0["y"], ax - p0["x"]) - p0["yaw"]
                self.get_logger().info("출발 방향 오차 %+.1f도%s"
                                       % (math.degrees(math.atan2(math.sin(b0), math.cos(b0))),
                                          " (경로 %.1fm 앞 점 기준)" % self.DEPART_AIM_M
                                          if getattr(self, "_depart_aim", None) else ""))
        self.get_logger().info("이동 목표 (%.2f, %.2f) %.0f도%s" % (
            x, y, math.degrees(yaw),
            ((" — 먼저 제자리 정렬%s" % self._plan_note(cur.get("plan")))
             if cur.get("turn") else " — 먼저 천천히 출발") if cur else
            (" — nav2 저속 출발" if depart and self.depart_mode == "nav2" else "")))
        if cur is not None:
            self._depart_goal, self._depart_seq = g, seq
            self._depart_xy, self._depart_yaw = (cur["x"], cur["y"]), cur["yaw"]
            self._depart_bearing = math.atan2(ay - cur["y"], ax - cur["x"])
            self._depart_t, self._depart_w_last = 0.0, 0.0
            self._depart_goal_xy = (ax, ay)
            plan = cur.get("plan") or {}
            self._depart_turn_dir = plan.get("dir", 0.0)
            self._depart_premove = plan.get("move", 0.0)
            self._depart_premove_xy = (cur["x"], cur["y"])
            self._depart_passes = 0
            self._depart_blocked_t = 0.0
            self._depart_wait_logged = False
            self._depart_phase = (("premove" if self._depart_premove else "turn")
                                  if cur.get("turn") else "creep")
            self.nav_msg = "천천히 출발하는 중"
            self._depart_timer = self.create_timer(1.0 / self.NAV_WATCH_HZ,
                                                   self._depart_tick)
        else:
            self._nav_send(g, seq)
            if depart and self.depart_mode == "nav2":
                self._slow_start()
        return True, "목표로 이동합니다 (%.2f, %.2f)" % (x, y)

    def _nav_accepted(self, fut, seq=None):
        if seq is not None and seq != getattr(self, "nav_seq", seq):
            # 이미 지난 목표다. 취소된 뒤에 받아들여졌으면 nav2 가 혼자 달리므로 끊는다.
            try:
                gh = fut.result()
                if gh.accepted:
                    gh.cancel_goal_async()
            except Exception:                                # noqa: BLE001
                pass
            return
        try:
            gh = fut.result()
        except Exception as e:                               # noqa: BLE001
            self.nav_state, self.nav_msg = "failed", str(e)
            return
        if not gh.accepted:
            self.nav_state, self.nav_msg = "failed", "nav2 가 목표를 거절했습니다"
            self.get_logger().warn(self.nav_msg)
            return
        self.nav_goal = gh
        self.nav_state = "active"
        gh.get_result_async().add_done_callback(
            lambda f: self._nav_done(f, seq))

    def _nav_done(self, fut, seq=None):
        if hasattr(self, "_appr_clear") and (seq is None or seq == getattr(self, "nav_seq", seq)):
            self._appr_clear()
        if seq is not None and seq != getattr(self, "nav_seq", seq):
            # 지난 목표의 결과다. 새 목표가 이미 달리고 있으므로 건드리지
            # 않는다 — 여기서 nav_goal 을 지우면 새 목표의 손잡이가 날아간다.
            self.get_logger().info("지난 이동 목표의 결과는 버린다")
            return
        self.nav_goal = None
        try:
            st = fut.result().status
        except Exception as e:                               # noqa: BLE001
            self.nav_state, self.nav_msg = "failed", str(e)
            return
        # action_msgs/GoalStatus: 4=SUCCEEDED 5=CANCELED 6=ABORTED
        early = (seq is not None and getattr(self, "_nav_early_seq", None) == seq)
        if st == 4 or (st == 5 and early):
            # 위치는 맞췄다. 방향은 nav2 가 느슨하게 두었으므로 여기서 잡는다.
            if self._final_turn():
                return              # 회전 중. 끝나면 _finish_arrival 이 마무리
            self._finish_arrival()
            return
        elif st == 5:
            self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
        else:
            self.nav_state, self.nav_msg = "failed", "이동에 실패했습니다"
        self.get_logger().info("이동 종료: %s" % self.nav_msg)
        self._nav_clear_path()

    def _nav_clear_path(self):
        self.nav_path = []

    # 도착 후 방향 보정.
    #
    # RPP 는 도착하면 따라갈 앞점이 발밑이 되어 방향을 못 맞춘다. 그래서
    # 위치는 nav2 가, 방향은 여기서 제자리 회전으로 맡는다.
    #
    # ⚠ nav2 의 Spin 을 쓰지 않는다. Spin 은 odom TF 를 보고 '시킨 각만큼
    #   돌았나' 를 재는 개루프다. TF 가 0.2 초쯤 늦게 오므로 최저 회전속도
    #   0.2 rad/s(11.5 도/s)에서 2.3 도를 더 돌고, 멈추는 감속에서 1.4 도가
    #   더 붙는다. 실제로 시킨 각과 무관하게 회전 방향으로 3~5 도씩 넘어갔고,
    #   세 번을 되풀이해도 1.4 도 안에 못 들어왔다(2026-09-14 측정:
    #   10.2도 시켜 15.0도, 4.8도 시켜 10.0도, 1.9도 시켜 6.8도).
    #
    #   그래서 맵 기준 닫힌 루프로 바꿨다. 핵심은 '돌고 - 멈추고 - 다시 재기'
    #   다. 멈춘 뒤에는 TF 가 늦어도 값이 맞으므로, 넘어간 만큼이 다음 회차의
    #   오차로 그대로 잡힌다.
    #
    # ⚠ 타이머는 이것 하나뿐이고 단계는 _turn_phase 로 넘긴다. 예전에
    #   회전용·대기용 타이머를 따로 만들다가, 주기 타이머를 일회성처럼 써서
    #   버려진 타이머가 계속 돌며 nav_state 를 덮어썼다. 움직이지도 않은
    #   목표가 '도착' 으로 끝났다(2026-09-14).

    # 목표 앞에서 nav2 를 먼저 끊는 거리 [m].
    #
    # ⚠ RPP 는 목표 5cm(xy_goal_tolerance) 안에 들어오면 목표 방향으로 제자리
    #   회전(0.6 rad/s)을 한다. StoppedGoalChecker 는 '멈춰 있어야' 도착이라
    #   돌고 있는 동안은 끝나지 않고, 그 회전이 끝난 뒤 아래 도착 후 보정이
    #   반대로 돌아 '왔다갔다' 가 됐다(2026-09-29 왕복 9회 중 2회, 한쪽으로
    #   34도/s 돌고 반대로 20도/s). 사용자: "한 번에 돌면서 멈춰야".
    #   그 회전이 시작되기 전(5cm 보다 조금 밖)에 목표를 취소하고 도착으로
    #   친다. 0.08 m/s 로 다가오므로 지연 동안 3~5cm 더 가 목표에 붙는다.
    #
    # ⚠ 0.06 -> 0.10 (같은 날). 5cm 제자리 회전은 막았지만, 그 앞 약 15cm
    #   부터 목표점이 코앞이 되며 RPP 곡률(2y/d^2)이 커져 지령이 34 도/s 까지
    #   올랐다(실제는 반경 제한으로 10 도/s). 그 곡선이 세지기 전에 끊는다.
    #   멈추는 자리는 목표에서 5~7cm 앞이 될 수 있다.
    NAV_ARRIVE_M  = 0.10
    NAV_WATCH_HZ  = 20.0

    # ---------------------------------------------------------- 주 실행기 멈춤 감시
    #
    # ⚠ 2026-11-23 재부팅 뒤 '저속 출발 중 위치(odom) 없음 (TF 가 1.5초 전 것)' 으로 왕복이 세 번 멈췄다. 밖에서 재면
    #   TF 는 0.02초 늦을 뿐 — api_node 의 TF 는 주 실행기(rclpy.spin)에서 받으므로, 어떤 콜백이 1초 넘게 붙잡고
    #   있으면 TF 가 쌓이지 않는다. 0.1초마다 표시를 남기고, STALL_WARN_S 넘게 안 바뀌면 그 순간 주 쓰레드가
    #   실행 중인 코드 위치를 남긴다. 로봇은 건드리지 않는다.
    STALL_WARN_S = 0.8

    def _hb_tick(self):
        self._hb = time.monotonic()

    # ---------------------------------------------------------- TF 낡음 계측 (2026-11-23, 원인 찾을 때까지)
    #   밖(다른 프로세스)에서는 odom TF 시각차 0.02초 이하인데 api_node 안에서만 1.5초 낡아 출발이 취소된다.
    #   TF 수신 콜백이 마지막으로 불린 때 · 받은 메시지 시각 · 그 순간 각 스레드가 하던 일을 남긴다. 로봇은 건드리지 않는다.
    TFW_WARN_S = 0.3

    def _tfw_setup(self):
        lis = getattr(self, "tf_listener", None)
        if lis is None:
            return
        self._tfw_rx = None            # (받은 시각 monotonic, 메시지 시각 초)
        self._tfw_n = 0
        orig = lis.callback

        def cb(data, _orig=orig):
            for t in data.transforms:
                if t.child_frame_id == "base_footprint" and t.header.frame_id == "odom":
                    self._tfw_rx = (time.monotonic(), t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
                    self._tfw_n += 1
            _orig(data)
        # 구독은 만들 때 콜백을 잡아 두므로 구독 객체의 콜백을 바꾼다
        try:
            lis.tf_sub.callback = cb
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("TF 계측 못 붙임: %s" % e)
            return
        threading.Thread(target=self._tfw_watch, daemon=True).start()

    def _tfw_watch(self):
        import sys as _sys
        import traceback as _tb
        bad = None
        while rclpy.ok():
            time.sleep(0.05)
            try:
                t = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                age = time.time() - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
            except Exception:                                # noqa: BLE001
                age = 99.0
            if age > self.TFW_WARN_S and bad is None:
                bad = (time.monotonic(), self._tfw_n)
                rx = self._tfw_rx
                rx_s = ("마지막 수신 %.2f초 전 (그 메시지 시각 %.2f초 전)"
                        % (time.monotonic() - rx[0], time.time() - rx[1])) if rx else "수신 없음"
                fr = _sys._current_frames()
                names = {th.ident: th.name for th in threading.enumerate()}
                stacks = []
                for tid, f in fr.items():
                    nm = names.get(tid, "?")
                    if tid == threading.get_ident():
                        continue
                    st = _tb.extract_stack(f)[-3:]
                    stacks.append("[%s] %s" % (nm, " <- ".join("%s:%d %s" % (os.path.basename(x.filename), x.lineno, x.name)
                                                               for x in reversed(st))))
                self.get_logger().warn("TF 낡음 %.2f초 — %s | 주 실행기 표시 %.2f초 전 | %s"
                                       % (age, rx_s, time.monotonic() - getattr(self, "_hb", time.monotonic()),
                                          " || ".join(stacks)[:1800]))
            elif age < 0.15 and bad is not None:
                self.get_logger().warn("TF 낡음 풀림 — %.2f초 동안, 그 사이 수신 %d개"
                                       % (time.monotonic() - bad[0], self._tfw_n - bad[1]))
                bad = None

    def _stall_watch(self):
        import sys as _sys
        import traceback as _tb
        main_id = threading.main_thread().ident
        bad_t0 = None
        while rclpy.ok():
            time.sleep(0.1)
            lag = time.monotonic() - getattr(self, "_hb", time.monotonic())
            if lag > self.STALL_WARN_S and bad_t0 is None:
                bad_t0 = time.monotonic() - lag
                fr = _sys._current_frames().get(main_id)
                where = " <- ".join("%s:%d %s" % (os.path.basename(f.filename), f.lineno, f.name)
                                    for f in reversed(_tb.extract_stack(fr)[-6:])) if fr else "?"
                self.get_logger().warn("주 실행기 %.1f초째 멈춤 — 지금: %s" % (lag, where))
            elif lag < 0.3 and bad_t0 is not None:
                self.get_logger().warn("주 실행기 멈춤 풀림 — 모두 %.1f초" % (time.monotonic() - bad_t0))
                bad_t0 = None

    # ---------------------------------------------------------- 막혀 섬
    #
    # ⚠ 2026-11-20 12:07 POI 1 -> 2 가던 중 nav2 충돌 감시(PolygonStop)가 10분 세워 왕복이 실패했다.
    #   어느 센서가 무엇을 봤는지 남는 기록이 없어 원인을 못 짚었다(라이다-지도 일치율은 그대로라
    #   카메라 쪽 추정). nav2 로 가는 중 STOP_DIAG_S 넘게 서 있으면 정지 영역 안 점을 센서별로 세어
    #   남기고 카메라 사진을 저장한다. STOP_ALERT_S 넘으면 상태 문구로 알린다 (RCS_LG 방식: 회피 없이
    #   정지, 20초 넘으면 사람을 부른다). 로봇을 움직이지는 않는다.
    STOP_ZONE = (-0.50, 0.60, 0.18)     # nav2.yaml PolygonStop 과 같게 (뒤, 앞, 반폭) [m]
    STOP_ZONE_PTS = 5                   # collision_monitor max_points 와 같게
    STOP_DIAG_S = 5.0
    STOP_ALERT_S = 20.0
    STOP_ALERT_EVERY_S = 60.0
    STOP_SNAP_KEEP = 100                # 저장해 둘 사진 수 (넘으면 오래된 것부터 지움)

    def _stop_zone_counts(self):
        """정지 영역 안 점: [(센서 이름, 점 수, 가장 가까운 (x, y) 또는 None)]."""
        x0, x1, hw = self.STOP_ZONE
        out = []
        for name, m, cam in self._obstacle_sources():
            if m is None:
                continue
            got = self._scan_xy(m, cam)
            if got is None:
                continue
            x, y = got
            k = (x >= x0) & (x <= x1) & (np.abs(y) <= hw)
            near = None
            if k.any():
                i = int(np.argmin(np.hypot(x[k], y[k])))
                near = (float(x[k][i]), float(y[k][i]))
            out.append((name, int(k.sum()), near))
        return out

    @staticmethod
    def _stop_counts_text(counts):
        if not counts:
            return "센서 자료 없음"
        return ", ".join("%s %d점%s" % (n, c, (" (가장 가까운 앞 %+.2f 옆 %+.2f)" % near) if near else "")
                         for n, c, near in counts)

    def _stop_snap(self, tag):
        """카메라 사진을 logs/stop_snap/ 에 남긴다. 저장한 파일 이름 목록."""
        d = os.path.expanduser("~/projects/UND_AMR/logs/stop_snap")
        try:
            os.makedirs(d, exist_ok=True)
            with self.lock:
                imgs = (("front", self.cam_jpeg), ("down", getattr(self, "cam_jpeg_down", None)))
            saved = []
            for k, jpg in imgs:
                if jpg:
                    fn = os.path.join(d, "%s_%s.jpg" % (tag, k))
                    with open(fn, "wb") as f:
                        f.write(jpg)
                    saved.append(os.path.basename(fn))
            files = sorted(os.listdir(d))
            for old in files[:max(0, len(files) - self.STOP_SNAP_KEEP)]:
                os.remove(os.path.join(d, old))
            return saved
        except OSError:
            return []

    def _stop_watch(self):
        dt = 0.5
        o = getattr(self, "odom", None)
        moving = (o is None or abs(o.twist.twist.linear.x) > 0.01
                  or abs(o.twist.twist.angular.z) > 0.02)
        driving = (self.nav_state == "active" and self.nav_goal is not None
                   and not getattr(self, "_turning", False))
        st = getattr(self, "_stopw", None)
        if moving or not driving:
            if st and st.get("logged"):
                self.get_logger().info("정지 감시: 다시 움직임 — %.0f초 서 있었음" % st["t"])
                if st.get("alerted") and self.nav_msg == st.get("alert_msg"):
                    self.nav_msg = st.get("msg0") or self.nav_msg
            self._stopw = None
            return
        if st is None:
            st = self._stopw = {"t": 0.0}
        st["t"] += dt
        if st["t"] >= self.STOP_DIAG_S and not st.get("logged"):
            st["logged"] = True
            counts = self._stop_zone_counts()
            tot = sum(c for _n, c, _p in counts)
            cur = self.tf_pose()
            snaps = self._stop_snap(time.strftime("stop_%m%d_%H%M%S"))
            self.get_logger().warn(
                "정지 감시: nav2 로 가는 중 %.0f초째 서 있음 — 자리 %s · 정지 영역 안 %s%s · 사진 %s"
                % (st["t"], ("(%.2f, %.2f, %.0f도)" % (cur["x"], cur["y"], math.degrees(cur["yaw"])))
                   if cur else "모름", self._stop_counts_text(counts),
                   "" if tot >= self.STOP_ZONE_PTS else " (영역이 비어 있음 — 충돌 감시 말고 다른 이유)",
                   ", ".join(snaps) or "없음"))
        if st["t"] >= self.STOP_ALERT_S:
            if not st.get("alerted"):
                st["alerted"], st["msg0"], st["last"] = True, self.nav_msg, -1e9
            st["alert_msg"] = "앞에 장애물 — %.0f초째 정지 (치워 주세요)" % st["t"]
            self.nav_msg = st["alert_msg"]
            if st["t"] - st["last"] >= self.STOP_ALERT_EVERY_S:
                st["last"] = st["t"]
                self.get_logger().warn("정지 감시: %.0f초째 막혀 있음 — %s"
                                       % (st["t"], self._stop_counts_text(self._stop_zone_counts())))

    def _nav_watch(self):
        if self.nav_state != "active" or self.nav_goal is None \
                or self.nav_target is None or getattr(self, "_turning", False):
            return
        if self.np_handoff():                # 직선 도착: 앞 자리에서 서지 않고 넘겨받기 (narrow_poi.py)
            return
        if getattr(self, "_nav_early_seq", None) == self.nav_seq:
            return                      # 이미 끊었다. 결과를 기다린다
        cur = self.tf_pose()
        if cur is None:
            return
        d = math.hypot(cur["x"] - self.nav_target[0], cur["y"] - self.nav_target[1])
        if d > self.NAV_ARRIVE_M:
            return
        # 결과(취소)가 오면 _nav_done 이 이 표시를 보고 도착으로 처리한다
        self._nav_early_seq = self.nav_seq
        self.get_logger().info("목표 %.0fmm 앞 — nav2 를 끊고 도착으로 처리" % (d * 1000))
        self.nav_goal.cancel_goal_async()
        self._pub_zero()

    # 끊은 뒤 목표점까지 곧게 기어 들어간다 (정밀 도착).
    #
    # ⚠ nav2 를 10cm 앞에서 끊으면 그만큼 모자라게 섰다. 사용자: "정확한
    #   위치에 서고 싶음"(2026-09-29). 멈추지 않고 그대로 목표점까지 곧게
    #   기어 들어가, 멈추는 데 드는 거리만큼 미리 끊는다. 목표점을 향한
    #   조향은 5cm 밖에서만(가까우면 곡률이 튄다 — RPP 가 그랬다), 그 안은
    #   곧게. nav2 충돌 감시를 거치지 않으므로 앞 장애물은 따로 본다.
    CREEP_V_MAX   = 0.06
    CREEP_V_MIN   = 0.03
    CREEP_KV      = 0.8       # v = KV x 남은 앞 거리
    # ⚠ 0.35 -> 0.12. 0.03 m/s 에서 끊은 뒤 3~4mm 만 더 갔다(2026-09-29 왕복,
    #   '남음 14~15mm' -> 최종 9~12mm). 추정값 0.35 로는 1cm 넘게 모자랐다.
    CREEP_LAG_S   = 0.12
    # ⚠ 0.05 -> 0.02, POS_TOL 0.015 -> 0.010 (같은 날). 좌우가 -2~-14mm 로
    #   한쪽에 남았다. 조향은 반경 0.5m 상한으로 묶여 있어 가까이서도 튀지 않는다.
    CREEP_STEER_M = 0.02      # 이 안에서는 조향하지 않는다
    # ⚠ 0.015 -> 0.005. POS_TOL(0.010)보다 크면 10~15mm 남은 재시도가 첫 주기에
    #   바로 멈춰 헛돌았다(2026-09-29 검토).
    CREEP_POS_OK  = 0.005     # 앞으로 이만큼 안 남았으면 기어가지 않는다
    CREEP_MAX_S   = 8.0
    # 멈춘 뒤 목표점까지 이보다 멀면 다시 들어간다. 옆으로 어긋났으면 목표점
    # 쪽으로 먼저 몸을 돌린다 — 곧게 기어서는 옆자리를 못 맞춘다(같은 날,
    # 10cm 앞 끊은 자리에서 좌우 -29/-53mm 가 그대로 남았다).
    POS_TOL       = 0.010
    POS_PASS_MAX  = 2
    FACE_LAT      = 0.010     # 좌우가 이 안(또는 앞 거리의 30% 안)이면 그냥 곧게
    # 목표점 쪽으로 몸을 돌리는 것은 남은 거리가 이보다 클 때만, 돌 각이 이
    # 안일 때만. 1~2cm 를 맞추려고 60도를 돌았다 돌아오거나, 지나친 몇 cm 를
    # 맞추려고 180도를 도는 것은 캐스터만 뒤집는다(2026-09-29 검토).
    FACE_MIN_M    = 0.02
    FACE_MAX      = math.radians(100)
    # ⚠ 2026-10-08 왕복 29회: 기어 들어간 뒤 좌우 -5~-15mm 가 매번 같은 쪽으로
    #   남아 최종 좌우 평균 -9mm(최대 -16). 20mm 넘을 때만 돌고 들어가 그때만
    #   +2~+3mm 로 맞았다. 그런데 이 도착은 끝에 어차피 165~175도를 돈다.
    #   목표점 쪽 75도가 그 회전과 같은 방향이면 회전을 둘로 나눌 뿐 더 돌지
    #   않는다 — 그때는 POS_TOL 부터 맞춘다. 대신 도착마다 약 4초 늘어난다.
    #   -> 오후 80회: 거리 평균 8.5 -> 3.9mm, 최대 16 -> 10mm, 구간 시간 그대로.
    # ⚠ 그래도 19/80 회가 좌우 6~10mm 로 남았다(10mm 아래는 안 맞춤). 10mm 딱
    #   경계는 '곧게' 로 분류돼 좌우를 못 고치는 재진입만 2번 했다(2026-10-08).
    #   같은 쪽으로 돌 때는 6mm 부터, 좌우 4mm 넘으면 곧게 말고 돌고 들어간다.
    POS_TOL_TURN  = 0.006
    FACE_LAT_TURN = 0.004

    def _pub_zero(self):
        """두 통로 모두에 0. base_node 는 우선 통로(/cmd_vel_noradius)가 0.3초
        안에 살아 있으면 /cmd_vel 을 무시하므로(2026-09-29), 정지는 둘 다 보낸다."""
        self.pub_twist.publish(Twist())
        self.pub_twist_free.publish(Twist())

    def _goal_offset(self):
        """목표점의 로봇 기준 (앞, 왼쪽) [m]. 못 구하면 None."""
        if self.nav_target is None:
            return None
        cur = self.tf_pose()
        if cur is None:
            return None
        dx = self.nav_target[0] - cur["x"]
        dy = self.nav_target[1] - cur["y"]
        c, s_ = math.cos(cur["yaw"]), math.sin(cur["yaw"])
        return c * dx + s_ * dy, -s_ * dx + c * dy

    TURN_TOL      = 0.012     # 약 0.7도. 이만큼 남으면 지령을 끊는다
    TURN_LAG_S    = 0.25      # 지령을 끊고도 더 도는 시간. 그만큼 미리 끊는다
    TURN_OK       = 0.035     # 약 2.0도. 가라앉은 뒤 이 안이면 합격
    # ⚠ 2026-11-16 0.35 -> 0.50: 제자리 회전에서 바퀴 속도가 명령과 상관없이 ±0.7rpm 쯤 출렁여(드라이브
    #   이득 kp 35~150 · ki 16~128 로 바꿔도 그대로), 느릴수록 끊겨 보였다 — 0.3 rad/s 20~26%, 0.6 rad/s
    #   12~14%(같은 날 같은 자리 실측). 빨리 돌수록 비율이 준다.
    TURN_W_MAX    = 0.50      # rad/s
    # 2026-11-18 사용자: '충전소에는 렉을 실고 가지 않아' - 충전소로 가는 출발 정렬 회전만 이만큼(POI 1 출발 166도 7.9초)
    TURN_W_MAX_NORACK = 0.70
    TURN_W_MIN    = 0.10      # 1 rpm. 그 아래 지령은 정수 rpm 에 뭉개진다
    # 회전 시작을 이만큼에 걸쳐 0 -> TURN_W_MAX 로 올린다 [s].
    # ⚠ 0 -> 20도/s 를 한 번에 주자 바퀴가 늦게 따라오다 0.4초 뒤 25~27도/s
    #   (엔코더)로 넘쳤고 차체는 캐스터가 보태 32~36도/s 로 '휙' 돌았다
    #   (2026-09-30 왕복 기록, 사용자 "회전이 너무 심함").
    TURN_RAMP_S   = 1.0
    TURN_KP       = 1.8
    TURN_HZ       = 20.0
    TURN_SPIN_MAX = 15.0      # 한 회차가 이보다 길면 뭔가 막힌 것이다
    TURN_PASS_MAX = 4         # '돌고 가라앉기' 최대 횟수
    TURN_SETTLE_S = 1.2       # 멈춘 뒤 가라앉기를 기다리는 시간
    # ⚠ 2026-11-20 POI 에 곧게 들어온 뒤 반대로 도는 도착(1↔2 147회)이 진입 끝 -> 도착 완료 평균 10.5초였다:
    #   고정 대기 1.2초 + 좁은 곳 속도 0.45 rad/s 로 177도 + 고정 대기 1.2초. 곧게 들어온 뒤에는 nav2 가 섞이지
    #   않으므로 선 것이 확인되면 TURN_SETTLE_MIN_S 만 기다리고, 열린 POI 에서 렉이 없으면(리프트 bottom)
    #   TURN_W_OPEN_NORACK 로 돈다. 렉을 들었거나 상태를 모르면 예전 그대로.
    TURN_SETTLE_MIN_S = 0.3
    TURN_W_OPEN_NORACK = 0.70

    def _turn_wmax(self):
        """도착 회전 최대 속도 [rad/s]."""
        if not getattr(self, "_np_turning", False):
            return self.TURN_W_MAX
        if getattr(self, "_np_mode", "narrow") == "open" and getattr(self, "lift", None) == "bottom":
            return self.TURN_W_OPEN_NORACK
        return self.NP_TURN_W

    def _turn_reset(self):
        """방향 보정을 깨끗이 치운다. 새 목표를 받을 때 반드시 부른다."""
        t = getattr(self, "_turn_timer", None)
        self._turn_timer = None
        if t is not None:
            try:
                t.cancel()                       # 먼저 잠재운다
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass
            self._pub_zero()      # 돌던 중이었으면 세운다
        self._turning = False
        self._turn_pass = 0
        self._pos_pass = 0
        self._face_yaw = None
        self._np_turning = False     # 취소되면 다음 일반 도착 회전에 좁은 곳 설정이 남지 않게

    def _yaw_err(self):
        """목표 방향까지 남은 각도 [rad]. 못 구하면 None."""
        if self.nav_target is None:
            return None
        cur = self.tf_pose()
        if cur is None:
            return None
        # 목표점을 향해 몸을 돌리는 중이면 그 방향이 목표
        face = getattr(self, "_face_yaw", None)
        d = (face if face is not None else self.nav_target[2]) - cur["yaw"]
        return math.atan2(math.sin(d), math.cos(d))

    def _final_turn(self):
        """방향 보정을 시작한다. 걸었으면 True."""
        if getattr(self, "_turning", False):
            return True                      # 이미 도는 중. 겹쳐 걸지 않는다
        err = self._yaw_err()
        if err is None:
            return False
        self._turning = True
        self._turn_pass = 0
        self._pos_pass = 0
        self._face_yaw = None
        _c = self.tf_pose()
        self._turn_start_yaw = _c["yaw"] if _c else None
        self._turn_backing = False
        self._turn_seq = getattr(self, "nav_seq", 0)
        # 멈추기 직전에 돌던 방향 (가라앉은 뒤에는 자이로가 0 이라 지금 잰다)
        g = getattr(self, "_gyro_z", 0.0)
        fresh = time.monotonic() - getattr(self, "_gyro_t", 0.0) < 0.3
        self._turn_hint = (1.0 if g > 0 else -1.0) if fresh and abs(g) > 0.03 else 0.0
        # 가라앉기부터 시작한다. 두 가지를 같이 노린다. 하나는 주행 직후
        # 비틀린 캐스터가 풀리기를 기다리는 것, 다른 하나는 nav2 의
        # velocity_smoother 가 조용해지기를 기다리는 것이다. 그게 멈추기
        # 전에는 collision_monitor 가 /cmd_vel 에 0 을 계속 얹어 우리 지령과
        # 번갈아 나간다.
        self._turn_phase = "creep"
        if (getattr(self, "cdock", None) or {}).get("state") == "moving" and getattr(self, "CD_SKIP_CREEP", False):
            self._turn_phase = "settle"      # 충전소 대기 자리: 기어 들어가지 않는다 (charger_dock.CD_SKIP_CREEP)
        self._turn_t = 0.0
        self._turn_timer = self.create_timer(1.0 / self.TURN_HZ, self._turn_tick)
        return True

    def _turn_tick(self):
        dt = 1.0 / self.TURN_HZ
        self._turn_t += dt
        # 새 목표가 들어왔으면 이 도착 동작은 끝났다(상태는 새 목표 것이니 건드리지 않는다)
        if getattr(self, "_turn_seq", None) is not None \
                and self._turn_seq != getattr(self, "nav_seq", self._turn_seq):
            self._turn_reset()
            return
        # 비상정지·수동·라이다 끊김이면 즉시 멈춘다. 이 동작은 nav2 를 거치지 않는다.
        if self.estop or self.control_mode != "auto" or getattr(self, "_scan_dead", False):
            self._turn_reset()
            self._pub_zero()
            self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
            return

        if self._turn_phase == "creep":
            off = self._goal_offset()
            fx, fy = off if off is not None else (0.0, 0.0)
            v = max(self.CREEP_V_MIN, min(self.CREEP_V_MAX, self.CREEP_KV * fx))
            stop = (off is None
                    or (self._turn_t <= dt * 1.5 and fx <= self.CREEP_POS_OK)
                    or fx <= v * self.CREEP_LAG_S
                    or abs(fy) > 2.0 * fx    # 옆에 있으면 곧게 가서는 못 닿는다
                    or self._turn_t > self.CREEP_MAX_S
                    or self._exit_front_blocked())
            if stop:
                if off is not None and self._turn_t > dt * 1.5:
                    self.get_logger().info("정밀 도착: 기어 들어간 뒤 앞 %.0fmm 좌우 %+.0fmm 남음"
                                           % (fx * 1000, fy * 1000))
                self._pub_zero()
                self._turn_phase = "settle"
                self._turn_t = 0.0
                if getattr(self, "_pos_pass", 0) == 0 and getattr(self, "_face_yaw", None) is None:
                    self._poi_snap_async(getattr(self, "nav_seq", None))
                return
            d = math.hypot(fx, fy)
            w = 0.0
            if d > self.CREEP_STEER_M:
                w = 2.0 * v * fy / (d * d)
                w = max(-v / 0.5, min(v / 0.5, w))   # 반경 0.5m 보다 급하게는 안 꺾는다
            t = Twist()
            t.linear.x = float(v)
            t.angular.z = float(w)
            # nav2 가 끊긴 뒤 /cmd_vel 에 남는 0 과 섞이지 않게 우선 통로로
            self.pub_twist_free.publish(t)
            return

        if self._turn_phase == "settle":
            if getattr(self, "_np_turning", False):
                # 곧게 들어온 뒤: 선 것이 확인되면 짧게 (TURN_SETTLE_MIN_S 주석)
                if self._turn_t < self.TURN_SETTLE_MIN_S or (
                        self._turn_t < self.TURN_SETTLE_S and not self._poi_still()):
                    return
            elif self._turn_t < self.TURN_SETTLE_S:
                return
            if getattr(self, "_poi_pending", False) and self._turn_t < self.TURN_SETTLE_S + 3.0:
                return                  # 라이다로 자리를 재는 중 (멈춘 뒤 0.3~1초)
            # --- 위치 먼저 ---
            if getattr(self, "_face_yaw", None) is not None:
                self._face_yaw = None        # 목표점 쪽으로 돌았다. 이제 곧게 간다
                self._turn_phase = "creep"
                self._turn_t = 0.0
                return
            off = self._goal_offset()
            if (off is not None and getattr(self, "_nav_precise", False)
                    and getattr(self, "_pos_pass", 0) < self.POS_PASS_MAX):
                fx, fy = off
                a = math.atan2(fy, fx)
                left = self._yaw_err()          # 마지막에 돌아야 할 각
                # 목표점 쪽으로 도는 것이 마지막 회전의 일부인가 (그러면 공짜다)
                same = left is not None and a * left > 0 and abs(left) >= abs(a)
                tol = self.POS_TOL_TURN if same else self.POS_TOL
                if math.hypot(fx, fy) > tol:
                    self._pos_pass = getattr(self, "_pos_pass", 0) + 1
                    self.nav_msg = "자리를 맞추는 중"
                    lat = self.FACE_LAT_TURN if same else self.FACE_LAT
                    if fx > 0 and abs(fy) <= max(lat, 0.3 * fx):
                        self._turn_phase = "creep"
                        self._turn_t = 0.0
                        self.get_logger().info("정밀 도착 %d회차: 앞 %.0fmm 좌우 %+.0fmm — 곧게"
                                               % (self._pos_pass, fx * 1000, fy * 1000))
                        return
                    cur = self.tf_pose()
                    face_min = tol if same else self.FACE_MIN_M
                    if (cur is not None and math.hypot(fx, fy) > face_min
                            and abs(a) <= self.FACE_MAX):
                        self._face_yaw = cur["yaw"] + math.atan2(fy, fx)
                        a = math.atan2(fy, fx)
                        self.get_logger().info("정밀 도착 %d회차: 앞 %.0fmm 좌우 %+.0fmm — "
                                               "목표점 쪽으로 %.0f도 돌고 들어감"
                                               % (self._pos_pass, fx * 1000, fy * 1000,
                                                  math.degrees(a)))
                        self._turn_phase = "spin"
                        d = self._turn_pick_dir(a, 1.0 if a > 0 else -1.0)
                        if d is None:
                            self._turn_blocked_end("회전할 공간이 없어 도착 보정을 멈췄습니다")
                            return
                        self._turn_dir = d
                        self._turn_t = 0.0
                        return
            err = self._yaw_err()
            if err is None:
                self._turn_stop("자세를 알 수 없어 방향 보정을 멈췄습니다")
                return
            if abs(err) <= getattr(self, "_turn_ok_n", self.TURN_OK):
                self._turn_stop()
                return
            if self._turn_pass >= getattr(self, "_turn_max_n",
                                          self.TURN_PASS_MAX):
                self._turn_stop()
                return
            self._turn_pass += 1
            self.nav_msg = "방향을 맞추는 중 (%.1f도)" % math.degrees(err)
            self.get_logger().info("도착 후 방향 보정 %d회차: %.2f도"
                                   % (self._turn_pass, math.degrees(err)))
            self._turn_phase = "spin"
            self._turn_dir = 1.0 if err > 0 else -1.0
            # 거의 반대쪽(150도 넘게)이면 어느 쪽으로 돌아도 비슷하다. 멈추기
            # 직전까지 돌던 방향으로 이어서 돈다 — 반대로 돌면 '왔다갔다' 로
            # 보였다(2026-09-29).
            hint = getattr(self, "_turn_hint", 0.0)
            if abs(err) > math.radians(150) and hint and self._turn_pass == 1:
                self._turn_dir = hint
            d = self._turn_pick_dir(err, self._turn_dir)
            if d is None:
                self._turn_blocked_end("회전할 공간이 없어 방향 보정을 멈췄습니다")
                return
            self._turn_dir = d
            self._turn_t = 0.0
            return

        # --- 도는 중 ---
        #
        # ⚠ 한 번 돌기 시작하면 그 방향으로만 돈다. 처음에는 오차 부호를 그대로
        #   따라갔는데, 목표를 지나치는 순간 부호가 뒤집혀 제자리에서 되돌았다.
        #   TF 가 늦게 오니 되돌면서 또 지나치고, 그래서 좌우로 흔들렸다
        #   (2026-09-14). 지나쳤으면 멈추고, 가라앉힌 뒤 다시 재서 판단한다.
        err = self._yaw_err()
        # ⚠ 2026-11-16 회전 시간 한도는 돌 각도에 맞춘다. 15초 고정이면 좁은 곳(0.20 rad/s)에서 180도가
        #   16초라 164도에서 끊기고, 서서 다시 재고 나머지 13도를 돌아 '뚝 끊겨' 보였다.
        wmax = self._turn_wmax()
        if self._turn_t <= 1.5 / self.TURN_HZ or getattr(self, "_turn_err0", None) is None:
            self._turn_err0 = abs(err) if err is not None else 0.0     # 이번 회차 시작 각도
        lim = max(self.TURN_SPIN_MAX, self._turn_err0 / max(wmax, 0.05) * 1.5
                  + self.TURN_RAMP_S + 2.0)
        if err is None or self._turn_t > lim:
            self._pub_zero()
            self._turn_phase = "settle"
            self._turn_t = 0.0
            return
        w = self.TURN_KP * err
        # 2026-11-16 좁은 POI(통로 안)에서 돌 때는 천천히 — 회전 중심이 덜 밀린다 (여유 약 4cm)
        wmax = self._turn_wmax()
        cap = max(self.TURN_W_MIN, wmax * min(1.0, self._turn_t / self.TURN_RAMP_S))
        w = max(-cap, min(cap, w))
        if abs(w) < self.TURN_W_MIN:
            w = math.copysign(self.TURN_W_MIN, w)
        # 처음 정한 방향으로 남은 각. 음수면 이미 지나친 것이다.
        rem = err * self._turn_dir
        # ⚠ ±180도 근처에서 오차가 반대 부호로 넘어간다. 멈출 때 살짝 되튕기기만
        #   해도 -179.5 가 +179.6 이 되어 '지나쳤다' 로 보고 한 바퀴를 버렸다
        #   (2026-09-29, 1회차 -179.5도 직후 2회차 -177.3도). 반 바퀴 넘게
        #   지나친 것은 있을 수 없으니 이어 붙인다.
        if rem < -math.pi / 2.0:
            rem += 2.0 * math.pi
        # 지령을 끊어도 TURN_LAG_S 만큼은 더 돈다. 그만큼 미리 끊는다.
        if rem <= max(self.TURN_TOL, abs(w) * self.TURN_LAG_S):
            self._pub_zero()
            self._turn_phase = "settle"
            self._turn_t = 0.0
            return
        # 2026-11-16 남은 회전이 쓸고 갈 영역에 물체(라이다·카메라)가 0.3초 넘게 계속 들어오면
        #   멈추고 다시 판단한다. 한두 번 튄 점에는 서지 않는다(출발 회전과 같은 기준).
        _pad = self.NP_TURN_PAD if getattr(self, "_np_turning", False) else None
        nh = self._sweep_hits(self._obstacle_pts(), self._turn_dir * max(rem, 0.05), pad=_pad)
        if nh >= self.SPIN_PTS:
            self._turn_block_t = getattr(self, "_turn_block_t", 0.0) + 1.0 / self.TURN_HZ
            if self._turn_block_t >= self.DEPART_BLOCK_CONFIRM_S:
                self._turn_block_t = 0.0
                self._pub_zero()
                self.get_logger().info("도착 회전: 돌 자리에 물체 %d점 (%s) — 멈추고 다시 판단"
                                       % (nh, self._sweep_sources(self._turn_dir * max(rem, 0.05))))
                self._turn_phase = "settle"
                self._turn_t = 0.0
                return
        else:
            self._turn_block_t = 0.0
        t = Twist()
        t.angular.z = float(self._turn_dir * abs(w))
        self.pub_twist.publish(t)

    def _turn_blocked_end(self, note):
        """도착 회전이 막혔다. 돌기 시작한 방향에서 많이 틀어졌으면 그 방향으로 되돌아 선다.

        ⚠ 2026-11-16 폭 95cm 통로에서 180도 돌다 53도에서 막혀 비스듬히 남았다 — 그러면
          곧게 후진해 나오지도 못한다. 지나온 영역은 비어 있으니 되돌아 들어온 방향으로 선다.
        """
        cur = self.tf_pose()
        y0 = getattr(self, "_turn_start_yaw", None)
        if (not getattr(self, "_turn_backing", False) and cur is not None and y0 is not None
                and abs(math.atan2(math.sin(cur["yaw"] - y0), math.cos(cur["yaw"] - y0)))
                > math.radians(3.0)):
            self._turn_backing = True
            gx, gy = self.nav_target[0], self.nav_target[1]
            self.nav_target = (gx, gy, y0)
            self._pos_pass = self.POS_PASS_MAX       # 자리 맞추기는 하지 않는다
            self._face_yaw = None
            self._turn_pass = 0
            self._turn_phase = "settle"
            self._turn_t = 0.0
            self.nav_msg = "회전할 공간이 없어 들어온 방향으로 되돌리는 중"
            self.get_logger().warn("도착 회전: 공간 부족 — 들어온 방향(%.0f도)으로 되돌려 섬"
                                   % math.degrees(y0))
            return
        self._turn_stop("회전할 공간이 없어 들어온 방향으로 서 있습니다"
                        if getattr(self, "_turn_backing", False) else note)

    def _turn_stop(self, note=None):
        t = getattr(self, "_turn_timer", None)
        self._turn_timer = None
        if t is not None:
            try:
                t.cancel()                       # 자기 콜백 안에서 없애므로
                self.destroy_timer(t)            # 잠재우는 것이 먼저다
            except Exception:                                # noqa: BLE001
                pass
        self._pub_zero()
        self._turning = False
        if getattr(self, "_turn_seq", None) is not None \
                and self._turn_seq != getattr(self, "nav_seq", self._turn_seq):
            return                      # 그 사이 새 목표가 들어왔다
        if note is None and getattr(self, "_turn_backing", False):
            note = "회전할 공간이 없어 들어온 방향으로 서 있습니다"
        self._turn_backing = False
        self._np_turning = False
        self._finish_arrival(note)

    def _finish_arrival(self, note=None):
        # 좁은 POI 의 앞 자리에 도착했으면 여기서 끝내지 않고 곧게 들어가기로 넘어간다
        if self.np_on_arrival():
            self._turn_reset()
            return
        err = self._yaw_err()
        cur = self.tf_pose() if self.nav_target is not None else None
        if cur is not None:
            gx, gy, gyaw = self.nav_target
            dx, dy = cur["x"] - gx, cur["y"] - gy
            # 목표 방향 기준 앞뒤(+ 지나침)·좌우(+ 왼쪽)
            along = math.cos(gyaw) * dx + math.sin(gyaw) * dy
            side = -math.sin(gyaw) * dx + math.cos(gyaw) * dy
            self.nav_pos_err = (along, side)
            self.get_logger().info("도착 위치 오차: 앞뒤 %+.0fmm 좌우 %+.0fmm (거리 %.0fmm)"
                                   % (along * 1000, side * 1000, math.hypot(dx, dy) * 1000))
        if note:
            self.nav_msg = "목표에 도착했습니다 (%s)" % note
        elif err is not None:
            self.nav_msg = "목표에 도착했습니다 (방향 오차 %.1f도)" % math.degrees(err)
        else:
            self.nav_msg = "목표에 도착했습니다"
        self.nav_state = "done"
        self._turn_reset()
        self._poi_rest_async(getattr(self, "nav_seq", None))

    # ---------------------------------------------------------- 지점 스캔 보정
    #
    # ⚠ 2026-10-14 무한 왕복 1시간을 라이다 스캔끼리 맞춰 따로 재 보니(tools/
    #   repeat_probe.py) 실제로 선 자리가 처음 자리에서 A 평균 15mm(최대 43),
    #   B 평균 26mm(최대 51) 벗어났다. 그동안 '도착 위치 오차' 는 1~5mm 였다.
    #   SLAM 위치가 1시간에 약 4cm 한쪽으로 흘렀다(B 좌우 -9 -> -49mm). 10/08 에
    #   api_node 를 다시 띄울 때마다 10cm 옮겨진 것도 같은 현상이다.
    #
    # 그래서 지점마다 처음 정확히 섰을 때의 스캔을 기준으로 남기고(지도별 파일),
    #   - 도착해 서 있을 때마다 기준과 맞춰 '실제로 서야 할 자리' 를 지금 SLAM
    #     좌표로 구해 둔다 (움직이지 않는다, 다음 방문 때 쓴다)
    #   - 다음에 그 지점으로 갈 때 그 자리를 처음부터 nav2 목표로 보낸다 —
    #     도착 뒤 따로 맞추는 동작 없이 한 번에 들어간다 (사용자 요청)
    #   - 보정값이 없거나 낡았으면(재시작 직후 등) nav2 를 끊는 순간 한 번 재서
    #     1cm 넘게 다르면 그때 목표점을 고친다 (기어 들어가기가 받아 준다)
    # SLAM 흐름은 느리다(1시간 4cm, 왕복 1회 약 1mm) — 지난 방문 값으로 충분하다.
    # ⚠ 2026-10-14: 보정을 얹어도 실제 흩어짐이 그대로였다(A 22, B 16mm) — SLAM
    #   위치가 설 때마다 15~20mm 튀어 지난 방문 값이 다음 방문에 안 맞는다.
    #   사용자 선택으로 SLAM 쪽을 먼저 고친다. 그동안 목표는 옮기지 않고
    #   도착마다 '실제 도착 오차(라이다)' 를 재서 로그로만 남긴다 (효과 확인용).
    POI_SCAN_CORRECT = False
    POI_SCAN_DIR     = "poi_scans"
    POI_REF_TOL      = 0.005    # SLAM 기준 이만큼 안에 섰을 때만 기준 스캔을 찍는다
    POI_INLIER_MIN   = 0.70     # 3cm 안에 든 점 비율. 사람·물건이 섞이면 떨어진다
    POI_FIT_MAX      = 0.015    # 잔차 중앙값 [m]
    POI_CORR_MAX     = 0.08     # 이보다 크게 옮기라면 잘못 맞춘 것으로 본다 [m]
    # 방향은 보정하지 않는다 — 도착 방향 맞추기 허용치(±2도)만큼 매번 다르게 서서
    # 라이다로 잰 방향 차이가 +0.9 / -3.5 / -1.7도로 흩어졌다(2026-10-14). 그걸
    # 보정으로 얹으면 흔들림을 키운다. 이 값은 '잘못 맞춘 것' 을 거르는 데만 쓴다.
    POI_CORR_MAX_YAW = math.radians(6.0)
    POI_CORR_STALE_S = 900.0    # 미리 얹는 보정값의 최대 나이
    POI_SNAP_M       = 0.010    # 끊는 순간 잰 값이 이만큼 넘게 다르면 목표를 고친다

    def _poi_path(self, poi):
        def safe(t):
            return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in str(t))
        return os.path.join(self.map_dir, self.POI_SCAN_DIR,
                            "%s__%s.json" % (safe(self.active_map), safe(poi["name"])))

    def _poi_ref(self, poi):
        """그 지점의 기준 스캔. 지점을 옮겼으면(좌표가 다르면) 없는 것으로 본다."""
        path = self._poi_path(poi)
        cache = self.__dict__.setdefault("_poi_refs", {})
        ref = cache.get(path)
        if ref is None:
            try:
                with open(path) as f:
                    d = json.load(f)
                ref = {"poi": tuple(d["poi"]), "slam": tuple(d["slam"]),
                       "pts": np.asarray(d["pts"], dtype=float)}
                ref["n"] = scan_icp.normals(ref["pts"])
            except (OSError, ValueError, KeyError):
                return None
            cache[path] = ref
        px, py, pyaw = ref["poi"]
        if (math.hypot(px - poi["x"], py - poi["y"]) > 0.001
                or abs(math.atan2(math.sin(pyaw - poi["yaw"]),
                                  math.cos(pyaw - poi["yaw"]))) > 0.002):
            return None
        return ref

    def _poi_apply(self, poi, x, y, yaw):
        """지난 방문에서 구해 둔 '실제로 서야 할 자리' 가 있으면 그리로 보낸다."""
        self._nav_poi = {"poi": poi, "path": self._poi_path(poi)}
        if not self.POI_SCAN_CORRECT:
            return x, y, yaw
        corr = self.__dict__.setdefault("_poi_corr", {}).get(self._nav_poi["path"])
        if corr is None or time.time() - corr[1] > self.POI_CORR_STALE_S \
                or self._poi_ref(poi) is None:
            return x, y, yaw
        cx, cy, cyaw = corr[0]
        self.get_logger().info("지점 %s: 라이다 보정 적용 (%+.0f, %+.0f mm, %+.2f도)"
                               % (poi["name"], (cx - x) * 1000, (cy - y) * 1000,
                                  math.degrees(cyaw - yaw)))
        return cx, cy, cyaw

    def _poi_laser(self, frame):
        lt = self.__dict__.setdefault("_poi_laser_tf", {})
        if frame not in lt:
            p = self.tf_pose("base_footprint", frame)
            if p is None:
                return None
            lt[frame] = (p["x"], p["y"], p["yaw"])
        return lt[frame]

    def _poi_snapshot(self, after=0.0, step=4):
        """최신 스캔(after 이후 찍힌 것)과 그 순간의 SLAM 자세. 없으면 None."""
        with self.lock:
            m = self.scan
        if m is None:
            return None
        st_ = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st_ < after:
            return None
        las = self._poi_laser(m.header.frame_id)
        slam = self.tf_pose_at(m.header.stamp)
        # ⚠ 기어 들어가 멈춘 직후에는 SLAM 이 지도를 다시 계산하느라 그 시각의
        #   map 좌표를 늦게 낸다 — 멈춰서 재기가 매번 실패했다(2026-10-14).
        #   완전히 서 있으면 최신 자세가 곧 그 시각 자세다.
        if slam is None and self._poi_still():
            slam = self.tf_pose()
        if las is None or slam is None:
            return None
        return (scan_icp.scan_points(m, las, step=step),
                (slam["x"], slam["y"], slam["yaw"]))

    def _poi_measure(self, ref, snap):
        """기준과 맞춰 '실제로 서야 할 자리'(지금 SLAM 좌표)와 실제 도착 오차를 낸다."""
        pts, slam = snap
        guess = scan_icp.relative(ref["slam"], slam)
        real, fit, inl = scan_icp.icp(pts, ref["pts"], guess, dst_n=ref.get("n"))
        if inl < self.POI_INLIER_MIN or fit > self.POI_FIT_MAX:
            return None, "스캔이 잘 안 맞음 (맞음 %.0f%%, 잔차 %.0fmm)" % (inl * 100, fit * 1000)
        g_in_ref = scan_icp.relative(ref["slam"], ref["poi"])
        cg = scan_icp.corrected_goal(slam, real, g_in_ref)
        px, py, pyaw = ref["poi"]
        dyaw = math.atan2(math.sin(cg[2] - pyaw), math.cos(cg[2] - pyaw))
        cg = (cg[0], cg[1], pyaw)               # 위치만 보정한다
        if math.hypot(cg[0] - px, cg[1] - py) > self.POI_CORR_MAX \
                or abs(dyaw) > self.POI_CORR_MAX_YAW:
            return None, "보정이 너무 큼 (%.0fmm, %.1f도)" % (
                math.hypot(cg[0] - px, cg[1] - py) * 1000, math.degrees(dyaw))
        # 실제 도착 오차: 목표 자세에서 본 지금 로봇 (앞, 왼쪽)
        err = scan_icp.relative(g_in_ref, real)
        return {"goal": cg, "err": err, "inl": inl}, None

    def _poi_thread(self, fn, *a):
        threading.Thread(target=fn, args=a, daemon=True).start()

    def _poi_snap_async(self, seq):
        np_ = getattr(self, "_nav_poi", None)
        if not self.POI_SCAN_CORRECT or np_ is None or self._poi_ref(np_["poi"]) is None:
            return
        self._poi_pending = True
        self._poi_thread(self._poi_snap_guard, seq, np_)

    def _poi_snap_guard(self, seq, np_):
        try:
            self._poi_snap(seq, np_, None)
        finally:
            self._poi_pending = False

    def _poi_snap(self, seq, np_, t0):
        """처음 기어 들어가 멈춘 순간 재서, 목표가 1cm 넘게 틀렸으면 고친다 (재시작 직후 등).

        ⚠ 처음에는 nav2 를 끊는 순간(아직 0.1~0.3 m/s 로 움직인다) 쟀다. 스캔 시각과
          그 순간 위치가 맞지 않아 68~92mm 틀린 보정을 냈고, 실제로 B 에 89mm
          어긋나 섰다(2026-10-14). 서 있을 때만 잰다 — 평소에는 미리 얹은 보정이
          맞아 1cm 안이라 아무 것도 안 한다.
        """
        ref = self._poi_ref(np_["poi"])
        if ref is None:
            return
        for _ in range(20):                     # 멈추기를 1초까지 기다린다
            if self._poi_still():
                break
            time.sleep(0.05)
        else:
            self.get_logger().info("지점 %s: 1초 안에 안 멈춰 라이다로 못 잼" % np_["poi"]["name"])
            return
        t0 = time.time()
        snap = None
        for _ in range(30):
            snap = self._poi_snapshot(after=t0)
            if snap is not None:
                break
            time.sleep(0.05)
        if snap is None:
            self.get_logger().info("지점 %s: 멈춰서 잴 스캔·자세를 1.5초 안에 못 얻음" % np_["poi"]["name"])
            return
        if not self._poi_still():
            self.get_logger().info("지점 %s: 재는 사이 다시 움직임" % np_["poi"]["name"])
            return
        res, why = self._poi_measure(ref, snap)
        if res is None:
            self.get_logger().info("지점 %s: 멈춰서 잰 라이다 보정 못 함 — %s" % (np_["poi"]["name"], why))
            return
        if seq != getattr(self, "nav_seq", None):
            return
        cur = self.nav_target
        if cur is None:
            return
        d = math.hypot(res["goal"][0] - cur[0], res["goal"][1] - cur[1])
        if d <= self.POI_SNAP_M:
            return
        self.nav_target = res["goal"]
        self.__dict__.setdefault("_poi_corr", {})[np_["path"]] = (res["goal"], time.time())
        self.get_logger().info("지점 %s: 멈춰서 잰 라이다 보정으로 목표를 %.0fmm 옮김"
                               % (np_["poi"]["name"], d * 1000))

    def _poi_rest_async(self, seq):
        np_ = getattr(self, "_nav_poi", None)
        if np_ is None:
            return
        self._poi_thread(self._poi_rest, seq, np_, time.time())

    def _poi_still(self):
        """완전히 섰나 (오도메트리 속도). 모르면 False."""
        o = getattr(self, "odom", None)
        if o is None:
            return False
        return (abs(o.twist.twist.linear.x) < 0.003
                and abs(o.twist.twist.angular.z) < 0.01)

    def _poi_rest(self, seq, np_, t0):
        """도착해 서 있을 때 잰다. 기준이 없으면 찍고, 있으면 다음 방문용 보정을 구한다."""
        poi = np_["poi"]
        # ⚠ 다음 목표로 돌기 시작한 뒤 찍은 스캔은 쓸모없다 — 측정 도구가 그렇게
        #   7도 돌아간 기준을 찍었다(2026-10-14). 섰을 때만, 그리고 그 사이 새
        #   목표가 들어오지 않았을 때만.
        if seq != getattr(self, "nav_seq", None) or not self._poi_still():
            return
        ref = self._poi_ref(poi)
        snap = None
        for _ in range(20):
            # 기준은 점을 줄이지 않는다 (점 간격이 곧 맞추는 정밀도라서)
            snap = self._poi_snapshot(after=t0, step=4 if ref is not None else 1)
            if snap is not None:
                break
            time.sleep(0.05)
        if snap is None or seq != getattr(self, "nav_seq", None) or not self._poi_still():
            return
        if ref is None:
            e = getattr(self, "nav_pos_err", None)
            if e is None or math.hypot(e[0], e[1]) > self.POI_REF_TOL:
                return              # 정확히 선 때만 기준으로 삼는다
            if getattr(self, "nav_target", None) != (poi["x"], poi["y"], poi["yaw"]):
                return              # 보정을 얹어 간 자리는 기준이 아니다
            pts, slam = snap
            d = {"map": self.active_map, "name": poi["name"],
                 "poi": [poi["x"], poi["y"], poi["yaw"]], "slam": list(slam),
                 "t": time.time(), "pts": np.round(pts, 4).tolist()}
            path = np_["path"]
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path + ".tmp", "w") as f:
                    json.dump(d, f)
                os.replace(path + ".tmp", path)
            except OSError as e:
                self.get_logger().warn("지점 기준 스캔을 저장하지 못했다: %s" % e)
                return
            self.__dict__.setdefault("_poi_refs", {}).pop(path, None)
            self.get_logger().info("지점 %s: 라이다 기준 스캔 저장 (점 %d개)" % (poi["name"], len(pts)))
            return
        res, why = self._poi_measure(ref, snap)
        if res is None:
            self.get_logger().info("지점 %s: 라이다 보정 못 함 — %s" % (poi["name"], why))
            return
        self.__dict__.setdefault("_poi_corr", {})[np_["path"]] = (res["goal"], time.time())
        g = res["goal"]
        self.get_logger().info(
            "지점 %s 실제 도착 오차(라이다): 앞뒤 %+.0fmm 좌우 %+.0fmm 방향 %+.2f도 — "
            "SLAM 틀어짐 (%+.0f, %+.0f mm)"
            % (poi["name"], res["err"][0] * 1000, res["err"][1] * 1000,
               math.degrees(res["err"][2]), (g[0] - poi["x"]) * 1000, (g[1] - poi["y"]) * 1000))

    def poi_scan_reset(self, name=None):
        """기준 스캔을 지운다 (주변이 바뀌었거나 지점 자리를 다시 잡을 때)."""
        d = os.path.join(self.map_dir, self.POI_SCAN_DIR)
        n = 0
        for p in self.pois:
            if name and p["name"] != name:
                continue
            path = self._poi_path(p)
            try:
                os.remove(path)
                n += 1
            except OSError:
                pass
            self.__dict__.setdefault("_poi_refs", {}).pop(path, None)
            self.__dict__.setdefault("_poi_corr", {}).pop(path, None)
        return True, "기준 스캔 %d개를 지웠습니다 (%s)" % (n, d)
        self._nav_clear_path()
        self.get_logger().info("이동 종료: %s" % self.nav_msg)

    def nav_cancel(self):
        """이동을 끊는다. nav2 목표·출발 기어가기·도착 후 동작 모두.

        ⚠ 지금 목표의 번호를 무효로 만든다(nav_seq +1). 그래야 (1) 목표 앞에서
          끊어 둔 취소 결과가 뒤늦게 와도 '도착' 으로 처리되지 않고, (2) 이미
          보냈지만 아직 받아들여지지 않은 목표가 나중에 받아들여져도 곧바로
          취소된다(_nav_accepted). 전에는 둘 다 정지 뒤에 로봇이 다시 움직일
          수 있었다(2026-09-29 검토).
        """
        if hasattr(self, "_appr_clear"):
            self._appr_clear()
        busy = (getattr(self, "_turning", False)
                or getattr(self, "_depart_timer", None) is not None
                or self.nav_goal is not None
                or self.nav_state in ("sending", "active"))
        self._turn_reset()
        self._depart_stop()
        if getattr(self, "_np_timer", None) is not None:
            busy = True
        self.np_cancel()
        self.sp_cancel()
        if getattr(self, "cleave", {}).get("state") == "leaving":
            busy = True
            self.cleave_cancel()
        self._nav_early_seq = None
        self.nav_seq = getattr(self, "nav_seq", 0) + 1
        if self.nav_goal is not None:
            try:
                self.nav_goal.cancel_goal_async()
            except Exception:                                # noqa: BLE001
                pass
            self.nav_goal = None
        self._pub_zero()
        if busy:
            self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
            self._nav_clear_path()
            return True, "이동을 취소했습니다"
        return True, "진행 중인 이동이 없습니다 (정지 지령만 보냄)"

    def nav_status(self):
        return {
            "available": self.nav_cli.server_is_ready(),
            "scan_ok": not self._scan_dead,
            "state": self.nav_state,
            "message": self.nav_msg,
            "target": ({"x": self.nav_target[0], "y": self.nav_target[1],
                        "yaw": self.nav_target[2]} if self.nav_target else None),
            "speed": self.nav_speed,
            "speed_min": self.NAV_SPEED_MIN,
            "speed_max": self.NAV_SPEED_MAX,
            "speed_applied": self._speed_sent,
            "loc_hold": self.loc_hold is not None,
            "loc_mon": getattr(self, "_locmon", None),
            "blocked_s": ((getattr(self, "_stopw", None) or {}).get("t")
                          if (getattr(self, "_stopw", None) or {}).get("alerted") else None),
        }

    # ---------------------------------------------------------- 주행 속도
    #
    # nav2 는 속도를 두 군데서 잡는다. 하나는 경로 추종기(RPP)의 목표 속도,
    # 다른 하나는 velocity_smoother 의 상한이다. 추종기만 올리면 smoother 가
    # 깎고, smoother 만 올리면 추종기가 애초에 그 속도를 안 낸다. 둘 다 바꾼다.
    #
    # 위는 base_node 의 max_linear_mps 와 같은 0.6 m/s 다. 그 위는 base_node
    # 가 어차피 깎는다(바퀴 rpm 상한 200 은 1.79 m/s 라 제약이 아니다).
    #
    # 빠르게 가도 안전한 것은 collision_monitor 의 FootprintApproach 가
    # 속도에 따라 반응 거리를 스스로 늘리기 때문이다. 그게 꺼져 있으면
    # 고정 크기 정지 구역(0.70 m)으로는 0.35 m/s 언저리가 한계다.
    #
    # 아래는 0.10 m/s 다. 바퀴 지령이 정수 rpm 이라 그 아래는 뭉개진다.

    NAV_SPEED_MIN = 0.10
    # 2026-11-13 0.60 -> 1.00 (base_node max_linear_mps 와 같이). 근거는 amr_params.yaml.
    NAV_SPEED_MAX = 1.00
    NAV_SPEED_DEFAULT = 0.25
    NAV_W_MAX = 0.6            # 회전 상한은 건드리지 않는다

    def _speed_path(self):
        return os.path.join(os.path.dirname(self.map_dir.rstrip("/")),
                            "und_amr_settings.json")

    def _speed_load(self):
        try:
            with open(self._speed_path()) as f:
                v = float(json.load(f).get("nav_speed"))
            return max(self.NAV_SPEED_MIN, min(self.NAV_SPEED_MAX, v))
        except Exception:                                    # noqa: BLE001
            return self.NAV_SPEED_DEFAULT

    def _speed_save(self):
        try:
            with open(self._speed_path(), "w") as f:
                json.dump({"nav_speed": self.nav_speed}, f, indent=1)
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn("주행 속도를 저장하지 못했다: %s" % e)

    def _p_double(self, name, v):
        return Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))

    def _p_darray(self, name, arr):
        return Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(x) for x in arr]))

    def _speed_push(self, v):
        """nav2 에 속도를 밀어 넣는다. 보내기만 하고 결과는 기다리지 않는다.

        HTTP 스레드에서 불리므로 기다리면 실행기와 얽혀 굳는다. 이 노드의
        다른 서비스 호출도 모두 같은 방식이다.
        """
        if getattr(self, "_spd_cli", None) is None:
            self._spd_cli = {
                "controller_server": self.create_client(
                    SetParameters, "/controller_server/set_parameters"),
                "velocity_smoother": self.create_client(
                    SetParameters, "/velocity_smoother/set_parameters"),
            }
        sent = 0
        c = self._spd_cli["controller_server"]
        if c.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            # RPP 는 desired_linear_vel, MPPI 는 vx_max, DWB 는 max_vel_x·max_speed_xy (2026-10-02).
            # 없는 쪽 이름은 그 값만 거절되고 나머지는 적용된다.
            req.parameters = [self._p_double("FollowPath.desired_linear_vel", v),
                              self._p_double("FollowPath.vx_max", v),
                              self._p_double("FollowPath.max_vel_x", v),       # DWB
                              self._p_double("FollowPath.max_speed_xy", v)]
            c.call_async(req)
            sent += 1
        c = self._spd_cli["velocity_smoother"]
        if c.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            req.parameters = [
                self._p_darray("max_velocity", [v, 0.0, self.NAV_W_MAX]),
                self._p_darray("min_velocity", [-v, 0.0, -self.NAV_W_MAX]),
            ]
            c.call_async(req)
            sent += 1
        return sent == 2

    def _speed_watch(self):
        """nav2 가 새로 뜨면 저장해 둔 속도를 다시 밀어 넣는다.

        nav2 를 껐다 켜면 파라미터가 nav2.yaml 값으로 돌아간다. 사람이
        기억해서 다시 넣어야 한다면 언젠가 잊는다.
        """
        up = self.nav_cli.server_is_ready()
        if not up:
            self._speed_sent = False
            return
        if self._speed_sent:
            return
        if self._speed_push(self._speed_eff()):
            self._speed_sent = True
            self.get_logger().info("주행 속도 %.2f m/s 적용" % self._speed_eff())

    def nav_speed_set(self, v):
        try:
            v = round(float(v), 2)
        except (TypeError, ValueError):
            return False, "속도 값이 잘못되었습니다", None
        if v < self.NAV_SPEED_MIN or v > self.NAV_SPEED_MAX:
            return False, ("주행 속도는 %.2f ~ %.2f m/s 사이여야 합니다"
                           % (self.NAV_SPEED_MIN, self.NAV_SPEED_MAX)), None
        self.nav_speed = v
        self._speed_save()
        if not self.nav_cli.server_is_ready():
            self._speed_sent = False
            return True, "nav2 가 없어 값만 저장했습니다 (%.2f m/s)" % v, v
        if not self._speed_push(self._speed_eff()):
            self._speed_sent = False
            return False, "nav2 파라미터 서비스가 응답하지 않습니다", None
        self._speed_sent = True
        self.get_logger().info("주행 속도 %.2f m/s" % v)
        if self._speed_eff() < v:
            return True, ("주행 속도를 %.2f m/s 로 저장했습니다. 렉을 든 상태라 "
                          "지금은 %.2f m/s 로 갑니다" % (v, self._speed_eff())), v
        return True, "주행 속도를 %.2f m/s 로 바꿨습니다" % v, v

    def map_edits_json(self):
        with self.lock:
            return {"map": self.active_map,
                    "editable": bool(self.active_map
                                     and self.map_source == "saved"),
                    "count": len(self.map_edits),
                    "shapes": list(self.map_edits)}

    def map_edits_apply(self, shapes):
        """화면에서 그린 편집을 지금 메인 맵에 적용하고 파일로 남긴다."""
        name = self.active_map
        if not name:
            return False, "적용된 지도가 없습니다"
        if self.map_source != "saved":
            return False, ("매핑 중에는 편집할 수 없습니다. "
                           "지도를 저장하고 적용한 뒤에 하세요")
        clean = self._clean_shapes(shapes if isinstance(shapes, list) else [])
        ok, err = self.save_edits(name, clean)
        if not ok:
            return False, "편집을 저장하지 못했습니다: %s" % err
        with self.lock:
            self.map_edits = clean
            self._compose_map()
        self.get_logger().info("'%s' 지도 편집 %d개 적용" % (name, len(clean)))
        if not clean:
            return True, "지도 편집을 모두 지웠습니다 (원본으로 되돌림)"
        # 지우개는 원 하나하나가 세어지므로 '곳', 가상벽·금지구역은 도형
        # 하나가 하나이므로 '개' 로 센다. 선과 영역도 구분해서 알린다.
        ne = sum(1 for c in clean if c["op"] == "erase")
        nl = sum(1 for c in clean if c["op"] == "wall" and c["kind"] == "line")
        nz = sum(1 for c in clean if c["op"] == "wall" and c["kind"] == "poly")
        parts = []
        if ne:
            parts.append("지운 자리 %d 곳" % ne)
        if nl:
            parts.append("가상벽 %d 개" % nl)
        if nz:
            parts.append("금지구역 %d 개" % nz)
        return True, "%s 을 지도에 적용했습니다" % " · ".join(parts)

    def map_load(self, name):
        """저장된 포즈그래프를 slam_toolbox 에 되돌려 넣는다.

        .pgm/.yaml 은 내비게이션용 결과물이라 이어서 매핑할 수 없다.
        .posegraph 가 있어야 지도를 이어 그리거나 그 위에서 위치추정을 한다."""
        base = self.map_path(name)
        if not base:
            return False, "잘못된 이름"
        if not os.path.exists(base + ".posegraph"):
            return False, "포즈그래프가 없어 불러올 수 없습니다 (.posegraph 필요)"
        # '적용' 은 그 지도 위에서 로봇 위치를 아는 것이 목적이다.
        # 지도를 이어 그리는 것은 별개 기능이라 여기서는 위치추정으로 띄운다.
        self.slam_kill()
        self._forget_map()
        if not self.slam_spawn(mode="localization", map_base=base):
            if self.adopt_saved_map(name):
                return True, "'%s' 를 지도로 적용했습니다 (위치추정 기동 실패)" % name
            return False, "SLAM 노드 기동 실패"
        # 위치추정 모드는 map_file_name 파라미터로 기동할 때 포즈그래프를
        # 이미 올린다. 여기서 deserialize 를 또 부르면 중복이고, 서비스가
        # 아직 준비되지 않아 실패하면 적용이 중간에 끊겨 active_map 이 어긋난다.
        # 표시용 지도는 저장본으로 고정한다. 그래야 위치추정 중 slam_toolbox 가
        # /map 을 다시 그려 내보내도 화면의 지도가 흔들리지 않는다.
        # (adopt_saved_map 이 active_map 기록까지 함께 처리한다)
        if not self.adopt_saved_map(name):
            self.active_map = os.path.basename(base)
            self._remember_active(self.active_map)
        return True, "'%s' 적용됨" % self.active_map

    # ---------------- 위치 재조정 ----------------
    #
    # 지도를 다시 불러오거나 노드를 재시작하면 로봇이 지도 어디에 있는지
    # 알 수 없다. 그때 위치를 다시 잡아 주는 기능이다.
    #   수동 - 사람이 화면에서 위치/방향을 찍어 준다
    #   자동 - 지금 라이다 스캔을 지도와 맞춰 가장 잘 맞는 자세를 찾는다

    def _publish_initialpose(self, x, y, yaw, cov_xy=0.25, cov_yaw=0.07):
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        c = [0.0] * 36
        c[0] = c[7] = float(cov_xy)      # x, y
        c[35] = float(cov_yaw)           # yaw
        m.pose.covariance = c
        self.pub_initpose.publish(m)

    # 수동 위치 고정 (2026-11-13, 사용자: "수동은 내가 움직이는대로 적용이 되어야하지 않아?"
    #   / "수동으로 위치고정하는 모드가 필요해서 수동 이랑 자동을 따로 만든거였어").
    #   /initialpose 만 보내면 slam_toolbox 가 그 자리 근처에서 스캔을 다시 맞춰 자기 자리로
    #   되돌린다 — 지도가 안 맞는 충전소 앞에서 9cm 당겨 갔다. 그래서 고정이면
    #   1) slam_toolbox 새 스캔 처리를 멈춘다 -> map->odom 이 그대로 묶인다.
    #   2) map 에서 원하는 자세 P 가 되도록 odom 자세 O' = (map->odom)^-1 * P 를 EKF
    #      /set_pose 로 넣는다. EKF 는 바퀴 · IMU 속도만 받으므로 그 값에서 이어 간다.
    #   이후는 라이다 보정 없이 바퀴 · IMU 로만 간다. relocalize_release() 나 자동
    #   재조정이 다시 켠다.
    def _slam_paused(self):
        """slam_toolbox paused_new_measurements. 모르면 None."""
        cli = self.__dict__.get("_slam_param_cli")
        if cli is None:
            cli = self.create_client(GetParameters, "/slam_toolbox/get_parameters")
            self._slam_param_cli = cli
        req = GetParameters.Request()
        req.names = ["paused_new_measurements"]
        r, err = self._call(cli, req, timeout=2.0)
        if r is None or not r.values:
            return None
        return bool(r.values[0].bool_value)

    def _slam_set_paused(self, want):
        st = self._slam_paused()
        if st is None:
            return False, "SLAM 상태를 모릅니다"
        if st == want:
            return True, None
        r, err = self._call(self.slam.get("pause"), Pause.Request(), timeout=3.0)
        if r is None:
            return False, err or "SLAM 이 응답하지 않습니다"
        return (self._slam_paused() == want), None

    def relocalize_release(self, quiet=False):
        """수동 고정을 풀고 라이다 보정(SLAM)을 다시 켠다."""
        if self.loc_hold is None and self._slam_paused() is not True:
            return True, "라이다 보정이 이미 켜져 있습니다"
        ok, err = self._slam_set_paused(False)
        if not ok:
            return False, "라이다 보정을 다시 켜지 못했습니다 (%s)" % (err or "?")
        self.loc_hold = None
        if not quiet:
            self.get_logger().info("수동 위치 고정 해제 — 라이다 보정 켬")
        return True, "라이다 보정을 다시 켰습니다"

    def relocalize_hold(self, x, y, yaw):
        ok, err = self._slam_set_paused(True)
        if not ok:
            return False, "라이다 보정을 멈추지 못해 고정하지 못했습니다 (%s)" % (err or "?")
        time.sleep(0.15)                       # 멈추기 전 마지막 보정이 TF 에 실릴 틈
        mo = self.tf_pose("map", "odom")
        if mo is None:
            self._slam_set_paused(False)
            return False, "map -> odom 을 모릅니다"
        c, s_ = math.cos(-mo["yaw"]), math.sin(-mo["yaw"])
        dx, dy = x - mo["x"], y - mo["y"]
        ox, oy = c * dx - s_ * dy, s_ * dx + c * dy
        oyaw = math.atan2(math.sin(yaw - mo["yaw"]), math.cos(yaw - mo["yaw"]))
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.pose.pose.position.x, m.pose.pose.position.y = float(ox), float(oy)
        m.pose.pose.orientation.z = math.sin(oyaw / 2.0)
        m.pose.pose.orientation.w = math.cos(oyaw / 2.0)
        cv = [0.0] * 36
        cv[0] = cv[7] = 1e-4
        cv[35] = 1e-4
        m.pose.covariance = cv
        self.pub_ekf_pose.publish(m)
        self.loc_hold = {"x": x, "y": y, "yaw": yaw, "t": time.time()}
        self.get_logger().info("수동 위치 고정 → (%.3f, %.3f, %.1f°) — 라이다 보정 끔"
                               % (x, y, math.degrees(yaw)))
        return True, ("위치를 (%.3f, %.3f) / %.1f° 로 고정했습니다 — 라이다 보정을 껐습니다. "
                      "[자동] 이나 [보정 켜기] 로 다시 켭니다" % (x, y, math.degrees(yaw)))

    def relocalize_manual(self, x, y, yaw, hold=False):
        if self.slam_mode != "localization":
            return False, "위치추정 중일 때만 재조정할 수 있습니다"
        if hold:
            return self.relocalize_hold(x, y, yaw)
        if self.loc_hold is not None or self._slam_paused():
            ok, msg = self.relocalize_release(quiet=True)
            if not ok:
                return False, msg
        self._publish_initialpose(x, y, yaw)
        # 이전 위치에서 쌓인 스캔 버퍼가 남아 있으면 새 위치와 싸운다.
        cli = self.slam.get("clear_buffer")
        if cli is not None and cli.service_is_ready():
            cli.call_async(EmptySrv.Request())
        self.get_logger().info("수동 위치 재조정 → (%.2f, %.2f, %.1f°)"
                               % (x, y, math.degrees(yaw)))
        return True, "위치를 (%.2f, %.2f) / %.0f° 로 재조정했습니다" % (
            x, y, math.degrees(yaw))

    # ---- 위치 감시 ----
    #
    # ⚠ 2026-11-13 "이동하다가 라이다 위치가 아예 틀어짐": 로그에 그 순간이 안 남아
    #   원인을 못 갈랐다. 1초마다 (1) SLAM 보정량 map->odom 이 갑자기 바뀌었는지
    #   (2) 지금 자세에서 스캔이 지도에 얼마나 맞는지와 근처(±0.3m, ±6도)에서 가장 잘
    #   맞는 자리가 얼마나 떨어졌는지를 logs/loc_monitor.csv 에 남기고, 튀면 경고한다.
    #   고치지는 않는다 — 기록을 보고 판단한다.
    LOCMON_JUMP_M = 0.05
    LOCMON_JUMP_DEG = 2.0
    LOCMON_OFF_M = 0.10
    LOCMON_OFF_DEG = 4.0
    LOCMON_GAIN = 0.06          # 근처 최고 점수가 이만큼 높아야 '틀어짐 의심'

    def _locmon_loop(self):
        self._api_t0 = time.monotonic()
        time.sleep(15.0)
        path = os.path.expanduser("~/projects/UND_AMR/logs/loc_monitor.csv")
        prev = None
        while rclpy.ok():
            t0 = time.monotonic()
            try:
                prev = self._locmon_tick(path, prev)
            except Exception as e:                           # noqa: BLE001
                self.get_logger().warn("위치 감시 오류: %s" % e, throttle_duration_sec=60.0)
            time.sleep(max(0.2, 1.0 - (time.monotonic() - t0)))

    def _locmon_tick(self, path, prev):
        if self.slam_mode != "localization" or not HAS_IMAGING:
            return None
        with self.lock:
            scan = self.scan
            meta = dict(self.map_meta)
            png = self.map_loc_png or self.map_png
            odom = self.odom
        if scan is None or png is None or not meta.get("available"):
            return prev
        pose = self.tf_pose_at(scan.header.stamp)
        mo = self.tf_pose("map", "odom")
        if pose is None or mo is None:
            return prev
        got = self._map_grid(png, meta)
        if got is None:
            return prev
        lik, res, ox, oy, H, free, lik_c, occ = got
        pts = self._scan_points(scan, max_pts=120)
        if len(pts) < 40:
            return prev
        now_sc = self._score_pose(lik, res, ox, oy, H, pts, pose["x"], pose["y"], pose["yaw"])
        r1 = self._match_scan(lik, res, ox, oy, H, pts, (pose["x"], pose["y"], pose["yaw"]),
                              0.30, 0.10, math.radians(6), math.radians(3))
        best = r1
        if r1 is not None:
            best = self._match_scan(lik, res, ox, oy, H, pts, r1[1],
                                    0.10, 0.025, math.radians(3), math.radians(1.5)) or r1
        bsc, bp = (best if best else (now_sc, (pose["x"], pose["y"], pose["yaw"])))
        dx, dy = bp[0] - pose["x"], bp[1] - pose["y"]
        dyaw = math.degrees(wrap_pi(bp[2] - pose["yaw"]))
        v = w = 0.0
        if odom is not None:
            v, w = odom.twist.twist.linear.x, odom.twist.twist.angular.z
        ev = ""
        if prev is not None:
            jm = math.hypot(mo["x"] - prev[0], mo["y"] - prev[1])
            jd = abs(math.degrees(wrap_pi(mo["yaw"] - prev[2])))
            if jm > self.LOCMON_JUMP_M or jd > self.LOCMON_JUMP_DEG:
                ev = "SLAM보정튐 %.0fmm %.1f도" % (jm * 1000, jd)
        off = math.hypot(dx, dy)
        if bsc - now_sc > self.LOCMON_GAIN and (off > self.LOCMON_OFF_M or abs(dyaw) > self.LOCMON_OFF_DEG):
            ev = (ev + " " if ev else "") + "틀어짐의심 %.0fmm %.1f도" % (off * 1000, dyaw)
        self._locmon = {"score": round(now_sc, 3), "best": round(bsc, 3),
                        "off_mm": round(off * 1000), "off_deg": round(dyaw, 1), "event": ev, "t": time.time()}
        h = self.__dict__.setdefault("_locmon_hist", [])
        h.append(now_sc)
        del h[:-10]
        if not getattr(self, "_boot_dock_done", False):
            self._boot_dock_check(pose)
        if ev:
            self.get_logger().warn("위치 감시: %s — 자리 (%.2f, %.2f, %.0f도) 점수 %.2f, 근처 최고 %.2f"
                                   " (속도 %.2f m/s %.2f rad/s)"
                                   % (ev, pose["x"], pose["y"], math.degrees(pose["yaw"]),
                                      now_sc, bsc, v, w))
        try:
            new = not os.path.exists(path)
            with open(path, "a", encoding="utf-8") as f:
                if new:
                    f.write("time,x,y,yaw_deg,mo_x,mo_y,mo_yaw_deg,v,w,score,best,"
                            "off_dx_mm,off_dy_mm,off_dyaw_deg,event\n")
                f.write("%s,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.0f,%.0f,%.1f,%s\n"
                        % (time.strftime("%H:%M:%S"), pose["x"], pose["y"],
                           math.degrees(pose["yaw"]), mo["x"], mo["y"],
                           math.degrees(mo["yaw"]), v, w, now_sc, bsc,
                           dx * 1000, dy * 1000, dyaw, ev))
        except OSError:
            pass
        return (mo["x"], mo["y"], mo["yaw"])

    # ---- 위치를 모르면 출발 거부 · 충전소에서 켜지면 충전소 자리로 (2026-11-23) ----
    #
    # ⚠ 2026-11-23 왕복이 POI 1 에서 끝나 마지막 자리가 POI 1 로 남은 채 로봇을 충전소로 옮겨 재부팅했다.
    #   SLAM 이 POI 1 에서 시작해 위치 감시가 '틀어짐의심 400~566mm'(일치 0.14, 평소 0.6~0.7)를 20분 동안
    #   582번 경고했는데, 왕복 시작을 막는 장치가 없어 충전소에서 엉뚱한 곳으로 출발했다.
    #   ① 최근 위치 감시 일치 점수 중앙값이 LOC_START_MIN 아래면 지점 이동 · 왕복 시작을 거부한다
    #      (수동 위치 고정 중이면 사람이 정한 것이라 막지 않는다).
    #   ② 기동 뒤 BOOT_DOCK_WINDOW_S 안에 충전소 V 홈이 등 뒤에 붙어 보이면(충전 전류와 무관 — 다 차면 0A)
    #      지금 위치가 충전소 POI 와 다를 때 충전소 도킹 자리로 위치를 맞춘다.
    LOC_START_MIN = 0.30        # 충전소 앞 평소 0.37~0.46, 위치를 잃었을 때 0.14~0.17
    LOC_FRESH_S = 5.0
    BOOT_DOCK_WINDOW_S = 120.0
    BOOT_DOCK_GAP = 0.06        # 붙어 있음: 틈 · 좌우 · 방향 [m, m, rad]
    BOOT_DOCK_LAT = 0.04
    BOOT_DOCK_YAW = math.radians(5.0)
    BOOT_DOCK_FIX_M = 0.10      # 이보다 어긋나 있으면 맞춘다
    BOOT_DOCK_FIX_YAW = math.radians(5.0)

    def _loc_unsure(self):
        """위치를 믿을 수 없으면 사람이 읽을 이유, 아니면 None."""
        lm = getattr(self, "_locmon", None)
        if not lm or getattr(self, "loc_hold", None) is not None:
            return None
        if time.time() - lm.get("t", 0.0) > self.LOC_FRESH_S:
            return None
        h = getattr(self, "_locmon_hist", [])
        if len(h) < 3:
            return None
        med = statistics.median(h[-3:])
        if med >= self.LOC_START_MIN:
            return None
        return ("위치가 지도와 맞지 않습니다 (일치 %.2f, 기준 %.2f) — 위치를 다시 잡은 뒤 출발하세요"
                % (med, self.LOC_START_MIN))

    def _docked_pose(self, scan):
        """충전소 V 홈이 등 뒤에 붙어 보이면 그 충전소 기준 로봇 지도 자세 (x, y, yaw, 충전소 이름), 아니면 None."""
        from und_amr_bringup.charger_detect import detect_scan
        chargers = [p for p in getattr(self, "pois", []) if p.get("kind") == "charger"]
        if len(chargers) != 1 or scan is None:
            return None
        las = self._poi_laser(scan.header.frame_id) or (0.0, 0.0, 0.0)
        c = detect_scan(scan.ranges, scan.angle_min, scan.angle_increment, las)
        if c is None:
            return None
        st_ = (c["x"], c["y"], c["yaw"])                     # 로봇 기준 충전소 앞면
        gap, lat, dyaw = self._cdock_err((0.0, 0.0, 0.0), st_)
        if abs(gap) > self.BOOT_DOCK_GAP or abs(lat) > self.BOOT_DOCK_LAT or abs(dyaw) > self.BOOT_DOCK_YAW:
            return None
        p = chargers[0]
        hl = self.CHARGER_HALF_LEN
        sm = (p["x"] - hl * math.cos(p["yaw"]), p["y"] - hl * math.sin(p["yaw"]), p["yaw"])   # 지도 위 앞면
        # 앞면(지도) = 로봇(지도) ⊕ st_  ->  로봇(지도) = 앞면(지도) ⊕ st_^-1
        cs, sn = math.cos(st_[2]), math.sin(st_[2])
        inv = (-cs * st_[0] - sn * st_[1], sn * st_[0] - cs * st_[1], -st_[2])
        c2, s2 = math.cos(sm[2]), math.sin(sm[2])
        rx = sm[0] + c2 * inv[0] - s2 * inv[1]
        ry = sm[1] + s2 * inv[0] + c2 * inv[1]
        return (rx, ry, wrap_pi(sm[2] + inv[2]), p.get("name", "?"))

    def _boot_dock_check(self, pose):
        t0 = getattr(self, "_api_t0", None)
        if t0 is None or time.monotonic() - t0 > self.BOOT_DOCK_WINDOW_S:
            self._boot_dock_done = True
            return
        if self.nav_state in ("sending", "active") or (getattr(self, "cdock", None) or {}).get(
                "state") not in (None, "idle", "done", "failed", "canceled"):
            return
        with self.lock:
            scan = self.scan
        try:
            got = self._docked_pose(scan)
        except Exception:                                    # noqa: BLE001
            got = None
        n = self.__dict__["_boot_dock_tries"] = getattr(self, "_boot_dock_tries", 0) + 1
        if got is None:
            if n >= 5:
                self._boot_dock_done = True                  # 충전소에 붙어 있지 않다
            return
        self._boot_dock_done = True
        rx, ry, ryaw, name = got
        off = math.hypot(rx - pose["x"], ry - pose["y"])
        dyaw = abs(wrap_pi(ryaw - pose["yaw"]))
        if off <= self.BOOT_DOCK_FIX_M and dyaw <= self.BOOT_DOCK_FIX_YAW:
            self.get_logger().info("기동: %s 에 붙어 있음 — 위치가 맞음 (차이 %.0fmm %.1f도)"
                                   % (name, off * 1000, math.degrees(dyaw)))
            return
        ok, msg = self.relocalize_manual(rx, ry, ryaw)
        self.get_logger().warn("기동: %s 에 붙어 있는데 위치가 (%.2f, %.2f, %.0f도) 로 %.0fmm %.0f도 어긋나 있음 — "
                               "충전소 도킹 자리 (%.2f, %.2f, %.0f도) 로 맞춤: %s"
                               % (name, pose["x"], pose["y"], math.degrees(pose["yaw"]), off * 1000,
                                  math.degrees(dyaw), rx, ry, math.degrees(ryaw), msg))

    def relocalize_auto(self, radius=2.0, yaw_span=math.pi, apply=False):
        """지금 스캔을 지도와 맞춰 가장 잘 맞는 자세를 찾는다.

        apply=False 면 찾기만 하고 로봇을 옮기지는 않는다. 화면에서 후보를
        먼저 보여 주고 사람이 확인을 누를 때 반영하기 위해서다.

        현재 추정 위치 주변만 성기게 훑고(coarse) 좋은 후보를 골라 다시
        촘촘히 훑는다(fine). 지도 전체를 뒤지지는 않으므로, 로봇이 완전히
        엉뚱한 곳에 있으면 수동으로 대략 찍어 준 뒤 쓰는 것이 확실하다.
        """
        if self.slam_mode != "localization":
            return False, "위치추정 중일 때만 재조정할 수 있습니다", None
        if not HAS_IMAGING:
            return False, "numpy/PIL 이 없어 자동 재조정을 할 수 없습니다", None
        with self.lock:
            scan = self.scan
            meta = dict(self.map_meta)
            # 가상벽은 라이다가 볼 수 없으므로 위치추정에서 뺀다
            png = self.map_loc_png or self.map_png
        if scan is None:
            return False, "라이다 스캔이 없습니다", None
        if not meta.get("available") or png is None:
            return False, "지도가 없습니다", None
        pose = self.tf_pose("map", "base_footprint")
        if pose is None:
            return False, "현재 위치를 알 수 없습니다", None

        got = self._map_grid(png, meta)
        if got is None:
            return False, "지도를 읽을 수 없습니다", None
        lik, res, ox, oy, H, free, lik_c, occ = got

        pts = self._scan_points(scan)
        if len(pts) < 40:
            return False, "유효한 스캔 점이 부족합니다", None

        cx, cy, cyaw = pose["x"], pose["y"], pose["yaw"]
        now = self._score_pose(lik, res, ox, oy, H, pts, cx, cy, cyaw)

        def rank(x, y, yaw, sc):
            """일치도에 '말이 되는가' 를 곱해 순위를 매긴다.

            벽을 뚫고 보는 자세와 지도에 없는 곳을 보는 자세는 그만큼
            깎는다. 일치도 1%p 차이로 순위가 뒤집히던 것을 막는다.
            """
            bl, un = self._ray_check(occ, free, res, ox, oy, H, pts, x, y, yaw)
            return sc * (1.0 - bl) * (1.0 - un), bl, un

        now_adj = rank(cx, cy, cyaw, now)[0]

        # 먼저 현재 위치 주변을 훑는다. 거기서 신통치 않으면 지도 전체를 본다.
        # (재시작 직후처럼 로봇이 지도 밖에 있을 수 있다)
        near = self._match_scan(lik, res, ox, oy, H, pts,
                                (cx, cy, cyaw), radius, 0.20,
                                yaw_span, math.radians(6))
        # 근처 탐색이 어중간하게 성공했다고 전역 탐색을 건너뛰면 안 된다.
        # 엉뚱한 곳의 30~40% 짜리 봉우리에 그대로 눌러앉는 일이 실제로
        # 있었다(정답에서 7~9m 떨어진 곳으로 이동). 항상 둘 다 훑고 비교한다.
        cands = []
        if near is not None:
            cands.append(("근처", near[1]))
        # 거친 단계는 너그러운 점수판(lik_c)으로 본다. 예민한 기준을 쓰면
        # 간격 탓에 어긋난 정답이 상위에 못 올라온다.
        for _, pose in self._match_global(lik_c, res, ox, oy, H, free, pts):
            cands.append(("전체", pose))
        if not cands:
            return False, "맞는 자세를 찾지 못했습니다", None

        # 후보를 각각 두 단계로 다듬고, 그중 가장 잘 맞는 것을 고른다.
        best = None
        used_global = False
        for src, pose in cands:
            r1 = self._match_scan(lik, res, ox, oy, H, pts, pose,
                                  0.40, 0.10, math.radians(12), math.radians(3))
            if r1 is None:
                continue
            r2 = self._match_scan(lik, res, ox, oy, H, pts, r1[1],
                                  0.10, 0.025, math.radians(4), math.radians(1))
            r = r2 or r1
            # 다듬기는 일치도로 하되, 후보끼리 견주는 것은 보정 점수로 한다.
            # 일치도만으로 고르면 8m 떨어진 51% 가 정답 52% 를 이긴다.
            adj, bl, un = rank(r[1][0], r[1][1], r[1][2], r[0])
            if best is None or adj > best[0]:
                best = (adj, r[1], r[0], bl, un)
                used_global = (src == "전체")
        if best is None:
            return False, "맞는 자세를 찾지 못했습니다", None
        adj, (bx, by, byaw), score, blocked, unknown = best
        moved = math.hypot(bx - cx, by - cy)
        dyaw = abs(math.degrees(math.atan2(math.sin(byaw - cyaw),
                                           math.cos(byaw - cyaw))))

        # 지금보다 나아지지 않으면 건드리지 않는다. 재조정이 오히려 위치를
        # 망가뜨린 적이 있어서, 개선이 확실할 때만 반영한다.
        # 견주는 값은 보정 점수다 - 일치도만 보면 벽을 뚫고 보는 자세가
        # 멀쩡한 자세를 이긴다.
        if adj < now_adj + 0.02:
            return False, ("지금 위치가 이미 더 잘 맞습니다 "
                           "(현재 %.0f%% / 후보 %.0f%%)"
                           % (now * 100, score * 100)), None
        if score < 0.25:
            return False, ("맞는 자세를 찾지 못했습니다 (최고 일치도 %.0f%%). "
                           "수동으로 대략 잡아 주세요" % (score * 100)), None
        # 라이다가 벽 너머를 보고 있는 자세는 아무리 점수가 높아도 그 자리가
        # 아니다. 실측에서 정답 근처는 22~43%, 엉뚱한 곳은 59~81% 였다.
        if blocked > 0.65:
            return False, ("찾은 자세가 지도와 맞지 않습니다 "
                           "(라이다 %.0f%% 가 벽 너머를 봅니다). "
                           "수동으로 대략 잡아 주세요" % (blocked * 100)), None

        pose_out = {"x": bx, "y": by, "yaw": byaw, "score": score,
                    "adj": round(adj, 4), "blocked": round(blocked, 3),
                    "unknown": round(unknown, 3),
                    "moved": moved, "dyaw": dyaw}
        if not apply:
            return True, ("후보를 찾았습니다%s — %.2f m / %.0f° 이동, "
                          "일치도 %.0f%% → %.0f%% (벽 너머 %.0f%%). "
                          "확인을 누르면 적용됩니다"
                          % (" (지도 전체 탐색)" if used_global else "",
                             moved, dyaw, now * 100, score * 100,
                             blocked * 100)), pose_out

        if self.loc_hold is not None:
            self.relocalize_release(quiet=True)
        self._publish_initialpose(bx, by, byaw)
        cli = self.slam.get("clear_buffer")
        if cli is not None and cli.service_is_ready():
            cli.call_async(EmptySrv.Request())
        self.get_logger().info(
            "자동 위치 재조정 → (%.2f, %.2f, %.1f°) 일치도 %.0f%% (이전 %.0f%%), "
            "벽 너머 %.0f%%"
            % (bx, by, math.degrees(byaw), score * 100, now * 100,
               blocked * 100))
        return True, ("자동 재조정 완료%s — %.2f m / %.0f° 이동, "
                      "일치도 %.0f%% → %.0f%% (벽 너머 %.0f%%)"
                      % (" (지도 전체 탐색)" if used_global else "",
                         moved, dyaw, now * 100, score * 100,
                         blocked * 100)), pose_out

    def _map_grid(self, png, meta):
        """지도 PNG 를 '벽에서 얼마나 가까운가' 점수판으로 바꾼다.

        스캔 점이 벽 셀에 정확히 얹히는지로 점수를 매기면, 벽이 한 셀
        두께라 조금만 어긋나도 0 점이 되어 엉뚱한 자세를 고른다. 그래서
        벽에서의 거리로 부드럽게 점수를 준다(likelihood field).
        scipy 를 쓸 수 없는 환경이라 4방향 팽창을 반복해 거리를 센다.
        """
        key = meta.get("seq")
        if self._lik_cache is not None and self._lik_cache[0] == key:
            return self._lik_cache[1]
        try:
            with PilImage.open(io.BytesIO(png)) as im:
                a = np.array(im.convert("L"))
        except Exception:                                    # noqa: BLE001
            return None
        occ = a < 100                       # 검정 = 점유
        free = a > 200                      # 흰색 = 자유공간 (미지는 205 회색)
        res = float(meta["resolution"])
        sigma = 0.15                        # 이 거리(m)만큼 벗어나면 점수 절반쯤
        steps = max(1, int(round(0.60 / res)))
        d = np.full(occ.shape, steps, dtype=np.int16)
        d[occ] = 0
        cur = occ.copy()
        for k in range(1, steps + 1):
            nxt = cur.copy()
            nxt[1:, :] |= cur[:-1, :]
            nxt[:-1, :] |= cur[1:, :]
            nxt[:, 1:] |= cur[:, :-1]
            nxt[:, :-1] |= cur[:, 1:]
            new = nxt & ~cur
            if not new.any():
                break
            d[new] = k
            cur = nxt
        dm = d.astype(np.float32) * res
        lik = np.exp(-(dm ** 2) / (2.0 * sigma * sigma))
        # 거친 탐색용은 훨씬 너그럽게 본다. 0.4m/10도 간격으로 훑을 때
        # 정답 자세도 그만큼 어긋나 있으므로, 예민한 기준을 쓰면 정답이
        # 상위에 못 올라오고 우연히 맞은 엉뚱한 자세가 1등이 된다.
        lik_coarse = np.exp(-(dm ** 2) / (2.0 * 0.50 * 0.50))
        o = meta.get("origin") or {}
        # occ 는 '광선이 벽을 뚫는가' 를 보는 데 쓴다 (_ray_check)
        out = (lik, res, float(o.get("x", 0.0)), float(o.get("y", 0.0)),
               occ.shape[0], free, lik_coarse, occ)
        self._lik_cache = (key, out)
        return out

    @staticmethod
    def _scan_points(scan, max_pts=180):
        """라이다 스캔을 로봇 기준 (x, y) 점들로. 계산량 때문에 솎아낸다."""
        rs = np.asarray(scan.ranges, dtype=np.float64)
        n = len(rs)
        ang = scan.angle_min + np.arange(n) * scan.angle_increment
        ok = np.isfinite(rs) & (rs > scan.range_min) & (rs < min(scan.range_max, 12.0))
        rs, ang = rs[ok], ang[ok]
        if len(rs) > max_pts:
            idx = np.linspace(0, len(rs) - 1, max_pts).astype(int)
            rs, ang = rs[idx], ang[idx]
        return np.stack([rs * np.cos(ang), rs * np.sin(ang)], axis=1)

    @staticmethod
    def _ray_check(occ, free, res, ox, oy, H, pts, x, y, yaw,
                   step=0.10, stop=0.20):
        """이 자세가 말이 되는가를 일치도와 따로 본다.

        일치도만으로는 엉뚱한 자리를 못 거른다. 벽 몇 개가 우연히 겹치면
        정답과 거의 같은 점수가 나온다 - 실측에서 정답 52%, 8m 떨어진
        오답 51% 였다. 1%p 차이라 잡음이 순위를 뒤집는다.

        그래서 두 가지를 더 본다.

          blocked : 로봇에서 스캔 점까지 가는 길이 지도의 벽을 뚫고
                    지나가는 비율. 진짜 자리라면 라이다는 벽 너머를 볼 수
                    없다. 실측에서 정답 근처 후보는 22~43%, 6~8m 떨어진
                    후보는 59~81% 로 뚜렷하게 갈렸다.
          unknown : 스캔 점이 아직 안 그린 곳에 떨어지는 비율. 지도에 없는
                    영역에서 벽이 잔뜩 보인다면 그 자리가 아니다.

        blocked 만 보면 안 된다. 텅 빈 곳에 놓인 자세는 광선이 지도 밖으로
        나가 버려 '벽을 안 뚫은' 것처럼 보인다(무작위 자세 중앙값 27%,
        정답 29%). 스캔이 벽에 얹히는지를 보는 일치도와 곱해서 써야 한다.
        """
        c, sn = math.cos(yaw), math.sin(yaw)
        rx = pts[:, 0] * c - pts[:, 1] * sn
        ry = pts[:, 0] * sn + pts[:, 1] * c
        gx = ((x + rx - ox) / res).astype(np.int32)
        gy = (H - 1 - (y + ry - oy) / res).astype(np.int32)
        inb = (gx >= 0) & (gx < occ.shape[1]) & (gy >= 0) & (gy < H)
        if inb.sum() < 20:
            return 1.0, 1.0
        ho = np.zeros(len(pts), bool)
        hf = np.zeros(len(pts), bool)
        ho[inb] = occ[gy[inb], gx[inb]]
        hf[inb] = free[gy[inb], gx[inb]]
        unknown = float((~ho & ~hf).mean())

        # 로봇에서 점까지 일정 간격으로 훑는다. 끝점 바로 앞은 건너뛴다 -
        # 점이 얹힌 벽 자체를 '뚫었다' 고 세면 전부 막힌 것이 된다.
        d = np.hypot(rx, ry)
        dmax = float(d.max()) if len(d) else 0.0
        if dmax <= stop:
            return 0.0, unknown
        t = (np.arange(1, max(2, int(dmax / step)) + 1) * step)[None, :]
        valid = t < (d[:, None] - stop)
        with np.errstate(divide="ignore", invalid="ignore"):
            f = t / d[:, None]
        sx = ((x + rx[:, None] * f - ox) / res).astype(np.int32)
        sy = (H - 1 - (y + ry[:, None] * f - oy) / res).astype(np.int32)
        ok = valid & (sx >= 0) & (sx < occ.shape[1]) & (sy >= 0) & (sy < H)
        hit = np.zeros(sx.shape, bool)
        hit[ok] = occ[sy[ok], sx[ok]]
        return float((hit & ok).any(axis=1).mean()), unknown

    @staticmethod
    def _score_pose(lik, res, ox, oy, H, pts, x, y, yaw):
        """그 자세로 스캔을 지도에 올렸을 때의 평균 일치도 (0~1)."""
        c, s = math.cos(yaw), math.sin(yaw)
        gx = (((pts[:, 0] * c - pts[:, 1] * s) + x - ox) / res).astype(np.int32)
        gy = (H - 1 - (((pts[:, 0] * s + pts[:, 1] * c) + y - oy) / res)).astype(np.int32)
        m = (gx >= 0) & (gx < lik.shape[1]) & (gy >= 0) & (gy < H)
        if m.sum() < 20:
            return 0.0
        # 지도 밖으로 나간 점은 0 점으로 쳐서 '밖으로 도망가면 점수가 오르는'
        # 일이 생기지 않게 한다.
        return float(lik[gy[m], gx[m]].sum() / len(pts))

    @classmethod
    def _match_scan(cls, lik, res, ox, oy, H, pts, center, span_xy, step_xy,
                    span_yaw, step_yaw):
        cx, cy, cyaw = center
        xs = np.arange(cx - span_xy, cx + span_xy + 1e-9, step_xy)
        ys = np.arange(cy - span_xy, cy + span_xy + 1e-9, step_xy)
        yaws = np.arange(cyaw - span_yaw, cyaw + span_yaw + 1e-9, step_yaw)
        best = None
        for yaw in yaws:
            for x in xs:
                for y in ys:
                    sc = cls._score_pose(lik, res, ox, oy, H, pts, x, y, yaw)
                    if best is None or sc > best[0]:
                        best = (sc, (float(x), float(y), float(yaw)))
        return best

    @classmethod
    def _match_global(cls, lik, res, ox, oy, H, free, pts,
                      cell_m=0.4, yaw_step=math.radians(10), topn=25):
        """지도 전체에서 자세를 찾는다.

        로봇이 지도 밖이나 아주 먼 곳에 있으면 주변만 훑어서는 못 찾는다.
        자유공간(흰색) 격자만 후보로 삼아 전부 훑는다. 미지 영역은 로봇이
        있을 수 없으므로 후보에서 뺀다 — 후보 수가 크게 줄어 속도가 산다.
        """
        stepc = max(1, int(round(cell_m / res)))
        sub = free[::stepc, ::stepc]
        rows, cols = np.nonzero(sub)
        if len(rows) == 0:
            return None
        cand_x = ox + (cols * stepc + 0.5) * res
        cand_y = oy + (H - 1 - rows * stepc + 0.5) * res
        # 1등만 가져가면, 거친 간격 탓에 정답이 2~3등으로 밀렸을 때 놓친다.
        # 상위 여러 개를 남겨 각각 정밀 탐색으로 다듬은 뒤 고른다.
        out = []
        for yaw in np.arange(-math.pi, math.pi, yaw_step):
            c, sn = math.cos(yaw), math.sin(yaw)
            rx = pts[:, 0] * c - pts[:, 1] * sn
            ry = pts[:, 0] * sn + pts[:, 1] * c
            for x, y in zip(cand_x, cand_y):
                gx = ((rx + x - ox) / res).astype(np.int32)
                gy = (H - 1 - ((ry + y - oy) / res)).astype(np.int32)
                m = (gx >= 0) & (gx < lik.shape[1]) & (gy >= 0) & (gy < H)
                if m.sum() < 20:
                    continue
                out.append((float(lik[gy[m], gx[m]].sum() / len(pts)),
                            (float(x), float(y), float(yaw))))
        if not out:
            return []
        out.sort(key=lambda t: -t[0])
        # 서로 너무 가까운 후보는 하나로 친다 (같은 봉우리)
        picked = []
        for sc, (x, y, yw) in out:
            if any(math.hypot(x - px, y - py) < 0.6 and
                   abs(math.atan2(math.sin(yw - pw), math.cos(yw - pw))) < math.radians(25)
                   for _, (px, py, pw) in picked):
                continue
            picked.append((sc, (x, y, yw)))
            if len(picked) >= topn:
                break
        return picked

    # ---------------- 장치 재시작 ----------------

    def lidar_restart(self, wait=8.0):
        """라이다 스트리밍을 껐다 켠다.

        드라이버에 리셋 서비스가 없어서 스캔을 stop/start 한다. 드라이버
        프로세스 자체가 죽어 있으면 launch 의 respawn 이 되살리므로, 여기서는
        스트리밍만 다시 걸어 주면 된다.
        """
        with self.lock:
            before = self.scan.header.stamp.sec if self.scan else 0
        try:
            # 2026-10-26 라이다 2대: 드라이버 서비스는 각자 토픽 이름(/scan_front_motor,
            # /scan_rear_motor). /scan 은 scan_merger 가 둘을 합쳐 낸다.
            for act in ("stop", "start"):
                for topic in ("scan_front", "scan_rear"):
                    subprocess.run(
                        ["ros2", "run", "bluesea2", "bluesea2_client", topic, act],
                        capture_output=True, timeout=10)
                time.sleep(1.5)
        except (OSError, subprocess.SubprocessError) as e:
            return False, "스캔 재시작 실패: %s" % e
        # 다시 들어오는지 확인한다. 껐다 켰는데 안 오면 알려 줘야 한다.
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            time.sleep(0.4)
            with self.lock:
                cur = self.scan.header.stamp.sec if self.scan else 0
            if cur and cur != before:
                return True, "라이다 스캔을 다시 시작했습니다"
        return False, "재시작했지만 스캔이 아직 들어오지 않습니다 (전원/랜선 확인)"

    def camera_reset(self, wait=25.0):
        """카메라를 하드웨어 리셋한다.

        realsense 드라이버가 hw_reset 서비스를 제공한다. USB 장치가 다시
        붙는 동안 영상이 잠깐 끊기므로, 돌아올 때까지 기다렸다가 알린다.
        """
        cli = self.cam_reset_cli
        if cli is None or not cli.wait_for_service(timeout_sec=2.0):
            return False, "카메라 드라이버가 실행 중이 아닙니다"
        with self.lock:
            before = self._cam_last
        cli.call_async(EmptySrv.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            time.sleep(0.5)
            with self.lock:
                cur = self._cam_last
            if cur > before + 0.5:
                return True, "카메라를 리셋했고 영상이 다시 들어옵니다"
        return False, ("리셋 명령은 보냈지만 영상이 아직 안 들어옵니다 "
                       "(USB 재연결에 시간이 걸릴 수 있음)")

    def _clear_active_if(self, name):
        if self.active_map == name:
            self.active_map = None
            self._remember_active(None)
            self._forget_map()

    def map_delete(self, name):
        base = self.map_path(name)
        if not base:
            return False, "잘못된 이름"
        stem = os.path.basename(base)
        removed = 0
        for f in os.listdir(self.map_dir):
            # .bag 은 디렉터리라 os.remove 로는 지워지지 않는다
            if f == stem or f.startswith(stem + "."):
                if self._rm(os.path.join(self.map_dir, f)):
                    removed += 1
        if not removed:
            return False, "해당 지도를 찾지 못했습니다"
        if self.active_map == stem:
            self.active_map = None
        return True, "'%s' 삭제 (%d개 파일)" % (stem, removed)

    def mapping_clear(self):
        r, err = self._call(self.slam.get("clear"), Clear.Request())
        if err:
            return False, err
        return True, "보정 내역 초기화"

    # ---------------------------------------------------------- 수동 방향 유지
    #
    # 조이스틱을 똑바로 밀어도 차체가 옆으로 밀린다. 캐스터가 출발할 때
    # 차체를 걷어차기 때문인데, 바퀴는 고르게 도는 채로 몸만 돌아가므로
    # 엔코더로는 안 보인다(2m 직진에서 엔코더로 설명 안 되는 회전 0.82도).
    # 자이로는 그것까지 보므로, 조이스틱이 '똑바로' 일 때 그 방향을 붙든다.
    #
    # 자율주행에는 걸지 않는다. nav2 는 이미 지도를 보고 되잡고 있어서,
    # 여기서 또 잡으면 둘이 싸운다.
    #
    # ⚠ 손을 떼면 멈추는 동작을 깨면 안 된다. 그래서 지령이 JOG_HOLD_S 안에
    #   다시 안 오면 즉시 0 을 보내고 손을 뗀다.

    JOG_HZ        = 20.0
    JOG_HOLD_S    = 0.30    # 이 시간 안에 새 지령이 없으면 멈춘다
    JOG_STRAIGHT  = 0.03    # 회전 지령이 이보다 작으면 '똑바로' 로 본다
    # ⚠ 세게 잡으면 안 된다. 1.5 / 0.15 로 뒀더니 0.15 m/s 에서 회전 반경이
    #   1m 라 눈에 띄게 휘청거렸다(2026-09-16, 렉에서 나올 때). 진입 조향에서
    #   같은 실수를 했던 것을 여기서 되풀이했다.
    #   0.05 rad/s 면 0.15 m/s 에서 반경 3m — 눈으로는 직선이다.
    JOG_KP        = 0.6
    JOG_W_MAX     = 0.05
    JOG_DEAD      = 0.009   # 약 0.5도. 이 안은 쫓지 않는다

    def _on_ekf(self, m):
        q = m.pose.pose.orientation
        self.ekf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def set_twist(self, linear, angular):
        if self.estop or self.control_mode != "manual":
            return False, "manual 모드에서만, 그리고 비상정지 해제 상태에서만 가능"
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        self._jog = (time.monotonic(), float(linear), float(angular))
        return True, "ok"

    def _jog_tick(self):
        j = getattr(self, "_jog", None)
        if j is None:
            return
        t0, v, w = j
        if (time.monotonic() - t0) > self.JOG_HOLD_S:
            self._jog = None
            self._jog_ref = None
            self._pub_zero()     # 손을 뗐다. 즉시 멈춘다
            return
        if self.estop or self.control_mode != "manual":
            self._jog = None
            self._jog_ref = None
            self._pub_zero()
            return
        yaw = getattr(self, "ekf_yaw", None)
        if abs(w) > self.JOG_STRAIGHT or abs(v) < 0.01 or yaw is None:
            # 사람이 돌리는 중이거나 서 있다. 붙들 방향을 지금 것으로 갱신만.
            self._jog_ref = yaw
        else:
            if self._jog_ref is None:
                self._jog_ref = yaw
            d = self._jog_ref - yaw
            d = math.atan2(math.sin(d), math.cos(d))
            if abs(d) < self.JOG_DEAD:
                w = 0.0                  # 이미 맞았다. 그냥 곧게 간다
            else:
                w = max(-self.JOG_W_MAX,
                        min(self.JOG_W_MAX, self.JOG_KP * d))
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub_twist.publish(t)

    def stop_now(self):
        self._jog = None          # 붙들고 있던 수동 지령도 놓는다
        self._jog_ref = None
        # ⚠ 0 을 한 번 보내도 도킹·빠져나오기 타이머가 다음 주기(0.05초)에
        #   다시 지령을 낸다(2026-09-23 관리자웹 검토 P1). 작업부터 끊는다.
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            self.job_cancel()
        # ⚠ nav2 목표도 끊는다. 0 한 번만 보내면 nav2 가 다음 주기에 다시
        #   주행 지령을 내 [정지]·[비상정지]·수동 전환이 이동을 못 세웠다
        #   (2026-09-29, 지점 왕복을 넣으며 발견).
        if getattr(self, "route", {}).get("state") in self.ROUTE_BUSY:
            self.route_cancel("정지 명령으로 왕복을 멈췄습니다", stop=False)
        if getattr(self, "cdock", {}).get("state") in self.CD_BUSY:
            self.cdock_cancel("정지 명령으로 충전소 도킹을 멈췄습니다", stop=False)
        self.cleave_cancel()
        self.np_cancel()
        # 도착 뒤 제자리 방향 맞추기는 nav2 가 아니라 여기 타이머가 돌린다.
        if (getattr(self, "nav_goal", None) is not None
                or getattr(self, "_turning", False)
                or getattr(self, "_depart_timer", None) is not None
                or getattr(self, "nav_state", "") in ("sending", "active")):
            self.nav_cancel()           # 출발 기어가기·도착 후 동작까지 모두
        self._pub_zero()
        # ⚠ 0 을 보내는 것만으로는 부족하다. base_node 에 가속 제한이 걸려
        #   있어 0 도 완만하게 내려간다(0.2 m/s 에서 22mm 더 간다).
        #   비상정지는 그 제한을 건너뛰어야 한다.
        cli = self.srv.get("stop_now")
        if cli is not None and cli.service_is_ready():
            cli.call_async(Trigger.Request())


class Handler(BaseHTTPRequestHandler):
    node = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _html(self, body):
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False, indent=1).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,PATCH,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n))
        except json.JSONDecodeError:
            return {}

    def do_OPTIONS(self):
        self._json({})

    def do_HEAD(self):
        """VS Code 포트 포워딩 등 프록시가 살아있는지 HEAD 로 먼저 확인한다.
        구현이 없으면 BaseHTTPRequestHandler 가 501 을 내고 브라우저까지 전달된다."""
        p = self.path.split("?")[0].rstrip("/") or "/"
        self.send_response(200 if p in ("/", "/api") else 200)
        self.send_header("Content-Type",
                         "text/html; charset=utf-8" if p == "/"
                         else "application/json; charset=utf-8")
        self.send_header("Content-Length", "0")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

    def do_GET(self):
        n = self.node
        p = self.path.split("?")[0].rstrip("/") or "/"
        base = "http://%s" % self.headers.get("Host", "localhost")

        if p == "/":
            self._html(PAGE)
        elif p == "/api":
            self._json({
                "Device": base + "/device/info",
                "Chassis": base + "/chassis/",
                "Battery": base + "/battery-state",
                "Scan": base + "/scan",
                "Maps": base + "/maps/",
                "Map": base + "/map/info",
                "Mapping": base + "/mapping/status",
                "Log": base + "/robot-log",
                "Camera": base + "/camera/info",
                "Nodes": base + "/nodes",
                "Topics": base + "/topics",
                "Services": base + "/services/",
            })
        elif p == "/device/info":
            self._json(n.device_info())
        elif p == "/chassis":
            self._json({
                "Status": {"url": base + "/chassis/status",
                           "help": "GET/PATCH 제어모드·모터상태"},
                "Pose": {"url": base + "/chassis/pose",
                         "help": "현재 위치 (odom 기준)"},
                "Twist": {"url": base + "/chassis/twist",
                          "help": "POST 수동 주행. manual 모드에서만"},
            })
        elif p == "/chassis/status":
            self._json(n.chassis_status())
        elif p == "/chassis/pose":
            self._json(n.chassis_pose())
        elif p == "/battery-state":
            self._json(n.battery_state())
        elif p == "/charger/status":
            self._json(n.cdock_status())
        elif p == "/route/status":
            self._json(n.route_status())
        elif p == "/battery-history":
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            try:
                hours = float((q.get("hours") or ["6"])[0])
            except ValueError:
                hours = 6.0
            self._json(n.battery_history(hours))
        elif p == "/scan":
            self._json(n.scan_json())
        elif p == "/map/info":
            with n.lock:
                meta = dict(n.map_meta)
            self._json(meta or {"available": False,
                                "hint": "SLAM(slam_toolbox) 이 실행 중이어야 /map 이 나옵니다"})
        elif p == "/map.png":
            with n.lock:
                png = n.map_png
            if png is None:
                self._json({"error": "no map"}, 404)
            else:
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(png)
        elif p == "/carts":
            self._json(n.carts_json())
        elif p == "/map/edits":
            self._json(n.map_edits_json())
        elif p == "/nav/status":
            self._json(n.nav_status())
        elif p == "/pois":
            self._json(n.poi_json())
        elif p == "/job/status":
            self._json(n.job_status())
        elif p == "/nav/depart_mode":
            self._json({"mode": n.depart_mode})
        elif p == "/nav/depart_params":
            self._json(n.depart_params())
        elif p == "/nav/speed":
            self._json({"speed": n.nav_speed,
                        "min": n.NAV_SPEED_MIN,
                        "max": n.NAV_SPEED_MAX,
                        "applied": n._speed_sent})
        elif p == "/nav/path":
            self._json(n.nav_path_json())
        elif p == "/carts/specs":
            self._json(n.cart_specs_json())
        elif p == "/network":
            self._json(n.net_status())
        elif p == "/network/wifi":
            ok, msg, lst = n.wifi_scan(
                self.path.find("rescan=0") < 0)
            self._json({"ok": ok, "message": msg, "networks": lst},
                       200 if ok else 503)
        elif p == "/mapping/status":
            self._json(n.mapping_status())
        elif p == "/robot-log":
            q = dict(x.split("=") for x in self.path.split("?")[1].split("&")
                     if "=" in x) if "?" in self.path else {}
            self._json(n.robot_log(int(q.get("n", 100)), q.get("level")))
        elif p == "/camera/info":
            with n.lock:
                info = dict(n.cam_info)
                has = n.cam_jpeg is not None
                has_down = getattr(n, "cam_jpeg_down", None) is not None
            self._json({"available": has, "down_available": has_down,
                        "imaging": HAS_IMAGING, **info})
        elif p in ("/camera/color.jpg", "/camera/color.mjpg"):
            self._camera(stream=p.endswith(".mjpg"))
        elif p in ("/camera/down.jpg", "/camera/down.mjpg"):
            self._camera(stream=p.endswith(".mjpg"), key="down")
        elif p == "/maps":
            self._json(n.maps_detail())
        elif p.startswith("/maps/") and p.endswith("/info"):
            info = n.map_info(p.split("/")[2])
            self._json(info or {"error": "정보 파일이 없습니다"},
                       200 if info else 404)
        elif p.startswith("/maps/") and p.endswith("/preview.png"):
            png = n.map_preview(p.split("/")[2])
            if png is None:
                self._json({"error": "no preview"}, 404)
            else:
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(png)
        elif p == "/nodes":
            self._json({"nodes": sorted(
                "/".join(x for x in (ns.rstrip("/"), name) if x)
                for name, ns in n.get_node_names_and_namespaces())})
        elif p == "/topics":
            self._json({"topics": {k: v[0] for k, v in
                                   sorted(n.get_topic_names_and_types())}})
        elif p == "/services":
            self._json({
                "Emergency Stop": base + "/services/estop",
                "Enable Motors": base + "/services/enable",
                "Release Motors": base + "/services/release",
                "Clear Errors": base + "/services/clear_errors",
                "Stop Now": base + "/services/stop",
            })
        elif p == "/ready":
            # 스크린샷 촬영용. 이 이미지를 문서 파싱 중에 삽입하면
            # load 이벤트가 그만큼 늦춰져 폴링 데이터가 채워진 뒤 캡처된다.
            try:
                ms = int(self.path.split("ms=")[1].split("&")[0])
            except (IndexError, ValueError):
                ms = 2000
            time.sleep(min(max(ms, 0), 15000) / 1000.0)
            png = (b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
                   b"\x00\x00\x00\x01\x08\x06\x00\x00\x00\x1f\x15\xc4\x89"
                   b"\x00\x00\x00\nIDATx\x9cc\x00\x01\x00\x00\x05\x00\x01"
                   b"\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82")
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(png)))
            self.end_headers()
            self.wfile.write(png)
        elif p in ("/favicon.ico", "/favicon.png"):
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
        else:
            self._json({"error": "not found", "path": p}, 404)

    def _camera(self, stream, key="front"):
        n = self.node
        attr = "cam_jpeg_down" if key == "down" else "cam_jpeg"
        if not stream:
            with n.lock:
                jpg = getattr(n, attr, None)
            if jpg is None:
                self._json({"error": "no camera image"}, 503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        try:
            while True:
                with n.lock:
                    jpg = getattr(n, attr, None)
                if jpg is None or jpg is last:
                    time.sleep(0.03)
                    continue
                last = jpg
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpg)).encode() +
                                 b"\r\n\r\n" + jpg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_PATCH(self):
        n = self.node
        p = self.path.rstrip("/")
        if p == "/chassis/status":
            b = self._body()
            mode = b.get("control_mode")
            if mode in ("auto", "manual"):
                n.control_mode = mode
                n.stop_now()
                self._json({"ok": True, "control_mode": mode})
            else:
                self._json({"error": "control_mode must be auto or manual"}, 400)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = self.node
        p = self.path.rstrip("/")
        b = self._body()

        if p == "/chassis/twist":
            ok, msg = n.set_twist(b.get("linear", 0.0), b.get("angular", 0.0))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/services/estop":
            n.estop = bool(b.get("enable", True))
            n.stop_now()
            if n.estop:
                n.lift_stop_async()
            self._json({"ok": True, "emergency_stop_pressed": n.estop})
        elif p == "/services/stop":
            n.stop_now()
            n.lift_stop_async()
            self._json({"ok": True})
        elif p == "/mapping/start":
            ok, msg = n.mapping_start()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/mapping/stop":
            ok, msg = n.mapping_stop()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/mapping/resume":
            ok, msg = n.mapping_resume()
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/map/edits":
            ok, msg = n.map_edits_apply(b.get("shapes", []))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/nav/goto":
            why = n._loc_unsure()
            ok, msg = (False, why) if why else n.nav_goto(b.get("x"), b.get("y"), b.get("yaw", 0.0))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/nav/cancel":
            ok, msg = n.nav_cancel()
            self._json({"ok": ok, "message": msg})
        elif p == "/pois":
            ok, msg, poi = n.poi_put(b)
            out = {"ok": ok, "message": msg}
            if poi:
                out["poi"] = poi
            self._json(out, 200 if ok else 409)
        elif p == "/pois/delete":
            ok, msg = n.poi_delete(b.get("name"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/pois/goto":
            ok, msg = n.poi_goto(b.get("name"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/route/start":
            ok, msg = n.route_start(b.get("a"), b.get("b"), b.get("repeat", 1),
                                    b.get("dwell", 0.0), lift=b.get("lift", True))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/poi/scan_reset":
            ok, msg = n.poi_scan_reset(b.get("name"))
            self._json({"ok": ok, "message": msg})
        elif p == "/route/cancel":
            ok, msg = n.route_cancel()
            self._json({"ok": ok, "message": msg})
        elif p == "/charger/dock":
            ok, msg = n.cdock_start(b.get("name"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/charger/cancel":
            ok, msg = n.cdock_cancel()
            self._json({"ok": ok, "message": msg})
        elif p == "/job/start":
            ok, msg = n.job_start(b.get("poi"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/job/exit":
            ok, msg = n.job_exit()
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/job/cancel":
            ok, msg = n.job_cancel()
            self._json({"ok": ok, "message": msg})
        elif p == "/nav/depart_mode":
            ok, msg = n.set_depart_mode(b.get("mode"))
            self._json({"ok": ok, "message": msg, "mode": n.depart_mode}, 200 if ok else 400)
        elif p == "/nav/depart_params":
            ok, msg = n.set_depart_params(b)
            self._json(dict(n.depart_params(), ok=ok, message=msg), 200 if ok else 400)
        elif p == "/nav/speed":
            ok, msg, v = n.nav_speed_set(b.get("speed"))
            out = {"ok": ok, "message": msg}
            if v is not None:
                out["speed"] = v
            self._json(out, 200 if ok else 409)
        elif p == "/carts/measure":
            ok, msg, res = n.cart_measure(b.get("seconds", 6.0),
                                          b.get("front", True),
                                          b.get("name", "NEW"))
            out = {"ok": ok, "message": msg}
            if res:
                out["result"] = res
            self._json(out, 200 if ok else 409)
        elif p == "/network/wifi/connect":
            ok, msg, res = n.wifi_connect(b.get("ssid"), b.get("password"),
                                          b.get("ip"))
            out = {"ok": ok, "message": msg}
            if res:
                out.update(res)
            self._json(out, 200 if ok else 409)
        elif p == "/network/mode":
            ok, msg, res = n.net_set_mode(b.get("mode"), b.get("ssid"))
            out = {"ok": ok, "message": msg}
            if res:
                out.update(res)
            self._json(out, 200 if ok else 409)
        elif p == "/network/ap":
            ok, msg = n.ap_set(b.get("ssid"), b.get("password"), b.get("band"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/network/ip":
            ok, msg, res = n.net_set_ip(b.get("connection"), b.get("ip"))
            out = {"ok": ok, "message": msg}
            if res:
                out.update(res)
            self._json(out, 200 if ok else 409)
        elif p == "/carts/unregister":
            ok, msg = n.cart_unregister(b.get("name"))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p == "/carts/register":
            ok, msg = n.cart_register(b.get("name"), b.get("width"),
                                      b.get("depth"), b.get("tol", 0.06),
                                      b.get("note", ""))
            self._json({"ok": ok, "message": msg}, 200 if ok else 409)
        elif p.startswith("/maps/") and p.endswith("/load"):
            ok, msg = n.map_load(p.split("/")[2])
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p.startswith("/maps/") and p.endswith("/note"):
            ok, msg = n.map_note(p.split("/")[2], b.get("note", ""))
            self._json({"ok": ok, "message": msg}, 200 if ok else 404)
        elif p.startswith("/maps/") and p.endswith("/delete"):
            ok, msg = n.map_delete(p.split("/")[2])
            self._json({"ok": ok, "message": msg}, 200 if ok else 404)
        elif p == "/devices/lidar/restart":
            ok, msg = n.lidar_restart()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/devices/camera/reset":
            ok, msg = n.camera_reset()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/localization/manual":
            try:
                ok, msg = n.relocalize_manual(
                    float(b["x"]), float(b["y"]), float(b["yaw"]),
                    hold=bool(b.get("hold", False)))
            except (KeyError, TypeError, ValueError):
                ok, msg = False, "x, y, yaw 가 필요합니다"
            self._json({"ok": ok, "message": msg}, 200 if ok else 400)
        elif p == "/localization/release":
            ok, msg = n.relocalize_release()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/localization/auto":
            # apply 를 주지 않으면 후보만 돌려준다. 화면에서 미리 보여 주고
            # 사람이 확인을 눌렀을 때 /localization/manual 로 반영한다.
            ok, msg, pose = n.relocalize_auto(
                float(b.get("radius", 2.0)), apply=bool(b.get("apply", False)))
            out = {"ok": ok, "message": msg}
            if pose:
                out["pose"] = pose
            self._json(out, 200 if ok else 503)
        elif p == "/mapping/pause":
            ok, msg = n.mapping_pause()
            self._json({"ok": ok, "message": msg, "paused": n.mapping_paused},
                       200 if ok else 503)
        elif p == "/mapping/save":
            ok, msg = n.mapping_save(str(b.get("name", "map")))
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p == "/mapping/clear":
            ok, msg = n.mapping_clear()
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p in ("/lift/up", "/lift/down", "/lift/stop"):
            err = None if p == "/lift/stop" else n.lift_busy_error()
            if err:
                self._json({"ok": False, "message": err}, 409)
                return
            ok, msg = n.call_motor_service("lift_" + p.rsplit("/", 1)[1])
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        elif p in ("/services/clear_errors", "/services/enable",
                   "/services/release"):
            ok, msg = n.call_motor_service(p.rsplit("/", 1)[1])
            self._json({"ok": ok, "message": msg}, 200 if ok else 503)
        else:
            self._json({"error": "not found"}, 404)


def main():
    rclpy.init()
    node = ApiNode()
    Handler.node = node
    # 대기열(backlog)을 늘린다. 기본값 5 는 이 화면에 좁다 - 브라우저가
    # 지도·라이다·카메라 스트림과 폴링을 한꺼번에 열어 새로고침 한 번에
    # 연결이 여러 개 몰린다. 넘치면 접속이 그냥 거절된다.
    ThreadingHTTPServer.request_queue_size = 128
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", node.port), Handler)
    except OSError as e:
        if getattr(e, "errno", None) != 98:                  # EADDRINUSE
            raise
        node.get_logger().error(
            "포트 %d 를 이미 다른 프로세스가 쓰고 있습니다.\n"
            "  이전에 띄운 api_node 가 살아 있을 가능성이 큽니다. 정리 후 다시 실행하세요:\n"
            "      pkill -f 'admin.launch'; pkill -9 -f api_node\n"
            "  또는 다른 포트로 띄우려면:\n"
            "      ros2 launch und_amr_bringup admin.launch.py api_port:=8091"
            % node.port)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 5분 주기 저장 사이의 배터리 이력을 잃지 않게, 오래 걸리거나
        # 실패할 수 있는 정리보다 먼저 저장한다.
        node._bat_hist_save(wait=True)
        try:
            node._remember_pose()        # 마지막 자리 (1초 주기 사이의 이동분)
        except Exception:                                    # noqa: BLE001
            pass
        srv.shutdown()
        node.bag_stop()
        node.slam_detach_or_kill()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
