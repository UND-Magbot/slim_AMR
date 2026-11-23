"""map->odom 보정을 한 번에 꺾지 않고 나눠서 반영한다 (2026-11-16).

⚠ 주행 S자: 위치추정 SLAM 은 0.2m 움직일 때마다 스캔을 지도에 맞추고 그 결과로
  map->odom 을 '계단' 처럼 고친다. 2026-11-16 기록에서 약 0.4m 마다 옆으로
  5~15mm (최대 65mm) 씩 한 번에 튀었고, 그 사이는 0.05mm 로 고요했다. 바퀴·IMU
  궤적은 곧은데 지도 위 자세만 계단처럼 옆으로 튀고, nav2 는 그때마다 경로로
  돌아가려 꺾어 눈에 S자로 보였다.

이 노드가 SLAM 대신 map->odom 을 낸다. SLAM 은 transform_publish_period:=0 으로
TF 를 안 내고(/pose 만 낸다), 여기서 /pose 와 그 스캔 시각의 odom->base 로 SLAM 이
냈을 map->odom 을 똑같이 계산한 뒤, 로봇 자리 기준으로 조금씩 따라간다.
  - 주행 중   : 시간상수 TAU_MOVE (보정을 0.x 초에 걸쳐 편다)
  - 멈춰 있음 : 시간상수 TAU_STILL (도착 정밀도용, 금방 맞춘다)
  - 크게 다름 : SNAP_M / SNAP_DEG 넘으면 바로 맞춘다 (위치 다시 잡기·재시작)

SLAM 의 transform_publish_period 가 0 일 때만 낸다. 매핑 중이거나 예전 방식으로 뜬
SLAM 이면(값이 0 이 아님) 아무것도 안 낸다 — TF 를 두 군데서 내지 않게.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import tf2_ros

TAU_MOVE = 0.8
TAU_STILL = 0.2
SNAP_M = 0.25
SNAP_DEG = 8.0
HZ = 50.0
FUTURE_S = 0.2          # slam_toolbox transform_timeout 과 같게 (미래 시각으로 낸다)
STILL_V = 0.01
STILL_W = 0.02
PARAM_PERIOD_S = 1.0
PARAM_FAIL_MAX = 5      # SLAM 이 바빠 몇 번 답을 못 해도 끄지 않는다

MAP, ODOM, BASE = "map", "odom", "base_footprint"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """a∘b (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2]))


def target_from(map_base, odom_base):
    """SLAM 이 내는 map->odom = map_base ∘ odom_base⁻¹."""
    return compose(map_base, inverse(odom_base))


def far(a, b, snap_m=SNAP_M, snap_deg=SNAP_DEG):
    return (math.hypot(a[0] - b[0], a[1] - b[1]) > snap_m
            or abs(wrap(a[2] - b[2])) > math.radians(snap_deg))


def blend(cur, tgt, odom_base, alpha):
    """로봇 자리에서 cur→tgt 로 alpha 만큼. 로봇 지도 자세를 직선으로 옮긴다.

    map->odom 의 (x, y, yaw) 를 따로 섞으면 odom 원점이 멀 때 회전 성분이
    로봇 자리를 엉뚱하게 흔든다. 로봇 자세 두 개를 섞고 거꾸로 푼다.
    """
    pc = compose(cur, odom_base)
    pt = compose(tgt, odom_base)
    p = (pc[0] + alpha * (pt[0] - pc[0]), pc[1] + alpha * (pt[1] - pc[1]),
         wrap(pc[2] + alpha * wrap(pt[2] - pc[2])))
    return target_from(p, odom_base)


def _tf_xyyaw(tr):
    q = tr.transform.rotation
    return (tr.transform.translation.x, tr.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class MapOdomSmoother(Node):
    def __init__(self):
        super().__init__("map_odom_smoother")
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.cur = None
        self.tgt = None
        self.v = self.w = 0.0
        self.enabled = False
        self._pending = None    # 켜지기 전에 온 /pose (2026-11-17 재부팅: 첫 /pose 를 버려 지도 좌표가 안 생김)
        self._param_fail = 0
        self._param_busy = False
        self._last = time.monotonic()
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self._on_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.pcli = self.create_client(GetParameters, "/slam_toolbox/get_parameters")
        self.create_timer(1.0 / HZ, self._tick)
        self.create_timer(PARAM_PERIOD_S, self._check_slam)

    # SLAM 이 TF 를 안 내도록 떠 있을 때만 켠다
    def _check_slam(self):
        if self._param_busy:
            return
        if not self.pcli.service_is_ready():
            self._param_result(None)
            return
        req = GetParameters.Request()
        req.names = ["transform_publish_period"]
        self._param_busy = True
        fut = self.pcli.call_async(req)
        fut.add_done_callback(self._param_done)

    def _param_done(self, fut):
        self._param_busy = False
        try:
            vals = fut.result().values
            self._param_result(vals[0].double_value if vals and vals[0].type == 3 else -1.0)
        except Exception:
            self._param_result(None)

    def _param_result(self, period):
        if period is None:
            self._param_fail += 1
            if self._param_fail < PARAM_FAIL_MAX or not self.enabled:
                return
            on = False
        else:
            self._param_fail = 0
            on = period == 0.0
        if on != self.enabled:
            self.get_logger().info("map->odom 부드럽게 내기 %s" % ("켬" if on else "끔 (SLAM 이 직접 냄)"))
            self.enabled = on
            if not on:
                self.cur = self.tgt = None
            elif self._pending is not None:
                m, self._pending = self._pending, None
                self._on_pose(m)

    def _on_odom(self, m):
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z

    def _odom_base(self, stamp=None):
        try:
            tr = self.buf.lookup_transform(ODOM, BASE, stamp if stamp is not None else Time())
            return _tf_xyyaw(tr)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

    def _on_pose(self, m):
        if not self.enabled:
            # ⚠ 2026-11-17 재부팅 때 SLAM 은 첫 스캔에서 /pose 를 한 번 내고, 서 있으면 다시 안 낸다.
            #   그게 켜지기(1초 뒤 파라미터 확인) 전에 와서 버려졌고, 지도 좌표가 없어 nav2 가 안 떴다.
            self._pending = m
            return
        ob = self._odom_base(Time.from_msg(m.header.stamp))
        if ob is None:
            ob = self._odom_base()
            if ob is None:
                return
        q = m.pose.pose.orientation
        mb = (m.pose.pose.position.x, m.pose.pose.position.y,
              math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        tgt = target_from(mb, ob)
        if self.cur is None or far(self.cur, tgt):
            if self.cur is not None:
                self.get_logger().info("SLAM 보정이 커서 바로 맞춤 (%.3fm, %.1f°)" % (
                    math.hypot(tgt[0] - self.cur[0], tgt[1] - self.cur[1]),
                    math.degrees(abs(wrap(tgt[2] - self.cur[2])))))
            self.cur = tgt
        self.tgt = tgt

    def _tick(self):
        now = time.monotonic()
        dt = min(now - self._last, 0.2)
        self._last = now
        if not self.enabled or self.cur is None:
            return
        if self.tgt is not None and self.tgt != self.cur:
            ob = self._odom_base()
            if ob is not None:
                still = abs(self.v) < STILL_V and abs(self.w) < STILL_W
                tau = TAU_STILL if still else TAU_MOVE
                a = 1.0 - math.exp(-dt / tau)
                self.cur = blend(self.cur, self.tgt, ob, a)
                if not far(self.cur, self.tgt, 0.0002, 0.01):
                    self.cur = self.tgt
        t = TransformStamped()
        st = self.get_clock().now() + Duration(seconds=FUTURE_S)
        t.header.stamp = st.to_msg()
        t.header.frame_id = MAP
        t.child_frame_id = ODOM
        t.transform.translation.x, t.transform.translation.y = self.cur[0], self.cur[1]
        t.transform.rotation.z = math.sin(self.cur[2] / 2)
        t.transform.rotation.w = math.cos(self.cur[2] / 2)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    n = MapOdomSmoother()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
