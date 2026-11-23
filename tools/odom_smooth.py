#!/usr/bin/env python3
"""오도메트리 방향이 매끄러운지 본다 — nav2 붙이기 전 점검.

직진 중 바퀴 오도메트리의 방향이 ±3도씩 오가는 것이 보였다(2026-09-11).
이것이 진짜 로봇의 흔들림인지, 바퀴 계측만의 잡음인지 갈라야 한다.
어느 쪽이든 자율주행에서 문제가 된다.

    로봇이 실제로 흔들림  -> 기구 문제 (캐스터 등)
    바퀴 계측만 흔들림    -> 컨트롤러가 그 가짜 흔들림을 쫓아가며
                            로봇을 실제로 흔들게 된다

세 가지를 같은 주행에서 나란히 재고, 각각에서 '직선 추세를 뺀 나머지'가
얼마나 되는지 본다. 추세는 실제 휨이고, 나머지가 떨림이다.

    바퀴 오도메트리   /odom            이 설정을 따라 계산됨
    자이로            /imu/base_link   독립. 200Hz 라 떨림을 가장 잘 본다
    지도              map->base_link   독립. 다만 15Hz 라 느린 것만 보인다

    python3 tools/odom_smooth.py --meters 1.5

앞쪽에 2.5 m 쯤 비워 두세요. 0.4m 안에 뭔가 들어오면 즉시 멈춥니다.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/home/und_amr1/projects/UND_AMR/tools")
from safe_stop import install_safe_stop          # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--meters", type=float, default=1.5)
ap.add_argument("--speed", type=float, default=0.15)
ap.add_argument("--prealign", type=float, default=0.3)
a = ap.parse_args()

STOP_M = 0.40


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class S(Node):
    def __init__(s):
        super().__init__("odom_smooth")
        s.odom = None
        s.scan = None
        s.gz = 0.0
        s.buf = Buffer()
        s.lis = TransformListener(s.buf, s)
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(Odometry, "/odom", lambda m: setattr(s, "odom", m), 10)
        # nav2 가 실제로 쓰는 것은 /odom 이 아니라 EKF 출력이다.
        # EKF 는 바퀴에서 x,y 속도만 받고 방위는 자이로에서 가져오도록
        # 설정돼 있으므로(config/ekf.yaml), 바퀴 yaw 잡음은 여기서 걸러진다.
        s.ekf = None
        s.create_subscription(Odometry, "/odometry/filtered",
                              lambda m: setattr(s, "ekf", m), 10)
        s.create_subscription(LaserScan, "/scan", lambda m: setattr(s, "scan", m),
                              qos_profile_sensor_data)
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        s.create_subscription(Imu, "/imu/base_link",
                              lambda m: setattr(s, "gz", m.angular_velocity.z), q)
        s.cli = s.create_client(Trigger, "/base_node/enable")

    def sp(s, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(s, timeout_sec=0.02)

    def ekf_yaw(s):
        return yaw_of(s.ekf.pose.pose.orientation) if s.ekf else None

    def map_yaw(s):
        try:
            t = s.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return yaw_of(t.transform.rotation)

    def front(s):
        m = s.scan
        if m is None:
            return 99.0
        v = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min < r < m.range_max):
                continue
            d = math.degrees(m.angle_min + i * m.angle_increment)
            if -40.0 <= d <= 40.0:
                v.append(r)
        return min(v) if v else 99.0

    def xy(s):
        p = s.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)


def detrend_rms(t, y):
    """직선 추세를 뺀 나머지의 rms [deg]. 추세=실제 휨, 나머지=떨림."""
    n = len(y)
    if n < 4:
        return 0.0, 0.0
    mt = statistics.mean(t)
    my = statistics.mean(y)
    den = sum((x - mt) ** 2 for x in t)
    slope = sum((t[i] - mt) * (y[i] - my) for i in range(n)) / den if den else 0.0
    res = [y[i] - (my + slope * (t[i] - mt)) for i in range(n)]
    return math.sqrt(sum(r * r for r in res) / n), slope


def main():
    rclpy.init()
    n = S()
    install_safe_stop(n)
    n.sp(3.0)
    if n.odom is None or n.scan is None:
        print("!! /odom 또는 /scan 미수신")
        return
    if n.map_yaw() is None:
        print("!! map->base_link 없음 — 위치추정이 돌고 있어야 합니다")
        return
    if not n.cli.wait_for_service(timeout_sec=5.0):
        print("!! /base_node/enable 없음")
        return
    clr = n.front()
    print("=" * 66)
    print("  전방 여유 %.2f m (필요 %.2f m)" % (clr, a.meters + 0.8))
    if clr < a.meters + 0.8:
        print("!! 여유 부족")
        return

    n.cli.call_async(Trigger.Request())
    n.sp(2.5)

    # 자이로 치우침을 먼저 뺀다. 안 빼면 추세에 섞인다.
    bias, cnt, t0 = 0.0, 0, time.time()
    while time.time() - t0 < 2.0:
        n.sp(0.05)
        bias += n.gz
        cnt += 1
    bias = bias / cnt if cnt else 0.0
    print("  자이로 바이어스 %+.5f rad/s" % bias)

    if a.prealign > 0:
        print("  캐스터 정렬 %.2f m ..." % a.prealign)
        drive(n, a.prealign, bias, record=False)
        n.sp(1.0)

    print("  측정 주행 %.2f m ..." % a.meters)
    rec = drive(n, a.meters, bias, record=True)
    if not rec:
        print("!! 기록이 없습니다")
        return

    t = [r[0] for r in rec]
    od = [math.degrees(r[1]) for r in rec]
    gy = [math.degrees(r[2]) for r in rec]
    mp = [math.degrees(r[3]) for r in rec if r[3] is not None]
    tm = [rec[i][0] for i in range(len(rec)) if rec[i][3] is not None]

    print()
    print("=" * 66)
    print("  주행 %.2f m, 표본 %d 개" % (rec[-1][4], len(rec)))
    print()
    print("  %-18s %10s %12s" % ("", "전체 회전", "떨림(추세 뺀 rms)"))
    ro, so = detrend_rms(t, od)
    rg, sg = detrend_rms(t, gy)
    print("  %-18s %+9.2f도 %11.2f도" % ("바퀴 오도메트리", od[-1] - od[0], ro))
    print("  %-18s %+9.2f도 %11.2f도" % ("자이로", gy[-1] - gy[0], rg))
    ek = [math.degrees(r[5]) for r in rec if len(r) > 5 and r[5] is not None]
    te = [rec[i][0] for i in range(len(rec)) if len(rec[i]) > 5 and rec[i][5] is not None]
    if len(ek) > 4:
        re_, se_ = detrend_rms(te, ek)
        print("  %-18s %+9.2f도 %11.2f도  <-- nav2 가 쓰는 값"
              % ("EKF 융합", ek[-1] - ek[0], re_))
    if len(mp) > 4:
        rm, sm = detrend_rms(tm, mp)
        print("  %-18s %+9.2f도 %11.2f도  (15Hz 라 느린 것만 보임)"
              % ("지도", mp[-1] - mp[0], rm))
    print()
    print("  판정")
    if len(ek) > 4:
        re2, _ = detrend_rms(te, ek)
        if re2 < 1.0:
            print("   EKF 출력 떨림 %.2f도 — nav2 는 이 값을 쓰므로 바퀴 잡음의"
                  % re2)
            print("   영향을 받지 않는다. ekf.yaml 이 바퀴에서 방위를 안 받고")
            print("   자이로에서 받도록 돼 있기 때문이다.")
        else:
            print("   ⚠ EKF 출력도 %.2f도 떤다. 바퀴 잡음이 새어 들고 있다." % re2)
            print("     ekf.yaml 의 odom0_config 에서 yaw 가 켜져 있는지 확인할 것.")
        print()
    if rg < 1e-9:
        print("   자이로 값이 없다. /imu/base_link 확인")
    elif ro > rg * 2.0 and ro > 0.5:
        print("   바퀴만 %.2f도 떨고 자이로는 %.2f도다 (%.1f배)."
              % (ro, rg, ro / rg))
        print("   로봇이 실제로 흔들리는 것이 아니라 바퀴 계측의 잡음이다.")
        print("   ekf.yaml 이 바퀴에서 방위를 안 받게 돼 있으므로(odom0_config 의")
        print("   yaw/vyaw 가 false), nav2 경로에는 영향이 없다. 다만 /odom 을")
        print("   직접 쓰는 도구나 진단은 이 잡음을 그대로 본다는 점은 알아둘 것.")
    elif max(ro, rg) > 2.0:
        # 기준 0.5도는 rms 5도짜리 계측 잡음을 잡으려고 잡았던 값이라
        # 펌웨어 수정(2026-09-11) 뒤에는 과민하다. 자이로 자체가 0.9도쯤
        # 나오므로, 그보다 확실히 큰 2도를 넘을 때만 기구를 의심한다.
        print("   바퀴 %.2f도 / 자이로 %.2f도 — 둘 다 크다." % (ro, rg))
        print("   두 독립 센서가 같이 떨면 로봇이 실제로 흔들리는 것이다.")
        print("   계측이 아니라 기구 쪽(캐스터·서스펜션)을 볼 것.")
    else:
        print("   바퀴 %.2f도 / 자이로 %.2f도 — 둘 다 작다. 문제 없음." % (ro, rg))
        if ro <= rg:
            print("   바퀴가 자이로만큼(또는 그보다) 매끄럽다.")
    print("=" * 66)
    n.destroy_node()
    rclpy.shutdown()


def drive(n, dist, bias, record):
    x0, y0, th0 = n.xy()
    m0 = n.map_yaw()
    e0 = n.ekf_yaw()
    t = Twist()
    t.linear.x = a.speed
    rec = []
    gy = 0.0
    tg = time.time()
    t0 = time.time()
    lim = dist / abs(a.speed) * 1.5 + 3.0
    while time.time() - t0 < lim:
        n.pub.publish(t)
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        dt = now - tg
        tg = now
        gy += (n.gz - bias) * dt
        x, y, th = n.xy()
        d = math.hypot(x - x0, y - y0)
        if record:
            mc = n.map_yaw()
            ec = n.ekf_yaw()
            rec.append((now - t0, wrap(th - th0), gy,
                        wrap(mc - m0) if mc is not None else None, d,
                        wrap(ec - e0) if (ec is not None and e0 is not None)
                        else None))
        if d >= dist:
            break
        if n.front() < STOP_M:
            print("      !! 0.4m 안에 장애물 — 정지")
            break
    for _ in range(12):
        n.pub.publish(Twist())
        n.sp(0.05)
    n.sp(1.0)
    return rec


if __name__ == "__main__":
    main()
