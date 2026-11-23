"""충전소 인식 거리별 치우침 측정 (2026-11-18).

사용자가 로봇을 충전소에 정확히 가운데 · 똑바로 붙여 둔 뒤 실행한다. 그 자세에서 지금 방향을 IMU(odom yaw)로
붙잡고 0.05 m/s 로 곧게 앞으로 DIST 만큼 빠지며, 스캔마다 충전소를 인식해 로봇 기준 좌우 · 방향을 기록한다.
참값은 '좌우 0 · 방향 0' 이므로 기록값(빠지는 동안 방향 변화는 빼고)이 곧 거리별 측정 치우침이다.

실행: python3 charger_calib.py [거리 m=1.0]
"""
import csv
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, "/home/und_amr1/ros2_ws/src/und_amr_bringup")
from und_amr_bringup.charger_detect import detect_scan   # noqa: E402

HL = 0.43675                  # 로봇 중심 -> 뒤끝
DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
V = 0.05
FRONT_CLEAR = DIST + 0.6      # 앞(차체 앞끝 0.437 + 이만큼) 비어 있어야 시작

rclpy.init()
n = rclpy.create_node("charger_calib")
buf = tf2_ros.Buffer()
tl = tf2_ros.TransformListener(buf, n)
pub = n.create_publisher(Twist, "/cmd_vel_noradius", 10)
st = {"scan": None}
n.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("scan", m), qos_profile_sensor_data)


def spin(sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


def pose(target, source, stamp=None):
    try:
        tr = buf.lookup_transform(target, source, rclpy.time.Time() if stamp is None else rclpy.time.Time.from_msg(stamp))
    except Exception:                                        # noqa: BLE001
        return None
    q = tr.transform.rotation
    return (tr.transform.translation.x, tr.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def stop():
    for _ in range(10):
        pub.publish(Twist())
        spin(0.05)


spin(1.5)
m = st["scan"]
o0 = pose("odom", "base_footprint")
las = pose("base_footprint", m.header.frame_id) if m is not None else None
if m is None or o0 is None or las is None:
    raise SystemExit("스캔/TF 없음 — 시작 안 함")
c = detect_scan(m.ranges, m.angle_min, m.angle_increment, las)
if c is None:
    raise SystemExit("충전소가 안 보임 — 붙어 있는지 확인")
gap0 = -c["x"] - HL
print("시작: 충전소 틈 %.0fmm, 인식 좌우 %+.1fmm 방향 %+.2f도" % (gap0 * 1000, c["y"] * 1000, math.degrees(c["yaw"])))
if gap0 > 0.06:
    raise SystemExit("충전소에 붙어 있지 않음 (틈 %.0fmm) — 시작 안 함" % (gap0 * 1000))
# 앞이 비었나 (차체 폭 안, 앞끝부터 FRONT_CLEAR)
r = np.asarray(m.ranges, float)
a = m.angle_min + np.arange(len(r)) * m.angle_increment + las[2]
ok = np.isfinite(r)
x = las[0] + r[ok] * np.cos(a[ok]); y = las[1] + r[ok] * np.sin(a[ok])
front = ((x > HL) & (x < HL + FRONT_CLEAR) & (np.abs(y) < 0.22)).sum()
if front > 3:
    raise SystemExit("앞 %.1fm 안에 물체 %d점 — 시작 안 함" % (FRONT_CLEAR, front))

rows = []
yaw_hold = o0[2]
last_stamp = None
t0 = time.time()
try:
    while time.time() - t0 < DIST / V + 8:
        rclpy.spin_once(n, timeout_sec=0.01)
        o = pose("odom", "base_footprint")
        if o is None:
            break
        gone = math.hypot(o[0] - o0[0], o[1] - o0[1])
        if gone >= DIST:
            break
        dyaw = math.atan2(math.sin(yaw_hold - o[2]), math.cos(yaw_hold - o[2]))
        t = Twist()
        t.linear.x = min(V, 0.02 + (time.time() - t0) * 0.1)
        t.angular.z = max(-0.1, min(0.1, 1.5 * dyaw))
        pub.publish(t)
        m = st["scan"]
        if m is None or m.header.stamp == last_stamp:
            continue
        last_stamp = m.header.stamp
        c = detect_scan(m.ranges, m.angle_min, m.angle_increment, las)
        os_ = pose("odom", "base_footprint", m.header.stamp) or o
        if c is None:
            continue
        # 참값: 시작 때 로봇 축 = 스테이션 축. 지금 로봇 기준으로 옮긴 참 스테이션과 인식 스테이션을 비교한다.
        yaw_now = math.atan2(math.sin(os_[2] - o0[2]), math.cos(os_[2] - o0[2]))     # 시작 대비 회전
        # 참 스테이션 앞면 중심(odom): 시작 로봇 중심에서 뒤로 HL (+ 시작 틈)
        fx = o0[0] - (HL + gap0) * math.cos(o0[2]); fy = o0[1] - (HL + gap0) * math.sin(o0[2])
        cs, sn = math.cos(-os_[2]), math.sin(-os_[2])
        tx = cs * (fx - os_[0]) - sn * (fy - os_[1]); ty = sn * (fx - os_[0]) + cs * (fy - os_[1])
        true_gap = -tx - HL
        # 인식: 로봇 기준 앞면 중심 (c.x, c.y), 바깥 방향 c.yaw (로봇 쪽 = +x 쪽이라 약 0도)
        meas_lat = c["y"] - ty                       # 로봇 기준 옆 방향 차이 [m]
        meas_yaw = math.atan2(math.sin(c["yaw"] + yaw_now), math.cos(c["yaw"] + yaw_now))
        rows.append((time.time() - t0, true_gap, -c["x"] - HL, meas_lat, math.degrees(meas_yaw), math.degrees(yaw_now), c["n"]))
finally:
    stop()

with open("/home/und_amr1/projects/UND_AMR/logs/charger_calib_%s.csv" % time.strftime("%H%M%S"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "true_gap", "meas_gap", "lat_bias", "yaw_bias_deg", "robot_turn_deg", "n_pts"])
    w.writerows(rows)
R = np.array(rows)
print("기록 %d개" % len(R))
for lo in np.arange(0.0, DIST, 0.1):
    k = (R[:, 1] >= lo) & (R[:, 1] < lo + 0.1)
    if k.sum() == 0:
        continue
    print("틈 %.1f~%.1fm: 좌우 치우침 %+5.1f±%.1fmm, 방향 치우침 %+5.2f±%.2f도, 틈 차이 %+5.1fmm (%d개)" % (
        lo, lo + 0.1, R[k, 3].mean() * 1000, R[k, 3].std() * 1000, R[k, 4].mean(), R[k, 4].std(),
        (R[k, 2] - R[k, 1]).mean() * 1000, k.sum()))
