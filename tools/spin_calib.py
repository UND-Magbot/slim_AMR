#!/usr/bin/env python3
"""제자리 회전 시험 — 회전 중심이 base_footprint 에서 얼마나 떨어져 있나 (라이다 위치 보정).

    python3 tools/spin_calib.py            +90도 x4, -90도 x4

⚠ 로봇이 제자리에서 돈다 (앞뒤로는 안 움직인다). 왕복·이동을 멈춘 뒤 쓴다.
  돌기 전에 차체가 쓸고 지나가는 원(반지름 SWEEP_R) 안에 물체가 있으면 시작하지 않는다.
  /cmd_vel_noradius (base_node 우선 통로) 로 직접 돌린다. 끝나거나 오류면 0 을 보낸다.

방법: 90도 돌 때마다 서서 라이다 스캔을 찍고, 돌기 전 스캔과 점-선 ICP 로 맞춰
      '실제로 옮겨진 만큼' 을 잰다. 오도메트리는 제자리라고 본다. 회전 중심이
      base_footprint 에서 c 만큼 떨어져 있으면 base_footprint 는 (I - R(θ))c 만큼
      옮겨진다 — 모든 회전에 하나의 c 가 맞으면(좌·우 모두) 라이다 위치 설정이
      틀린 것이고, 회전마다 제각각이면 바퀴 미끄러짐이다.

왜 (2026-10-14): 왕복 중 제자리 회전 4번(-72~-86도) 모두 base_footprint 가 실제로
약 5cm 옮겨졌다(오도메트리는 0). 회전 중심 (-38~-39, +2~+3)mm. 모델(URDF)의
구동축 위치는 '차체 정중앙 가정, 줄자 확인 필요' 였다. 줄자로는 못 잰다.
"""
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/und_amr_bringup"))
from und_amr_bringup import scan_icp   # noqa: E402

D = math.degrees
W = 0.30                 # rad/s
STEP = math.radians(90)
SWEEP_R = 0.55           # 차체 외접 반경 0.455 + 여유
BODY = (0.50, -0.45, 0.16)   # 자기 몸에 맞는 점 거르기: 앞, 뒤, 반폭 [m] (넉넉히)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    n = Node("spin_calib")
    pub = n.create_publisher(Twist, "/cmd_vel_noradius", 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    lock = threading.Lock()
    S = {"scan": None, "v": 0.0, "w": 0.0, "od": None}

    def on_scan(m):
        with lock:
            S["scan"] = (time.time(), m)

    def on_odom(m):
        with lock:
            S["v"], S["w"] = m.twist.twist.linear.x, m.twist.twist.angular.z
            S["od"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
    n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odometry/filtered", on_odom, 20)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    def send(w):
        t = Twist()
        t.angular.z = float(w)
        pub.publish(t)

    def stop():
        for _ in range(10):
            send(0.0)
            time.sleep(0.03)

    def still():
        with lock:
            return abs(S["v"]) < 0.003 and abs(S["w"]) < 0.01

    las = None
    t0 = time.time()
    while las is None and time.time() - t0 < 10:
        time.sleep(0.2)
        with lock:
            m = S["scan"][1] if S["scan"] else None
        if m is None or S["od"] is None:
            continue
        try:
            tr = buf.lookup_transform("base_footprint", m.header.frame_id, rclpy.time.Time())
            las = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
        except Exception:                                    # noqa: BLE001
            pass
    if las is None:
        sys.exit("스캔/TF/오도메트리를 못 받았다")

    def grab():
        """선 뒤 새 스캔 하나 (점 전부) 와 그때 오도메트리."""
        t_s = time.time()
        while not still() and time.time() - t_s < 3.0:
            time.sleep(0.05)
        time.sleep(0.8)
        t_a = time.time()
        while time.time() - t_a < 1.0:
            with lock:
                ts, m = S["scan"]
                od = S["od"]
            if ts > t_a + 0.08:
                return scan_icp.scan_points(m, las, step=1), od
            time.sleep(0.03)
        return None, None

    # 둘레 확인
    pts, _ = grab()
    if pts is None:
        sys.exit("스캔 없음")
    r = np.hypot(pts[:, 0], pts[:, 1])
    inside_body = (pts[:, 0] < BODY[0]) & (pts[:, 0] > BODY[1]) & (np.abs(pts[:, 1]) < BODY[2])
    near = (r < SWEEP_R) & ~inside_body
    if near.sum() > 3:
        sys.exit("차체가 쓸고 지나갈 원(%.2fm) 안에 점 %d개 — 시작하지 않는다" % (SWEEP_R, near.sum()))
    print("둘레 비었음 (원 %.2fm 안 점 %d개). 시작" % (SWEEP_R, near.sum()), flush=True)

    res = []
    try:
        for sign in (+1, -1):
            for k in range(4):
                p0, o0 = grab()
                # 오도메트리 방향으로 90도 돈다 (미리 끊는 몫: 0.3 rad/s 에서 약 3도)
                start = o0[2]
                t_s = time.time()
                while True:
                    with lock:
                        yaw = S["od"][2]
                    turned = math.atan2(math.sin(yaw - start), math.cos(yaw - start)) * sign
                    if turned < 0 and time.time() - t_s > 1.0 and abs(turned) > math.pi / 2:
                        turned += 2 * math.pi
                    if turned >= STEP - math.radians(3) or time.time() - t_s > 15:
                        break
                    ramp = min(1.0, (time.time() - t_s) / 1.0)
                    send(sign * max(0.1, W * ramp))
                    time.sleep(0.05)
                stop()
                p1, o1 = grab()
                if p0 is None or p1 is None:
                    print("  스캔 못 얻음 — 건너뜀", flush=True)
                    continue
                odr = scan_icp.relative(o0, o1)
                real, fit, inl = scan_icp.icp(p1[::3], p0, odr, dst_n=scan_icp.normals(p0))
                res.append((odr, real, inl))
                print("  %+d번째 회전 %+6.1f도 | 실제 이동 앞 %+6.1f 좌 %+6.1f mm, 실제 회전 %+6.2f도 | 오도메트리 앞 %+5.1f 좌 %+5.1f mm | 맞음 %.0f%%"
                      % (sign * (k + 1), D(odr[2]), real[0] * 1000, real[1] * 1000, D(real[2]),
                         odr[0] * 1000, odr[1] * 1000, inl * 100), flush=True)
    finally:
        stop()

    good = [x for x in res if x[2] >= 0.6]
    if len(good) < 3:
        print("쓸 만한 회전이 부족하다")
        return
    # (I - R(θ)) c = 실제 이동 - 오도메트리 이동  을 모두 쌓아 c 를 푼다
    A, b = [], []
    for odr, real, _ in good:
        th = real[2]
        c, s = math.cos(th), math.sin(th)
        A += [[1 - c, s], [-s, 1 - c]]
        b += [real[0] - odr[0], real[1] - odr[1]]
    A, b = np.array(A), np.array(b)
    cxy, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = (A @ cxy - b).reshape(-1, 2)
    rr = np.hypot(resid[:, 0], resid[:, 1]) * 1000
    print("== 회전 중심 (base_footprint 기준): 앞 %+.1f mm, 좌 %+.1f mm | 회전마다 남는 차이 평균 %.1f 최대 %.1f mm"
          % (cxy[0] * 1000, cxy[1] * 1000, rr.mean(), rr.max()))
    for sign in (+1, -1):
        sub = [x for x in good if x[0][2] * sign > 0]
        if len(sub) >= 2:
            A2, b2 = [], []
            for odr, real, _ in sub:
                th = real[2]
                c, s = math.cos(th), math.sin(th)
                A2 += [[1 - c, s], [-s, 1 - c]]
                b2 += [real[0] - odr[0], real[1] - odr[1]]
            c2, *_ = np.linalg.lstsq(np.array(A2), np.array(b2), rcond=None)
            print("   %s 회전만: 앞 %+.1f mm, 좌 %+.1f mm" % ("왼쪽(+)" if sign > 0 else "오른쪽(-)",
                                                         c2[0] * 1000, c2[1] * 1000))
    dyaw = [D(real[2] - odr[2]) for odr, real, _ in good]
    print("   회전각: 실제 - 오도메트리 평균 %+.2f도 (90도당)" % (sum(dyaw) / len(dyaw)))
    ex.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
