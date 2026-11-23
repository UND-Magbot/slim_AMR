#!/usr/bin/env python3
"""제자리 회전 중심 실측 — 제자리에서 돌 때 로봇 자리가 실제로 얼마나 옮겨지나 (2026-11-20).

    python3 tools/spin_center.py                 지금 자리에서 기본 순서로 돈다
    python3 tools/spin_center.py --w 0.45        회전 속도 [rad/s]
    python3 tools/spin_center.py --seq 180,-180,360

⚠ 로봇을 움직인다(/cmd_vel_noradius 로 직접 돈다). 서 있고 주변이 비어 있을 때만.
  Ctrl-C 하면 0 을 보내고 멈춘다.

왜 (2026-11-20)
  POI 1 에 회전 없이(충전소 쪽에서) 들어왔을 때와 180도 돌고 선 실제 자리가 늘 +15/-33mm 달랐다
  (SLAM 은 -16mm 만 봄). 회전 중에는 SLAM 이 위치를 안 고치고 오도메트리는 base_footprint 를
  축으로 돈다고 본다 — 실제 회전 중심이 어긋나 있으면 돈 만큼 자리가 옮겨지는데 아무도 모른다.

방법
  멈출 때마다 스캔 5장 + SLAM(map) · 오도메트리(odom) 자세를 찍는다. 스캔끼리 ICP 로 맞춰 실제 이동을 구한다.
  · 같은 방향끼리(0도 대 0도) : 라이다 설치 오차가 상쇄 — 한 번 돌고 올 때 미끄러지는 양만 남는다
  · 180도 차이              : 회전 중심 어긋남 (I-R)c 에 라이다 설치 오차가 섞인다
"""
import argparse
import math
import os
import statistics as st
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/und_amr_bringup"))
from und_amr_bringup import scan_icp   # noqa: E402

N_SCANS = 5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--w", type=float, default=0.70)
    ap.add_argument("--seq", default="180,-180,180,-180,-180,180,-180,180,360,-360")
    ap.add_argument("--settle", type=float, default=1.5)
    a = ap.parse_args()
    seq = [float(x) for x in a.seq.split(",")]

    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("spin_center")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    pub = node.create_publisher(Twist, "/cmd_vel_noradius", 10)
    scans, lock = [], threading.Lock()

    def on_scan(m):
        with lock:
            scans.append((time.time(), m))
            del scans[:-40]
    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def tf2d(p, c):
        tr = buf.lookup_transform(p, c, rclpy.time.Time())
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)

    t0 = time.time()
    laser = None
    while laser is None and time.time() - t0 < 15:
        try:
            with lock:
                fid = scans[-1][1].header.frame_id if scans else None
            if fid:
                laser = tf2d("base_footprint", fid)
        except Exception:                                    # noqa: BLE001
            time.sleep(0.2)
    if laser is None:
        sys.exit("TF 를 못 읽었다")

    def capture():
        time.sleep(a.settle)
        t_a = time.time()
        while True:
            with lock:
                got = [m for (ts, m) in scans if ts > t_a]
            if len(got) >= N_SCANS or time.time() - t_a > 3.0:
                break
            time.sleep(0.05)
        pts = np.concatenate([scan_icp.scan_points(m, laser, step=2) for m in got[:N_SCANS]])
        return {"pts": pts, "odom": tf2d("odom", "base_footprint"), "map": tf2d("map", "base_footprint")}

    def spin(deg):
        """odom 방향으로 deg 만큼 돈다 (도착 회전과 같은 방식: 비례 감속, 저속에서 끊음)."""
        y0 = tf2d("odom", "base_footprint")[2]
        target, d = math.radians(deg), (1.0 if deg > 0 else -1.0)
        done, t_s, prev = 0.0, time.time(), y0
        while True:
            y = tf2d("odom", "base_footprint")[2]
            dy = math.atan2(math.sin(y - prev), math.cos(y - prev))
            prev, done = y, done + dy
            rem = (target - done) * d
            el = time.time() - t_s
            w = min(a.w * min(1.0, el / 1.0), max(0.10, 1.8 * rem))
            if rem <= max(0.012, w * 0.25) or el > 30:
                break
            t = Twist()
            t.angular.z = d * w
            pub.publish(t)
            time.sleep(0.05)
        stop()

    caps = []
    try:
        caps.append(capture())
        head = 0.0
        print("시작: SLAM (%.3f, %.3f, %.1f도)" % (caps[0]["map"][0], caps[0]["map"][1],
                                               math.degrees(caps[0]["map"][2])), flush=True)
        for k, deg in enumerate(seq):
            spin(deg)
            head += deg
            c = capture()
            c["head"] = head
            caps.append(c)
            print("  %2d/%d  %+5.0f도 돌고 섬 (누적 %+.0f)" % (k + 1, len(seq), deg, head), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
    caps[0]["head"] = 0.0

    # 기준: 방향별 첫 촬영
    refs = {}
    print("\n%-6s %-6s | %-24s | %-22s | %-22s | %s" % ("회", "방향", "실제(라이다) 앞/왼 mm 도", "오도메트리 앞/왼 mm",
                                                       "SLAM 앞/왼 mm", "ICP 잔차·맞음"))
    rows = {"same": [], "cross": []}
    for i, c in enumerate(caps):
        hk = int(round(((c["head"] % 360) + 360) % 360))
        hk = 0 if hk in (0, 360) else hk
        base = caps[0]
        guess = scan_icp.relative(base["odom"], c["odom"])
        (rx, ry, ryaw), fit, inl = scan_icp.icp(c["pts"], base["pts"], guess=guess)
        od = scan_icp.relative(base["odom"], c["odom"])
        sl = scan_icp.relative(base["map"], c["map"])
        kind = "same" if hk == 0 else "cross"
        if i > 0:
            rows[kind].append((rx, ry, ryaw, od, sl))
        dyaw = math.atan2(math.sin(ryaw - math.radians(hk)), math.cos(ryaw - math.radians(hk)))
        print("%-6d %-6d | %+7.1f %+7.1f %+6.2f | %+7.1f %+7.1f | %+7.1f %+7.1f | %4.1fmm %3.0f%%"
              % (i, hk, rx * 1e3, ry * 1e3, math.degrees(dyaw),
                 od[0] * 1e3, od[1] * 1e3, sl[0] * 1e3, sl[1] * 1e3, fit * 1e3, inl * 100))
    for kind, name in (("same", "같은 방향(0도)으로 돌아온 뒤"), ("cross", "180도 돈 뒤")):
        v = rows[kind]
        if len(v) >= 2:
            print("\n%s %d회: 실제 앞 %+.1f±%.1f  왼 %+.1f±%.1f mm | 오도메트리는 %+.1f/%+.1f | SLAM %+.1f/%+.1f"
                  % (name, len(v), st.mean(x[0] for x in v) * 1e3, st.pstdev(x[0] for x in v) * 1e3,
                     st.mean(x[1] for x in v) * 1e3, st.pstdev(x[1] for x in v) * 1e3,
                     st.mean(x[3][0] for x in v) * 1e3, st.mean(x[3][1] for x in v) * 1e3,
                     st.mean(x[4][0] for x in v) * 1e3, st.mean(x[4][1] for x in v) * 1e3))
    v = rows["cross"]
    if v:
        mx = st.mean(x[0] for x in v) / 2.0
        my = st.mean(x[1] for x in v) / 2.0
        print("→ 180도 회전 기준 '겉보기 회전 중심' (처음 자세 기준, 라이다 설치 오차 포함): 앞 %+.1f 왼 %+.1f mm"
              % (mx * 1e3, my * 1e3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
