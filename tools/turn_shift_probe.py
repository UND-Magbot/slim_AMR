#!/usr/bin/env python3
"""제자리 회전 동안 로봇이 실제로 얼마나 옮겨지나 — 회전 전후 라이다 스캔을 맞춰 잰다.

    python3 tools/turn_shift_probe.py            300초 (왕복을 돌려 둘 것)

듣기만 한다. '섰다(0.5초) -> 제자리에서 돈다(전진 없이 20도 넘게) -> 섰다' 를
찾아, 돌기 전 스캔과 돈 뒤 스캔을 점-선 ICP 로 맞춘다. 같은 동안 오도메트리
(/odometry/filtered)가 말하는 이동과 비교한다.

왜 (2026-10-14): 매 구간 출발 직후 SLAM 이 로봇을 진행 방향 앞으로 11~13cm
옮겼다(25구간 중 거의 전부). 출발 직전에는 제자리에서 약 170도 돈다. 회전의
실제 중심이 base_footprint 와 어긋나 있으면 회전 동안 실제로 옮겨지는데
오도메트리는 제자리라고 보고, 첫 정합 때 SLAM 이 한꺼번에 고친다.
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/und_amr_bringup"))
from und_amr_bringup import scan_icp   # noqa: E402

D = math.degrees


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=300.0)
    a = ap.parse_args()

    import rclpy
    import tf2_ros
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    n = Node("turn_shift_probe")
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

    las = None
    while las is None:
        time.sleep(0.2)
        with lock:
            m = S["scan"][1] if S["scan"] else None
        if m is None:
            continue
        try:
            tr = buf.lookup_transform("base_footprint", m.header.frame_id, rclpy.time.Time())
            las = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
        except Exception:                                    # noqa: BLE001
            pass

    def still():
        with lock:
            return abs(S["v"]) < 0.003 and abs(S["w"]) < 0.01

    def grab():
        t0 = time.time()
        while time.time() - t0 < 1.0:
            with lock:
                ts, m = S["scan"]
                od = S["od"]
            if ts > t0 + 0.08:
                return scan_icp.scan_points(m, las, step=1), od
            time.sleep(0.03)
        return None, None

    print("기록 %.0f초 — 제자리 회전 전후를 잰다" % a.t, flush=True)
    res = []
    end = time.time() + a.t
    state, still_t, before = "wait_still", None, None
    while time.time() < end:
        time.sleep(0.05)
        with lock:
            v, w = S["v"], S["w"]
        if state == "wait_still":
            if still():
                still_t = still_t or time.time()
                if time.time() - still_t > 0.5:
                    before = grab()
                    state = "wait_turn" if before[0] is not None else "wait_still"
            else:
                still_t = None
        elif state == "wait_turn":
            if abs(v) > 0.01:
                state, still_t = "wait_still", None          # 회전 없이 출발했다
            elif abs(w) > 0.05:
                state = "turning"
        elif state == "turning":
            if abs(v) > 0.02:
                state, still_t = "wait_still", None          # 돌면서 전진 — 제자리 아님
            elif still():
                still_t = still_t or time.time()
                if time.time() - still_t > 0.5:
                    after = grab()
                    state, still_t = "wait_still", None
                    if after[0] is None:
                        continue
                    (p0, o0), (p1, o1) = before, after
                    odr = scan_icp.relative(o0, o1)          # 오도메트리: 돌기 전 자세에서 본 돈 뒤 자세
                    if abs(D(odr[2])) < 20:
                        continue
                    nrm = scan_icp.normals(p0)
                    real, fit, inl = scan_icp.icp(p1[::3], p0, odr, dst_n=nrm)
                    res.append((odr, real, inl))
                    print("  회전 %+6.1f도 | 실제 이동 앞 %+6.1f 좌 %+6.1f mm, 회전 %+6.1f도 | 오도메트리 이동 앞 %+5.1f 좌 %+5.1f mm, 회전 %+6.1f도 | 맞음 %.0f%%"
                          % (D(odr[2]), real[0] * 1000, real[1] * 1000, D(real[2]),
                             odr[0] * 1000, odr[1] * 1000, D(odr[2]), inl * 100), flush=True)
            else:
                still_t = None
    if res:
        good = [r for r in res if r[2] >= 0.6]
        if good:
            sh = [math.hypot(r[1][0] - r[0][0], r[1][1] - r[0][1]) * 1000 for r in good]
            dy = [D(r[1][2] - r[0][2]) for r in good]
            print("== %d번: 회전 동안 실제 이동과 오도메트리 차이 평균 %.0fmm (최대 %.0f), 회전각 차이 평균 %+.2f도"
                  % (len(good), sum(sh) / len(sh), max(sh), sum(dy) / len(dy)))
    ex.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
