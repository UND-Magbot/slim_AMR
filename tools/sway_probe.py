#!/usr/bin/env python3
"""주행 중 S자 원인 가르기 — 위치 보정(SLAM)이 툭 뛸 때 nav2 지령이 꺾이는지 본다.

    python3 tools/sway_probe.py            120초 기록 후 분석 (왕복을 돌려 둘 것)
    python3 tools/sway_probe.py -t 300

듣기만 한다(로봇을 움직이지 않는다). 기록 :
  /tf map->odom        SLAM 이 위치를 고칠 때마다 바뀐다
  /odometry/filtered   EKF (odom 기준 자세)
  /cmd_vel, /cmd_vel_noradius   지령 회전
  /imu/base_link       자이로 (실제 차체 회전)

판정 (2026-10-02, 지령이 차체보다 0.2~0.7초 먼저 흔들린다는 기록에서 출발)
  - 보정 직후 0.3초 안에 지령이 다른 때보다 크게 꺾이면  -> 위치 보정이 원인
  - 보정과 무관하게 꺾이면                                -> nav2 조향 자체(설정)
  - 지도 위 자세로 직선 구간의 좌우 흔들림 폭(mm)도 낸다
"""
import argparse
import csv
import math
import os
import statistics as st
import time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def record(sec, path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = Node("sway_probe")
    s = {"mo": (0.0, 0.0, 0.0), "od": (0.0, 0.0, 0.0), "v": 0.0, "cmd": 0.0,
         "cmd_t": 0.0, "g": 0.0, "jump": 0}

    def on_tf(m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                tr = t.transform
                new = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
                old = s["mo"]
                if (math.hypot(new[0] - old[0], new[1] - old[1]) > 1e-4
                        or abs(new[2] - old[2]) > 1e-4):
                    s["jump"] += 1
                s["mo"] = new

    def on_ekf(m):
        p = m.pose.pose
        s["od"] = (p.position.x, p.position.y, yaw_of(p.orientation))
        s["v"] = m.twist.twist.linear.x

    def on_cmd(m):
        s["cmd"], s["cmd_t"] = m.angular.z, time.time()

    n.create_subscription(TFMessage, "/tf", on_tf, 200)
    n.create_subscription(Odometry, "/odometry/filtered", on_ekf, 50)
    n.create_subscription(Twist, "/cmd_vel", on_cmd, 50)
    n.create_subscription(Twist, "/cmd_vel_noradius", on_cmd, 50)
    n.create_subscription(Imu, "/imu/base_link",
                          lambda m: s.__setitem__("g", m.angular_velocity.z),
                          qos_profile_sensor_data)
    # ⚠ 메시지는 별도 스레드에서 계속 처리한다. spin_once 를 한 번씩 부르면
    #   자이로(200Hz)에 밀려 /odom 이 줄을 서 1~2초 늦게 기록됐다(2026-10-02).
    import threading
    from rclpy.executors import SingleThreadedExecutor
    _ex = SingleThreadedExecutor()
    _ex.add_node(n)
    threading.Thread(target=_ex.spin, daemon=True).start()
    t0 = time.time()
    nxt = t0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "mo_x", "mo_y", "mo_yaw", "od_x", "od_y", "od_yaw",
                    "v", "cmd_w", "gyro", "jump"])
        while time.time() - t0 < sec:
            time.sleep(0.005)
            if time.time() >= nxt:
                nxt += 0.02
                cmd = s["cmd"] if time.time() - s["cmd_t"] < 0.5 else 0.0
                w.writerow(["%.3f" % (time.time() - t0)]
                           + ["%.5f" % x for x in s["mo"] + s["od"]]
                           + ["%.4f" % s["v"], "%.5f" % cmd, "%.5f" % s["g"], s["jump"]])
                s["jump"] = 0
    n.destroy_node()
    rclpy.shutdown()


def analyze(path):
    R = list(csv.DictReader(open(path)))
    T = [float(r["t"]) for r in R]
    V = [float(r["v"]) for r in R]
    C = [math.degrees(float(r["cmd_w"])) for r in R]
    J = [int(r["jump"]) for r in R]
    MO = [(float(r["mo_x"]), float(r["mo_y"]), float(r["mo_yaw"])) for r in R]
    OD = [(float(r["od_x"]), float(r["od_y"]), float(r["od_yaw"])) for r in R]

    def map_pose(i):
        mx, my, myaw = MO[i]
        ox, oy, oyaw = OD[i]
        c, s_ = math.cos(myaw), math.sin(myaw)
        return mx + c * ox - s_ * oy, my + s_ * ox + c * oy, myaw + oyaw

    # 1) 보정 크기 (주행 중) : 보정 직전 지도 자세와 직후 지도 자세 차이
    hops = []
    for i in range(1, len(R)):
        if J[i] and V[i] > 0.15:
            a, b = map_pose(i - 1), map_pose(i)
            dyaw = math.degrees(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
            # 0.02초 동안 실제 이동분(EKF)을 빼서 보정분만
            ox = math.hypot(OD[i][0] - OD[i - 1][0], OD[i][1] - OD[i - 1][1])
            d = abs(math.hypot(b[0] - a[0], b[1] - a[1]) - ox)
            hops.append((i, d * 1000, dyaw))
    big = [h for h in hops if h[1] > 3 or abs(h[2]) > 0.2]
    print("주행 중 위치 보정 %d회 (%.1f초에 1번), 3mm·0.2도 넘는 것 %d회"
          % (len(hops), (T[-1] - T[0]) / max(1, len(hops)), len(big)))
    if hops:
        print("  보정 크기: 위치 평균 %.1fmm 최대 %.1fmm, 방향 평균 %.2f도 최대 %.2f도"
              % (st.mean(h[1] for h in hops), max(h[1] for h in hops),
                 st.mean(abs(h[2]) for h in hops), max(abs(h[2]) for h in hops)))

    # 2) 보정 직후 지령 변화 vs 평소
    k = int(0.3 / 0.02)
    near, far = [], []
    hop_i = set()
    for h in hops:
        hop_i.update(range(h[0], h[0] + k))
    for i in range(len(C) - 5):
        if V[i] < 0.2:
            continue
        d = abs(C[i + 5] - C[i])            # 0.1초 동안 지령 변화
        (near if i in hop_i else far).append(d)
    if near and far:
        print("0.1초 지령 변화: 보정 직후 0.3초 평균 %.2f 도/s (2도 넘는 비율 %.0f%%) / 평소 %.2f (%.0f%%)"
              % (st.mean(near), 100 * sum(x > 2 for x in near) / len(near),
                 st.mean(far), 100 * sum(x > 2 for x in far) / len(far)))
        hop_big = set()
        for h in big:
            hop_big.update(range(h[0], h[0] + k))
        nb = [abs(C[i + 5] - C[i]) for i in sorted(hop_big) if i < len(C) - 5 and V[i] > 0.2]
        if nb:
            print("  큰 보정 직후만: 평균 %.2f 도/s (n=%d)" % (st.mean(nb), len(nb)))

    # 3) 직선 구간 좌우 흔들림 (지도 자세, 구간마다 직선 맞춤 뒤 잔차)
    i = 0
    rows = []
    while i < len(R):
        if V[i] > 0.25:
            j = i
            while j < len(R) and V[j] > 0.25:
                j += 1
            a, b = i + 25, j - 25
            if b - a > 100:
                P = [map_pose(q)[:2] for q in range(a, b)]
                x0, y0 = P[0]
                x1, y1 = P[-1]
                L = math.hypot(x1 - x0, y1 - y0)
                if L > 0.5:
                    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
                    lat = [((x - x0) * nx + (y - y0) * ny) * 1000 for x, y in P]
                    # 1초 평균으로 위치 보정 계단 제거 전후 둘 다
                    rows.append((T[a], L, max(lat) - min(lat), st.pstdev(lat)))
            i = j
        i += 1
    if rows:
        print("직선 구간 좌우 흔들림 (지도 자세, 양 끝 잇는 선 기준):")
        for r in rows:
            print("  t=%5.1f 길이 %.2fm  폭 %.0fmm  표준편차 %.1fmm" % r)
        print("  평균 폭 %.0fmm" % st.mean(r[2] for r in rows))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-t", type=float, default=120.0, help="기록 시간 [s]")
    ap.add_argument("--analyze", help="기록한 CSV 를 분석만")
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze)
        return
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs",
                        time.strftime("sway_%m%d_%H%M.csv"))
    print("기록 %.0f초 -> %s" % (a.t, os.path.normpath(path)))
    record(a.t, path)
    analyze(path)


if __name__ == "__main__":
    main()
