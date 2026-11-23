#!/usr/bin/env python3
"""자이로 배율 재기 — 제자리 회전 전후 라이다 스캔을 맞대 실제 회전각을 구하고
EKF(자이로 적분) 회전각과 비교한다.

    python3 tools/gyro_scale_probe.py            180초 듣기 (왕복을 돌려 둘 것)
    python3 tools/gyro_scale_probe.py -t 300

듣기만 한다. 도착 후 제자리 회전처럼 '멈춤 -> 제자리 회전 -> 멈춤' 을 찾아,
멈춰 있는 동안의 스캔(여러 장 평균)을 회전 전후로 원형 상관해 실제 각을 낸다.
옮겨 간 거리가 2cm 넘으면 버린다(라이다가 회전 중심에서 1.5cm 떨어져 있어
제자리 회전에서는 거의 안 움직인다).

결과 '실제/자이로' 가 1 보다 크면 자이로가 덜 읽는 것. (2026-10-02, 회전 뒤 첫
SLAM 보정이 회전과 같은 쪽으로 0.4~1.4도라 자이로 배율을 의심)
"""
import argparse
import math
import statistics as st
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=180.0)
    a = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    n = Node("gyro_scale_probe")
    s = {"v": 1.0, "w": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "uw": 0.0, "t": 0.0}

    def on_ekf(m):
        p, q = m.pose.pose, m.pose.pose.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if s["t"]:
            s["uw"] += math.atan2(math.sin(y - s["yaw"]), math.cos(y - s["yaw"]))
        s.update(x=p.position.x, y=p.position.y, yaw=y, t=time.time())

    n.create_subscription(Odometry, "/odometry/filtered", on_ekf, 50)
    n.create_subscription(Odometry, "/odom", lambda m: s.update(
        v=abs(m.twist.twist.linear.x), w=abs(m.twist.twist.angular.z)), 20)
    scans = []
    n.create_subscription(LaserScan, "/scan", lambda m: scans.append(m), qos_profile_sensor_data)

    # ⚠ 메시지는 별도 스레드에서 계속 처리한다. spin_once 를 한 번씩 부르면
    #   자이로(200Hz)에 밀려 /odom 이 줄을 서 1~2초 늦게 기록됐다(2026-10-02).
    import threading
    from rclpy.executors import SingleThreadedExecutor
    _ex = SingleThreadedExecutor()
    _ex.add_node(n)
    threading.Thread(target=_ex.spin, daemon=True).start()
    rests = []          # (끝 시각, 평균 스캔, 누적 yaw, x, y)
    cur = None          # 지금 멈춤 구간 {t0, sum, cnt, ...}
    still_t = None
    t_end = time.time() + a.t
    while time.time() < t_end:
        time.sleep(0.02)
        now = time.time()
        still = s["v"] < 0.003 and s["w"] < 0.003
        if still:
            still_t = still_t or now
        else:
            still_t = None
            if cur and cur["cnt"] >= 3:
                rests.append(cur)
            cur = None
        while scans:
            m = scans.pop(0)
            if still_t and now - still_t > 0.4:
                if cur is None:
                    cur = {"inc": m.angle_increment, "n": len(m.ranges),
                           "sum": [0.0] * len(m.ranges), "cnt_i": [0] * len(m.ranges),
                           "cnt": 0, "yaw": s["uw"], "x": s["x"], "y": s["y"]}
                if len(m.ranges) == cur["n"]:
                    for i, r in enumerate(m.ranges):
                        if m.range_min < r < min(m.range_max, 8.0):
                            cur["sum"][i] += r
                            cur["cnt_i"][i] += 1
                    cur["cnt"] += 1
                    cur["yaw"], cur["x"], cur["y"] = s["uw"], s["x"], s["y"]
    if cur and cur["cnt"] >= 3:
        rests.append(cur)
    n.destroy_node()
    rclpy.shutdown()

    def avg(r):
        return [r["sum"][i] / r["cnt_i"][i] if r["cnt_i"][i] else None for i in range(r["n"])]

    ratios = []
    print("멈춤 구간 %d개" % len(rests))
    for r0, r1 in zip(rests, rests[1:]):
        dyaw = r1["yaw"] - r0["yaw"]
        moved = math.hypot(r1["x"] - r0["x"], r1["y"] - r0["y"])
        if abs(math.degrees(dyaw)) < 30 or moved > 0.02 or r0["n"] != r1["n"]:
            continue
        A, B, N, inc = avg(r0), avg(r1), r0["n"], r0["inc"]
        # 회전 dyaw 면 이전 스캔의 i 번 방향이 이후 스캔의 i - dyaw/inc 번에 온다
        k0 = int(round(-dyaw / inc))
        best = []
        for k in range(k0 - int(math.radians(6) / inc), k0 + int(math.radians(6) / inc) + 1):
            e, c = 0.0, 0
            for i in range(N):
                j = (i + k) % N
                if A[i] is not None and B[j] is not None:
                    d = A[i] - B[j]
                    e += min(d * d, 0.04)
                    c += 1
            if c > N * 0.3:
                best.append((e / c, k))
        if len(best) < 3:
            continue
        best.sort(key=lambda x: x[1])
        i = min(range(len(best)), key=lambda q: best[q][0])
        if 0 < i < len(best) - 1:
            y0, y1, y2 = best[i - 1][0], best[i][0], best[i + 1][0]
            den = y0 - 2 * y1 + y2
            frac = 0.5 * (y0 - y2) / den if den > 0 else 0.0
        else:
            frac = 0.0
        true = -(best[i][1] + frac) * inc
        ratio = true / dyaw
        ratios.append(ratio)
        print("  자이로 %+8.2f도  스캔 %+8.2f도  차 %+5.2f도  실제/자이로 %.4f  (이동 %.0fmm, 오차 %.4f)"
              % (math.degrees(dyaw), math.degrees(true), math.degrees(true - dyaw), ratio,
                 moved * 1000, best[i][0]))
    if ratios:
        print("실제/자이로 평균 %.4f (중앙 %.4f, n=%d) -> 자이로가 %.2f%% %s 읽음"
              % (st.mean(ratios), st.median(ratios), len(ratios),
                 abs(st.median(ratios) - 1) * 100, "덜" if st.median(ratios) > 1 else "더"))


if __name__ == "__main__":
    main()
