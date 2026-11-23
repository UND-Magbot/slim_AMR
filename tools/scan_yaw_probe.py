#!/usr/bin/env python3
"""라이다로 잰 차체 회전 vs 자이로 vs 바퀴 엔코더 — 어느 것이 진짜 차체 회전인가.

    python3 tools/scan_yaw_probe.py -t 40     40초 듣기 (그동안 계단 시험이나 주행)

듣기만 한다. 연속 스캔 두 장의 '먼 점(2~8m)' 거리 배열을 각도로 밀어 맞춰
스캔 사이 회전각을 낸다(이동에 따른 시차를 줄이려 먼 점만). 10Hz 회전속도를
자이로·엔코더(같은 구간 평균)와 비교한다. (2026-10-02, 윤활 뒤 자이로만 떠는 것을
보고 카메라(IMU) 받침 떨림을 의심)
"""
import argparse
import math
import statistics as st
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=40.0)
    a = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    rclpy.init()
    n = Node("scan_yaw_probe")
    g, o, sc = [], [], []
    n.create_subscription(Imu, "/imu/base_link", lambda m: g.append((time.monotonic(), m.angular_velocity.z)), qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom", lambda m: o.append((time.monotonic(), m.twist.twist.angular.z, m.twist.twist.linear.x)), 50)
    n.create_subscription(LaserScan, "/scan", lambda m: sc.append((time.monotonic(), m.angle_increment, list(m.ranges))), qos_profile_sensor_data)
    # ⚠ 메시지는 별도 스레드에서 계속 처리한다. spin_once 를 한 번씩 부르면
    #   자이로(200Hz)에 밀려 /odom 이 줄을 서 1~2초 늦게 기록됐다(2026-10-02).
    import threading
    from rclpy.executors import SingleThreadedExecutor
    _ex = SingleThreadedExecutor()
    _ex.add_node(n)
    threading.Thread(target=_ex.spin, daemon=True).start()
    t_end = time.monotonic() + a.t
    while time.monotonic() < t_end:
        time.sleep(0.01)
    n.destroy_node(); rclpy.shutdown()

    def mean_in(arr, t0, t1, k=1):
        v = [x[k] for x in arr if t0 <= x[0] < t1]
        return st.mean(v) if v else float("nan")

    D = math.degrees
    rows = []
    for (t0, inc, A), (t1, _, B) in zip(sc, sc[1:]):
        N = len(A)
        if len(B) != N or t1 - t0 > 0.2:
            continue
        def err(k):
            e, c = 0.0, 0
            for i in range(0, N, 2):
                ra, rb = A[i], B[(i + k) % N]
                if 2.0 < ra < 8.0 and 2.0 < rb < 8.0:
                    d = ra - rb
                    e += min(d * d, 0.04); c += 1
            return e / c if c > 50 else 1e9
        ks = list(range(-12, 13))
        es = [err(k) for k in ks]
        i = min(range(len(ks)), key=lambda q: es[q])
        frac = 0.0
        if 0 < i < len(ks) - 1:
            den = es[i - 1] - 2 * es[i] + es[i + 1]
            frac = 0.5 * (es[i - 1] - es[i + 1]) / den if den > 0 else 0.0
        dyaw = -(ks[i] + frac) * inc          # 차체가 +돌면 스캔은 -로 밀린다
        rows.append((t0, D(dyaw / (t1 - t0)), D(mean_in(g, t0, t1)), D(mean_in(o, t0, t1)),
                     mean_in(o, t0, t1, 2)))
    mv = [r for r in rows if r[4] > 0.2]
    print("스캔 쌍 %d, 주행 중 %d" % (len(rows), len(mv)))
    if len(mv) > 10:
        for name, k in (("자이로", 2), ("엔코더", 3)):
            for lag in (0, 1, 2, 3, 5, 8, 10):
                xs = [mv[q][1] for q in range(len(mv) - lag)]
                ys = [mv[q + lag][k] for q in range(len(mv) - lag)] if True else None
                mx, my = st.mean(xs), st.mean(ys)
                cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
                cx = math.sqrt(sum((x - mx) ** 2 for x in xs)); cy = math.sqrt(sum((y - my) ** 2 for y in ys))
                print("  라이다 회전 vs %s (%.1f초 뒤): 상관 %.2f" % (name, lag * 0.1, cov / (cx * cy) if cx * cy else float("nan")))
        print("  표준편차: 라이다 %.2f, 자이로 %.2f, 엔코더 %.2f 도/s" % tuple(st.pstdev(r[k] for r in mv) for k in (1, 2, 3)))
        print("  시간    라이다  자이로  엔코더  (도/s)")
        for r in mv[::2][:80]:
            print("  %6.1f  %+6.1f  %+6.1f  %+6.1f" % (r[0] - rows[0][0], r[1], r[2], r[3]))


if __name__ == "__main__":
    main()
