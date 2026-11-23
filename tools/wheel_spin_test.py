#!/usr/bin/env python3
"""제자리 회전에서 두 바퀴 속도를 비교한다 — 드라이브 쪽 차이인지 바닥 쪽인지 가르기.

    python3 tools/wheel_spin_test.py            좌·우로 반 바퀴씩 3회
    python3 tools/wheel_spin_test.py -w 0.4 -n 4

제자리 회전은 두 바퀴가 같은 속도로 반대로 돈다. 캐스터가 버티는 힘은 도는
방향과 거의 무관하다.
  - 어느 쪽으로 돌든 같은 바퀴가 빠르다   -> 드라이브·모터 쪽 차이
  - 도는 방향에 따라 빠른 바퀴가 바뀐다   -> 바닥·캐스터·무게 쪽

안전 : 로봇 중심에서 SWEEP 안에 라이다 점이 있으면 시작하지 않고, 도는 중에도
       보면 멈춘다(차체 대각 반경 약 0.46m). 작업·이동 중이면 시작하지 않는다.
(2026-09-30, 로봇을 들 수 없어 구동륜을 띄우는 시험 대신)
"""
import argparse
import json
import math
import statistics as st
import sys
import time
import urllib.request

SWEEP = 0.55
LIDAR_X = -0.015
SEP = 0.1789
WHEEL_R = 0.1701 / 2


def api(path):
    with urllib.request.urlopen("http://localhost:8090" + path, timeout=3) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-w", type=float, default=0.5, help="회전 속도 [rad/s]")
    ap.add_argument("-n", type=int, default=3, help="방향마다 횟수")
    ap.add_argument("-a", type=float, default=180.0, help="한 번에 도는 각 [도]")
    a = ap.parse_args()

    if api("/job/status").get("state") in ("moving", "looking", "staging",
                                           "entering", "preflip", "exiting",
                                           "exit_fix"):
        print("도킹 작업 중이라 시작하지 않습니다")
        return 1
    if api("/nav/status").get("state") in ("sending", "active") \
            or api("/route/status").get("busy"):
        print("이동·왕복 중이라 시작하지 않습니다")
        return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu, JointState, LaserScan

    rclpy.init()
    n = Node("wheel_spin_test")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    s = {"wl": None, "wr": None, "jt": 0.0, "gyro": 0.0, "gt": 0.0, "scan": None}

    def on_js(m):
        try:
            i, j = m.name.index("left_wheel_joint"), m.name.index("right_wheel_joint")
        except ValueError:
            return
        s["wl"], s["wr"] = m.velocity[i] * WHEEL_R, m.velocity[j] * WHEEL_R
        s["jt"] = time.monotonic()

    def on_imu(m):
        s["gyro"], s["gt"] = m.angular_velocity.z, time.monotonic()

    n.create_subscription(JointState, "/joint_states", on_js, 50)
    n.create_subscription(Imu, "/imu/base_link", on_imu, qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan", lambda m: s.__setitem__("scan", m),
                          qos_profile_sensor_data)

    def near():
        m = s["scan"]
        if m is None:
            return True
        ang = m.angle_min
        for r in m.ranges:
            if m.range_min < r < m.range_max:
                x = r * math.cos(ang) + LIDAR_X
                y = r * math.sin(ang)
                if math.hypot(x, y) < SWEEP:
                    return True
            ang += m.angle_increment
        return False

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)

    t_end = time.monotonic() + 4.0
    while (s["scan"] is None or s["wl"] is None or s["gt"] == 0.0) \
            and time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if s["scan"] is None or s["wl"] is None or s["gt"] == 0.0:
        print("라이다·바퀴 속도·자이로 중 안 들어오는 것이 있습니다")
        return 1
    if near():
        print("로봇 둘레 %.2fm 안에 물체가 있어 시작하지 않습니다" % SWEEP)
        return 1

    res = {1: [], -1: []}
    try:
        for k in range(a.n):
            for d in (1, -1):
                yaw, last = 0.0, time.monotonic()
                t0 = last
                rec = []
                last_jt = 0.0
                while abs(math.degrees(yaw)) < a.a:
                    rclpy.spin_once(n, timeout_sec=0.02)
                    now = time.monotonic()
                    yaw += s["gyro"] * (now - last)
                    last = now
                    if now - t0 > 20.0 or near():
                        print("  시간 초과 또는 물체 — 멈춤")
                        raise KeyboardInterrupt
                    t = Twist()
                    t.angular.z = d * a.w
                    pub.publish(t)
                    if s["jt"] != last_jt:
                        last_jt = s["jt"]
                        rec.append((now - t0, s["wl"], s["wr"], s["gyro"]))
                stop()
                time.sleep(1.5)
                for _ in range(20):
                    rclpy.spin_once(n, timeout_sec=0.02)
                res[d].append(rec)
                show(d, rec, a.w)
    except KeyboardInterrupt:
        pass
    finally:
        stop()

    print("\n=== 요약 (정상 구간 = 시작 1초 뒤 ~ 끝 0.5초 전, 크기만 비교)")
    tgt = a.w * SEP / 2.0 * 1000
    for d, name in ((1, "왼쪽 회전(반시계)"), (-1, "오른쪽 회전(시계)")):
        ls, rs, pk = [], [], []
        for rec in res[d]:
            m = steady(rec)
            if m:
                ls.append(m[0]); rs.append(m[1])
            if rec:
                start = [r for r in rec if 0.2 < r[0] < 1.2]
                if start:
                    k = max(start, key=lambda r: abs(abs(r[2]) - abs(r[1])))
                    pk.append((abs(k[2]) - abs(k[1])) * 1000)
        if ls:
            print("  %s: 왼바퀴 %.1f / 오른바퀴 %.1f mm/s (목표 %.1f), 우-좌 %+.1f | "
                  "출발 1초 최대 우-좌 %s" % (name, st.mean(ls), st.mean(rs), tgt,
                                        st.mean(rs) - st.mean(ls),
                                        " ".join("%+.0f" % x for x in pk)))
    n.destroy_node()
    rclpy.shutdown()
    return 0


def steady(rec):
    if not rec:
        return None
    t_end = rec[-1][0]
    mid = [r for r in rec if 1.0 < r[0] < t_end - 0.5]
    if len(mid) < 5:
        return None
    return (st.mean(abs(r[1]) for r in mid) * 1000,
            st.mean(abs(r[2]) for r in mid) * 1000)


def show(d, rec, w):
    m = steady(rec)
    name = "왼쪽 회전" if d > 0 else "오른쪽 회전"
    if not m:
        print("%s: 기록 부족" % name)
        return
    print("%s: 정상 구간 왼바퀴 %.1f / 오른바퀴 %.1f mm/s (우-좌 %+.1f), %.1f초"
          % (name, m[0], m[1], m[1] - m[0], rec[-1][0]))


if __name__ == "__main__":
    sys.exit(main())
