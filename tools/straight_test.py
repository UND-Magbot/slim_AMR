#!/usr/bin/env python3
"""조향 없는 직진 시험 — 차체가 저절로 도는 것이 한쪽으로 치우치는지 본다.

    python3 tools/straight_test.py              1.5m 전진·후진 4번씩
    python3 tools/straight_test.py -d 1.0 -n 3  1.0m, 3번씩

회전 명령 0 으로 /cmd_vel 에 직접 속도를 낸다(api_node 를 거치지 않는다).
구간마다 자이로(실제 회전)와 엔코더(바퀴가 계산한 회전)의 누적 회전을 낸다.

  - 매번 같은 방향으로 돈다  -> 기구 비대칭 (캐스터 높이·예압, 회전축 기울기,
                               바퀴 지름). 맞추면 비교적 쉽게 고쳐진다
  - 방향이 무작위           -> 캐스터가 뒤집히며 찬다 (캐스터 자체 문제)
  - 자이로만 돌고 엔코더는 안 돎 -> 구동륜이 미끄러진다

안전 : 진행 방향 차체 끝에서 0.40m 안(폭 ±0.20m)에 라이다 점이 있으면 멈춘다.
       api_node 가 도킹 중이면 시작하지 않는다. Ctrl-C 로 언제든 멈춘다.
(2026-09-29, 캐스터 흔들림 원인 가르기)
"""
import argparse
import json
import math
import sys
import time
import urllib.request

HALF_L = 0.435          # 구동축에서 차체 끝까지 [m]
LIDAR_X = -0.015        # 구동축 기준 라이다 위치 [m]
GUARD = 0.40            # 차체 끝에서 이 안에 점이 있으면 정지 [m]
GUARD_W = 0.20          # 좌우 반폭 [m]
FLIP_M = 0.30           # 출발 뒤 캐스터가 뒤집히는 구간으로 보는 거리 [m]


def api(path):
    with urllib.request.urlopen("http://localhost:8090" + path, timeout=3) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-d", type=float, default=1.5, help="한 구간 거리 [m]")
    ap.add_argument("-n", type=int, default=4, help="전진·후진 쌍 횟수")
    ap.add_argument("-v", type=float, default=0.10, help="속도 [m/s]")
    ap.add_argument("--ff", nargs=2, type=float, metavar=("전진도", "후진도"),
                    help="출발 예측 보정: 전진·후진 출발에서 캐스터가 돌리는 각 "
                         "[도, +=반시계]. 그만큼 --ff-m 동안 미리 반대로 돈다")
    ap.add_argument("--ff-m", type=float, default=0.30, help="예측 보정 거리 [m]")
    a = ap.parse_args()

    st = api("/job/status").get("state")
    if st in ("moving", "looking", "staging", "entering", "preflip", "exiting",
              "exit_fix"):
        print("도킹 작업 중이라 시작하지 않습니다 (%s)" % st)
        return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan

    rclpy.init()
    n = Node("straight_test")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    s = {"gyro": 0.0, "gt": 0.0, "ow": 0.0, "x": None, "y": None, "scan": None}

    def on_imu(m):
        s["gyro"] = m.angular_velocity.z
        s["gt"] = time.monotonic()

    def on_odom(m):
        s["ow"] = m.twist.twist.angular.z
        s["x"], s["y"] = m.pose.pose.position.x, m.pose.pose.position.y

    def on_scan(m):
        s["scan"] = m

    n.create_subscription(Imu, "/imu/base_link", on_imu, qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom", on_odom, 20)
    n.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)

    def blocked(sign):
        m = s["scan"]
        if m is None:
            return True
        ang = m.angle_min
        for r in m.ranges:
            if m.range_min < r < m.range_max:
                x = r * math.cos(ang) + LIDAR_X
                y = r * math.sin(ang)
                if abs(y) < GUARD_W and HALF_L < sign * x < HALF_L + GUARD:
                    return True
            ang += m.angle_increment
        return False

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)

    t_end = time.monotonic() + 3.0
    while ((s["x"] is None or s["scan"] is None or s["gt"] == 0.0)
           and time.monotonic() < t_end):
        rclpy.spin_once(n, timeout_sec=0.05)
    if s["x"] is None or s["gt"] == 0.0 or s["scan"] is None:
        print("오도메트리·자이로·라이다 중 안 들어오는 것이 있습니다")
        return 1

    results = []
    try:
        for i in range(a.n):
            for sign, name in ((1, "전진"), (-1, "후진")):
                x0, y0 = s["x"], s["y"]
                g_flip = g_rest = o_flip = o_rest = 0.0
                g_cum, marks = 0.0, {}
                last = time.monotonic()
                why = "완료"
                while True:
                    rclpy.spin_once(n, timeout_sec=0.02)
                    now = time.monotonic()
                    dt, last = now - last, now
                    gone = math.hypot(s["x"] - x0, s["y"] - y0)
                    if gone >= a.d:
                        break
                    if blocked(sign):
                        why = "앞에 물체 — 정지"
                        break
                    if now - s["gt"] > 0.3:
                        why = "자이로 끊김 — 정지"
                        break
                    t = Twist()
                    t.linear.x = sign * a.v
                    if a.ff and gone < a.ff_m:
                        kick = math.radians(a.ff[0] if sign > 0 else a.ff[1])
                        t.angular.z = -kick * a.v / a.ff_m
                    pub.publish(t)
                    g_cum += s["gyro"] * dt
                    for mk in (0.05, 0.10, 0.15, 0.20, 0.30):
                        if gone >= mk and mk not in marks:
                            marks[mk] = math.degrees(g_cum)
                    if gone < FLIP_M:
                        g_flip += s["gyro"] * dt
                        o_flip += s["ow"] * dt
                    else:
                        g_rest += s["gyro"] * dt
                        o_rest += s["ow"] * dt
                stop()
                time.sleep(1.0)
                for _ in range(20):
                    rclpy.spin_once(n, timeout_sec=0.02)
                results.append((name, g_flip, g_rest, o_flip, o_rest))
                print("    누적 자이로 " + " ".join(
                    "%.0fcm:%+.1f" % (k * 100, v) for k, v in sorted(marks.items())))
                print("%d %s : 출발 0.3m 자이로 %+5.1f도 엔코더 %+5.1f도 | "
                      "나머지 자이로 %+5.1f도 엔코더 %+5.1f도  (%s)" % (
                          i + 1, name, math.degrees(g_flip), math.degrees(o_flip),
                          math.degrees(g_rest), math.degrees(o_rest), why))
                if why != "완료":
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        stop()

    for name in ("전진", "후진"):
        rs = [r for r in results if r[0] == name]
        if not rs:
            continue
        tot = [math.degrees(r[1] + r[2]) for r in rs]
        same = all(t > 0 for t in tot) or all(t < 0 for t in tot)
        verdict = ("판정하려면 2회 이상 필요" if len(rs) < 2 else
                   "매번 같은 방향 (기구 비대칭 의심)" if same else
                   "방향 제각각 (캐스터 뒤집힘 의심)")
        print("%s %d회 전체 회전: %s  -> %s" % (
            name, len(rs), " ".join("%+.1f" % t for t in tot), verdict))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
