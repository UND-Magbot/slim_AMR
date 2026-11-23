#!/usr/bin/env python3
"""출발할 때 두 바퀴가 같이 도는지 — 바퀴별 속도(엔코더)를 따로 잰다.

    python3 tools/wheel_start_test.py                0.04, 0.10 m/s 를 3회씩
    python3 tools/wheel_start_test.py -v 0.06 -n 5   0.06 m/s 로 5회

앞으로 DIST 가서 서고, 뒤로 DIST 돌아온다(제자리 유지). 회전 지령은 0.
/joint_states 의 좌·우 바퀴 속도(엔코더, 20Hz)와 자이로를 출발 뒤 1.5초
동안 기록해, 어느 바퀴가 먼저 나가는지와 속도 차이를 낸다.

  - 매번 같은 바퀴가 늦다   -> 그쪽 부하(캐스터·무게)나 그쪽 드라이브
  - 늦는 쪽이 들쭉날쭉      -> 정지 마찰(버티다 풀림)
  - 공중에 띄워도 같다면    -> 드라이브·모터 쪽

안전 : 진행 방향 차체 끝에서 0.40m 안(폭 ±0.20m)에 라이다 점이 있으면 서고
       끝낸다. 도킹·이동·왕복 중이면 시작하지 않는다. Ctrl-C 로 멈춘다.
(2026-09-30, 출발 때 좌우 바퀴 속도 차 원인 가르기)
"""
import argparse
import json
import math
import sys
import time
import urllib.request

HALF_L = 0.435
LIDAR_X = -0.015
GUARD = 0.40
GUARD_W = 0.20
REC_S = 1.5
DIST = 0.30


def api(path):
    with urllib.request.urlopen("http://localhost:8090" + path, timeout=3) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-v", type=float, nargs="*", default=[0.04, 0.10],
                    help="출발 속도들 [m/s]")
    ap.add_argument("-n", type=int, default=3, help="속도마다 왕복 횟수")
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
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState, LaserScan

    rclpy.init()
    n = Node("wheel_start_test")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    s = {"wl": None, "wr": None, "jt": 0.0, "gyro": 0.0, "gt": 0.0,
         "x": None, "y": None, "scan": None}
    rad = {"l": 0.1701 / 2, "r": 0.1701 / 2}

    def on_js(m):
        try:
            i, j = m.name.index("left_wheel_joint"), m.name.index("right_wheel_joint")
        except ValueError:
            return
        s["wl"] = m.velocity[i] * rad["l"]
        s["wr"] = m.velocity[j] * rad["r"]
        s["jt"] = time.monotonic()

    def on_imu(m):
        s["gyro"], s["gt"] = m.angular_velocity.z, time.monotonic()

    def on_odom(m):
        s["x"], s["y"] = m.pose.pose.position.x, m.pose.pose.position.y

    n.create_subscription(JointState, "/joint_states", on_js, 50)
    n.create_subscription(Imu, "/imu/base_link", on_imu, qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom", on_odom, 20)
    n.create_subscription(LaserScan, "/scan", lambda m: s.__setitem__("scan", m),
                          qos_profile_sensor_data)

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

    t_end = time.monotonic() + 4.0
    while (s["x"] is None or s["scan"] is None or s["wl"] is None) \
            and time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if s["x"] is None or s["scan"] is None or s["wl"] is None:
        print("오도메트리·라이다·바퀴 속도 중 안 들어오는 것이 있습니다")
        return 1

    results = []
    try:
        for v in a.v:
            for k in range(a.n):
                for sign, name in ((1, "전진"), (-1, "후진")):
                    if blocked(sign):
                        print("%s 쪽에 물체 — 끝냅니다" % name)
                        raise KeyboardInterrupt
                    x0, y0 = s["x"], s["y"]
                    t0 = time.monotonic()
                    rec = []
                    last_jt = 0.0
                    while True:
                        rclpy.spin_once(n, timeout_sec=0.02)
                        now = time.monotonic()
                        gone = math.hypot(s["x"] - x0, s["y"] - y0)
                        if gone >= DIST and now - t0 > REC_S:
                            break
                        if now - t0 > 15.0 or blocked(sign):
                            print("  %s 중 멈춤 (시간 초과 또는 물체)" % name)
                            break
                        t = Twist()
                        t.linear.x = sign * v if gone < DIST else 0.0
                        pub.publish(t)
                        if s["jt"] != last_jt and now - t0 <= REC_S:
                            last_jt = s["jt"]
                            rec.append((now - t0, s["wl"] * sign, s["wr"] * sign,
                                        s["gyro"]))
                    stop()
                    time.sleep(1.5)
                    for _ in range(20):
                        rclpy.spin_once(n, timeout_sec=0.02)
                    results.append((v, name, rec))
                    report(v, name, rec)
    except KeyboardInterrupt:
        pass
    finally:
        stop()

    print("\n=== 요약 (출발 1.5초 안, + 는 오른쪽 바퀴가 빠름)")
    for v in a.v:
        for name in ("전진", "후진"):
            rs = [summ(r[2], v) for r in results if r[0] == v and r[1] == name]
            rs = [x for x in rs if x]
            if not rs:
                continue
            lead = ["오른쪽" if x[0] > 0 else "왼쪽" for x in rs]
            print("  %.2f m/s %s: 최대 속도차 %s mm/s | 먼저 나간 바퀴 %s | "
                  "50%% 도달 좌/우 %s 초" % (
                      v, name, " ".join("%+.0f" % (x[0] * 1000) for x in rs),
                      " ".join(lead),
                      " ".join("%.2f/%.2f" % (x[1], x[2]) for x in rs)))
    n.destroy_node()
    rclpy.shutdown()
    return 0


def summ(rec, v):
    if not rec:
        return None
    d = max((r[2] - r[1] for r in rec), key=abs)
    def reach(idx):
        return next((r[0] for r in rec if r[idx] >= 0.5 * v), float("nan"))
    return d, reach(1), reach(2)


def report(v, name, rec):
    x = summ(rec, v)
    if not x:
        print("%.2f %s: 기록 없음" % (v, name))
        return
    print("%.2f m/s %s: 좌우 속도차 최대 %+.0f mm/s, 50%% 도달 좌 %.2f / 우 %.2f 초"
          % (v, name, x[0] * 1000, x[1], x[2]))
    print("    t    좌mm/s  우mm/s  자이로도/s")
    for r in rec[::3]:
        print("  %.2f  %6.0f  %6.0f  %+6.1f" % (r[0], r[1] * 1000, r[2] * 1000,
                                               math.degrees(r[3])))


if __name__ == "__main__":
    sys.exit(main())
