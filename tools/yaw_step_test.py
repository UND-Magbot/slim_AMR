#!/usr/bin/env python3
"""주행 중 회전 응답 재기 — 곧게 가며 작은 조향 계단을 주고, 바퀴(엔코더)와
차체(자이로)가 얼마나 늦게·얼마나 크게 따라오는지 본다.

    python3 tools/yaw_step_test.py               0.35 m/s, ±3 도/s 계단, 1회
    python3 tools/yaw_step_test.py -v 0.45 -a 3 -n 2

순서 : 0 -> v 로 가속(조향 0, 1.5초 유지) -> +a 1초 -> 0 1초 -> -a 1초 -> 0 1초 -> 정지.
       -n 2 이상이면 제자리 180도 돌아 같은 계단을 반대로 한 번 더(되돌아옴).
지령은 /cmd_vel 에 낸다(nav2 와 같은 길: base_node 가속 제한·회전 반경 제한).
안전 : 진행 방향 차체 끝에서 0.6m 안(폭 ±0.25m)에 라이다 점이 있으면 즉시 멈춘다.
       이동·왕복·도킹 중이면 시작하지 않는다. Ctrl-C 로 언제든 멈춘다.
기록 : logs/yawstep_MMDD_HHMM.csv (t, 지령 v·w, 엔코더 w, 자이로 w, 속도)
(2026-10-02, 속도를 올리면 회전 흔들림이 커져 base_node 회전속도 제어 설계용)
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys
import time
import urllib.request

HALF_L = 0.435
# 캐스터 점수 기준 (같은 시험, 0.35 m/s ±8 도/s): (직진 떨림 도/s, 조향 넘침 배)
BASE_PRE = (1.36, 1.32)     # 2026-10-02 12:38·12:49, 앞 캐스터 교체 후·윤활 전
BASE_POST = (5.2, 2.0)      # 2026-10-02 13:45·13:48, 캐스터 윤활 후
LIDAR_X = -0.015
GUARD = 0.60
GUARD_W = 0.25


def api(path):
    with urllib.request.urlopen("http://localhost:8090" + path, timeout=3) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-v", type=float, default=0.35)
    ap.add_argument("-a", type=float, default=3.0, help="조향 계단 [도/s]")
    ap.add_argument("-n", type=int, default=1)
    ap.add_argument("-r", type=float, default=0.0, help="조향을 이 시간에 걸쳐 올리고 내린다 [s] (0=계단)")
    a = ap.parse_args()
    if api("/job/status").get("state") in ("moving", "looking", "staging", "entering",
                                           "preflip", "exiting", "exit_fix"):
        print("도킹 작업 중"); return 1
    if api("/nav/status").get("state") in ("sending", "active") or api("/route/status").get("busy"):
        print("이동·왕복 중"); return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init()
    n = Node("yaw_step_test")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    s = {"ow": 0.0, "v": 0.0, "g": 0.0, "scan": None, "ot": 0.0}
    dg = {k: "" for k in ("pos1", "pos2", "cmd1", "cmd2", "act1", "act2", "iq1", "iq2")}

    def on_diag(m):
        for st_ in m.status:
            if not st_.name.startswith("motor/axis"):
                continue
            ax = st_.name[-1]
            kv = {k.key: k.value for k in st_.values}
            dg["pos" + ax], dg["cmd" + ax] = kv.get("position", ""), kv.get("cmd_rpm", "")
            dg["act" + ax], dg["iq" + ax] = kv.get("act_rpm", ""), kv.get("iq", "")
    n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)
    n.create_subscription(Odometry, "/odom", lambda m: s.update(
        ow=m.twist.twist.angular.z, v=m.twist.twist.linear.x, ot=time.monotonic()), 20)
    n.create_subscription(Imu, "/imu/base_link", lambda m: s.__setitem__("g", m.angular_velocity.z),
                          qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan", lambda m: s.__setitem__("scan", m),
                          qos_profile_sensor_data)

    # ⚠ 메시지는 별도 스레드에서 계속 처리한다. spin_once 를 한 번씩 부르면
    #   자이로(200Hz)에 밀려 /odom 이 줄을 서 1~2초 늦게 기록됐다(2026-10-02).
    import threading
    from rclpy.executors import SingleThreadedExecutor
    _ex = SingleThreadedExecutor()
    _ex.add_node(n)
    threading.Thread(target=_ex.spin, daemon=True).start()
    def blocked():
        m = s["scan"]
        if m is None:
            return True
        ang = m.angle_min
        for r in m.ranges:
            if m.range_min < r < m.range_max:
                x = r * math.cos(ang) + LIDAR_X
                y = r * math.sin(ang)
                if abs(y) < GUARD_W and HALF_L < x < HALF_L + GUARD:
                    return True
            ang += m.angle_increment
        return False

    def stop():
        for _ in range(10):
            pub.publish(Twist()); time.sleep(0.02)

    t_end = time.monotonic() + 4
    while (s["scan"] is None or not s["ot"]) and time.monotonic() < t_end:
        time.sleep(0.05)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs",
                        time.strftime("yawstep_%m%d_%H%M%S.csv"))
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "run", "cmd_v", "cmd_w", "enc_w", "gyro_w", "v",
                "pos1", "pos2", "cmd1", "cmd2", "act1", "act2", "iq1", "iq2"])
    wr = math.radians(a.a)
    plan = [(a.v / 0.2 + 1.5, 0.0), (1.0, wr), (1.0, 0.0), (1.0, -wr), (1.0, 0.0)]
    t00 = time.monotonic()
    ok = True
    try:
        for run in range(a.n):
            if run:
                # 제자리 180도 (자이로 적분)
                yaw, last = 0.0, time.monotonic()
                while abs(yaw) < math.pi * 0.98:
                    time.sleep(0.02)
                    now = time.monotonic(); yaw += s["g"] * (now - last); last = now
                    t = Twist(); t.angular.z = 0.4; pub.publish(t)
                stop(); time.sleep(1.5)
            if blocked():
                print("앞에 물체 — 시작하지 않음"); ok = False; break
            t0 = time.monotonic()
            prev_w = 0.0
            for dur, wc in plan:
                te = time.monotonic() + dur
                while time.monotonic() < te:
                    time.sleep(0.01)
                    if blocked():
                        raise RuntimeError("앞에 물체")
                    wcmd = wc
                    if a.r > 0:          # 이전 값에서 a.r 초에 걸쳐 바꾼다
                        k_ = min(1.0, (dur - (te - time.monotonic())) / a.r)
                        wcmd = prev_w + (wc - prev_w) * k_
                    t = Twist(); t.linear.x = a.v; t.angular.z = wcmd; pub.publish(t)
                    w.writerow(["%.3f" % (time.monotonic() - t00), run, a.v, "%.5f" % wcmd,
                                "%.5f" % s["ow"], "%.5f" % s["g"], "%.4f" % s["v"]]
                               + [dg[k] for k in ("pos1", "pos2", "cmd1", "cmd2",
                                                  "act1", "act2", "iq1", "iq2")])
                    time.sleep(0.02)
                prev_w = wc
            stop(); time.sleep(1.0)
    except (KeyboardInterrupt, RuntimeError) as e:
        print("멈춤:", e); ok = False
    finally:
        stop(); f.close()
        n.destroy_node(); rclpy.shutdown()
    print("기록:", os.path.normpath(path))
    analyze(path)
    sc = caster_score(path)
    if sc:
        print("캐스터 점수: 직진 떨림 %.2f 도/s (윤활 전 %.2f, 윤활 후 %.2f) | 조향 넘침 %.2f배 (윤활 전 %.2f, 윤활 후 %.2f)"
              % (sc[0], BASE_PRE[0], BASE_POST[0], sc[1], BASE_PRE[1], BASE_POST[1]))
    return 0 if ok else 1


def caster_score(path):
    """캐스터 상태 숫자 셋 (2026-10-02, 캐스터 마찰 조정용).

    직진 떨림   : 조향 0 으로 0.3 m/s 넘게 갈 때, 자이로에서 0.3초 평균을 뺀 빠른
                  성분의 표준편차 [도/s]. 캐스터 회전축이 너무 자유로우면 커진다.
    조향 넘침   : 조향 계단에서 0.3초 평균 자이로 최대 / 지령. 1.0 이 이상적.
    """
    R = list(csv.DictReader(open(path)))
    if not R:
        return None
    D = math.degrees
    T = [float(r["t"]) for r in R]; C = [D(float(r["cmd_w"])) for r in R]
    G = [D(float(r["gyro_w"])) for r in R]; V = [float(r["v"]) for r in R]
    dt = (T[-1] - T[0]) / max(1, len(T) - 1)
    k = max(1, int(0.15 / dt))
    S = [st.mean(G[max(0, i - k):i + k + 1]) for i in range(len(G))]
    idx = [i for i in range(len(R)) if V[i] > 0.3 and abs(C[i]) < 0.1]
    flutter = st.pstdev([G[i] - S[i] for i in idx]) if len(idx) > 10 else float("nan")
    over = []
    i = 1
    while i < len(T):
        if abs(C[i]) > 0.5 and abs(C[i - 1]) <= 0.5 and V[i] > 0.2:
            sgn = 1 if C[i] > 0 else -1
            amp = abs(C[i])
            win = [j for j in range(i, len(T)) if T[j] - T[i] < 1.4]
            over.append(max(S[j] * sgn for j in win) / amp)
            while i < len(T) and abs(C[i]) > 0.5:
                i += 1
        i += 1
    return flutter, (st.mean(over) if over else float("nan"))


def analyze(path):
    R = list(csv.DictReader(open(path)))
    if not R:
        return
    D = math.degrees
    T = [float(r["t"]) for r in R]; C = [D(float(r["cmd_w"])) for r in R]
    E = [D(float(r["enc_w"])) for r in R]; G = [D(float(r["gyro_w"])) for r in R]
    # 지령 계단마다: 지연(목표 변화의 50% 도달), 넘침, 정상값
    for i in range(1, len(T)):
        if abs(C[i] - C[i - 1]) > 0.5:
            c0, c1 = C[i - 1], C[i]
            seg = [j for j in range(i, len(T)) if T[j] - T[i] < 1.0]
            def resp(X):
                b0 = st.mean(X[max(0, i - 10):i])
                half = b0 + 0.5 * (c1 - c0)
                t50 = next((T[j] - T[i] for j in seg if (X[j] - half) * (c1 - c0) >= 0), float("nan"))
                pk = max((X[j] - b0) * (1 if c1 > c0 else -1) for j in seg)
                end = st.mean(X[j] for j in seg[-10:])
                return t50, pk / abs(c1 - c0), (end - b0) / (c1 - c0)
            e, g = resp(E), resp(G)
            print("  지령 %+.1f -> %+.1f 도/s | 바퀴: 50%% %.2f초, 최대 %.2f배, 1초 뒤 %.2f배 | 차체: 50%% %.2f초, 최대 %.2f배, 1초 뒤 %.2f배"
                  % (c0, c1, e[0], e[1], e[2], g[0], g[1], g[2]))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--analyze":
        analyze(sys.argv[2]); sys.exit(0)
    sys.exit(main())
