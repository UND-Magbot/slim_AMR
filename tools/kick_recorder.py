#!/usr/bin/env python3
"""차체 '튐' 기록·판정 — 지령은 거의 0 인데 차체가 도는 순간을 가려낸다.

    python3 tools/kick_recorder.py              Ctrl-C 로 멈출 때까지 기록
    python3 tools/kick_recorder.py -t 300       300초만 기록
    python3 tools/kick_recorder.py --analyze logs/kick_0929_1030.csv

도킹·나오기 중 캐스터가 차체를 돌릴 때 원인은 셋 중 하나다(2026-09-29).
  ① 전류 한계   : 튈 때 한쪽 바퀴 전류가 한계 근처 → 전류 한계를 올린다
  ② 속도 루프   : 엔코더에도 좌우 속도 차가 찍힌다(바퀴가 실제로 밀림)
                  → 드라이브 속도 루프 이득을 올린다
  ③ 미끄러짐    : 엔코더는 곧게 도는데 자이로만 돈다 → 구동륜 접지(하드웨어)

기록 : /imu/base_link (자이로 z = 실제 차체 회전), /odom (엔코더로 계산한 회전),
       /cmd_vel·/cmd_vel_noradius (지령 회전), /diagnostics (바퀴 전류 iq)
판정 : |자이로| > 3 도/s 이고 |지령| < 1 도/s 인 구간을 '튐' 으로 묶어,
       그 구간의 엔코더 회전 / 자이로 회전 비율과 최대 전류를 낸다.
"""
import argparse
import csv
import math
import sys
import time

KICK_GYRO = math.radians(3.0)     # 이보다 빨리 돌면 튐 후보 [rad/s]
KICK_CMD = math.radians(1.0)      # 지령이 이보다 작을 때만 (지령이 만든 회전 제외)


def record(seconds, path):
    import rclpy
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    import signal
    stop = {"now": False}
    # SIGINT 를 직접 받는다. rclpy 가 먼저 받으면 spin 중에 예외 종류가
    # 매번 달라져(ExternalShutdownException, RCLError) 판정을 못 냈다.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__("now", True))
    n = Node("kick_recorder")
    st = {"gyro": 0.0, "odom_w": 0.0, "odom_v": 0.0, "cmd_w": 0.0,
          "cmd_t": 0.0, "iq1": None, "iq2": None, "act1": None, "act2": None}

    def on_imu(m):
        st["gyro"] = m.angular_velocity.z

    def on_odom(m):
        st["odom_w"] = m.twist.twist.angular.z
        st["odom_v"] = m.twist.twist.linear.x

    def on_cmd(m):
        st["cmd_w"] = m.angular.z
        st["cmd_t"] = time.time()

    def on_diag(m):
        for s in m.status:
            kv = {k.key: k.value for k in s.values}
            ax = "1" if "1" in s.name else ("2" if "2" in s.name else None)
            if ax is None:
                continue
            try:
                st["iq" + ax] = (float(kv["iq"]) if kv.get("iq") not in
                                 (None, "None") else None)
                st["act" + ax] = float(kv.get("act_rpm", "nan"))
            except ValueError:
                pass

    # IMU 는 best effort 로 나온다. 기본(reliable)으로 구독하면 QoS 가 안 맞아
    # 한 개도 안 들어온다(2026-09-29 첫 기록에서 자이로가 비었다).
    from rclpy.qos import qos_profile_sensor_data
    n.create_subscription(Imu, "/imu/base_link", on_imu, qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom", on_odom, 50)
    n.create_subscription(Twist, "/cmd_vel", on_cmd, 50)
    n.create_subscription(Twist, "/cmd_vel_noradius", on_cmd, 50)
    n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)

    # ⚠ 메시지는 별도 스레드에서 계속 처리한다. spin_once 를 한 번씩 부르면
    #   자이로(200Hz)에 밀려 /odom 이 줄을 서 1~2초 늦게 기록됐다(2026-10-02).
    import threading
    from rclpy.executors import SingleThreadedExecutor
    _ex = SingleThreadedExecutor()
    _ex.add_node(n)
    threading.Thread(target=_ex.spin, daemon=True).start()

    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "gyro_z", "odom_w", "odom_v", "cmd_w", "iq1", "iq2",
                "act1", "act2"])
    t0 = time.time()
    nxt = t0
    print("기록 중: %s (Ctrl-C 로 멈춤)" % path)
    try:
        while (rclpy.ok() and not stop["now"]
               and (seconds <= 0 or time.time() - t0 < seconds)):
            time.sleep(0.01)
            now = time.time()
            if now >= nxt:
                nxt += 0.05                      # 20 Hz
                cmd = st["cmd_w"] if now - st["cmd_t"] < 0.5 else 0.0
                w.writerow(["%.3f" % (now - t0), "%.5f" % st["gyro"],
                            "%.5f" % st["odom_w"], "%.4f" % st["odom_v"],
                            "%.5f" % cmd, st["iq1"], st["iq2"],
                            st["act1"], st["act2"]])
    except KeyboardInterrupt:
        pass
    except Exception as exc:                      # noqa: BLE001
        # SIGINT 를 rclpy 가 먼저 받으면 ExternalShutdownException 으로 온다.
        # 그래도 모은 기록은 닫고 판정까지 한다.
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    f.close()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("저장: %s" % path)


def analyze(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            def num(k):
                try:
                    return float(r[k])
                except (TypeError, ValueError):
                    return None
            rows.append({k: num(k) for k in r})
    kicks, cur = [], None
    for r in rows:
        hit = (abs(r["gyro_z"]) > KICK_GYRO and abs(r["cmd_w"]) < KICK_CMD
               and abs(r["odom_v"] or 0.0) > 0.005)
        if hit:
            cur = cur or []
            cur.append(r)
        elif cur:
            if len(cur) >= 3:                     # 0.15초 이상 이어진 것만
                kicks.append(cur)
            cur = None
    print("%s : %.0f초, 튐 %d건" % (path, rows[-1]["t"] if rows else 0,
                                   len(kicks)))
    if not kicks:
        return
    for k in kicks:
        dt = 0.05
        g = sum(r["gyro_z"] for r in k) * dt
        o = sum(r["odom_w"] for r in k) * dt
        iqs = [abs(x) for r in k for x in (r["iq1"], r["iq2"]) if x is not None]
        ratio = (o / g) if abs(g) > 1e-6 else float("nan")
        if ratio != ratio:
            verdict = "?"
        elif ratio < 0.3:
            verdict = "③ 미끄러짐 (엔코더는 거의 안 돎)"
        elif iqs and max(iqs) > 8.0:
            verdict = "① 전류 한계 근처"
        else:
            verdict = "② 바퀴가 실제로 밀림 (속도 루프)"
        print("  t=%6.2fs %.2f초  자이로 %+.1f도  엔코더 %+.1f도 (비율 %.2f)  "
              "최대 전류 %s  -> %s" % (
                  k[0]["t"], len(k) * dt, math.degrees(g), math.degrees(o),
                  ratio, ("%.1fA" % max(iqs)) if iqs else "-", verdict))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-t", type=float, default=0.0, help="기록 시간 [s], 0 이면 Ctrl-C 까지")
    ap.add_argument("--analyze", help="기록한 CSV 를 판정만 한다")
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze)
        return 0
    path = "/home/und_amr1/projects/UND_AMR/logs/kick_%s.csv" % time.strftime(
        "%m%d_%H%M")
    record(a.t, path)
    analyze(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
