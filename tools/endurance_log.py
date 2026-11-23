#!/usr/bin/env python3
"""장시간 반복 주행 기록 — 배터리 소모 · 주행 거리 · 왕복 횟수 (2026-10-27, 8시간 시험).

    python3 tools/endurance_log.py -t 30600           8.5시간, 10초마다 (logs/endurance_MMDD_HHMM.csv)
    python3 tools/endurance_log.py --report logs/endurance_1027_0925.csv

듣기만 한다. 기록 열
  t, voltage, current, power_w, soc, remaining_ah, energy_wh, temperature  (BMS, /battery-state)
  lap, route_state                                                          (/route/status)
  dist_m       : 시작부터 바퀴 오도메트리(/odom)로 잰 누적 주행 거리
  moving_s     : 시작부터 움직인(0.02 m/s 넘게) 누적 시간
  iq1, iq2     : 모터 전류 [A] (/chassis/status, 있으면)
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import threading
import time
import urllib.request

API = "http://localhost:8090"
COLS = ["t", "voltage", "current", "power_w", "soc", "remaining_ah", "energy_wh",
        "temperature", "lap", "route_state", "dist_m", "moving_s", "iq1", "iq2"]


def get(path):
    with urllib.request.urlopen(API + path, timeout=2.0) as f:
        return json.loads(f.read())


def record(sec, period, path):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    rclpy.init()
    n = Node("endurance_log")
    lock = threading.Lock()
    S = {"last": None, "dist": 0.0, "moving": 0.0, "t": None}

    def on_odom(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        now = time.monotonic()
        with lock:
            if S["last"] is not None:
                d = math.hypot(p[0] - S["last"][0], p[1] - S["last"][1])
                if d < 0.5:                       # 튐(재시작 등)은 빼고
                    S["dist"] += d
            if S["t"] is not None and abs(m.twist.twist.linear.x) > 0.02:
                S["moving"] += min(now - S["t"], 0.5)
            S["last"], S["t"] = p, now
    n.create_subscription(Odometry, "/odom", on_odom, 20)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(COLS)
    print("기록: %s (%.0f초, %.0f초마다)" % (path, sec, period), flush=True)
    end = time.time() + sec
    while time.time() < end:
        t0 = time.time()
        row = {"t": "%.1f" % t0}
        try:
            b = get("/battery-state")
            for k in ("voltage", "current", "power_w", "soc", "remaining_ah", "energy_wh", "temperature"):
                row[k] = b.get(k)
        except Exception:                                    # noqa: BLE001
            pass
        try:
            r = get("/route/status")
            row["lap"], row["route_state"] = r.get("lap"), r.get("state")
        except Exception:                                    # noqa: BLE001
            pass
        try:
            m = get("/chassis/status").get("motors", {})
            row["iq1"] = (m.get("axis1") or {}).get("iq")
            row["iq2"] = (m.get("axis2") or {}).get("iq")
        except Exception:                                    # noqa: BLE001
            pass
        with lock:
            row["dist_m"] = "%.2f" % S["dist"]
            row["moving_s"] = "%.0f" % S["moving"]
        w.writerow([row.get(c, "") for c in COLS])
        f.flush()
        time.sleep(max(0.0, period - (time.time() - t0)))
    f.close()
    ex.shutdown()
    rclpy.shutdown()


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def report(path):
    R = list(csv.DictReader(open(path)))
    R = [r for r in R if fnum(r["soc"]) is not None]
    if len(R) < 2:
        print("기록 부족")
        return
    a, b = R[0], R[-1]
    hrs = (float(b["t"]) - float(a["t"])) / 3600.0
    dsoc = fnum(a["soc"]) - fnum(b["soc"])
    dah = (fnum(a["remaining_ah"]) or 0) - (fnum(b["remaining_ah"]) or 0)
    dwh = (fnum(a["energy_wh"]) or 0) - (fnum(b["energy_wh"]) or 0)
    dist = fnum(b["dist_m"]) - fnum(a["dist_m"])
    mov = fnum(b["moving_s"]) - fnum(a["moving_s"])
    laps = (fnum(b["lap"]) or 0) - (fnum(a["lap"]) or 0)
    pw = [-fnum(r["power_w"]) for r in R if fnum(r["power_w"]) is not None]
    print("== %s" % os.path.basename(path))
    print("  시간 %.2f시간 | 왕복 %d회 | 주행 %.0fm (움직인 시간 %.0f%%)" % (hrs, laps, dist, 100 * mov / max(1, hrs * 3600)))
    print("  배터리 %s%% -> %s%% (%.0f%%p) | %.2fV -> %.2fV | 쓴 용량 %.2fAh / %.0fWh"
          % (a["soc"], b["soc"], dsoc, fnum(a["voltage"]), fnum(b["voltage"]), dah, dwh))
    if pw:
        print("  평균 소비 전력 %.1fW (최대 %.1fW) | 시간당 %.1f%%p, %.2fAh" % (st.mean(pw), max(pw), dsoc / max(hrs, 1e-6), dah / max(hrs, 1e-6)))
    if dsoc > 0:
        print("  이 속도면 100%%->0%% 까지 약 %.1f시간 (배터리 설계 25Ah 기준 %.1f시간)"
              % (100.0 / (dsoc / hrs), 25.0 / max(dah / hrs, 1e-6)))
    if dist > 0:
        print("  주행 1km 당 %.0fWh" % (dwh / (dist / 1000.0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=30600.0)
    ap.add_argument("--period", type=float, default=10.0)
    ap.add_argument("--report")
    a = ap.parse_args()
    if a.report:
        report(a.report)
        return
    path = os.path.expanduser(time.strftime("~/projects/UND_AMR/logs/endurance_%m%d_%H%M.csv"))
    record(a.t, a.period, path)
    report(path)


if __name__ == "__main__":
    main()
