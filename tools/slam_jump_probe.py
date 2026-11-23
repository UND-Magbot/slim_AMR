#!/usr/bin/env python3
"""SLAM 보정(map->odom)이 언제 얼마나 튀는지 — 주행 중 / 제자리 회전 중 / 정지 중으로 나눈다.

    python3 tools/slam_jump_probe.py            900초 기록 (왕복을 돌려 둘 것)
    python3 tools/slam_jump_probe.py -t 600
    python3 tools/slam_jump_probe.py --report logs/slamjump_1014_1510.csv

듣기만 한다. 기록 (logs/slamjump_MMDD_HHMM.csv)
  t, mo_x, mo_y, mo_yaw   : map->odom (SLAM 이 고친 몫)
  v, w                    : /odometry/filtered 속도 (그 순간 로봇 상태)

튐 : map->odom 이 한 번에 3mm 또는 0.1도 넘게 바뀐 것. 로봇 위치로 환산한
     크기(로봇이 지도에서 순간이동한 거리)로 잰다.

왜 (2026-10-14): 라이다 스캔끼리 따로 재 보니 SLAM 위치가 1시간에 약 4cm
흘렀고, 지점에 설 때마다 15~20mm 튀었다. 사용자 선택으로 SLAM 쪽을 고친다.
설정을 바꿀 때마다 이 도구로 전후를 비교한다.
"""
import argparse
import csv
import math
import os
import statistics as st
import threading
import time

JUMP_M = 0.003
JUMP_RAD = math.radians(0.1)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def record(sec, path):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = Node("slam_jump_probe")
    lock = threading.Lock()
    st_ = {"v": 0.0, "w": 0.0, "ox": 0.0, "oy": 0.0, "oyaw": 0.0}
    rows = []

    def on_odom(m):
        with lock:
            st_["v"] = m.twist.twist.linear.x
            st_["w"] = m.twist.twist.angular.z
            st_["ox"] = m.pose.pose.position.x
            st_["oy"] = m.pose.pose.position.y
            st_["oyaw"] = yaw_of(m.pose.pose.orientation)

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                tr = t.transform
                with lock:
                    rows.append((time.time(), tr.translation.x, tr.translation.y,
                                 yaw_of(tr.rotation), st_["v"], st_["w"],
                                 st_["ox"], st_["oy"], st_["oyaw"]))

    n.create_subscription(Odometry, "/odometry/filtered", on_odom, 20)
    n.create_subscription(TFMessage, "/tf", on_tf, 100)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    print("기록: %s (%.0f초)" % (path, sec), flush=True)
    end = time.time() + sec
    while time.time() < end:
        time.sleep(1.0)
    ex.shutdown()
    with lock, open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "mo_x", "mo_y", "mo_yaw", "v", "w", "od_x", "od_y", "od_yaw"])
        for r in rows:
            w.writerow(["%.3f" % r[0]] + ["%.5f" % v for v in r[1:]])
    rclpy.shutdown()


def robot_shift(a, b):
    """map->odom 이 a -> b 로 바뀔 때 로봇이 지도에서 옮겨진 거리 (오도메트리 자리 기준)."""
    def apply(mo, od):
        c, s = math.cos(mo[2]), math.sin(mo[2])
        return mo[0] + c * od[0] - s * od[1], mo[1] + s * od[0] + c * od[1]
    od = (b["od_x"], b["od_y"])
    p1 = apply((a["mo_x"], a["mo_y"], a["mo_yaw"]), od)
    p2 = apply((b["mo_x"], b["mo_y"], b["mo_yaw"]), od)
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def report(path):
    R = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    if len(R) < 2:
        print("기록 없음")
        return
    jumps = []
    for a, b in zip(R, R[1:]):
        dyaw = abs(math.atan2(math.sin(b["mo_yaw"] - a["mo_yaw"]), math.cos(b["mo_yaw"] - a["mo_yaw"])))
        dxy = math.hypot(b["mo_x"] - a["mo_x"], b["mo_y"] - a["mo_y"])
        if dxy < JUMP_M and dyaw < JUMP_RAD:
            continue
        v, w = b["v"], b["w"]
        kind = "주행" if abs(v) > 0.02 else ("제자리회전" if abs(w) > 0.05 else "정지")
        jumps.append((b["t"], kind, robot_shift(a, b) * 1000, D(dyaw)))
    dur = R[-1]["t"] - R[0]["t"]
    print("== %s  %.0f초, map->odom %d건" % (os.path.basename(path), dur, len(R)))
    # 상태별 시간
    tm = {"주행": 0.0, "제자리회전": 0.0, "정지": 0.0}
    for a, b in zip(R, R[1:]):
        k = "주행" if abs(a["v"]) > 0.02 else ("제자리회전" if abs(a["w"]) > 0.05 else "정지")
        tm[k] += min(b["t"] - a["t"], 0.5)
    for k in ("주행", "제자리회전", "정지"):
        js = [j for j in jumps if j[1] == k]
        if js:
            mm = [j[2] for j in js]
            print("  %-6s %4.0f초 | 튐 %3d번 (분당 %.1f) | 로봇 순간이동 평균 %.1f 최대 %.1f mm | 방향 최대 %.2f도"
                  % (k, tm[k], len(js), len(js) / max(tm[k], 1) * 60, st.mean(mm), max(mm),
                     max(j[3] for j in js)))
        else:
            print("  %-6s %4.0f초 | 튐 없음" % (k, tm[k]))
    big = sorted(jumps, key=lambda j: -j[2])[:8]
    print("  큰 튐:", ", ".join("%s %s %.0fmm" % (time.strftime("%H:%M:%S", time.localtime(j[0])), j[1], j[2])
                              for j in big))


D = math.degrees


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=900.0)
    ap.add_argument("--report")
    a = ap.parse_args()
    if a.report:
        report(a.report)
        return
    path = os.path.expanduser(time.strftime("~/projects/UND_AMR/logs/slamjump_%m%d_%H%M.csv"))
    record(a.t, path)
    report(path)


if __name__ == "__main__":
    main()
