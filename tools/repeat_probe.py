#!/usr/bin/env python3
"""반복정밀도 실측 — 라이다 스캔끼리 맞춰서 '실제로' 처음 자리에서 얼마나 비켜 섰는지 잰다.

    python3 tools/repeat_probe.py              3600초 기록 (왕복을 돌려 둘 것)
    python3 tools/repeat_probe.py -t 1800
    python3 tools/repeat_probe.py --report logs/repeat_1014_1250.csv   기록만 다시 요약

듣기만 한다(로봇을 움직이지 않는다).

왜 따로 재나 (2026-10-14)
  api_node 의 '도착 위치 오차' 는 SLAM 이 낸 위치 기준이다. 10/08 에 api_node 를
  다시 띄울 때마다 위치가 약 10cm 옮겨졌는데(라이다-지도 일치율 23~31% -> 57~59%),
  그동안 도착 오차는 3.8mm 로 찍혔다. 위치 추정이 틀어지면 그 숫자에는 안 보인다.

방법
  왕복이 지점에서 기다리는 동안(/route/status state=waiting) 스캔 5장을 찍는다.
  지점마다 처음 선 스캔을 기준으로, 이후 스캔을 ICP(점-점, 2D)로 맞춰 '기준 자세에서
  본 지금 로봇 자리' 를 구한다. 지도도 SLAM 도 안 쓰므로 위치 추정이 틀어져도 상관없다.
  같은 순간 SLAM 이 낸 위치 차이도 같이 적어, 둘의 차이(= 위치 추정 틀어짐)를 본다.

기록 열 (기준 자세 기준, 앞 +, 왼쪽 +)
  real_fwd/real_left/real_yaw  : 라이다로 잰 실제 차이 [mm, mm, 도]
  slam_fwd/slam_left/slam_yaw  : SLAM 이 낸 위치 차이
  fit_mm, inlier               : ICP 잔차 중앙값, 맞은 점 비율 (나쁘면 사람·물체가 섞인 것)
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import sys
import threading
import time
import urllib.request

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/und_amr_bringup"))
from und_amr_bringup import scan_icp   # noqa: E402  api_node 와 같은 점-선 ICP (2026-10-14)

API = "http://localhost:8090"
N_SCANS = 5
RANGE_MAX = 8.0
LASER_XY_YAW = None          # base_footprint <- laser (TF 에서 한 번 읽는다)


# ------------------------------------------------------------------ ICP
def kabsch(a, b):
    """b ≈ R a + t 가 되는 R, t (2D)."""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    return R, cb - R @ ca


def icp(src, dst, iters=50):
    """src(지금 로봇 기준 점) 를 dst(기준 자세 기준 점) 에 맞춘다.
    반환: 기준 자세에서 본 지금 로봇 자리 (x, y, yaw), 잔차 중앙값[m], 맞은 비율."""
    R, t = np.eye(2), np.zeros(2)
    thr = 0.20
    for k in range(iters):
        s = src @ R.T + t
        d2 = ((s[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(s)), j])
        m = dist < thr
        if m.sum() < 30:
            break
        dR, dt = kabsch(s[m], dst[j[m]])
        R, t = dR @ R, dR @ t + dt
        thr = max(0.03, min(thr, 3.0 * float(np.median(dist[m]))))
        if abs(math.atan2(dR[1, 0], dR[0, 0])) < 1e-5 and np.hypot(*dt) < 1e-5:
            break
    s = src @ R.T + t
    d2 = ((s[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
    dist = np.sqrt(d2.min(1))
    return (float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]),
            float(np.median(dist)), float(np.mean(dist < 0.03)))


def scan_points(m):
    r = np.asarray(m.ranges, dtype=float)
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r > 0.12) & (r < RANGE_MAX)
    r, a = r[ok], a[ok]
    lx, ly, lyaw = LASER_XY_YAW
    return np.stack([lx + r * np.cos(a + lyaw), ly + r * np.sin(a + lyaw)], 1)


# ------------------------------------------------------------------ 기록
def rel(p0, p1):
    """p0 자세에서 본 p1 (앞, 왼쪽, 방향)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    d = p1[2] - p0[2]
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(d), math.cos(d))


def http_json(path):
    with urllib.request.urlopen(API + path, timeout=1.0) as f:
        return json.loads(f.read())


def record(sec, path):
    global LASER_XY_YAW
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("repeat_probe")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    scans = []
    lock = threading.Lock()

    def on_scan(m):
        with lock:
            scans.append((time.time(), m))
            del scans[:-40]
    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def tf2d(a, b):
        tr = buf.lookup_transform(a, b, rclpy.time.Time())
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    t0 = time.time()
    while LASER_XY_YAW is None and time.time() - t0 < 15:
        try:
            with lock:
                fid = scans[-1][1].header.frame_id if scans else None
            if fid:
                LASER_XY_YAW = tf2d("base_footprint", fid)
        except Exception:                                    # noqa: BLE001
            pass
        time.sleep(0.2)
    if LASER_XY_YAW is None:
        sys.exit("라이다 TF(base_footprint <- laser)를 못 읽었다")

    ref = {}                                  # 지점 -> (점들 목록, slam 자세)
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "target", "n", "real_fwd", "real_left", "real_yaw",
                "slam_fwd", "slam_left", "slam_yaw", "fit_mm", "inlier"])
    print("기록: %s (%.0f초)" % (path, sec), flush=True)
    seen = None
    prev_state = "waiting"          # 켜자마자 이미 기다리는 중이면 건너뛴다 (끝물일 수 있다)
    end = time.time() + sec
    while time.time() < end:
        try:
            r = http_json("/route/status")
        except Exception:                                    # noqa: BLE001
            time.sleep(0.5)
            continue
        key = (r.get("lap"), r.get("target"), r.get("state"))
        # ⚠ '이동 중 -> 대기' 로 바뀌는 순간에만 찍는다. 대기가 끝나 다음 목표로
        #   돌기 시작한 뒤에도 0.5초쯤 상태가 'waiting' 으로 남아, 그때 찍은 기준
        #   스캔이 7도 돌아가 있었다(2026-10-14, A +7.1도 두 번 똑같이).
        was, prev_state = prev_state, r.get("state")
        if r.get("state") != "waiting" or was == "waiting" or key == seen:
            time.sleep(0.1)
            continue
        seen = key
        tgt = r.get("target")
        time.sleep(0.3)                       # 멈춘 직후 떨림이 가라앉게
        t_a = time.time()
        while time.time() - t_a < 2.0:
            with lock:
                got = [m for (ts, m) in scans if ts > t_a]
            if len(got) >= N_SCANS:
                break
            time.sleep(0.05)
        try:
            slam = tf2d("map", "base_footprint")
        except Exception:                                    # noqa: BLE001
            slam = None
        # 찍는 동안 움직였으면 버린다 (0.3초 사이 SLAM 자세 비교)
        try:
            time.sleep(0.3)
            slam3 = tf2d("map", "base_footprint")
            if slam is not None and (math.hypot(slam3[0] - slam[0], slam3[1] - slam[1]) > 0.002
                                     or abs(slam3[2] - slam[2]) > math.radians(0.2)):
                print("  %s: 찍는 동안 움직임 — 버림" % tgt, flush=True)
                continue
        except Exception:                                    # noqa: BLE001
            pass
        pts = [scan_points(m) for m in got[:N_SCANS]]
        if len(pts) < 2 or slam is None:
            print("  %s: 스캔/위치 부족 — 건너뜀" % tgt, flush=True)
            continue
        if tgt not in ref:
            # 기준은 점을 다 쓴다 (점-선 ICP 의 법선도 한 번만 구한다)
            ref[tgt] = (pts, slam, scan_icp.normals(pts[0]))
            print("  %s: 기준 스캔 저장 (SLAM %.3f, %.3f, %.1f도)"
                  % (tgt, slam[0], slam[1], math.degrees(slam[2])), flush=True)
            continue
        rpts, rslam, rn = ref[tgt]
        dst = rpts[0]
        guess = rel(rslam, slam)
        res = [scan_icp.icp(p[::4], dst, guess, dst_n=rn) for p in pts]
        x = st.median(v[0][0] for v in res)
        y = st.median(v[0][1] for v in res)
        yaw = st.median(v[0][2] for v in res)
        fit = st.median(v[1] for v in res)
        inl = st.median(v[2] for v in res)
        sf, sl, sy = rel(rslam, slam)
        w.writerow(["%.2f" % time.time(), tgt, len(res), "%.1f" % (x * 1000), "%.1f" % (y * 1000),
                    "%.2f" % math.degrees(yaw), "%.1f" % (sf * 1000), "%.1f" % (sl * 1000),
                    "%.2f" % math.degrees(sy), "%.1f" % (fit * 1000), "%.2f" % inl])
        f.flush()
        print("  %s: 실제 앞 %+6.1f 좌 %+6.1f mm %+5.2f도 | SLAM 앞 %+6.1f 좌 %+6.1f mm | 맞음 %.0f%% 잔차 %.0fmm"
              % (tgt, x * 1000, y * 1000, math.degrees(yaw), sf * 1000, sl * 1000,
                 inl * 100, fit * 1000), flush=True)
    f.close()
    ex.shutdown()
    rclpy.shutdown()


def report(path):
    R = list(csv.DictReader(open(path)))
    if not R:
        print("기록 없음")
        return
    for tgt in sorted(set(r["target"] for r in R)):
        xs = [r for r in R if r["target"] == tgt and float(r["inlier"]) >= 0.5]
        if not xs:
            continue
        def col(k):
            return [float(r[k]) for r in xs]
        rf, rl, ry = col("real_fwd"), col("real_left"), col("real_yaw")
        sf, sl = col("slam_fwd"), col("slam_left")
        dist = [math.hypot(a, b) for a, b in zip(rf, rl)]
        print("== 지점 %s  %d회 (맞음 50%% 미만 %d회 뺌)"
              % (tgt, len(xs), sum(1 for r in R if r["target"] == tgt) - len(xs)))
        print("  실제(라이다)  앞 %+5.1f ±%4.1f mm | 좌 %+5.1f ±%4.1f mm | 방향 %+.2f ±%.2f도 | 기준에서 거리 평균 %.1f 최대 %.1f mm"
              % (st.mean(rf), st.pstdev(rf), st.mean(rl), st.pstdev(rl), st.mean(ry), st.pstdev(ry),
                 st.mean(dist), max(dist)))
        print("  SLAM 생각    앞 %+5.1f ±%4.1f mm | 좌 %+5.1f ±%4.1f mm"
              % (st.mean(sf), st.pstdev(sf), st.mean(sl), st.pstdev(sl)))
        e = [math.hypot(a - c, b - d) for a, b, c, d in zip(rf, rl, sf, sl)]
        print("  실제와 SLAM 차이(위치 추정 틀어짐) 평균 %.1f 최대 %.1f mm" % (st.mean(e), max(e)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", type=float, default=3600.0)
    ap.add_argument("--report")
    a = ap.parse_args()
    if a.report:
        report(a.report)
        return
    path = os.path.expanduser(time.strftime("~/projects/UND_AMR/logs/repeat_%m%d_%H%M.csv"))
    try:
        record(a.t, path)
    finally:
        if os.path.exists(path):
            report(path)


if __name__ == "__main__":
    main()
