#!/usr/bin/env python3
"""라이다 2대 장착 위치·방향 측정 (2026-10-26, 차체 변경 · 라이다 2대 대각선 장착).

    python3 tools/lidar_calib.py            시험 전체 (회전 + 직진)
    python3 tools/lidar_calib.py --dry      움직이지 않고 둘레 확인 · 몸체 점 표시만

⚠ 로봇이 움직인다: 제자리 좌·우 90도 4번씩, 앞으로 0.4m 갔다 뒤로 0.4m.
  /cmd_vel_noradius (base_node 우선 통로) 로 직접 몬다 — nav2 · 왕복을 멈춘 뒤 쓴다.
  움직이기 전 둘레(중심에서 0.6m)와 앞뒤 0.5m 에 물체가 있으면 시작하지 않고,
  움직이는 동안 차체에서 0.15m 안으로 물체가 들어오면 바로 선다.

방법 (각 라이다를 자기 좌표 그대로 쓴다 — URDF 값은 안 쓴다)
  위치 : 90도 돌 때마다 선 채로 스캔을 찍어 앞 스캔과 점-선 ICP. 라이다 원점은
         회전 중심(구동축 가운데) c 를 돌아 (I - R(θ))c 만큼 옮겨진다 → c (라이다 좌표).
  방향 : 앞으로 0.4m 간 전후를 맞춰, 그 움직임이 라이다 좌표에서 어느 쪽인지 →
         라이다 yaw ψ. 그때 라이다 위치 p = -R(ψ) c (base_footprint 좌표).
  확인 : 두 라이다를 구한 값으로 base_footprint 에 옮겨 같은 벽이 겹치는지 본다.

도면: 앞 왼쪽 라이다는 왼쪽으로 90도(ψ≈+90), 뒤 오른쪽은 오른쪽으로 90도(ψ≈-90),
중심선에서 좌우 135mm. 어느 토픽이 앞인지는 이 시험으로 정해진다.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/und_amr_bringup"))
from und_amr_bringup import scan_icp   # noqa: E402

D = math.degrees
TOPICS = ("scan", "scan2")
W_TURN = 0.30
V_LIN = 0.05
DIST = 0.40
# 차체 (도면 873.5 x 328.8). 라이다 좌표에서의 대략 사각형 — 두 라이다 모두 대각선
# 모서리에 ±90도 돌려 달려 있어 자기 좌표에서는 같은 모양이다(대칭):
#   차체 중심 ≈ (-0.135, +0.39), 긴 쪽이 라이다 y 축 방향.
HALF_L, HALF_W = 0.4368, 0.1644
BODY_C = (-0.135, 0.39)
SWEEP_R = 0.60
STOP_MARGIN = 0.15


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def polar(m, step=1):
    r = np.asarray(m.ranges, dtype=float)[::step]
    a = m.angle_min + np.arange(0, len(m.ranges), step) * m.angle_increment
    ok = np.isfinite(r) & (r > 0.05) & (r < 12.0)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)


def body_dist(p):
    """라이다 좌표 점 -> 대략 차체 사각형 바깥까지 거리 (안이면 음수)."""
    dx = np.abs(p[:, 0] - BODY_C[0]) - HALF_W
    dy = np.abs(p[:, 1] - BODY_C[1]) - HALF_L
    out = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    ins = np.minimum(np.maximum(dx, dy), 0)
    return out + ins


class SelfMask:
    """선 채로 찍은 몇 장에서 늘 같은 자리에 보이는 가까운 점(거치대 · 브래킷)."""

    def __init__(self, scans):
        R = np.array([np.asarray(m.ranges, dtype=float) for m in scans])
        R[~np.isfinite(R)] = np.nan
        self.med = np.nanmedian(R, 0)
        self.inc = scans[0].angle_increment
        self.amin = scans[0].angle_min
        self.near = np.isfinite(self.med) & (self.med < 0.60)

    def obstacle_pts(self, m):
        r = np.asarray(m.ranges, dtype=float)
        a = self.amin + np.arange(len(r)) * self.inc
        ok = np.isfinite(r) & (r > 0.05)
        # 거치대: 같은 각도에 늘 보이던 거리와 3cm 안이면 자기 몸.
        # 돌면서 거치대가 떨려 거리가 흔들린다 — 몸체 방향의 0.6m 안 점은 다 몸으로 본다
        # (첫 시험에서 돌기 시작하자마자 4점으로 멈췄다, 2026-10-26).
        own = self.near & ok & ((np.abs(r - np.nan_to_num(self.med, nan=99)) < 0.03) | (r < 0.60))
        k = ok & ~own
        p = np.stack([r[k] * np.cos(a[k]), r[k] * np.sin(a[k])], 1)
        return p[body_dist(p) > 0.02]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--analyze", help="저장해 둔 스캔(npz)으로 계산만 다시")
    a = ap.parse_args()
    if a.analyze:
        analyze(list(np.load(a.analyze, allow_pickle=True)["steps"]))
        return

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    n = Node("lidar_calib")
    pub = n.create_publisher(Twist, "/cmd_vel_noradius", 10)
    lock = threading.Lock()
    S = {t: [] for t in TOPICS}
    O = {"v": 0.0, "w": 0.0, "od": None}

    def on_scan(m, t):
        with lock:
            S[t].append((time.time(), m))
            del S[t][:-30]

    def on_odom(m):
        with lock:
            O["v"], O["w"] = m.twist.twist.linear.x, m.twist.twist.angular.z
            O["od"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
    for t in TOPICS:
        n.create_subscription(LaserScan, "/" + t, lambda m, t=t: on_scan(m, t), qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odometry/filtered", on_odom, 20)
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    def send(v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(t)

    def stop():
        for _ in range(10):
            send(0.0, 0.0)
            time.sleep(0.03)

    def still():
        with lock:
            return abs(O["v"]) < 0.003 and abs(O["w"]) < 0.01

    def latest(t, after=0.0):
        with lock:
            xs = [m for ts, m in S[t] if ts > after]
        return xs[-1] if xs else None

    t0 = time.time()
    while time.time() - t0 < 10 and (O["od"] is None or any(latest(t) is None for t in TOPICS)):
        time.sleep(0.2)
    if O["od"] is None or any(latest(t) is None for t in TOPICS):
        sys.exit("스캔 2개 / 오도메트리를 못 받았다")

    # 몸체 점 (선 채로 2초)
    time.sleep(2.0)
    with lock:
        masks = {t: SelfMask([m for _, m in S[t][-25:]]) for t in TOPICS}

    def obstacles(t):
        m = latest(t)
        return masks[t].obstacle_pts(m) if m is not None else np.zeros((0, 2))

    # 둘레 확인
    bad = []
    for t in TOPICS:
        p = obstacles(t)
        dc = np.hypot(p[:, 0] - BODY_C[0], p[:, 1] - BODY_C[1])
        ring = dc < SWEEP_R
        # 앞뒤(라이다 y 축 방향, 차체 끝에서 0.5m)
        ends = (np.abs(p[:, 0] - BODY_C[0]) < HALF_W + 0.05) & (np.abs(p[:, 1] - BODY_C[1]) < HALF_L + 0.5)
        print("  /%s: 몸체로 뺀 각도 %d개, 둘레 %.2fm 안 물체 점 %d개, 앞뒤 0.5m 안 %d개"
              % (t, int(masks[t].near.sum()), SWEEP_R, int(ring.sum()), int(ends.sum())), flush=True)
        if ring.sum() > 3 or ends.sum() > 3:
            bad.append(t)
    if bad:
        stop()
        sys.exit("주변에 물체가 있어 시작하지 않는다 (%s)" % ", ".join(bad))
    if a.dry:
        print("둘레 비었음 (--dry: 움직이지 않음)")
        return

    strikes = {t: 0 for t in TOPICS}

    def guard(kind):
        """움직이는 중 침범 감시. 점 8개 넘게 두 번 연속이면 True."""
        for t in TOPICS:
            p = obstacles(t)
            if kind == "turn":
                hit = np.hypot(p[:, 0] - BODY_C[0], p[:, 1] - BODY_C[1]) < SWEEP_R - 0.05
            else:
                hit = body_dist(p) < STOP_MARGIN
            strikes[t] = strikes[t] + 1 if hit.sum() > 8 else 0
            if strikes[t] >= 2:
                q = p[hit]
                ang = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
                print("  ⚠ /%s 에 물체 %d점 (라이다 기준 각도 %.0f~%.0f도, 거리 %.2f~%.2fm) — 정지"
                      % (t, int(hit.sum()), ang.min(), ang.max(),
                         np.hypot(q[:, 0], q[:, 1]).min(), np.hypot(q[:, 0], q[:, 1]).max()), flush=True)
                return True
        return False

    def grab():
        t_s = time.time()
        while not still() and time.time() - t_s < 3.0:
            time.sleep(0.05)
        time.sleep(0.8)
        t_a = time.time()
        out = {}
        for t in TOPICS:
            while time.time() - t_a < 1.5:
                m = latest(t, after=t_a + 0.08)
                if m is not None:
                    out[t] = masks[t].obstacle_pts(m)     # 몸체 · 거치대 뺀 점
                    break
                time.sleep(0.03)
        with lock:
            od = O["od"]
        return out, od

    steps = []
    aborted = False
    try:
        # ---- 제자리 회전
        for sign in (+1, -1):
            for k in range(4):
                s0, o0 = grab()
                start, t_s = o0[2], time.time()
                while True:
                    with lock:
                        yaw = O["od"][2]
                    turned = math.atan2(math.sin(yaw - start), math.cos(yaw - start)) * sign
                    if turned >= math.radians(87) or time.time() - t_s > 15:
                        break
                    if guard("turn"):
                        aborted = True
                        break
                    send(0.0, sign * max(0.1, W_TURN * min(1.0, (time.time() - t_s) / 1.0)))
                    time.sleep(0.05)
                stop()
                if aborted:
                    raise KeyboardInterrupt
                s1, o1 = grab()
                steps.append({"kind": "turn", "odr": scan_icp.relative(o0, o1), "s0": s0, "s1": s1})
                print("  회전 %+5.1f도 (오도메트리)" % D(steps[-1]["odr"][2]), flush=True)
        # ---- 직진 (앞으로 갔다 뒤로)
        for direction in (+1, -1):
            s0, o0 = grab()
            t_s = time.time()
            while True:
                with lock:
                    od = O["od"]
                d = math.hypot(od[0] - o0[0], od[1] - o0[1])
                if d >= DIST - 0.01 or time.time() - t_s > 20:
                    break
                if guard("line"):
                    aborted = True
                    break
                send(direction * V_LIN * min(1.0, (time.time() - t_s) / 1.0 + 0.2), 0.0)
                time.sleep(0.05)
            stop()
            if aborted:
                raise KeyboardInterrupt
            s1, o1 = grab()
            steps.append({"kind": "line", "odr": scan_icp.relative(o0, o1), "s0": s0, "s1": s1})
            print("  직진 %+.0fmm (오도메트리)" % (steps[-1]["odr"][0] * 1000), flush=True)
    except KeyboardInterrupt:
        print("중단됨")
    finally:
        stop()
    raw = os.path.expanduser(time.strftime("~/projects/UND_AMR/logs/lidar_calib_%m%d_%H%M.npz"))
    np.savez(raw, steps=np.array(steps, dtype=object))
    print("스캔 저장:", raw)
    ex.shutdown()
    rclpy.shutdown()
    analyze(steps)


def analyze(steps):
    """저장한 단계별 스캔으로 두 라이다 위치 · 방향을 구한다 (로봇 안 움직임)."""
    def solve_c(pairs):
        A, b = [], []
        for (x, y, th) in pairs:
            c, s = math.cos(th), math.sin(th)
            A += [[1 - c, s], [-s, 1 - c]]
            b += [x, y]
        cL, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
        res = (np.array(A) @ cL - np.array(b)).reshape(-1, 2)
        return cL, float(np.hypot(res[:, 0], res[:, 1]).mean() * 1000)

    def turn_guess(c, th):
        cc, ss = math.cos(th), math.sin(th)
        return ((1 - cc) * c[0] + ss * c[1], -ss * c[0] + (1 - cc) * c[1], th)

    def near(p, rmax=10.0):
        return p[np.hypot(p[:, 0], p[:, 1]) < rmax]

    def best_icp(src, dst, guesses):
        """여러 출발값 중 가장 잘 맞는 것. (맞음 비율, -잔차) 순."""
        dst_n = scan_icp.normals(dst)
        best = None
        for g in guesses:
            rr, fit, inl = scan_icp.icp(src[::2], dst, g, dst_n=dst_n)
            if best is None or (inl, -fit) > (best[2], -best[1]):
                best = (rr, fit, inl)
        return best

    # ⚠ 2026-10-26 첫 계산: 둘레가 넓게 비어 가장 가까운 벽이 1.2~1.9m, 먼 점은
    #   21m 까지 — 맞음 비율이 최고 53% (잔차 21mm) 라 '50% 이상' 기준에 거의 다
    #   버려졌다. 10m 안 점만 쓰고, 출발값 여러 개, '30% 이상 · 잔차 30mm 이하'.
    # ⚠ 90도 돌면 몸체에 가리는 쪽이 바뀌어 두 스캔이 절반쯤만 겹친다 — 잔차 중앙값은
    #   안 겹친 점까지 세어 커진다(15~130mm). 그래도 회전마다 구한 구동축 자리는 ±3mm 로
    #   같았다. 잔차는 보지 않고 맞음 비율만 본다.
    OK_INL, OK_FIT = 0.35, 1.0
    result = {}
    for t in TOPICS:
        for st_ in steps:
            for k in ("s0", "s1"):
                if t in st_[k]:
                    st_[k][t] = near(st_[k][t])
        turns = [st_ for st_ in steps if st_["kind"] == "turn" and t in st_["s0"] and t in st_["s1"]]
        lines = [st_ for st_ in steps if st_["kind"] == "line" and t in st_["s0"] and t in st_["s1"]]
        if len(turns) < 4:
            print("/%s: 회전 자료 부족" % t)
            continue
        # 출발값: 구동축이 차체 가운데 근처 (도면) — 라이다 좌표로 BODY_C
        c = np.array(BODY_C)
        for it in range(3):
            got = []
            spread = 0.10 if it == 0 else 0.03
            for st_ in turns:
                gs = [turn_guess(c + np.array([dx, dy]), st_["odr"][2])
                      for dx in (-spread, 0, spread) for dy in (-spread, 0, spread)]
                rr, fit, inl = best_icp(st_["s1"][t], st_["s0"][t], gs)
                if inl >= OK_INL and fit <= OK_FIT:
                    got.append(rr)
            if len(got) < 4:
                break
            c, resid = solve_c(got)
        if len(got) < 4:
            print("/%s: 회전 맞추기 실패 (맞는 회전 %d개)" % (t, len(got)))
            continue
        print("  /%s 회전 %d개 맞음 | 구동축 = 라이다 좌표 (%+.1f, %+.1f)mm | 회전마다 남는 차이 %.1fmm"
              % (t, len(got), c[0] * 1000, c[1] * 1000, resid))
        # 방향: 직진. 몸체 긴 쪽이 라이다 y 축이므로 ψ 는 +90 또는 -90 근처 — 둘 다 해 본다.
        psis = []
        for st_ in lines:
            Dd = st_["odr"][0]
            gs = [(math.cos(-psi0) * Dd, math.sin(-psi0) * Dd, st_["odr"][2])
                  for psi0 in (math.pi / 2, -math.pi / 2)]
            rr, fit, inl = best_icp(st_["s1"][t], st_["s0"][t], gs)
            (x, y, th) = rr
            if inl < OK_INL or fit > OK_FIT:
                print("  /%s 직진 맞추기 실패 (맞음 %.0f%%)" % (t, inl * 100))
                continue
            cc, ss = math.cos(th), math.sin(th)
            ax = x - ((1 - cc) * c[0] + ss * c[1])
            ay = y - (-ss * c[0] + (1 - cc) * c[1])
            sgn = 1.0 if Dd > 0 else -1.0
            psis.append(-math.atan2(sgn * ay, sgn * ax))
            print("  /%s 직진 %+.0fmm: 라이다 좌표 이동 (%+.1f, %+.1f)mm 맞음 %.0f%% -> 방향 %+.2f도"
                  % (t, Dd * 1000, x * 1000, y * 1000, inl * 100, D(psis[-1])))
        if not psis:
            continue
        psi = math.atan2(sum(math.sin(v) for v in psis), sum(math.cos(v) for v in psis))
        cc, ss = math.cos(psi), math.sin(psi)
        p = (-(cc * c[0] - ss * c[1]), -(ss * c[0] + cc * c[1]))
        result[t] = {"x": p[0], "y": p[1], "yaw": psi, "spin_resid_mm": resid,
                     "yaw_each_deg": [D(v) for v in psis]}
        print("== /%s: 위치 (앞 %+.1f, 좌 %+.1f) mm, 방향 %+.2f도 (%s)"
              % (t, p[0] * 1000, p[1] * 1000, D(psi),
                 "앞 왼쪽" if p[0] > 0 and p[1] > 0 else "뒤 오른쪽" if p[0] < 0 and p[1] < 0 else "?"))
    if len(result) == 2 and steps:
        s = steps[-1]["s1"]
        pts = {}
        for t in TOPICS:
            r = result[t]
            c_, s_ = math.cos(r["yaw"]), math.sin(r["yaw"])
            q = s[t]
            pts[t] = np.stack([r["x"] + c_ * q[:, 0] - s_ * q[:, 1], r["y"] + s_ * q[:, 0] + c_ * q[:, 1]], 1)
        rel, fit, inl = scan_icp.icp(near(pts["scan2"])[::2], near(pts["scan"]), (0, 0, 0),
                                     dst_n=scan_icp.normals(near(pts["scan"])))
        print("== 확인: /scan2 를 /scan 에 맞추려면 (%+.1f, %+.1f)mm %+.2f도 더 옮겨야 함 (맞음 %.0f%%) — 0 에 가까울수록 좋다"
              % (rel[0] * 1000, rel[1] * 1000, D(rel[2]), inl * 100))
        result["_check"] = {"dx": rel[0], "dy": rel[1], "dyaw": rel[2], "inlier": inl}
    out = os.path.expanduser(time.strftime("~/projects/UND_AMR/logs/lidar_calib_%m%d_%H%M.json"))
    with open(out, "w") as f:
        json.dump(result, f, indent=1)
    print("저장:", out)


if __name__ == "__main__":
    main()
