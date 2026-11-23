#!/usr/bin/env python3
"""새 대차(렉) 규격 재기 — 라이다로 다리 네 개를 찾아 가로 x 세로를 잰다.

carts.yaml 에 들어가는 width/depth 는 줄자로 잰 외곽 치수가 아니라
'다리 중심 간 거리' 다. 라이다가 보는 값과 같은 기준으로 재야 인식이
맞아떨어지므로, 등록 전에 이 도구로 재서 그 값을 그대로 넣는다.

등록된 규격을 쓰지 않는다는 점이 cart_detector 와 다르다. 후보 네 개를
고르는 조합을 전부 훑어 '가장 직사각형에 가까운' 조합을 찾는다.
따라서 처음 보는 대차도 잴 수 있다.

    쓰는 법
      1) 대차를 로봇 앞 1~2m 에, 다른 물건 없이 홀로 세운다
      2) python3 tools/cart_measure.py
      3) 마지막에 찍히는 yaml 줄을 config/carts.yaml 에 붙인다

    옵션
      --frames 60      몇 프레임 모아 중앙값을 낼지 (기본 60, 약 12초)
      --range 3.0      이 거리 안에서만 찾는다 [m]
      --name LG3       yaml 에 찍을 이름
      --front          로봇 정면(+x) 쪽 조합만 본다
"""
import argparse
import itertools
import math
import os
import statistics
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cart_match import cluster_scan, leg_candidates          # noqa: E402


def best_rectangles(pts, rel_tol=0.08, side_min=0.20, side_max=1.60,
                    front_only=False):
    """후보 중 네 개로 만들 수 있는 직사각형을 규격 없이 찾는다.

    등록표와 대조하지 않으므로 '얼마나 직사각형다운가' 만 본다.
    변 길이에 비례한 상대 오차를 쓴다 — 큰 대차일수록 절대 오차는 커진다.
    """
    out = []
    for idx in itertools.combinations(range(len(pts)), 4):
        p = [pts[i] for i in idx]
        if front_only and min(q[0] for q in p) <= 0.0:
            continue
        for order in itertools.permutations(range(4)):
            if order[0] != min(order):          # 회전 중복 제거
                continue
            q = [p[i] for i in order]
            s = [math.hypot(q[i][0] - q[(i + 1) % 4][0],
                            q[i][1] - q[(i + 1) % 4][1]) for i in range(4)]
            d1 = math.hypot(q[0][0] - q[2][0], q[0][1] - q[2][1])
            d2 = math.hypot(q[1][0] - q[3][0], q[1][1] - q[3][1])
            a, b = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            if not (side_min <= a <= side_max and side_min <= b <= side_max):
                continue
            scale = (a + b) / 2
            # 마주보는 변끼리 같고, 대각선 두 개가 같아야 직사각형
            e = max(abs(s[0] - s[2]), abs(s[1] - s[3]),
                    abs(d1 - d2) / 1.5) / scale
            if e > rel_tol:
                continue
            cx = sum(t[0] for t in p) / 4
            cy = sum(t[1] for t in p) / 4
            P = np.array(p) - np.array([cx, cy])
            _, _, vt = np.linalg.svd(P, full_matrices=False)
            out.append({"a": max(a, b), "b": min(a, b), "err": e,
                        "x": cx, "y": cy,
                        "yaw": math.atan2(vt[0][1], vt[0][0]),
                        "dist": math.hypot(cx, cy), "idx": idx})
            break                               # 한 조합당 하나면 충분
    # 가까운 것 우선, 같으면 더 직사각형다운 것 우선
    out.sort(key=lambda r: (round(r["dist"], 1), r["err"]))
    return out


class Measure(Node):

    def __init__(self, args):
        super().__init__("cart_measure")
        self.args = args
        self.acc = []
        self.frames = 0
        self.last_cands = 0
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        print("라이다 기다리는 중... (대차를 로봇 앞 1~2m 에 홀로 두세요)")

    def _on_scan(self, m):
        if self.frames >= self.args.frames:
            return
        x, y, groups = cluster_scan(m.ranges, m.angle_min, m.angle_increment,
                                    rmax=self.args.range, gap=0.05)
        cands = leg_candidates(x, y, groups, wmin=0.010, wmax=0.100, min_pts=2)
        self.last_cands = len(cands)
        self.frames += 1
        if len(cands) < 4:
            print("  [%2d/%d] 다리 후보 %d개 — 4개 미만"
                  % (self.frames, self.args.frames, len(cands)))
            return
        pts = [(c["x"], c["y"]) for c in cands]
        rects = best_rectangles(pts, front_only=self.args.front)
        if not rects:
            print("  [%2d/%d] 후보 %d개 — 직사각형 없음"
                  % (self.frames, self.args.frames, len(cands)))
            return
        r = rects[0]
        self.acc.append(r)
        print("  [%2d/%d] %.0f x %.0f mm   앞 %.2fm 옆 %+.2fm %+.0f°  "
              "(후보 %d개, 직각오차 %.1f%%)"
              % (self.frames, self.args.frames, r["a"] * 1000, r["b"] * 1000,
                 r["x"], r["y"], math.degrees(r["yaw"]), len(cands),
                 r["err"] * 100))

    def done(self):
        return self.frames >= self.args.frames


def report(acc, name):
    if len(acc) < 3:
        print("\n측정 실패 — 직사각형이 잡힌 프레임이 %d개뿐입니다." % len(acc))
        print("대차를 3m 안, 정면 가까이 두고 주변 물건을 치운 뒤 다시 해 보세요.")
        return 1
    A = sorted(r["a"] for r in acc)
    B = sorted(r["b"] for r in acc)
    ma, mb = statistics.median(A), statistics.median(B)
    sa = statistics.pstdev(A) if len(A) > 1 else 0.0
    sb = statistics.pstdev(B) if len(B) > 1 else 0.0
    # 흔들림이 큰 쪽에 맞춰 허용 오차를 잡되, 최소 40mm 는 준다.
    # (다리 하나가 스캔마다 붙었다 떨어지면 변 하나가 통째로 흔들린다)
    tol = max(0.040, round(4 * max(sa, sb), 3))

    print("\n" + "=" * 58)
    print("측정 결과  (%d 프레임 중 %d 프레임 유효)" % (acc[0].get("_n", len(acc)), len(acc)))
    print("=" * 58)
    print("  가로(width)   %6.1f mm   흔들림 ±%.1f mm   범위 %.0f~%.0f"
          % (ma * 1000, sa * 1000, A[0] * 1000, A[-1] * 1000))
    print("  세로(depth)   %6.1f mm   흔들림 ±%.1f mm   범위 %.0f~%.0f"
          % (mb * 1000, sb * 1000, B[0] * 1000, B[-1] * 1000))
    if max(sa, sb) > 0.020:
        print("\n  ⚠ 흔들림이 20mm 를 넘습니다. 다리 하나가 스캔마다 끊겼다")
        print("    붙었다 할 수 있습니다. 대차를 조금 옮겨 다시 재 보세요.")
    print("\nconfig/carts.yaml 에 붙일 줄 — carts 목록에 \"%s\" 도 추가하세요:\n" % name)
    print("    %s:" % name)
    print("      width: %.3f" % ma)
    print("      depth: %.3f" % mb)
    print("      tol: %.3f" % tol)
    print("      note: \"cart_measure 실측\"")
    print()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--range", type=float, default=3.0)
    ap.add_argument("--name", default="NEW")
    ap.add_argument("--front", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Measure(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        print("\n(중단)")
    rc = report(node.acc, args.name)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
