"""스캔 합치기 — 라이다 바로 뒤쪽(차체를 가로지르는 방향) 가까운 점 버리기 (2026-10-30)."""
import math

import numpy as np

from und_amr_bringup.scan_merger import ScanMerger


def make():
    n = ScanMerger.__new__(ScanMerger)
    n.out_frame = "laser"
    n.rmin, n.rmax = 0.05, 40.0
    n.mount_r = 0.08
    n.blind_half, n.blind_r = math.radians(15.0), 0.60
    n.sh_min, n.sh_win, n.sh_rmax = math.radians(5.0), 3, 5.0
    n.box = (-0.43675 - 0.045, 0.43675 + 0.045, 0.1644 + 0.045)
    n.ext = {"laser_front": (0.407, 0.145, math.radians(91.4))}   # 앞 왼쪽, 왼쪽을 본다
    n._warned = set()
    return n


def scan(points):
    """{센서각(도): 거리} -> LaserScan 비슷한 것 (0.15도 2400칸, 나머지 inf)."""
    m = type("S", (), {})()
    m.header = type("H", (), {"frame_id": "laser_front"})()
    m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / 2400
    m.range_min, m.range_max = 0.05, 40.0
    r = np.full(2400, np.inf)
    for deg, d in points.items():
        k = int(round((math.radians(deg) + math.pi) / m.angle_increment))
        r[k - 4:k + 5] = d                    # 이웃이 같은 거리라 그림자 거르기에 안 걸림
    m.ranges = r.tolist()
    return m


def xy(n, deg, d):
    lx, ly, lyaw = n.ext["laser_front"]
    a = math.radians(deg) + lyaw
    return lx + d * math.cos(a), ly + d * math.sin(a)


def near(P, q, tol=0.02):
    return len(P) and (np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) < tol).any()


def test_phantom_behind_lidar_is_dropped():
    # 2026-10-27 (171도, 0.34m) · 2026-10-30 (170도, 0.40m) 로봇 오른쪽 앞 유령 점
    n = make()
    for deg, d in ((171.0, 0.34), (170.0, 0.40), (-175.0, 0.5)):
        P = n.points(scan({deg: d}))
        assert not near(P, xy(n, deg, d)), (deg, d)


def test_far_behind_and_other_directions_kept():
    n = make()
    for deg, d in ((170.0, 1.6), (180.0, 4.5), (150.0, 0.7), (0.0, 0.4), (45.0, 0.5), (-90.0, 0.5)):
        q = xy(n, deg, d)
        P = n.points(scan({deg: d}))
        assert near(P, q), (deg, d)
