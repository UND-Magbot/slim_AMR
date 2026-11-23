"""2D 라이다 스캔 정합 — 주행 캘리브레이션용.

두 스캔 사이의 (dx, dy, dtheta) 를 구한다. 벽 한 방향만 보는 대신
360도 형상을 통째로 맞추므로 평평하지 않은 벽이나 잡음에 강하다.

  match(A, B, meta, init=...)   직진/원호 구간용 (ICP)
  angular_shift(A, B, inc, ...) 회전 전용 (거리 프로파일 원형상관)
  match_rotation(...)           회전 구간용 (상관 -> ICP 보정)

ICP 는 큰 이동에서 지역최소에 빠지므로 반드시 엔코더 기반 초기값을 준다.
초기값을 +-15% 흔들어도 같은 값으로 수렴하는지 확인하면 신뢰할 수 있다.

2026-08-25 조립 후 캘리브레이션에서 작성. 직진 정합 정확도는
시뮬레이션 검증 기준 2m 이동에서 1~2mm.
"""
import math
import numpy as np


def scan_to_xy(ranges, angle_min, angle_increment, rmin=0.05, rmax=12.0):
    r = np.asarray(ranges, dtype=np.float64)
    ang = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    r, ang = r[ok], ang[ok]
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)


def _nearest(src, dst, chunk=512):
    idx = np.empty(len(src), dtype=np.int64)
    dist = np.empty(len(src), dtype=np.float64)
    for i in range(0, len(src), chunk):
        s = src[i:i + chunk]
        d = np.linalg.norm(s[:, None, :] - dst[None, :, :], axis=2)
        j = np.argmin(d, axis=1)
        idx[i:i + chunk] = j
        dist[i:i + chunk] = d[np.arange(len(s)), j]
    return idx, dist


def icp(src, dst, init=(0.0, 0.0, 0.0), iters=60, trim=0.75, reject=1.5):
    """src 를 dst 에 맞추는 강체변환 (tx, ty, theta)."""
    tx, ty, th = init
    P = src.copy()
    hist = []
    keep = None
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        Pt = P @ R.T + np.array([tx, ty])
        j, d = _nearest(Pt, dst)
        keep = d < reject
        if keep.sum() < 20:
            break
        thr = np.quantile(d[keep], trim)
        keep = keep & (d <= max(thr, 0.02))
        if keep.sum() < 20:
            break
        A, B = Pt[keep], dst[j[keep]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        D = np.diag([1.0, np.linalg.det(Vt.T @ U.T)])
        Rk = Vt.T @ D @ U.T
        tk = cb - Rk @ ca
        dth = math.atan2(Rk[1, 0], Rk[0, 0])
        newR = Rk @ R
        th = math.atan2(newR[1, 0], newR[0, 0])
        tx, ty = Rk @ np.array([tx, ty]) + tk
        hist.append(float(np.sqrt((np.linalg.norm(A - B, axis=1) ** 2).mean())))
        if it > 5 and abs(dth) < 1e-6 and np.linalg.norm(tk) < 1e-6:
            break
    return (tx, ty, th), {"rms": hist[-1] if hist else float("nan"),
                          "iters": len(hist),
                          "pairs": int(keep.sum()) if keep is not None else 0}


def match(scan_a, scan_b, meta, init=(0.0, 0.0, 0.0), rmax=12.0):
    """스캔 A(이전)와 B(이후)로부터 로봇이 움직인 (dx, dy, dtheta).

    라이다 프레임 기준. 전진하면 dx 가 양수."""
    A = scan_to_xy(scan_a, meta.angle_min, meta.angle_increment, rmax=rmax)
    B = scan_to_xy(scan_b, meta.angle_min, meta.angle_increment, rmax=rmax)
    (tx, ty, th), info = icp(B, A, init=init)
    info["n_a"], info["n_b"] = len(A), len(B)
    return (tx, ty, th), info


def angular_shift(a_ranges, b_ranges, angle_increment, guess_rad=0.0,
                  search_deg=45.0, rmax=15.0):
    """회전각을 거리 프로파일 원형상관으로 구한다 (지역최소 없음).

    ⚠ 라이다가 회전중심에서 떨어져 있으면 편향이 생긴다(0.15m 오프셋,
      20도 회전에서 +0.85도). match_rotation 처럼 ICP 로 보정해서 쓸 것."""
    A = np.asarray(a_ranges, dtype=np.float64)
    B = np.asarray(b_ranges, dtype=np.float64)
    n = len(A)
    va = np.isfinite(A) & (A > 0) & (A < rmax)
    vb = np.isfinite(B) & (B > 0) & (B < rmax)
    Af, Bf = np.where(va, A, 0.0), np.where(vb, B, 0.0)
    g = int(round(guess_rad / angle_increment))
    half = int(round(math.radians(search_deg) / angle_increment))
    shifts = list(range(g - half, g + half + 1))
    costs = []
    for s in shifts:
        m = va & np.roll(vb, s)
        if m.sum() < n * 0.15:
            costs.append(np.inf)
            continue
        d = Af[m] - np.roll(Bf, s)[m]
        costs.append(float(np.sqrt((d * d).mean())))
    costs = np.array(costs)
    k = int(np.argmin(costs))
    frac = 0.0
    if 0 < k < len(costs) - 1 and np.isfinite(costs[k - 1:k + 2]).all():
        y0, y1, y2 = costs[k - 1], costs[k], costs[k + 1]
        den = y0 - 2 * y1 + y2
        if abs(den) > 1e-12:
            frac = max(-1.0, min(1.0, 0.5 * (y0 - y2) / den))
    lo, hi = max(0, k - 10), min(len(costs) - 1, k + 10)
    sharp = float(min(costs[lo], costs[hi]) / costs[k]) if costs[k] > 0 else float("nan")
    return (shifts[k] + frac) * angle_increment, {
        "rms": float(costs[k]), "sharpness": sharp}


def match_rotation(a_ranges, b_ranges, meta, lever=0.0, guess_rad=0.0,
                   search_deg=40.0, rmax=12.0, reject=0.5):
    """제자리 회전 구간 전용: 각도상관으로 초기값 -> ICP 로 보정."""
    th0, ainfo = angular_shift(a_ranges, b_ranges, meta.angle_increment,
                               guess_rad=guess_rad, search_deg=search_deg, rmax=rmax)
    tx0 = lever * (math.cos(th0) - 1.0)
    ty0 = lever * math.sin(th0)
    A = scan_to_xy(a_ranges, meta.angle_min, meta.angle_increment, rmax=rmax)
    B = scan_to_xy(b_ranges, meta.angle_min, meta.angle_increment, rmax=rmax)
    (tx, ty, th), info = icp(B, A, init=(tx0, ty0, th0), reject=reject)
    info["theta0_deg"] = math.degrees(th0)
    info["sharpness"] = ainfo["sharpness"]
    return (tx, ty, th), info
