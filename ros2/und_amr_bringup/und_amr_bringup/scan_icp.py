"""라이다 스캔끼리 맞추기 (2D 점-점 ICP) — 지점 반복정밀도 보정용 (2026-10-14).

지도도 SLAM 도 안 쓰고, 같은 자리에서 찍은 두 스캔이 얼마나 어긋났는지로
'기준 자세에서 본 지금 로봇 자리' 를 구한다.

⚠ 왜 필요한가: 10/14 무한 왕복 1시간을 라이다로 따로 재 보니 실제로 선 자리가
  처음 자리에서 A 평균 15mm(최대 43), B 평균 26mm(최대 51) 벗어났다. 그동안
  SLAM 은 1~5mm 라고 봤다. SLAM 위치가 1시간에 약 4cm 한쪽으로 흘렀다
  (B 좌우 -9 -> -49mm). 도착 제어는 SLAM 기준으로는 맞게 섰다.

scipy 는 이 PC 에서 numpy 2 와 안 맞아 못 쓴다(2026-10-08). numpy 만 쓴다.
"""
import math

import numpy as np

RANGE_MIN = 0.12
RANGE_MAX = 8.0


def scan_points(msg, laser_xy_yaw, step=1):
    """LaserScan -> base_footprint 기준 점 (N, 2). laser_xy_yaw 는 base <- laser."""
    r = np.asarray(msg.ranges, dtype=float)[::step]
    a = msg.angle_min + np.arange(0, len(msg.ranges), step) * msg.angle_increment
    ok = np.isfinite(r) & (r > RANGE_MIN) & (r < RANGE_MAX)
    r, a = r[ok], a[ok]
    lx, ly, lyaw = laser_xy_yaw
    return np.stack([lx + r * np.cos(a + lyaw), ly + r * np.sin(a + lyaw)], 1)


def _kabsch(a, b):  # (예전 점-점 방식, 남겨 둠)
    """b ≈ R a + t 가 되는 R, t."""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    return R, cb - R @ ca


def normals(dst, radius=0.06, min_n=4):
    """기준 점마다 주변 점들로 벽 방향을 구해 법선을 낸다. 못 구하면 (0, 0).

    ⚠ 점-점 ICP 는 벽 위 점 간격만큼 '벽을 따라' 어긋난 자리에 멈춘다. 점을
      4개 중 1개로 줄인 기준(3m 앞 간격 3cm)으로 합성 시험에서 10mm 틀렸다
      (2026-10-14). 벽 면(선)에 맞추면 벽을 따라 미끄러지는 몫이 없다.
    """
    d2 = ((dst[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
    out = np.zeros_like(dst)
    for i in range(len(dst)):
        nb = dst[d2[i] < radius * radius]
        if len(nb) < min_n:
            continue
        c = nb - nb.mean(0)
        w, v = np.linalg.eigh(c.T @ c)
        if w[1] <= 0 or w[0] > 0.1 * w[1]:
            continue                        # 선처럼 늘어서지 않았다 (모서리·잡음)
        out[i] = v[:, 0]                    # 작은 고유값 방향 = 법선
    return out


def icp(src, dst, guess=(0.0, 0.0, 0.0), iters=40, dst_n=None):
    """src(지금 로봇 기준 점)를 dst(기준 자세 기준 점)에 맞춘다 (점-선).

    guess : 기준 자세에서 본 지금 로봇 자리 추정 (SLAM 차이를 넣는다)
    dst_n : normals(dst) — 기준마다 한 번만 구해 두면 빠르다
    반환  : (x, y, yaw) 기준 자세에서 본 지금 로봇 자리, 잔차 중앙값 [m], 맞은 비율
    """
    if dst_n is None:
        dst_n = normals(dst)
    has_n = np.abs(dst_n).sum(1) > 0
    c, s = math.cos(guess[2]), math.sin(guess[2])
    R = np.array([[c, -s], [s, c]])
    t = np.array([guess[0], guess[1]], dtype=float)
    thr = 0.20
    for _ in range(iters):
        p = src @ R.T + t
        d2 = ((p[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(p)), j])
        m = (dist < thr) & has_n[j]
        if m.sum() < 30:
            break
        pm, q, nn = p[m], dst[j[m]], dst_n[j[m]]
        r = ((pm - q) * nn).sum(1)
        A = np.stack([nn[:, 0] * -pm[:, 1] + nn[:, 1] * pm[:, 0], nn[:, 0], nn[:, 1]], 1)
        x, *_ = np.linalg.lstsq(A, -r, rcond=None)
        da, dx, dy = float(x[0]), float(x[1]), float(x[2])
        cd, sd = math.cos(da), math.sin(da)
        dR = np.array([[cd, -sd], [sd, cd]])
        R, t = dR @ R, dR @ t + np.array([dx, dy])
        thr = max(0.03, min(thr, 3.0 * float(np.median(dist[m]))))
        if abs(da) < 1e-6 and math.hypot(dx, dy) < 1e-6:
            break
    p = src @ R.T + t
    d = np.sqrt(((p[:, None, :] - dst[None, :, :]) ** 2).sum(-1).min(1))
    return ((float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])),
            float(np.median(d)), float(np.mean(d < 0.03)))


# ---------------------------------------------------------------- 자세 계산
def compose(a, b):
    """a 자세에서 b 만큼 간 자세 (a ⊕ b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    yaw = a[2] + b[2]
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(yaw), math.cos(yaw)))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """a 자세에서 본 b (a⁻¹ ⊕ b)."""
    return compose(inverse(a), b)


def corrected_goal(slam_now, real_in_ref, goal_in_ref):
    """지금 SLAM 자세, 라이다로 잰 '기준 자세에서 본 지금 자리', 기준 자세에서 본
    목표점으로, 실제로 서야 할 자리를 지금의 SLAM(지도) 좌표로 낸다."""
    return compose(compose(slam_now, inverse(real_in_ref)), goal_in_ref)
