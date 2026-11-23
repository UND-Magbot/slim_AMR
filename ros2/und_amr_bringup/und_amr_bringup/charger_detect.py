"""충전 스테이션 라이다 인식 (2026-11-11).

스테이션 앞면은 안으로 오목한 피라미드라 라이다 높이(201.5mm)에서 자르면
평평한 앞면 사이에 V 홈이 보인다. 2026-11-11 실측 (로봇 뒤 683mm, 61장 평균):
  홈 입구 폭 273mm, 깊이 83mm, 두 변이 앞면에서 31.5 / 30.9도 (꼭지각 118도),
  변 직선 잔차 0.6~0.8mm, 양옆 평평한 앞면 약 4cm.

detect() 는 스캔 점(센서 = base 와 같은 xy 의 'laser' 프레임)에서 이 홈을 찾아
앞면 중심 (홈 입구 가운데)과 바깥 방향(홈에서 열린 쪽)을 낸다.
- 꼭짓점 후보마다 양쪽 VEE_REACH 안 점에 직선을 맞춘다.
- 꼭지각이 VEE_ANGLE ± ANGLE_TOL, 두 변 잔차 RESID_MAX 안, 각 변 점 MIN_PTS 이상.
- 홈은 센서 쪽으로 열려 있어야 한다(오목).
- 변 직선이 홈 끝을 지나서도 이어지면(긴 벽 모서리) 버린다 — 스테이션은 홈 끝에서
  평평한 앞면으로 꺾인다.
"""
import math

import numpy as np

VEE_ANGLE = 118.0          # 꼭지각 [도]
ANGLE_TOL = 12.0
VEE_DEPTH = 0.083          # 꼭짓점에서 앞면까지 [m]
VEE_HALF = 0.137           # 홈 입구 반폭 [m]
VEE_REACH = 0.12           # 꼭짓점에서 이 거리 안 점으로 변을 맞춘다
RESID_MAX = 0.006
MIN_PTS = 8
MAX_RANGE = 3.0
GAP = 0.05                 # 이웃 점이 이보다 멀면 다른 물체


def _line(P):
    """점들의 직선: (중심, 단위 방향, RMS 잔차)."""
    c = P.mean(0)
    _, s, vt = np.linalg.svd(P - c, full_matrices=False)
    d = vt[0]
    res = math.sqrt(max(s[-1] ** 2, 0.0) / len(P)) if len(s) > 1 else 0.0
    return c, d, res


def detect(x, y, sensor=(0.0, 0.0)):
    """x, y: 각도 순서의 점 (같은 길이, inf/nan 은 빼고 넣어도 됨).
    찾으면 dict(x, y, yaw, apex, angle, depth, resid, score), 못 찾으면 None.
    (x, y) = 앞면 중심, yaw = 스테이션 바깥 방향 (홈이 열린 쪽, 로봇 쪽)."""
    P = np.c_[np.asarray(x, float), np.asarray(y, float)]
    ok = np.isfinite(P).all(1) & (np.hypot(P[:, 0] - sensor[0], P[:, 1] - sensor[1]) < MAX_RANGE)
    P = P[ok]
    if len(P) < 2 * MIN_PTS:
        return None
    S = np.asarray(sensor, float)
    cut = np.r_[0, np.nonzero(np.hypot(*np.diff(P, axis=0).T) > GAP)[0] + 1, len(P)]
    best = None
    for s0, s1 in zip(cut[:-1], cut[1:]):
        Q = P[s0:s1]
        if len(Q) < 2 * MIN_PTS or np.hypot(*(Q[-1] - Q[0])) < 0.2:
            continue
        for k in range(MIN_PTS, len(Q) - MIN_PTS):
            A = Q[k]
            dist = np.hypot(Q[:, 0] - A[0], Q[:, 1] - A[1])
            Lp, Rp = Q[:k + 1][dist[:k + 1] < VEE_REACH], Q[k:][dist[k:] < VEE_REACH]
            if len(Lp) < MIN_PTS or len(Rp) < MIN_PTS:
                continue
            cL, dL, eL = _line(Lp)
            cR, dR, eR = _line(Rp)
            if max(eL, eR) > RESID_MAX:
                continue
            # 꼭짓점에서 각 변으로 나가는 방향
            uL = dL if np.dot(cL - A, dL) > 0 else -dL
            uR = dR if np.dot(cR - A, dR) > 0 else -dR
            ang = math.degrees(math.acos(float(np.clip(np.dot(uL, uR), -1, 1))))
            if abs(ang - VEE_ANGLE) > ANGLE_TOL:
                continue
            ax = uL + uR
            ax /= np.linalg.norm(ax)
            if np.dot(S - A, ax) <= 0:            # 센서 쪽으로 열린 홈만
                continue
            # 두 변 직선의 교점을 꼭짓점으로
            M = np.c_[uL, -uR]
            if abs(np.linalg.det(M)) < 1e-6:
                continue
            t = np.linalg.solve(M, (cR - cL))
            apex = cL + uL * t[0]
            # 홈 끝을 지나서도 변 직선이 이어지면 긴 벽 모서리다
            longer = 0
            for u, c in ((uL, cL), (uR, cR)):
                far = Q[(np.hypot(Q[:, 0] - apex[0], Q[:, 1] - apex[1]) > VEE_REACH + 0.06)
                        & (np.hypot(Q[:, 0] - apex[0], Q[:, 1] - apex[1]) < VEE_REACH + 0.15)]
                if len(far):
                    n = np.array([-u[1], u[0]])
                    if np.median(np.abs((far - c) @ n)) < 0.01:
                        longer += 1
            if longer:
                continue
            score = (max(eL, eR) / RESID_MAX + abs(ang - VEE_ANGLE) / ANGLE_TOL)
            if best is None or score < best["score"]:
                face = apex + ax * VEE_DEPTH
                best = {"x": float(face[0]), "y": float(face[1]),
                        "yaw": math.atan2(ax[1], ax[0]),
                        "apex": (float(apex[0]), float(apex[1])),
                        "angle": ang, "resid": max(eL, eR), "score": score,
                        "n": len(Lp) + len(Rp)}
    return best


def detect_scan(ranges, angle_min, angle_inc, sensor=(0.0, 0.0, 0.0)):
    """LaserScan 값으로. sensor = 스캔 프레임의 base 기준 (x, y, yaw). 결과도 base 기준."""
    r = np.asarray(ranges, float)
    a = angle_min + np.arange(len(r)) * angle_inc + sensor[2]
    ok = np.isfinite(r) & (r > 0.05)
    x = sensor[0] + r[ok] * np.cos(a[ok])
    y = sensor[1] + r[ok] * np.sin(a[ok])
    # ⚠ 360도 스캔은 -180도에서 시작해 +180도에서 끝난다. 스테이션이 로봇 바로 뒤에
    #   있으면 V 홈이 배열 처음과 끝으로 갈라져 못 찾았다(2026-11-11 첫 시험).
    #   이웃 점 사이가 가장 먼 곳(끝-처음 이음 포함)에서 시작하게 돌린다.
    if len(x) > 2:
        gx, gy = np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]])
        k = int(np.argmax(np.hypot(gx, gy))) + 1
        x, y = np.roll(x, -k), np.roll(y, -k)
    return detect(x, y, sensor[:2])
