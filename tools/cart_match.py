"""대차 인식 — 다리 후보 중 4개를 골라 규격에 맞는 사각형을 찾는다.

후보를 전부 다리로 가정하면, 접지선이나 잡동사니가 하나만 끼어도 크기가
안 맞아 인식에 실패한다. 그래서 후보 중 4개를 고르는 조합을 모두 훑어
등록된 대차 규격에 가장 잘 맞는 조합을 찾는다.

다리 모양(원형/사각/기타)과 굵기는 보지 않는다. 캐스터는 바퀴가 향한
방향에 따라 폭이 두 배까지 달라져(실측 35~72mm) 판별 기준이 못 된다.
쓰는 것은 '네 다리가 이루는 가로 x 세로' 뿐이다.
"""
import itertools
import math

import numpy as np


def cluster_scan(ranges, angle_min, angle_inc, rmax=3.0, gap=0.05,
                 rmin=0.05):
    """스캔을 덩어리로 끊는다. 인접 점 간격이 gap 보다 벌어지면 다른 물체."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    x, y, a = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), ang[ok]
    o = np.argsort(a)
    x, y = x[o], y[o]
    out, cur = [], [0]
    for i in range(1, len(x)):
        if math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) > gap:
            out.append(cur)
            cur = []
        cur.append(i)
    out.append(cur)
    return x, y, out


def leg_candidates(x, y, groups, wmin=0.010, wmax=0.100, min_pts=2,
                   radius=0.020):
    """다리 후보 = 작고 고립된 덩어리. 모양은 보지 않는다.

    라이다는 다리의 '로봇 쪽 표면' 만 보므로 덩어리 중심은 실제 다리
    중심보다 앞으로 나와 있다. 반지름만큼 뒤로 밀어 보정한다.

    미는 양은 '고정 반지름' 을 쓴다. 예전에는 그때그때 보이는 덩어리 폭의
    절반을 썼는데, 보이는 폭이 캐스터가 향한 방향에 따라 두 배까지 달라져
    (실측 15~51mm) 같은 대차도 보는 각도마다 크기가 달라졌다.
    2026-08-28 실측 비교 — 짧은 변 흔들림 폭÷2 ±2.1mm / 고정 20mm ±0.6mm.

    반지름의 절대값이 정확할 필요는 없다. 모든 대차를 같은 값으로 재고
    같은 값으로 등록하므로 치우침은 상쇄된다. 중요한 것은 '시점이 바뀌어도
    같은 값이 나오는 것' 이다. radius=None 이면 예전처럼 폭의 절반을 쓴다.
    """
    out = []
    for g in groups:
        if len(g) < min_pts:
            continue
        w = math.hypot(x[g][-1] - x[g][0], y[g][-1] - y[g][0])
        if not (wmin <= w <= wmax):
            continue
        cx, cy = x[g].mean(), y[g].mean()
        d = math.hypot(cx, cy)
        if d < 1e-6:
            continue
        # 표면 -> 중심 보정 (라이다 반대쪽으로 반지름만큼)
        r = (w / 2.0) if radius is None else radius
        k = (d + r) / d
        out.append({"x": cx * k, "y": cy * k, "raw": (cx, cy),
                    "w": w, "n": len(g), "r": r})
    return out


def _quad_yaw(q, a, b, square_tol=0.02):
    """사각형의 방향. 네 점의 주성분이 아니라 '변' 에서 구한다.

    주성분(SVD)으로 구하면 정사각형에서는 긴 축이라는 게 없어 방향이
    정해지지 않는다. S300(765x765) 을 넣고 재 보니 라이다 잡음 1.6mm
    만으로 방향이 -177~180 도를 오갔다(표준편차 101도). 변에서 구하면
    정사각형도 90도 주기 안에서 흔들림 없이 나온다.

    네 변은 90도씩 돌아가 있으므로 4배각으로 평균 내면 네 변을 모두
    쓰면서 대표 방향 하나가 나온다. 직사각형이면 거기서 긴 변 쪽으로
    확정한다(기존 주성분 방식과 같은 값). 정사각형은 90도 주기가
    방향의 전부다 — 어느 변이 '앞' 인지는 라이다만으로 알 수 없다.
    """
    ang = [math.atan2(q[(i + 1) % 4][1] - q[i][1],
                      q[(i + 1) % 4][0] - q[i][0]) for i in range(4)]
    base = math.atan2(sum(math.sin(4 * t) for t in ang),
                      sum(math.cos(4 * t) for t in ang)) / 4.0
    if abs(a - b) <= square_tol:
        return base                            # 이미 -45~45도
    lo = ang[0] if a >= b else ang[1]          # 긴 변 (a = 변 0,2 의 길이)
    best = None
    for k in range(4):
        c = base + k * math.pi / 2
        # 축 방향이므로 180도 주기로 견준다
        d = abs(math.atan2(math.sin(2 * (c - lo)), math.cos(2 * (c - lo))))
        if best is None or d < best[0]:
            best = (d, c)
    # 다리 네 개로는 대차의 앞뒤를 가릴 수 없다. 방향은 180도 주기의
    # '축' 이므로 -90~90도로 접어 값을 하나로 고정한다. 접지 않으면
    # 같은 자세인데도 110도와 -70도를 오간다.
    return (best[1] + math.pi / 2) % math.pi - math.pi / 2


def _rect_score(p, spec_a, spec_b, tol):
    """네 점이 spec_a x spec_b 직사각형인가. 0~1 점수, 안 맞으면 None."""
    best = None
    for order in itertools.permutations(range(4)):
        if order[0] != min(order):        # 회전 중복 제거
            continue
        q = [p[i] for i in order]
        s = [math.hypot(q[i][0] - q[(i + 1) % 4][0],
                        q[i][1] - q[(i + 1) % 4][1]) for i in range(4)]
        d1 = math.hypot(q[0][0] - q[2][0], q[0][1] - q[2][1])
        d2 = math.hypot(q[1][0] - q[3][0], q[1][1] - q[3][1])
        # 마주보는 변끼리 같아야 하고, 대각선 두 개가 같아야 직사각형
        if abs(s[0] - s[2]) > tol or abs(s[1] - s[3]) > tol:
            continue
        if abs(d1 - d2) > tol * 1.5:
            continue
        a, b = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
        for ea, eb in ((spec_a, spec_b), (spec_b, spec_a)):
            err = math.hypot(a - ea, b - eb)
            if err > tol * 2:
                continue
            sc = max(0.0, 1.0 - err / (tol * 2))
            if best is None or sc > best[0]:
                best = (sc, a, b, err, order)
    return best


def match_carts(cands, specs, tol=0.06, min_score=0.25):
    """후보 중 4개를 골라 등록된 대차와 맞춰 본다.

    후보가 많아도 조합 수는 적다(10개면 210가지). 접지선 같은 가짜가
    끼어도 진짜 네 개가 여전히 가장 잘 맞는다.
    """
    pts = [(c["x"], c["y"]) for c in cands]
    raws = [c.get("raw", (c["x"], c["y"])) for c in cands]
    found = []
    for idx in itertools.combinations(range(len(pts)), 4):
        p = [pts[i] for i in idx]
        for name, sp in specs.items():
            r = _rect_score(p, sp["width"], sp["depth"], sp.get("tol", tol))
            if r is None or r[0] < min_score:
                continue
            cx = sum(q[0] for q in p) / 4
            cy = sum(q[1] for q in p) / 4
            # 사각형의 꼭짓점 차례. 조합 번호순으로 두면 다리를 이어 그릴 때
            # 대각선으로 엇갈린 나비 모양이 될 수 있어, 사각형 차례로 세운다.
            order = r[4]
            corner = [idx[i] for i in order]
            yaw = _quad_yaw([pts[i] for i in corner], r[1], r[2])
            found.append({"cart": name, "score": r[0], "a": r[1], "b": r[2],
                          "err": r[3], "x": cx, "y": cy, "yaw": yaw,
                          "legs": [pts[i] for i in corner],
                          # 카메라 투영용 표면 좌표 (보정 전)
                          "legs_raw": [raws[i] for i in corner],
                          "idx": idx})
    found.sort(key=lambda f: -f["score"])
    # 같은 다리를 공유하는 중복 결과는 하나만 남긴다
    kept = []
    for f in found:
        if any(len(set(f["idx"]) & set(k["idx"])) >= 2 for k in kept):
            continue
        kept.append(f)
    return kept


def best_rectangles(pts, rel_tol=0.08, side_min=0.20, side_max=1.60,
                    front_only=True):
    """등록표를 쓰지 않고, 후보 넷으로 만들 수 있는 직사각형을 찾는다.

    처음 보는 대차를 재려고 만든 것이다. match_carts 는 '등록된 규격에
    얼마나 맞는가' 를 보지만, 여기서는 '얼마나 직사각형다운가' 만 본다.

    오차는 변 길이에 비례한 상대값으로 본다 - 큰 대차일수록 절대 오차는
    당연히 커진다. front_only 는 로봇 정면 쪽만 본다. 뒤쪽 벽 모서리가
    더 그럴듯한 사각형으로 잡히는 일이 실제로 있었다.
    """
    out = []
    for idx in itertools.combinations(range(len(pts)), 4):
        p = [pts[i] for i in idx]
        if front_only and min(q[0] for q in p) <= 0.10:
            continue
        for order in itertools.permutations(range(4)):
            if order[0] != min(order):        # 회전 중복 제거
                continue
            q = [p[i] for i in order]
            sd = [math.hypot(q[i][0] - q[(i + 1) % 4][0],
                             q[i][1] - q[(i + 1) % 4][1]) for i in range(4)]
            d1 = math.hypot(q[0][0] - q[2][0], q[0][1] - q[2][1])
            d2 = math.hypot(q[1][0] - q[3][0], q[1][1] - q[3][1])
            a, b = (sd[0] + sd[2]) / 2, (sd[1] + sd[3]) / 2
            if not (side_min <= a <= side_max and side_min <= b <= side_max):
                continue
            scale = (a + b) / 2
            e = max(abs(sd[0] - sd[2]), abs(sd[1] - sd[3]),
                    abs(d1 - d2) / 1.5) / scale
            if e > rel_tol:
                continue
            cx = sum(t[0] for t in p) / 4
            cy = sum(t[1] for t in p) / 4
            out.append({"a": max(a, b), "b": min(a, b), "err": e,
                        "x": cx, "y": cy,
                        "yaw": _quad_yaw(q, a, b),
                        "dist": math.hypot(cx, cy), "idx": idx})
            break                             # 한 조합당 하나면 충분
    # 가까운 것 우선, 같으면 더 직사각형다운 것 우선
    out.sort(key=lambda r: (round(r["dist"], 1), r["err"]))
    return out
