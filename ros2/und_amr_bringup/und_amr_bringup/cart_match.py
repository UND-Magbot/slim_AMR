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
                   radius=0.020, end_seg=None):
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

    end_seg 를 주면, 다리보다 넓은 덩어리의 양 끝 end_seg 길이도 후보로
    낸다. 다리가 뒤의 물체와 5cm 안으로 붙으면 한 덩어리가 되어 통째로
    버려졌다(2026-09-28: ls_origin 뒤 다리 둘이 158/268mm 덩어리로 묶여
    1.5m 에서 후보 3개 -> 인식 실패). 끝에 붙은 다리는 끝에서 건진다.
    가짜 후보가 조금 늘지만 조합 검사가 규격에 맞는 넷만 고른다.
    """
    out = []
    for g in groups:
        if len(g) < min_pts:
            continue
        w = math.hypot(x[g][-1] - x[g][0], y[g][-1] - y[g][0])
        if w > wmax and end_seg:
            for sub in _end_parts(x, y, g, end_seg):
                c = _leg_from(x, y, sub, wmin, min_pts, radius)
                if c is not None:
                    c["end"] = True
                    out.append(c)
            continue
        if not (wmin <= w <= wmax):
            continue
        c = _leg_from(x, y, g, 0.0, min_pts, radius)
        if c is not None:
            out.append(c)
    return out


def _end_parts(x, y, g, seg, step=0.012, skip=3):
    """덩어리 양 끝에서 seg 길이 안에 드는 점들.

    ⚠ 끝점을 그대로 기준으로 잡으면 안 된다. 다리와 뒤 물체의 경계에서는
      라이다가 둘 사이 거리를 섞어 돌려주는 점이 1~2개 튀어 나온다. 그 점을
      기준으로 자르면 조각에 점이 2개만 들어 다리 자리가 5~10cm 틀린다
      (2026-09-28, 1.77m 에서 인식 35/60). 점이 촘촘히 이어지기 시작하는
      곳(간격 step 이하)부터 자른다. 최대 skip 개까지 건너뛴다 (-> 55/60).
    """
    parts = []
    for ends in (list(g), list(g)[::-1]):
        k = 0
        while (k < min(skip, len(ends) - 2)
               and math.hypot(x[ends[k + 1]] - x[ends[k]],
                              y[ends[k + 1]] - y[ends[k]]) > step):
            k += 1
        e = ends[k:]
        x0, y0 = x[e[0]], y[e[0]]
        parts.append(sorted(i for i in e
                            if math.hypot(x[i] - x0, y[i] - y0) <= seg))
    return parts


def _leg_from(x, y, g, wmin, min_pts, radius):
    """점 묶음 하나를 다리 후보로. 너무 작거나 원점이면 None."""
    if len(g) < min_pts:
        return None
    w = math.hypot(x[g][-1] - x[g][0], y[g][-1] - y[g][0])
    if w < wmin:
        return None
    cx, cy = x[g].mean(), y[g].mean()
    d = math.hypot(cx, cy)
    if d < 1e-6:
        return None
    # 표면 -> 중심 보정 (라이다 반대쪽으로 반지름만큼)
    r = (w / 2.0) if radius is None else radius
    k = (d + r) / d
    return {"x": cx * k, "y": cy * k, "raw": (cx, cy),
            "w": w, "n": len(g), "r": r}


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
        # ⚠ 그것만으로는 모자란다. 네 점을 엇갈리게 이으면 '나비 모양' 이
        #   되는데, 그때는 진짜 변 두 개가 대각선 자리로 가고 진짜 대각선
        #   두 개가 변 자리로 온다. 마주보는 변도 같고 대각선 두 개도 같아서
        #   위 검사를 그대로 통과한다.
        #
        #   실제로 그렇게 걸렸다(2026-09-21). 423x566 렉을 567x708 로 읽고
        #   있었다 — 708 은 변이 아니라 대각선이다. 그 값으로 등록해 두었던
        #   탓에 err 가 1.5mm 로 나와 완벽해 보였다.
        #
        #   대각선은 반드시 sqrt(가로^2 + 세로^2) 여야 한다. 나비 모양이면
        #   여기서 484mm 가 어긋나 걸러진다.
        if abs(d1 - math.hypot(a, b)) > tol * 1.5:
            continue
        for ea, eb in ((spec_a, spec_b), (spec_b, spec_a)):
            err = math.hypot(a - ea, b - eb)
            if err > tol * 2:
                continue
            sc = max(0.0, 1.0 - err / (tol * 2))
            if best is None or sc > best[0]:
                best = (sc, a, b, err, order)
    return best


def _candidate_quads(pts, specs, tol):
    """규격에 맞을 수 있는 4점 묶음만 낸다. (idx, 이름, 규격)

    ⚠ 모든 4점 조합을 훑으면 후보 39개에서 8만 개가 넘는다. 넓은 덩어리의
      양 끝을 후보로 넣은 뒤(leg_end_seg) 렉 밑처럼 주변이 복잡한 곳에서
      후보가 39개까지 늘었고, 검출기가 CPU 100% 로 스캔 한 장에 3초 넘게
      걸려 인식 결과가 3.6초씩 묵었다 — 나오기가 '렉을 확인하지 못함' 으로
      거부됐다(2026-09-29).
      직사각형의 네 점은 서로의 거리가 가로·세로·대각선 중 하나여야 한다.
      그 거리에 맞는 쌍만 이어 두고 그 안에서 4점을 찾는다. 허용 폭은
      _rect_score 가 받아들이는 범위(변 tol*2, 대각선 약 tol*3.5)보다
      넉넉한 tol*4 라, 예전에 찾던 것은 빠지지 않는다.
    """
    n = len(pts)
    dist = [[math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
             for j in range(n)] for i in range(n)]
    for name, sp in specs.items():
        t = sp.get("tol", tol) * 4.0
        lens = (sp["width"], sp["depth"], math.hypot(sp["width"], sp["depth"]))
        nb = [set() for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if min(abs(dist[i][j] - L) for L in lens) <= t:
                    nb[i].add(j)
                    nb[j].add(i)
        for i in range(n):
            ni = {j for j in nb[i] if j > i}
            for j in sorted(ni):
                c2 = {k for k in ni & nb[j] if k > j}
                for k in sorted(c2):
                    for l in sorted(x for x in c2 & nb[k] if x > k):
                        yield (i, j, k, l), name, sp


def match_carts(cands, specs, tol=0.06, min_score=0.25):
    """후보 중 4개를 골라 등록된 대차와 맞춰 본다.

    후보가 많아도 조합 수는 적다(10개면 210가지). 접지선 같은 가짜가
    끼어도 진짜 네 개가 여전히 가장 잘 맞는다.
    """
    pts = [(c["x"], c["y"]) for c in cands]
    raws = [c.get("raw", (c["x"], c["y"])) for c in cands]
    found = []
    for idx, name, sp in _candidate_quads(pts, specs, tol):
        p = [pts[i] for i in idx]
        if True:
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


def match_carts3(cands, specs, tol=0.06, min_score=0.25):
    """다리 셋으로 대차를 찾는다. 넷째 다리는 계산으로 채운다.

    ⚠ 2026-11-13 넥·라이다 재배치 뒤 ls_origin 오른쪽 뒤 다리 하나가 점 0~3개로만
      잡혀(라이다 201.5mm 가 캐스터 판 높이와 겹침) 같은 자리에서 182장 중 111장만
      인식됐다 — 빠진 것은 늘 그 다리 하나였다. 나머지 셋은 2~7mm 로 정확했다.

    직사각형의 세 꼭짓점 A-B-C (B 가 모서리)는 |AB|·|BC| 가 가로·세로, |AC| 가
    대각선이어야 한다. 넷째 D = A + C - B. 넷보다 약한 증거이므로 점수를 0.8 배로
    낮추고, 받아들일지는 부르는 쪽(최근 확실한 인식 근처 / 카메라 확인)이 정한다.
    """
    pts = [(c["x"], c["y"]) for c in cands]
    raws = [c.get("raw", (c["x"], c["y"])) for c in cands]
    n = len(pts)
    found = []
    for name, sp in specs.items():
        t = sp.get("tol", tol)
        W, D = sp["width"], sp["depth"]
        diag = math.hypot(W, D)
        for b in range(n):
            for a in range(n):
                if a == b:
                    continue
                dab = math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])
                if min(abs(dab - W), abs(dab - D)) > t * 2:
                    continue
                for c in range(a + 1, n):
                    if c == b:
                        continue
                    dbc = math.hypot(pts[c][0] - pts[b][0], pts[c][1] - pts[b][1])
                    dac = math.hypot(pts[a][0] - pts[c][0], pts[a][1] - pts[c][1])
                    if abs(dac - diag) > t * 1.5:
                        continue
                    best = None
                    for ea, eb in ((W, D), (D, W)):
                        err = math.hypot(dab - ea, dbc - eb)
                        if err <= t * 2 and (best is None or err < best):
                            best = err
                    if best is None:
                        continue
                    # 직각 확인: 대각선 검사만으로는 각이 크게 틀어진 것도 지나간다
                    v1 = (pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])
                    v2 = (pts[c][0] - pts[b][0], pts[c][1] - pts[b][1])
                    cosang = (v1[0] * v2[0] + v1[1] * v2[1]) / max(1e-9, dab * dbc)
                    if abs(cosang) > 0.10:               # 90 +- 약 6도
                        continue
                    sc = max(0.0, 1.0 - best / (t * 2)) * 0.8
                    if sc < min_score:
                        continue
                    d4 = (pts[a][0] + pts[c][0] - pts[b][0],
                          pts[a][1] + pts[c][1] - pts[b][1])
                    r4 = (raws[a][0] + raws[c][0] - raws[b][0],
                          raws[a][1] + raws[c][1] - raws[b][1])
                    q = [pts[a], pts[b], pts[c], d4]
                    qa = (dab + math.hypot(pts[c][0] - d4[0], pts[c][1] - d4[1])) / 2
                    qb = (dbc + math.hypot(pts[a][0] - d4[0], pts[a][1] - d4[1])) / 2
                    found.append({
                        "cart": name, "score": sc, "a": qa, "b": qb, "err": best,
                        "x": (pts[a][0] + pts[c][0]) / 2,
                        "y": (pts[a][1] + pts[c][1]) / 2,
                        "yaw": _quad_yaw(q, qa, qb),
                        "legs": q,
                        "legs_raw": [raws[a], raws[b], raws[c], r4],
                        "idx": (a, b, c), "inferred": 3})
    found.sort(key=lambda f: -f["score"])
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
