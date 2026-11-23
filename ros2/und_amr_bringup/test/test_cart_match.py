"""대차 인식 계산 회귀 시험.

여기 있는 숫자는 전부 2026-08-27~28 에 실물에서 재서 확인한 값이다.
값이 나빠지면 이 시험이 먼저 알려 준다.
"""
import math

import numpy as np
import pytest

LIDAR_NOISE = 0.0016      # 라이다 반복 정밀도 실측 1.6 mm


def fake_legs(cm, width, depth, yaw_deg, seed, noise=LIDAR_NOISE, dist=1.8):
    """규격을 알고 있는 대차를 그 자세로 놓았을 때의 다리 네 개."""
    rng = np.random.default_rng(seed)
    base = [(+depth / 2, +width / 2), (+depth / 2, -width / 2),
            (-depth / 2, -width / 2), (-depth / 2, +width / 2)]
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    out = []
    for x, y in base:
        out.append({"x": (x * c - y * s + dist) + rng.normal(0, noise),
                    "y": (x * s + y * c) + rng.normal(0, noise)})
    return out


def yaw_spread(vals):
    """방향은 180도 주기의 축이다. 그 주기로 접어 흩어짐을 잰다."""
    ref = vals[0]
    folded = [((v - ref + 90) % 180) - 90 for v in vals]
    return float(np.std(folded))


# ---------------------------------------------------------------- 방향

@pytest.mark.parametrize("name,w,d", [
    ("직사각형 LG", 0.70, 0.50),
    ("직사각형 LG2", 0.64, 0.545),
    ("정사각형 S300", 0.765, 0.765),      # 주성분으로 구하면 여기서 무너진다
])
def test_방향이_잡음에도_안정적이다(cm, name, w, d):
    """정사각형은 긴 축이 없어 주성분(SVD)으로는 방향이 정해지지 않는다.

    2026-08-28 실측: 주성분 방식일 때 표준편차 101도. 변에서 구하도록
    바꾼 뒤 0.1도. 0.5도를 넘으면 그 방식으로 되돌아간 것이다.
    """
    spec = {name: {"width": w, "depth": d, "tol": 0.06}}
    yaws = []
    for seed in range(40):
        r = cm.match_carts(fake_legs(cm, w, d, 20.0, seed), spec)
        assert r, "%s 를 못 찾았다" % name
        yaws.append(math.degrees(r[0]["yaw"]))
    assert yaw_spread(yaws) < 0.5, "방향 흔들림 %.2f도" % yaw_spread(yaws)


def test_방향은_90도_주기로_접힌_축이다(cm):
    """다리 넷으로는 앞뒤를 가릴 수 없다. -90~90도 안으로 낸다."""
    spec = {"LG2": {"width": 0.64, "depth": 0.545, "tol": 0.06}}
    for deg in (0, 20, 70, 155, 200, 300):
        r = cm.match_carts(fake_legs(cm, 0.64, 0.545, deg, 1), spec)
        assert r
        y = math.degrees(r[0]["yaw"])
        assert -90.0 <= y <= 90.0, "%d도 -> %.1f도" % (deg, y)


# ---------------------------------------------------------------- 조합 탐색

def test_가짜가_스물둘_섞여도_진짜_넷을_찾는다(cm):
    """후보를 전부 다리로 보면 크기가 안 맞아 인식 자체가 실패한다.

    현장에서 접지선이 다리로 잡혀 겪은 문제다. 실측에서 후보 26개
    (진짜 4 + 가짜 22) 중 진짜만 골라냈다.
    """
    w, d = 0.64, 0.545
    real = fake_legs(cm, w, d, 15.0, 7)
    rng = np.random.default_rng(11)
    cands = list(real)
    while len(cands) < 26:
        cands.append({"x": float(rng.uniform(0.4, 2.8)),
                      "y": float(rng.uniform(-1.4, 1.4))})
    r = cm.match_carts(cands, {"LG2": {"width": w, "depth": d, "tol": 0.06}})
    assert r, "가짜가 섞이자 못 찾았다"
    best = r[0]
    assert best["cart"] == "LG2"
    # 고른 넷이 실제로 진짜 다리인가 (진짜는 목록 앞 네 개)
    assert set(best["idx"]) == {0, 1, 2, 3}, "엉뚱한 넷을 골랐다: %s" % (best["idx"],)
    assert best["err"] < 0.02, "규격 오차 %.3f m" % best["err"]


def test_규격이_어긋나면_점수가_떨어진다(cm):
    """2026-08-28 S300 사건. 도면값 765x765 를 넣었더니 실측 707x688 과
    95 mm 어긋나 점수가 문턱(0.25)에 걸쳐 인식률이 11% 로 깜빡였다."""
    real_w, real_d = 0.707, 0.688
    legs = fake_legs(cm, real_w, real_d, 5.0, 3)
    도면 = cm.match_carts(legs, {"S300": {"width": 0.765, "depth": 0.765,
                                          "tol": 0.06}}, min_score=0.0)
    실측 = cm.match_carts(legs, {"S300": {"width": real_w, "depth": real_d,
                                          "tol": 0.06}}, min_score=0.0)
    assert 도면 and 실측
    assert 도면[0]["score"] < 0.25, "도면값인데 문턱을 넘었다"
    assert 실측[0]["score"] > 0.9, "실측값인데 점수가 낮다"


def test_다리를_사각형_차례로_돌려준다(cm):
    """조합 번호순으로 두면 이어 그릴 때 나비 모양으로 엇갈린다."""
    def crosses(p, q, r, s):
        def cr(o, a, b):
            return ((a[0]-o[0])*(b[1]-o[1])) - ((a[1]-o[1])*(b[0]-o[0]))
        return (cr(p, q, r)*cr(p, q, s) < 0) and (cr(r, s, p)*cr(r, s, q) < 0)

    spec = {"LG2": {"width": 0.64, "depth": 0.545, "tol": 0.06}}
    for seed in range(12):
        r = cm.match_carts(fake_legs(cm, 0.64, 0.545, 33.0, seed), spec)
        assert r
        L = r[0]["legs"]
        assert not crosses(L[0], L[1], L[2], L[3])
        assert not crosses(L[1], L[2], L[3], L[0])


# ---------------------------------------------------------------- 다리 후보

def test_고정_반지름이_보이는_폭보다_덜_흔들린다(cm):
    """보이는 폭은 캐스터 방향에 따라 몇 배까지 달라진다(실측 15~51 mm).

    그 절반을 보정에 쓰면 같은 대차도 각도마다 크기가 달라진다.
    2026-08-28 실측: 짧은 변 흔들림 폭÷2 가 ±2.1 mm, 고정 20 mm 가 ±0.6 mm.
    """
    rng = np.random.default_rng(5)
    fixed, bywidth = [], []
    for _ in range(30):
        pts, groups, xs, ys = [], [], [], []
        # 네 다리. 보이는 폭만 매번 다르게 준다 (캐스터 방향이 바뀌는 상황)
        for cx, cy in ((1.5, 0.3), (2.0, 0.3), (2.0, -0.3), (1.5, -0.3)):
            w = float(rng.uniform(0.015, 0.055))
            n = 5
            g0 = len(xs)
            for k in range(n):
                t = (k / (n - 1) - 0.5) * w
                xs.append(cx - t * cy / math.hypot(cx, cy))
                ys.append(cy + t * cx / math.hypot(cx, cy))
            groups.append(list(range(g0, g0 + n)))
        X, Y = np.array(xs), np.array(ys)
        a = cm.leg_candidates(X, Y, groups, radius=0.020)
        b = cm.leg_candidates(X, Y, groups, radius=None)
        assert len(a) == 4 and len(b) == 4
        side = lambda c: math.hypot(c[0]["x"]-c[1]["x"], c[0]["y"]-c[1]["y"])
        fixed.append(side(a))
        bywidth.append(side(b))
    assert np.std(fixed) < np.std(bywidth), (
        "고정 반지름이 더 흔들린다: %.4f vs %.4f" % (np.std(fixed), np.std(bywidth)))


def test_덩어리_끊기는_간격으로_나눈다(cm):
    """점 사이가 gap 보다 벌어지면 다른 물체로 본다."""
    ang = np.linspace(-0.4, 0.4, 200)
    r = np.full_like(ang, 2.0)
    r[100:] = 2.6                      # 가운데서 60 cm 껑충 뛴다
    x, y, groups = cm.cluster_scan(r, float(ang[0]),
                                   float(ang[1] - ang[0]), rmax=3.0, gap=0.05)
    assert len(groups) >= 2, "떨어진 두 물체를 하나로 묶었다"


# ---------------------------------------------------------------- 규격 없이 재기

def test_등록표_없이도_사각형을_찾는다(cm):
    """새 대차를 재는 화면이 쓰는 함수. 규격을 모르는 상태로 잰다."""
    w, d = 0.70, 0.52
    legs = fake_legs(cm, w, d, 10.0, 2)
    got = cm.best_rectangles([(c["x"], c["y"]) for c in legs])
    assert got, "사각형을 못 찾았다"
    a, b = got[0]["a"], got[0]["b"]
    assert abs(max(a, b) - max(w, d)) < 0.01
    assert abs(min(a, b) - min(w, d)) < 0.01


def test_로봇_뒤쪽은_보지_않는다(cm):
    """뒤쪽 벽 모서리가 더 그럴듯한 사각형으로 잡히는 일이 있었다."""
    behind = [(-1.2, 0.3), (-1.8, 0.3), (-1.8, -0.3), (-1.2, -0.3)]
    assert cm.best_rectangles(behind, front_only=True) == []
    assert cm.best_rectangles(behind, front_only=False), "앞쪽만 끄면 찾아야 한다"


def test_leg_merged_with_background_is_recovered_from_cluster_end(cm):
    # 2026-09-28: 뒤 다리가 뒤 물체와 붙어 넓은 덩어리가 되면 버려졌다.
    import numpy as np
    xs = [1.0 + 0.004 * i for i in range(7)]            # 다리 (24mm)
    xs += [1.03 + 0.004 * i for i in range(40)]          # 붙은 물체
    x = np.array(xs); y = np.full(len(xs), 0.5)
    g = [list(range(len(xs)))]
    assert cm.leg_candidates(x, y, g) == []
    c = cm.leg_candidates(x, y, g, end_seg=0.04)
    assert any(abs(q["raw"][0] - 1.012) < 0.01 for q in c)


def test_cluster_end_skips_mixed_pixel_outlier(cm):
    # 경계에서 튄 점 하나가 끝에 붙어 있어도 다리 자리를 제대로 잡는다.
    import numpy as np
    xs = [0.95] + [1.0 + 0.004 * i for i in range(7)] \
        + [1.03 + 0.004 * i for i in range(40)]
    x = np.array(xs); y = np.full(len(xs), 0.5)
    c = cm.leg_candidates(x, y, [list(range(len(xs)))], end_seg=0.04)
    assert any(abs(q["raw"][0] - 1.015) < 0.008 for q in c)


def test_pruned_quad_search_matches_brute_force(cm):
    # 2026-09-29: 후보가 39개로 늘어 전수 조합이 스캔 한 장에 3초를 넘겼다.
    # 거리로 걸러 낸 탐색이 전수 탐색과 똑같은 결과를 내야 한다.
    import itertools
    import random
    specs = {"ls_origin": {"width": 0.423, "depth": 0.566, "tol": 0.04}}

    def brute(c):
        orig = cm._candidate_quads
        cm._candidate_quads = lambda pts, sp, tol: (
            (idx, n, s) for idx in itertools.combinations(range(len(pts)), 4)
            for n, s in sp.items())
        try:
            return cm.match_carts(c, specs, min_score=0.2)
        finally:
            cm._candidate_quads = orig
    for seed in range(40):
        r = random.Random(seed)
        c = [{"x": r.uniform(-1.5, 1.5), "y": r.uniform(-1.5, 1.5)}
             for _ in range(r.randint(4, 16))]
        cx, cy, yw = r.uniform(-1, 1), r.uniform(-1, 1), r.uniform(-1.5, 1.5)
        for sx, sy in ((.283, .2115), (.283, -.2115), (-.283, .2115), (-.283, -.2115)):
            c.append({"x": cx + sx * math.cos(yw) - sy * math.sin(yw),
                      "y": cy + sx * math.sin(yw) + sy * math.cos(yw)})
        a = [(f["cart"], f["idx"]) for f in brute(c)]
        b = [(f["cart"], f["idx"]) for f in cm.match_carts(c, specs, min_score=0.2)]
        assert a == b


def test_three_legs_infer_fourth():
    """2026-11-13 ls_origin 뒤 다리 하나가 자주 안 잡혔다 — 셋으로 넷째를 계산."""
    import math as _m
    from und_amr_bringup.cart_match import match_carts3
    W, D = 0.423, 0.566
    yaw = _m.radians(-8.5)
    c, s = _m.cos(yaw), _m.sin(yaw)
    cx, cy = 1.33, 0.006
    corners = [(-D / 2, -W / 2), (D / 2, -W / 2), (D / 2, W / 2), (-D / 2, W / 2)]
    legs = [(cx + a * c - b * s, cy + a * s + b * c) for a, b in corners]
    cands = [{"x": x, "y": y} for x, y in [legs[0], legs[2], legs[3]]]
    cands += [{"x": 2.5, "y": -1.0}, {"x": 0.4, "y": 1.7}]       # 잡동사니
    spec = {"ls_origin": {"width": W, "depth": D, "tol": 0.04}}
    r = match_carts3(cands, spec)
    assert r and r[0]["cart"] == "ls_origin" and r[0]["inferred"] == 3
    assert abs(r[0]["x"] - cx) < 1e-6 and abs(r[0]["y"] - cy) < 1e-6
    d4 = r[0]["legs"][3]
    assert min(_m.hypot(d4[0] - x, d4[1] - y) for x, y in legs) < 1e-6


def test_three_legs_reject_non_right_angle():
    from und_amr_bringup.cart_match import match_carts3
    # 변 길이는 맞지만 각이 70도 — 대각선도 어긋나 걸러져야 한다
    import math as _m
    W, D = 0.423, 0.566
    b = (1.0, 0.0)
    a = (1.0 + D, 0.0)
    ang = _m.radians(70)
    cpt = (1.0 + W * _m.cos(ang), W * _m.sin(ang))
    cands = [{"x": p[0], "y": p[1]} for p in (a, b, cpt)]
    assert match_carts3(cands, {"r": {"width": W, "depth": D, "tol": 0.04}}) == []
