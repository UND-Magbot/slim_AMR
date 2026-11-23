"""자동 위치 재조정 판정 회귀 시험.

일치도만 보면 엉뚱한 자리를 못 거른다 — 실측에서 정답 52% 대 오답 51% 였다.
'라이다는 벽 너머를 볼 수 없다' 는 판정이 그 1%p 를 가른다.
"""
import math

import numpy as np
import pytest

RES, OX, OY = 0.05, 0.0, 0.0
H, W = 200, 200            # 10 m x 10 m


@pytest.fixture(scope="module")
def room():
    """가운데를 벽으로 나눈 방. 왼쪽 칸과 오른쪽 칸이 생긴다."""
    occ = np.zeros((H, W), bool)
    occ[0, :] = occ[H - 1, :] = True          # 위아래 바깥벽
    occ[:, 0] = occ[:, W - 1] = True          # 좌우 바깥벽
    occ[:, W // 2] = True                     # 가운데 칸막이
    free = ~occ
    return occ, free


def w2p(x, y):
    return int((x - OX) / RES), H - 1 - int((y - OY) / RES)


def scan_from(occ, x, y, n=120, rmax=9.0):
    """그 자리에서 라이다를 쏘면 보이는 점들 (로봇 기준 좌표)."""
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for t in np.arange(0.15, rmax, RES / 2):
            gx = int((x + t * math.cos(a) - OX) / RES)
            gy = H - 1 - int((y + t * math.sin(a) - OY) / RES)
            if not (0 <= gx < W and 0 <= gy < H):
                break
            if occ[gy, gx]:
                pts.append((t * math.cos(a), t * math.sin(a)))
                break
    return np.array(pts)


def adj(ApiNode, occ, free, pts, x, y, yaw=0.0):
    """실제 판정식. 벽뚫음과 미지영역을 함께 본다.

    벽뚫음만 보면 안 된다 - 텅 빈 곳에 놓인 자세는 광선이 지도 밖으로 나가
    '안 뚫은' 것처럼 보인다. 실제로 칸막이 건너편은 벽뚫음 0 이지만
    미지영역이 0.36 이라 여기서 걸린다.
    """
    b, u = ApiNode._ray_check(occ, free, RES, OX, OY, H, pts, x, y, yaw)
    return (1.0 - b) * (1.0 - u), b, u


def test_정답_자세는_설명이_된다(ApiNode, room):
    occ, free = room
    pts = scan_from(occ, 2.5, 5.0)
    assert len(pts) > 80, "시험 준비: 스캔 점이 모자람"
    score, b, u = adj(ApiNode, occ, free, pts, 2.5, 5.0)
    assert b < 0.10, "정답인데 벽 뚫음 %.2f" % b
    assert u < 0.10, "정답인데 미지영역 %.2f" % u
    assert score > 0.85


@pytest.mark.parametrize("x,y,왜", [
    (3.3, 5.0, "0.8 m 어긋남 — 광선이 칸막이를 뚫는다"),
    (4.2, 5.0, "1.7 m 어긋남"),
    (7.5, 5.0, "칸막이 건너편 — 뚫지는 않지만 지도에 없는 곳을 본다"),
    (2.5, 2.0, "3 m 아래"),
])
def test_어긋난_자세는_정답보다_낮다(ApiNode, room, x, y, 왜):
    """일치도 1%p 차이로 순위가 뒤집히던 것을 이 판정이 가른다.

    2026-08-28 실측: 정답 52% 대 오답 51% 였는데, 벽뚫음은 정답 근처
    22~43% 대 6~8 m 떨어진 곳 59~81% 로 뚜렷하게 갈렸다.
    """
    occ, free = room
    pts = scan_from(occ, 2.5, 5.0)
    good = adj(ApiNode, occ, free, pts, 2.5, 5.0)[0]
    bad, b, u = adj(ApiNode, occ, free, pts, x, y)
    assert bad < good * 0.85, (
        "%s: 정답 %.2f 인데 이 자세가 %.2f (벽뚫음 %.2f, 미지 %.2f)"
        % (왜, good, bad, b, u))


def test_두_지표는_서로를_보완한다(ApiNode, room):
    """벽뚫음만 쓰면 지도 밖을 보는 자세를 놓친다."""
    occ, free = room
    pts = scan_from(occ, 2.5, 5.0)
    _, b, u = adj(ApiNode, occ, free, pts, 7.5, 5.0)   # 칸막이 건너편
    assert b < 0.10, "여기는 벽을 뚫지 않는다 (그래서 벽뚫음만으론 못 잡는다)"
    assert u > 0.20, "미지영역이 이 자세를 잡아야 한다"


def test_맞는_자세일수록_일치도가_높다(ApiNode, room):
    occ, free = room
    lik = np.where(occ, 1.0, 0.0).astype(np.float32)
    pts = scan_from(occ, 2.5, 5.0)
    at = ApiNode._score_pose(lik, RES, OX, OY, H, pts, 2.5, 5.0, 0.0)
    off = ApiNode._score_pose(lik, RES, OX, OY, H, pts, 2.5, 5.0,
                              math.radians(35))
    assert at > off, "제자리(%.2f)가 35도 틀어진 것(%.2f)보다 낮다" % (at, off)


def test_지도_밖으로_나가면_점수를_주지_않는다(ApiNode, room):
    """밖으로 도망가면 점수가 오르는 일이 생기면 안 된다."""
    occ, free = room
    lik = np.where(occ, 1.0, 0.0).astype(np.float32)
    pts = scan_from(occ, 2.5, 5.0)
    inside = ApiNode._score_pose(lik, RES, OX, OY, H, pts, 2.5, 5.0, 0.0)
    outside = ApiNode._score_pose(lik, RES, OX, OY, H, pts, 60.0, 60.0, 0.0)
    assert outside < inside


def test_스캔_점을_솎아낸다(ApiNode):
    """전역 탐색은 계산량이 커서 점을 줄여 쓴다."""
    class FakeScan:
        angle_min = -math.pi
        angle_increment = 2 * math.pi / 1200
        range_min, range_max = 0.05, 20.0
        ranges = [2.0] * 1200
    out = ApiNode._scan_points(FakeScan(), max_pts=180)
    assert 0 < len(out) <= 180
    assert out.shape[1] == 2
