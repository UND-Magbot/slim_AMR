"""지도 편집 합성 회귀 시험.

원본을 고치지 않는다는 약속과, 가상벽이 위치추정 지도에 새지 않는다는
약속이 이 기능의 전부다. 둘 다 여기서 지킨다.
"""
import io
import math
import threading

import numpy as np
import pytest
from PIL import Image

RES, OX, OY = 0.05, -2.0, -1.5
W, H = 120, 90


def w2p(x, y):
    """월드 -> 픽셀. _score_pose 와 같은 규칙."""
    return int((x - OX) / RES), H - 1 - int((y - OY) / RES)


BLACK_AT = (0.6, 0.6)          # 여기에 검은 덩어리를 둔다


def blank_map():
    """BLACK_AT 자리에 검은 덩어리 하나가 있는 흰 지도."""
    a = np.full((H, W), 255, np.uint8)
    c, r = w2p(*BLACK_AT)
    a[r - 3:r + 4, c - 3:c + 4] = 0
    buf = io.BytesIO()
    Image.fromarray(a, "L").save(buf, "PNG")
    return buf.getvalue()


def px(png):
    with Image.open(io.BytesIO(png)) as im:
        return np.array(im.convert("L"))


@pytest.fixture
def node(ApiNode):
    n = object.__new__(ApiNode)
    n.lock = threading.Lock()
    n._map_seq = 0
    n.map_base_png = blank_map()
    n.map_png = None
    n.map_loc_png = None
    n.map_edits = []
    n.map_meta = {"resolution": RES, "origin": {"x": OX, "y": OY}, "seq": 0}
    return n


def test_편집이_없으면_원본_그대로(node):
    node._compose_map()
    assert node.map_png == node.map_base_png
    assert node.map_loc_png == node.map_base_png


def test_지우개는_흰색으로_만든다(ApiNode, node):
    c, r = w2p(0.6, 0.6)                       # 검은 덩어리 자리
    base = px(node.map_base_png)
    assert base[r, c] == 0, "시험 준비가 틀렸다 — 그 자리가 검지 않다"
    node.map_edits = ApiNode._clean_shapes(
        [{"op": "erase", "kind": "circle", "x": 0.6, "y": 0.6, "r": 0.12}])
    node._compose_map()
    assert px(node.map_png)[r, c] == 255
    assert px(node.map_loc_png)[r, c] == 255, "지우개는 위치추정에도 반영돼야 한다"


def test_지우개는_반경_밖을_건드리지_않는다(ApiNode, node):
    node.map_edits = ApiNode._clean_shapes(
        [{"op": "erase", "kind": "circle", "x": 0.6, "y": 0.6, "r": 0.12}])
    node._compose_map()
    c, r = w2p(1.2, 0.6)                       # 0.6 m 떨어진 곳
    assert px(node.map_png)[r, c] == px(node.map_base_png)[r, c]


def test_가상벽은_화면에만_들어가고_위치추정에는_안_들어간다(ApiNode, node):
    """이 기능의 핵심. 실제로 없는 벽을 위치추정 지도에 넣으면
    '여기서 벽이 보여야 한다' 고 기대해 맞는 자세의 점수가 깎인다."""
    node.map_edits = ApiNode._clean_shapes(
        [{"op": "wall", "kind": "line",
          "points": [[-1.0, 0.0], [1.0, 0.0]], "width": 0.2}])
    node._compose_map()
    disp, loc = px(node.map_png), px(node.map_loc_png)
    c, r = w2p(0.0, 0.0)
    assert disp[r, c] == 0, "화면용 지도에 가상벽이 안 들어갔다"
    assert loc[r, c] != 0, "가상벽이 위치추정 지도로 샜다"
    assert (loc == px(node.map_base_png)).all(), "위치추정 지도가 원본과 달라졌다"


def test_금지구역은_감싼_안쪽을_채운다(ApiNode, node):
    node.map_edits = ApiNode._clean_shapes(
        [{"op": "wall", "kind": "poly",
          "points": [[-1.0, -0.6], [-0.2, -0.6], [-0.2, 0.2], [-1.0, 0.2]]}])
    node._compose_map()
    a = px(node.map_png)
    inside = w2p(-0.6, -0.2)
    outside = w2p(0.6, -0.2)
    assert a[inside[1], inside[0]] == 0, "안쪽이 안 채워졌다"
    assert a[outside[1], outside[0]] != 0, "바깥까지 채웠다"


def test_편집을_비우면_원본이_픽셀까지_되돌아온다(ApiNode, node):
    before = px(node.map_base_png).copy()
    node.map_edits = ApiNode._clean_shapes([
        {"op": "erase", "kind": "circle", "x": 0.6, "y": 0.6, "r": 0.2},
        {"op": "wall", "kind": "line", "points": [[-1, 0], [1, 0]], "width": 0.2},
    ])
    node._compose_map()
    assert not (px(node.map_png) == before).all(), "편집이 반영되지 않았다"
    node.map_edits = []
    node._compose_map()
    assert (px(node.map_png) == before).all(), "원본으로 못 돌아왔다"


def test_합성할_때마다_seq_가_오른다(node):
    """자동 위치 재조정이 거리장 캐시를 seq 로 무효화한다.
    안 오르면 편집이 위치추정에 반영되지 않는다."""
    node._compose_map()
    first = node.map_meta["seq"]
    node._compose_map()
    assert node.map_meta["seq"] > first


def test_지도_밖_도형은_그냥_넘어간다(ApiNode, node):
    node.map_edits = ApiNode._clean_shapes(
        [{"op": "erase", "kind": "circle", "x": 900.0, "y": 900.0, "r": 0.2}])
    node._compose_map()                       # 예외 없이 끝나야 한다
    assert node.map_png


# ---------------------------------------------------------------- 입력 걸러내기

def test_이상한_도형은_버린다(ApiNode):
    """잘못된 한 줄 때문에 지도가 통째로 지워지는 일은 없어야 한다."""
    bad = [
        {"op": "erase", "kind": "circle", "x": 0, "y": 0, "r": 99},    # 너무 큼
        {"op": "erase", "kind": "circle", "x": 0, "y": 0},             # r 없음
        {"op": "keepout", "kind": "circle", "x": 0, "y": 0, "r": 0.2},  # 모르는 op
        {"op": "erase", "kind": "circle", "x": 1e9, "y": 0, "r": 0.2},  # 말도 안 되는 좌표
        {"op": "wall", "kind": "line", "points": [[0, 0]]},             # 점이 하나
        {"op": "wall", "kind": "poly", "points": [[0, 0], [1, 1]]},     # 점이 둘
        {"op": "wall", "kind": "line", "points": [[0, 0], [1, 1]], "width": 9},
        "문자열",
        {"op": "erase", "kind": "circle", "x": 0.5, "y": 0.5, "r": 0.2},  # 이것만 정상
    ]
    out = ApiNode._clean_shapes(bad)
    assert len(out) == 1, "걸러내기가 %d개를 통과시켰다" % len(out)
    assert out[0]["op"] == "erase"


def test_선과_영역은_통과시킨다(ApiNode):
    ok = [
        {"op": "wall", "kind": "line", "points": [[0, 0], [1, 0], [1, 1]],
         "width": 0.25},
        {"op": "wall", "kind": "poly", "points": [[0, 0], [1, 0], [1, 1]]},
    ]
    out = ApiNode._clean_shapes(ok)
    assert len(out) == 2
    assert out[0]["kind"] == "line" and out[0]["width"] == 0.25
    assert out[1]["kind"] == "poly" and len(out[1]["points"]) == 3


def test_개수_상한을_지킨다(ApiNode):
    many = [{"op": "erase", "kind": "circle", "x": 0.1 * i, "y": 0, "r": 0.1}
            for i in range(200)]
    assert len(ApiNode._clean_shapes(many, limit=50)) == 50
