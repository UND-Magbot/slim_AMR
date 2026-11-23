"""카메라 장애물 노드 — 바닥은 무시하고, 바닥 위 물건만, 잡음 한 장은 안 낸다 (2026-10-27)."""
import math

import numpy as np
import pytest

from und_amr_bringup.camera_obstacles import CameraObstacles, quat_matrix

FX = FY = 383.937
CX, CY = 324.932, 237.373
W, H = 640, 480
CAM = (0.467, 0.0, 0.230)        # base 기준 카메라 (URDF)


class Q:                          # optical -> base: RPY(-90, 0, -90)
    x, y, z, w = -0.5, 0.5, -0.5, 0.5


def make_node():
    n = CameraObstacles.__new__(CameraObstacles)
    n.every, n.dec = 1, 4
    n.min_depth, n.max_range = 0.15, 2.0
    n.fl_m, n.fl_s = 0.05, 0.02
    n.max_h, n.cell = 1.8, 0.05
    n.min_pts, n.nf, n.hits = 3, 3, 2
    n.half = math.radians(44.0)
    n.inc = math.radians(0.5)
    n.nbins = int(round(2 * n.half / n.inc))
    n.K = (FX, FY, CX, CY)
    n.ext = (quat_matrix(Q), np.array(CAM), (CAM[0], CAM[1], 0.0))
    n.rays = None
    n.n = 0
    n.hist = []
    n.lidar_clear, n.lidar_age, n.lidar = 0.10, 0.5, None
    n.scan_frame = "camera_link"
    n.out = []
    n.pub = type("P", (), {"publish": lambda self, m: n.out.append(m)})()
    return n


def depth_image(box=None, noise_px=None):
    """바닥(높이 0)과, 있으면 상자 앞면 [x, y0, y1, 높이] 를 그린 깊이 영상 [mm]."""
    v, u = np.mgrid[0:H, 0:W].astype(float)
    ry = (v - CY) / FY                       # optical y = 아래
    rx = (u - CX) / FX
    z = np.full((H, W), 0.0)
    down = ry > 1e-6
    z[down] = CAM[2] / ry[down]              # 바닥까지 깊이
    z[z > 6.0] = 0.0
    if box is not None:
        bx, y0, y1, bh = box
        d = bx - CAM[0]
        yb = -rx * d                         # 그 깊이에서 base y
        hb = CAM[2] - ry * d                 # 그 깊이에서 높이
        hit = (yb > y0) & (yb < y1) & (hb > 0) & (hb < bh) & ((z == 0) | (z > d))
        z[hit] = d
    if noise_px is not None:
        r, c = noise_px
        z[r:r + 8, c:c + 8] = 0.6                # 공중에 뜬 점 한 덩어리
    img = type("I", (), {})()
    img.data = (z * 1000).astype(np.uint16).tobytes()
    img.height, img.width, img.encoding = H, W, "16UC1"
    from builtin_interfaces.msg import Time
    depth_image.k = getattr(depth_image, "k", 0) + 1     # 장마다 다른 시각
    img.header = type("Hd", (), {"stamp": Time(sec=depth_image.k),
                                 "frame_id": "camera_depth_optical_frame"})()
    return img


def run(n, img):
    n.on_depth(img)
    return np.asarray(n.out[-1].ranges, dtype=float)


def test_flat_floor_is_free_not_obstacle():
    n = make_node()
    for _ in range(3):
        r = run(n, depth_image())
    assert not np.isfinite(r).any()          # 장애물 없음
    # 실제 시야 80도(fx 384, 폭 640) — 88도 칸 중 가장자리는 nan
    assert np.isinf(r).mean() > 0.8          # 보이는 방향은 모두 '비었음'(지우기)


def test_low_box_in_front_is_seen():
    # 라이다(201.5mm) 아래 높이 120mm 상자, 축에서 1.2m (카메라에서 0.73m)
    n = make_node()
    for _ in range(3):
        r = run(n, depth_image(box=(1.2, -0.15, 0.15, 0.12)))
    a = -n.half + np.arange(len(r)) * n.inc
    front = np.abs(a) < math.radians(8)
    assert np.isfinite(r[front]).all()
    assert abs(np.nanmin(r[front]) - 0.733) < 0.06
    assert not np.isfinite(r[np.abs(a) > math.radians(25)]).any()   # 옆은 깨끗


def test_same_frame_sent_twice_counts_once():
    from builtin_interfaces.msg import Time
    n = make_node()
    img = depth_image(noise_px=(200, 300))
    img.header.stamp = Time(sec=5)
    n.on_depth(img)
    n.on_depth(img)                         # 같은 장을 다시 받음
    assert len(n.out) == 1
    assert not np.isfinite(np.asarray(n.out[-1].ranges, dtype=float)).any()


def test_one_frame_blob_is_not_published():
    n = make_node()
    r = run(n, depth_image(noise_px=(200, 300)))
    assert not np.isfinite(r).any()          # 한 장만 보인 것은 안 낸다
    run(n, depth_image())
    run(n, depth_image())
    r = run(n, depth_image(noise_px=(200, 300)))
    assert not np.isfinite(r).any()          # 마지막 3장 중 1장뿐이면 안 낸다
    r = run(n, depth_image(noise_px=(200, 300)))
    assert np.isfinite(r).any()              # 이어서 보이면 낸다


def test_beyond_max_range_is_left_to_lidar():
    n = make_node()
    for _ in range(3):
        r = run(n, depth_image(box=(3.0, -0.3, 0.3, 0.5)))
    assert not np.isfinite(r).any()


def test_every_skips_frames():
    n = make_node()
    n.every = 3
    for _ in range(6):
        n.on_depth(depth_image())
    assert len(n.out) == 2


@pytest.mark.parametrize("enc", ["rgb8", "32FC1"])
def test_unknown_encoding_is_ignored(enc):
    n = make_node()
    img = depth_image()
    img.encoding = enc
    n.on_depth(img)
    assert n.out == []


def test_cells_the_lidar_already_sees_are_left_to_lidar():
    """라이다가 못 보는 것만 낸다 (2026-10-30). 라이다 점이 상자 앞면에 있으면 빼고,
    스캔이 오래되면 빼지 않는다."""
    import time
    n = make_node()
    face = np.stack([np.full(13, 1.2), np.linspace(-0.15, 0.15, 13)], 1)
    n.lidar = (time.monotonic(), face)
    for _ in range(3):
        r = run(n, depth_image(box=(1.2, -0.15, 0.15, 0.12)))
    assert not np.isfinite(r).any()
    n.lidar = (time.monotonic() - 2.0, face)    # 오래된 스캔
    r = run(n, depth_image(box=(1.2, -0.15, 0.15, 0.12)))
    assert np.isfinite(r).any()


def test_points_on_the_robot_body_are_ignored():
    """숙인 카메라에 보이는 차체 앞부분은 장애물이 아니다 (2026-11-11)."""
    n = make_node()
    n.body = (-0.48675, 0.48675, 0.2144)
    B = np.array([[0.40, 0.0, 0.15], [0.45, 0.10, 0.10], [0.80, 0.0, 0.15], [0.80, 0.01, 0.15],
                  [0.80, 0.02, 0.15]])
    cells = n.obstacle_cells(B, CAM[0], CAM[1])
    assert set(cells) == {(16, 0)}            # 0.80m 앞 상자만 (점 3개)


def test_neck_under_the_camera_is_ignored():
    """넥 앞면 (2026-11-11 실측: base x 0.40~0.46, y 0.10~0.28, 높이 0.10~0.51) 은 버리고
    로봇 앞 물건은 본다."""
    n = make_node()
    n.body = (-0.48675, 0.48675, 0.2144)
    n.self_x, n.self_hw = 0.50, 0.35
    neck = [[0.42, 0.25, h] for h in (0.15, 0.30, 0.45)]
    front = [[0.72, 0.27, 0.30]] * 3
    cells = n.obstacle_cells(np.array(neck + front), CAM[0], CAM[1])
    assert set(cells) == {(14, 5)}


def test_range_max_beyond_obstacle_range_so_free_is_not_marked():
    """nav2 는 inf 를 range_max - 0.0001 로 바꾼다. 그 값이 표시 한계(max_range) 밖이어야
    빈 방향 끝에 장애물이 찍히지 않는다 (2026-11-11 이동 취소)."""
    n = make_node()
    m = None
    for _ in range(3):
        n.on_depth(depth_image())
    m = n.out[-1]
    assert m.range_max - 1e-4 > n.max_range + 0.1
