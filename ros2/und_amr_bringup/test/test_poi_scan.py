"""지점 라이다 기준 스캔 보정 시험 (2026-10-14).

- 처음 정확히 선 도착에서 기준 스캔을 저장한다 (보정을 얹어 간 자리는 기준이 아니다)
- 서 있을 때 기준과 맞춰 '실제로 서야 할 자리' 를 구하고, 다음 방문 때 목표에 미리 얹는다
- nav2 를 끊는 순간 잰 값이 1cm 넘게 다르면 목표점을 고친다 (1cm 안이면 그대로)
- 스캔이 안 맞거나(사람·물건) 보정이 너무 크면 쓰지 않는다
- 지점을 옮기면 예전 기준 스캔은 버린다
"""
import importlib.util
import math
import pathlib
import sys
import threading
import time

import numpy as np
import pytest

SRC = pathlib.Path(__file__).resolve().parent.parent / "und_amr_bringup"


@pytest.fixture(scope="module")
def mods():
    sys.path.insert(0, str(SRC.parent))
    spec = importlib.util.spec_from_file_location("api_node", SRC / "api_node.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["api_node"] = mod
    spec.loader.exec_module(mod)
    from und_amr_bringup import scan_icp
    return mod.ApiNode, scan_icp


def _walls():
    W = []
    for x in np.linspace(-3, 3, 400):
        W += [(x, 2.0), (x, -1.5)]
    for y in np.linspace(-1.5, 2, 240):
        W += [(3.0, y), (-2.5, y)]
    for a in np.linspace(0, 2 * math.pi, 80):
        W.append((1 + 0.2 * math.cos(a), 0.5 + 0.2 * math.sin(a)))
    return np.array(W)


def _seen_from(si, world_pts, pose, step=1, shift=0):
    """세계 점들을 pose 에 선 로봇이 본 점(로봇 기준)으로. step 으로 성기게."""
    world_pts = world_pts[shift::step]
    inv = si.inverse(pose)
    c, s = math.cos(inv[2]), math.sin(inv[2])
    return world_pts @ np.array([[c, -s], [s, c]]).T + np.array(inv[:2])


POI = {"name": "B", "kind": "point", "x": 0.0, "y": 0.0, "yaw": 0.0}


def _node(ApiNode, tmp_path):
    n = ApiNode.__new__(ApiNode)
    n.map_dir = str(tmp_path)
    n.active_map = "m1"
    n.pois = [dict(POI)]
    n.lock = threading.Lock()
    n.logs = []
    n.get_logger = lambda: type("L", (), {"info": lambda s_, m: n.logs.append(m),
                                          "warn": lambda s_, m: n.logs.append(m)})()
    n.nav_seq = 1
    n._poi_still = lambda: True
    n.POI_SCAN_CORRECT = True            # 보정 동작 자체를 시험한다 (운행 기본값은 꺼짐)
    return n


def test_reference_then_correction_then_applied(mods, tmp_path):
    ApiNode, si = mods
    W = _walls()
    n = _node(ApiNode, tmp_path)
    # 1) 첫 도착: SLAM 기준 목표에 2mm 로 섰다 -> 기준 스캔 저장
    n.nav_target = (0.0, 0.0, 0.0)
    n.nav_pos_err = (0.002, 0.0)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    np_ = {"poi": POI, "path": n._poi_path(POI)}
    n._poi_rest(1, np_, 0.0)
    assert pathlib.Path(np_["path"]).exists() and n._poi_ref(POI) is not None
    # 2) 나중 도착: SLAM 은 목표(0,0)에 섰다고 하지만 실제로는 (+20, -15mm, 0.5도) 에 서 있다
    real = (0.020, -0.015, math.radians(0.5))
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, real, 4, 1), (0.0, 0.0, 0.0))
    n._poi_rest(1, np_, 0.0)
    g = n._poi_corr[np_["path"]][0]
    # 실제로 서야 할 자리는 SLAM 좌표로 (-20, +15mm, -0.5도) 쪽이다
    assert abs(g[0] + 0.020) < 0.003 and abs(g[1] - 0.015) < 0.003
    assert g[2] == POI["yaw"]                 # 방향은 보정하지 않는다
    assert any("실제 도착 오차" in m for m in n.logs)
    # 3) 다음 방문: 그 자리를 목표로 보낸다
    x, y, yaw = n._poi_apply(POI, 0.0, 0.0, 0.0)
    assert (x, y, yaw) == g


def test_no_reference_from_corrected_or_sloppy_arrival(mods, tmp_path):
    ApiNode, si = mods
    W = _walls()
    n = _node(ApiNode, tmp_path)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    np_ = {"poi": POI, "path": n._poi_path(POI)}
    n.nav_target, n.nav_pos_err = (0.0, 0.0, 0.0), (0.012, 0.0)      # 12mm 로 섰다
    n._poi_rest(1, np_, 0.0)
    assert n._poi_ref(POI) is None
    n.nav_target, n.nav_pos_err = (0.01, 0.0, 0.0), (0.001, 0.0)      # 보정 얹은 자리
    n._poi_rest(1, np_, 0.0)
    assert n._poi_ref(POI) is None


def _with_ref(mods, tmp_path):
    ApiNode, si = mods
    W = _walls()
    n = _node(ApiNode, tmp_path)
    n.nav_target, n.nav_pos_err = (0.0, 0.0, 0.0), (0.0, 0.0)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    np_ = {"poi": POI, "path": n._poi_path(POI)}
    n._poi_rest(1, np_, 0.0)
    return n, si, W, np_


def test_snap_moves_target_only_when_over_1cm(mods, tmp_path):
    n, si, W, np_ = _with_ref(mods, tmp_path)
    n._nav_poi = np_
    # 6mm 어긋남 -> 그대로
    n.nav_target = (0.0, 0.0, 0.0)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.006, 0.0, 0.0), 4, 1), (0.0, 0.0, 0.0))
    n._poi_snap(1, np_, None)
    assert n.nav_target == (0.0, 0.0, 0.0)
    # 30mm 어긋남 -> 목표를 옮긴다
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.030, 0.0), 4, 1), (0.0, 0.0, 0.0))
    n._poi_snap(1, np_, None)
    assert abs(n.nav_target[1] + 0.030) < 0.003
    # 그 사이 새 목표가 들어왔으면 손대지 않는다
    n.nav_target = (0.0, 0.0, 0.0)
    n.nav_seq = 2
    n._poi_snap(1, np_, None)
    assert n.nav_target == (0.0, 0.0, 0.0)


def test_bad_match_or_huge_correction_is_ignored(mods, tmp_path):
    n, si, W, np_ = _with_ref(mods, tmp_path)
    # 주변이 완전히 달라졌다 (다른 방)
    other = np.array([(x, 0.7) for x in np.linspace(-1, 1, 300)])
    n._poi_snapshot = lambda after=0.0, step=4: (other, (0.0, 0.0, 0.0))
    n._poi_rest(1, np_, 0.0)
    assert np_["path"] not in getattr(n, "_poi_corr", {})
    # 15cm 어긋났다고 나오면 잘못 맞춘 것으로 본다
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.15, 0.0, 0.0), 4, 1), (0.0, 0.0, 0.0))
    n._poi_rest(1, np_, 0.0)
    assert np_["path"] not in getattr(n, "_poi_corr", {})


def test_moved_poi_drops_reference_and_stale_corr_not_applied(mods, tmp_path):
    n, si, W, np_ = _with_ref(mods, tmp_path)
    n._poi_corr = {np_["path"]: ((0.01, 0.0, 0.0), time.time() - 2 * n.POI_CORR_STALE_S)}
    assert n._poi_apply(POI, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)          # 낡은 보정
    moved = dict(POI, x=0.05)
    assert n._poi_ref(moved) is None                                     # 지점을 옮겼다
    ok, msg = n.poi_scan_reset("B")
    assert ok and n._poi_ref(POI) is None


def test_no_measure_while_moving_or_after_new_goal(mods, tmp_path):
    ApiNode, si = mods
    W = _walls()
    n = _node(ApiNode, tmp_path)
    n.nav_target, n.nav_pos_err = (0.0, 0.0, 0.0), (0.0, 0.0)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    np_ = {"poi": POI, "path": n._poi_path(POI)}
    n._poi_still = lambda: False                 # 돌기 시작했다
    n._poi_rest(1, np_, 0.0)
    assert n._poi_ref(POI) is None
    n._poi_still = lambda: True
    n.nav_seq = 2                                # 다음 목표가 들어왔다
    n._poi_rest(1, np_, 0.0)
    assert n._poi_ref(POI) is None


def test_snap_waits_until_still(mods, tmp_path):
    n, si, W, np_ = _with_ref(mods, tmp_path)
    n._nav_poi = np_
    n.nav_target = (0.0, 0.0, 0.0)
    n._poi_snapshot = lambda after=0.0, step=4: (_seen_from(si, W, (0.0, 0.030, 0.0), 4, 1), (0.0, 0.0, 0.0))
    n._poi_still = lambda: False                 # 아직 움직인다 -> 재지 않는다
    n._poi_snap(1, np_, None)
    assert n.nav_target == (0.0, 0.0, 0.0)


def test_correction_off_by_default_only_measures(mods, tmp_path):
    ApiNode, si = mods
    assert ApiNode.POI_SCAN_CORRECT is False
    n = _node(ApiNode, tmp_path)
    n.POI_SCAN_CORRECT = False
    n._poi_corr = {n._poi_path(POI): ((0.02, 0.0, 0.0), time.time())}
    assert n._poi_apply(POI, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
