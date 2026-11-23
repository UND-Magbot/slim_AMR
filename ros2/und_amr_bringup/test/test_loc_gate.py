"""위치를 모르면 출발 거부 · 충전소에서 켜지면 충전소 자리로 (2026-11-23).

POI 1 에서 끝난 뒤 로봇을 충전소로 옮겨 재부팅 -> SLAM 이 POI 1 에서 시작, 위치 감시가 582번 경고했는데도 출발했다.
"""
import math
import threading
import time
import types

import pytest


@pytest.fixture
def ApiNode():
    from und_amr_bringup.api_node import ApiNode
    return ApiNode


def _node(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.logs = []
    lg = lambda *a, **k: n.logs.append(a[0])               # noqa: E731
    n.get_logger = lambda: types.SimpleNamespace(info=lg, warn=lg, error=lg)
    n.loc_hold = None
    n._locmon = {"score": 0.14, "t": time.time()}
    n._locmon_hist = [0.15, 0.14, 0.16]
    return n


def test_unsure_when_scores_low(ApiNode):
    n = _node(ApiNode)
    assert "위치가 지도와 맞지 않습니다" in n._loc_unsure()
    n._locmon_hist = [0.40, 0.38, 0.45]                      # 충전소 앞 평소
    assert n._loc_unsure() is None
    n._locmon_hist = [0.15, 0.14, 0.16]
    n.loc_hold = {"x": 0}                                    # 사람이 고정
    assert n._loc_unsure() is None
    n.loc_hold = None
    n._locmon["t"] = time.time() - 60                        # 오래된 값
    assert n._loc_unsure() is None


def test_route_and_poi_goto_refuse_when_unsure(ApiNode):
    n = _node(ApiNode)
    ok, msg = n.route_start("1", "2", 1)
    assert not ok and "위치" in msg
    ok, msg = n.poi_goto("1")
    assert not ok and "위치" in msg


CHG = {"name": "충전소1", "kind": "charger", "x": 0.80, "y": -0.10, "yaw": math.radians(158)}


def _docked_scan(monkeypatch, ApiNode, gap=-0.02, lat=0.0):
    import und_amr_bringup.charger_detect as cd
    hl = ApiNode.CHARGER_HALF_LEN
    monkeypatch.setattr(cd, "detect_scan", lambda *a, **k: {"x": -(hl + gap), "y": -lat, "yaw": 0.0})
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id="laser"), ranges=[],
                                 angle_min=0.0, angle_increment=0.0)


def test_docked_pose_from_charger_poi(monkeypatch, ApiNode):
    n = _node(ApiNode)
    n.pois = [dict(CHG), {"name": "1", "kind": "point", "x": 4.17, "y": -1.93, "yaw": -0.35}]
    n._poi_laser = lambda f: (0.0, 0.0, 0.0)
    scan = _docked_scan(monkeypatch, ApiNode, gap=-0.02)
    rx, ry, ryaw, name = n._docked_pose(scan)
    # 틈 -20mm: POI(도킹 자세)에서 충전소 쪽으로 20mm
    assert name == "충전소1"
    assert abs(rx - (CHG["x"] - 0.02 * math.cos(CHG["yaw"]))) < 1e-6
    assert abs(ry - (CHG["y"] - 0.02 * math.sin(CHG["yaw"]))) < 1e-6
    assert abs(ryaw - CHG["yaw"]) < 1e-9
    assert n._docked_pose(_docked_scan(monkeypatch, ApiNode, gap=0.30)) is None   # 떨어져 있음


def _boot(monkeypatch, ApiNode, pose):
    n = _node(ApiNode)
    n.pois = [dict(CHG)]
    n._poi_laser = lambda f: (0.0, 0.0, 0.0)
    n.scan = _docked_scan(monkeypatch, ApiNode)
    n.nav_state, n.cdock = "idle", None
    n._api_t0 = time.monotonic()
    n.moved = []
    n.relocalize_manual = lambda x, y, yaw, hold=False: n.moved.append((x, y, yaw)) or (True, "ok")
    n._boot_dock_check(pose)
    return n


def test_boot_on_charger_with_wrong_pose_relocalizes(monkeypatch, ApiNode):
    n = _boot(monkeypatch, ApiNode, {"x": 4.41, "y": -1.99, "yaw": -0.10})
    assert len(n.moved) == 1 and n._boot_dock_done
    assert abs(n.moved[0][0] - CHG["x"]) < 0.05


def test_boot_on_charger_with_right_pose_leaves_it(monkeypatch, ApiNode):
    n = _boot(monkeypatch, ApiNode, {"x": 0.79, "y": -0.09, "yaw": CHG["yaw"]})
    assert not n.moved and n._boot_dock_done
