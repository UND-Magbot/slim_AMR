"""nav2 로 가다 막혀 선 원인 기록 · 20초 알림 (2026-11-20, 12:07 PolygonStop 10분 정지)."""
import threading
import types

import numpy as np
import pytest


@pytest.fixture
def ApiNode():
    from und_amr_bringup.api_node import ApiNode
    return ApiNode


def _odom(v=0.0, w=0.0):
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=w))))


def _node(ApiNode, pts_cam=((0.40, 0.02),) * 8):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.logs = []
    lg = lambda *a: n.logs.append(a[0])                    # noqa: E731
    n.get_logger = lambda: types.SimpleNamespace(info=lg, warn=lg, error=lg)
    n.nav_state, n.nav_goal, n.nav_msg = "active", object(), "이동 중"
    n._turning = False
    n.odom = _odom()
    n.tf_pose = lambda *a, **k: {"x": 5.5, "y": -2.5, "yaw": 0.0}
    n._obstacle_sources = lambda: [("라이다", "L", False), ("카메라 down", "D", True)]
    xy = {"L": (np.array([2.0]), np.array([0.0])),
          "D": (np.array([p[0] for p in pts_cam]), np.array([p[1] for p in pts_cam]))}
    n._scan_xy = lambda m, cam: xy[m]
    n.snaps = []
    n._stop_snap = lambda tag: n.snaps.append(tag) or [tag + "_front.jpg"]
    return n


def _ticks(n, secs):
    for _ in range(int(secs / 0.5)):
        n._stop_watch()


def test_logs_sources_then_alerts_then_clears(ApiNode):
    n = _node(ApiNode)
    _ticks(n, 4.5)
    assert not n.logs
    _ticks(n, 1.0)
    assert len(n.logs) == 1 and "카메라 down 8점" in n.logs[0] and "라이다 0점" in n.logs[0]
    assert len(n.snaps) == 1
    _ticks(n, 15.0)
    assert "20초째 정지" in n.nav_msg or "21초째 정지" in n.nav_msg
    assert n._stopw["alerted"]
    n.odom = _odom(v=0.1)
    n._stop_watch()
    assert n.nav_msg == "이동 중" and n._stopw is None
    assert "다시 움직임" in n.logs[-1]


def test_empty_zone_is_flagged(ApiNode):
    n = _node(ApiNode, pts_cam=((2.0, 0.5),))
    _ticks(n, 5.5)
    assert "영역이 비어 있음" in n.logs[0]


def test_not_watching_when_not_driving(ApiNode):
    n = _node(ApiNode)
    n.nav_state = "done"
    _ticks(n, 30.0)
    assert not n.logs and n._stopw is None
