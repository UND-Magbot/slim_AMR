"""POI 앞 축 위 경유점 (2026-11-23): nav2 가 비스듬히 들어오지 않게 경유점 -> 서지 않고 앞 자리로."""
import math
import threading
import types

import pytest

from test_narrow_poi import make


@pytest.fixture
def ApiNode():
    from und_amr_bringup.api_node import ApiNode
    return ApiNode


POI = {"name": "일반", "x": 3.0, "y": 0.0, "yaw": 0.0}


def _node(ApiNode, start):
    n = make(ApiNode, start, POI)
    n.SP_VIA_ON = True                      # 기본은 꺼 둠 (2026-11-23 실주행 결과)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n._np_pre_pose = lambda q, d_max=None, d_min=None: (
        q["x"] - d_max * math.cos(q["yaw"]), q["y"] - d_max * math.sin(q["yaw"]), q["yaw"], d_max)
    n.sent = []
    n.nav_goto = lambda x, y, yaw, **k: (n.sent.append((x, y, yaw)) or (True, ""))
    return n


def _path(deg):
    a = math.radians(deg)
    return [(3.0 - 2.0 * math.cos(a) * t, -2.0 * math.sin(a) * t) for t in (1.0, 0.75, 0.5, 0.25, 0.0)]


def test_far_along_axis_goes_via_point(ApiNode):
    n = _node(ApiNode, (-1.0, 0.3, 0.0))
    ok, _ = n._sp_decide(POI, _path(8.0))
    assert ok and n._np["state"] == "via"
    x, y, yaw = n.sent[0]
    assert abs(x - (3.0 - 0.6 - n.SP_VIA_M)) < 1e-6 and abs(y) < 1e-6      # 앞 자리보다 0.6m 바깥, 축 위


def test_no_via_when_robot_inside(ApiNode):
    """로봇이 경유점보다 안쪽(옆)에 있으면 돌아가지 않는다."""
    n = _node(ApiNode, (2.0, 1.5, 0.0))
    ok, _ = n._sp_decide(POI, _path(10.0))
    assert ok and n._np["state"] == "pre"
    assert abs(n.sent[0][0] - 2.4) < 1e-6


def test_no_via_when_path_comes_from_side(ApiNode):
    n = _node(ApiNode, (-1.0, 0.0, 0.0))
    ok, _ = n._sp_decide(POI, _path(30.0))
    assert ok and n._np.get("via") is None


def test_switch_to_pre_without_stopping(ApiNode):
    n = _node(ApiNode, (-1.0, 0.3, 0.0))
    n._sp_decide(POI, _path(8.0))
    n._slow_timer = None
    n.nav_state = "active"
    goals = []
    n._nav_send = lambda g, seq: goals.append((g, seq))
    n._nav_goal_msg = lambda x, y, yaw: (x, y, yaw)
    seq0 = n.nav_seq
    n._sim.update(x=1.45, y=0.02, yaw=math.radians(-2))     # 경유점(1.8) 0.4m 안
    assert not n.np_handoff()
    assert n._np["state"] == "pre" and n.nav_seq == seq0 + 1
    assert goals and abs(goals[0][0][0] - 2.4) < 1e-6 and goals[0][1] == n.nav_seq
    assert n.nav_target[:2] == (2.4, 0.0)
