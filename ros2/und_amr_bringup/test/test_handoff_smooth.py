"""POI 앞 자리 접근 감속 · 달리며 넘겨받은 직선의 감속/조향 제한 (2026-11-23, '휘청거림 없게')."""
import math
import types

import pytest

from test_narrow_poi import make, run


@pytest.fixture
def ApiNode():
    from und_amr_bringup.api_node import ApiNode
    return ApiNode


POI = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}


def _pre(n, x, y=0.0, yaw=0.0):
    n._sim.update(x=x, y=y, yaw=yaw)
    n._np.update(mode="open", entry=dict(POI), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)


def test_approach_limit_ramps_down_smoothly(ApiNode):
    n = make(ApiNode, (0.0, 0.0, 0.0), POI)
    n._slow_timer = None
    _pre(n, -0.5)                                  # 앞 자리에서 1.9m - 아직 제한 없음
    assert not n.np_handoff() and getattr(n, "_appr_lim", None) is None
    lims = []
    for f in [0.95, 0.75, 0.55, 0.35, 0.15, 0.05]:
        x = 1.4 - (n.SP_HANDOFF_M + f * n.SP_APPR_RAMP_M)     # 감속 구간 안
        _pre(n, x)
        assert not n.np_handoff()
        lims.append(n._appr_lim)
    assert all(a >= b for a, b in zip(lims, lims[1:]))           # 다가갈수록 줄기만
    assert lims[0] > lims[-1] + 0.03 and abs(lims[-1] - n.SP_APPR_V) < 0.01   # 끝은 넘겨받기 속도


def test_handoff_clears_limit(ApiNode):
    n = make(ApiNode, (0.0, 0.0, 0.0), POI)
    n._slow_timer = None
    _pre(n, 1.0)
    n.np_handoff()
    _pre(n, 1.15, 0.03, math.radians(-4))
    assert n.np_handoff()
    assert getattr(n, "_appr_lim", None) is None


def test_running_handoff_no_hard_brake_or_steer_jump(ApiNode):
    """0.5 m/s 로 축 왼쪽 8.7cm · -12.6도에서 넘겨받아도 감속 SP_DEC_A, 회전 변화 SP_W_ACC 안."""
    n = make(ApiNode, (1.15, 0.087, math.radians(-12.6)), POI)
    n._slow_timer = None
    n._np.update(mode="open", entry=dict(POI), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.5), angular=types.SimpleNamespace(z=0.0))))
    assert n.np_handoff()
    dt = 1.0 / n.NP_HZ
    vs, ws = [0.5], [0.0]
    for _ in range(200):
        if getattr(n, "_np_timer", None) is None:
            break
        n._np_tick()
        s = n._sim
        vs.append(s["v"]); ws.append(s["w"])
        s["yaw"] += s["w"] * dt
        s["x"] += s["v"] * math.cos(s["yaw"]) * dt
        s["y"] += s["v"] * math.sin(s["yaw"]) * dt
    # 끝 감속 구간 전까지의 감속은 SP_DEC_A 를 넘지 않는다
    big = [(a - b) / dt for a, b in zip(vs[:40], vs[1:41])]
    assert max(big) <= n.SP_DEC_A + 1e-6
    dw = [abs(b - a) / dt for a, b in zip(ws[:-1], ws[1:-1])]     # 마지막(끝에서 서며 0)은 뺀다
    assert max(dw) <= n.SP_W_ACC + 1e-6
    s = n._sim
    assert abs(s["y"]) < 0.015 and abs(math.degrees(s["yaw"])) < 3.0
