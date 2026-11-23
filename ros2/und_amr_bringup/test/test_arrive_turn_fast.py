"""곧게 들어온 뒤 반대로 도는 도착 회전 단축 (2026-11-20).

1↔2 147회: 진입 끝 -> 도착 완료 평균 10.5초 (고정 대기 1.2 + 0.45 rad/s 177도 + 고정 대기 1.2).
"""
import math
import types

from test_narrow_poi import ApiNode, make   # noqa: F401  (fixture 재사용)


def _odom(v=0.0, w=0.0):
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=w))))


def _turn_node(ApiNode, mode="open", lift="bottom"):
    poi = {"name": "1", "x": 2.0, "y": 0.0, "yaw": math.pi}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.nav_target = (2.0, 0.0, math.pi)
    n.lift = lift
    n._np_mode = mode
    n._np_turning = True
    n._turning = True
    n._turn_phase, n._turn_t, n._turn_pass = "settle", 0.0, 0
    n._pos_pass = n.POS_PASS_MAX
    n._face_yaw = None
    n._turn_seq = None
    n.pub_twist = n.pub_twist_free
    n.odom = _odom()
    return n


def test_open_poi_without_rack_turns_faster(ApiNode):
    assert _turn_node(ApiNode)._turn_wmax() == ApiNode.TURN_W_OPEN_NORACK
    assert _turn_node(ApiNode, lift="top")._turn_wmax() == ApiNode.NP_TURN_W       # 렉
    assert _turn_node(ApiNode, lift=None)._turn_wmax() == ApiNode.NP_TURN_W        # 모름
    assert _turn_node(ApiNode, mode="narrow")._turn_wmax() == ApiNode.NP_TURN_W    # 좁은 곳
    n = _turn_node(ApiNode)
    n._np_turning = False
    assert n._turn_wmax() == ApiNode.TURN_W_MAX                                     # 일반 도착


def _ticks_until_spin(n, moving):
    dt = 1.0 / n.TURN_HZ
    k = 0
    while n._turn_phase == "settle" and k < 100:
        n.odom = _odom(w=0.05 if moving(k * dt) else 0.0)
        n._turn_tick()
        k += 1
    return k * dt


def test_settle_short_when_still(ApiNode):
    t = _ticks_until_spin(_turn_node(ApiNode), lambda t: False)
    assert t <= ApiNode.TURN_SETTLE_MIN_S + 0.11


def test_settle_waits_while_still_moving(ApiNode):
    t = _ticks_until_spin(_turn_node(ApiNode), lambda t: t < 0.7)
    assert 0.7 <= t <= 0.85
    t = _ticks_until_spin(_turn_node(ApiNode), lambda t: True)          # 안 멈추면 예전 한도
    assert abs(t - ApiNode.TURN_SETTLE_S) <= 0.11


def test_normal_arrival_keeps_full_settle(ApiNode):
    n = _turn_node(ApiNode)
    n._np_turning = False
    t = _ticks_until_spin(n, lambda t: False)
    assert t >= ApiNode.TURN_SETTLE_S - 0.01


def test_cancel_clears_np_turning(ApiNode):
    n = _turn_node(ApiNode)
    n._turn_timer = None
    n._turn_reset()
    assert n._np_turning is False
