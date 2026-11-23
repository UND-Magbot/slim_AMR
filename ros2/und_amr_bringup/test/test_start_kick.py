"""제자리 회전 뒤 출발 비틀림 미리 받기 (2026-11-19).

POI 빠져나오기 76회: 180도 제자리 회전 뒤 서 있다 출발해도 0.2~0.9초에 차체가 직전 회전 방향으로
약 0.05 rad/s 더 돌았다(방향 +1.6도까지, 이어서 ±2도 흔들림).
"""
import math
import threading
import types

from test_narrow_poi import ApiNode, make   # noqa: F401  (fixture 재사용)


def _odom(v, w):
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=w))))


def _feed(n, v, w, secs, hz=20.0):
    import time as _t
    for _ in range(int(secs * hz)):
        n._run_t = _t.monotonic() - 1.0 / hz
        n._on_odom(_odom(v, w))


def _spin_node(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n._sess_travel = lambda m: None
    return n


def test_spin_sign_tracks_last_in_place_turn(ApiNode):
    n = _spin_node(ApiNode)
    _feed(n, 0.0, -0.5, 1.0)
    assert n._spin_sign == -1
    _feed(n, 0.0, +0.3, 0.05)                    # 2도 안 되는 반대 보정은 무시
    assert n._spin_sign == -1
    _feed(n, 0.0, 0.0, 4.0)                      # 서 있어도 남는다
    assert n._spin_sign == -1
    _feed(n, 0.05, 0.0, 1.0)                     # 5cm: 아직
    assert n._spin_sign == -1
    _feed(n, 0.05, 0.0, 1.2)                     # 0.11m 움직이면 지움
    assert n._spin_sign == 0
    _feed(n, 0.0, +0.5, 1.0)
    assert n._spin_sign == 1


def _out(ApiNode, spin, kick_w=None, dist=0.05, dist2=0.0, kick2=None):
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [dict(poi, kind="point")]
    n._spin_sign = spin
    if kick_w is not None:
        n.SP_KICK_W = kick_w
    if kick2 is not None:
        n.SP_KICK2_W = kick2
    plan = n.sp_out_plan(5.0, 0.0)
    n.sp_out(plan, lambda: None)
    # 모의: 지령은 0.3초 늦게 들어가고, 출발 0.2~0.9초에 직전 회전 방향으로 dist rad/s 가 더해진다
    dt = 1.0 / n.NP_HZ
    q, t, peak, ws = [], 0.0, 0.0, []
    s = n._sim
    while getattr(n, "_np_timer", None) is not None and t < 20.0:
        n._np_tick()
        ws.append(s["w"])
        q.append((s["v"], s["w"]))
        v, w = q.pop(0) if len(q) > 6 else (0.0, 0.0)
        if 0.2 <= t < 0.9:
            w += spin * dist
        if 0.07 <= abs(s["x"] - 2.0) < 0.20:        # 둘째 비틀림: 간 거리에 묶여 반대쪽
            w -= spin * dist2
        s["yaw"] += w * dt
        s["x"] += v * math.cos(s["yaw"]) * dt
        s["y"] += v * math.sin(s["yaw"]) * dt
        peak = max(peak, abs(s["yaw"]))
        t += dt
    return n, math.degrees(peak), ws


def test_kick_feedforward_cancels_start_twist(ApiNode):
    _, off, _ = _out(ApiNode, -1, kick_w=0.0, kick2=0.0)
    n, on, ws = _out(ApiNode, -1, kick2=0.0)
    assert n._np_kick == 1                       # 오른쪽으로 돌았으니 왼쪽으로 미리
    assert ws[2] > 0.03                          # 첫 지령부터 들어간다
    assert on < 0.5 * off, (on, off)


def test_no_kick_without_recent_spin(ApiNode):
    n, _, ws = _out(ApiNode, 0, dist=0.0)
    assert n._np_kick == 0 and max(abs(w) for w in ws) < 1e-6


def test_second_kick_feedforward(ApiNode):
    _, off, _ = _out(ApiNode, -1, dist2=0.045, kick2=0.0)
    _, on, _ = _out(ApiNode, -1, dist2=0.045)
    assert on < 0.6 * off, (on, off)


def test_both_kicks_together(ApiNode):
    _, off, _ = _out(ApiNode, +1, kick_w=0.0, kick2=0.0, dist2=0.045)
    _, on, _ = _out(ApiNode, +1, dist2=0.045)
    assert on < 0.5 * off, (on, off)


def test_speed_ramps_after_hold_instead_of_jumping(ApiNode):
    """붙잡기(0.06m)가 끝난 뒤 0.05 -> 0.20 이 한 번에 뛰지 않는다."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [dict(poi, kind="point")]
    n.SP_RAMP_A = 0.15                             # 기본은 꺼 둠 (효과 없음) — 기능만 확인
    n.sp_out(n.sp_out_plan(5.0, 0.0), lambda: None)
    dt, vs, s = 1.0 / n.NP_HZ, [], n._sim
    while getattr(n, "_np_timer", None) is not None and len(vs) < 400:
        n._np_tick()
        vs.append(s["v"])
        s["x"] += s["v"] * dt
    jumps = [b - a for a, b in zip(vs, vs[1:])]
    assert max(jumps) < 0.15 * dt + 1e-6 + 0.01, max(jumps)
    assert max(vs) > 0.19                          # 설정 속도까지는 간다



def _end_w(ApiNode, look_min):
    """빠져나오기 끝 10cm 에서 좌우 2mm 가 남아 있을 때 내는 조향 최대값."""
    keep = ApiNode.SP_OUT_LOOK_MIN
    ApiNode.SP_OUT_LOOK_MIN = look_min
    try:
        poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
        n = make(ApiNode, (2.0, 0.0, 0.0), poi)
        n.lock = threading.Lock()
        n.map_png, n.map_meta = None, {}
        n.pois = [dict(poi, kind="point")]
        n.sp_out(n.sp_out_plan(5.0, 0.0), lambda: None)
        dt, s, wmax = 1.0 / n.NP_HZ, n._sim, 0.0
        while getattr(n, "_np_timer", None) is not None:
            end = 2.0 + n.SP_OUT_M_FWD - 0.10      # 빠져나오기 끝 10cm
            if s["x"] > end:
                s["y"] = 0.002                     # 실제처럼 좌우 2mm 가 남아 있다
            n._np_tick()
            if s["x"] > end:
                wmax = max(wmax, abs(s["w"]))
            s["x"] += s["v"] * dt
        return wmax
    finally:
        ApiNode.SP_OUT_LOOK_MIN = keep


def test_open_exit_does_not_bend_at_end_for_mm_lateral(ApiNode):
    """빠져나오기 끝에서 좌우 몇 mm 를 지우려고 크게 꺾지 않는다 (2026-11-20, 실측 끝 -0.9도)."""
    w0, w4 = _end_w(ApiNode, 0.0), _end_w(ApiNode, 0.40)
    assert w4 < 0.5 * w0, (w0, w4)
