"""렉 없을 때 POI 출발(후진 빠져나오기 + 큰 회전) 단축 (2026-11-20, 사용자: 8초 이하)."""
import math
import threading

from test_narrow_poi import ApiNode, make   # noqa: F401  (fixture 재사용)


def _node(ApiNode, lift):
    poi = {"name": "1", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [dict(poi, kind="point")]
    n.lift = lift
    n.SP_OUT_FWD_ONLY = False          # 아래 '뒤로 빠져나오기' 시험은 앞이 막혀 뒤로 나오는 경우의 거리 규칙
    return n


def test_goal_behind_still_exits_forward_short(ApiNode):
    n = _node(ApiNode, "bottom")
    n.SP_OUT_FWD_ONLY = True
    p, d, dist = n.sp_out_plan(-3.0, 0.0)                                # 목적지가 뒤
    assert d == 1 and dist == ApiNode.SP_OUT_M_FWD


def test_reverse_exit_is_short_without_rack(ApiNode):
    p, d, dist = _node(ApiNode, "bottom").sp_out_plan(-3.0, 0.0)       # 목적지가 뒤
    assert d == -1 and dist == ApiNode.SP_OUT_M_NORACK
    p, d, dist = _node(ApiNode, "top").sp_out_plan(-3.0, 0.0)          # 렉을 들었을 수 있음
    assert d == -1 and dist == ApiNode.SP_OUT_M
    p, d, dist = _node(ApiNode, None).sp_out_plan(-3.0, 0.0)           # 모름
    assert dist == ApiNode.SP_OUT_M
    p, d, dist = _node(ApiNode, "bottom").sp_out_plan(5.0, 0.0)        # 앞으로 빠져나오기는 그대로
    assert d == 1 and dist == ApiNode.SP_OUT_M


def _run_exit(n, goal):
    n.sp_out(n.sp_out_plan(*goal), lambda: None)
    dt, s, t = 1.0 / n.NP_HZ, n._sim, 0.0
    while getattr(n, "_np_timer", None) is not None and t < 20.0:
        n._np_tick()
        s["x"] += s["v"] * dt
        t += dt
    return t, s["x"]


def test_reverse_exit_time_and_position(ApiNode):
    t_old, x_old = _run_exit(_node(ApiNode, "top"), (-3.0, 0.0))
    t_new, x_new = _run_exit(_node(ApiNode, "bottom"), (-3.0, 0.0))
    assert abs(x_new - (2.0 - ApiNode.SP_OUT_M_NORACK)) < 0.015
    assert abs(x_old - (2.0 - ApiNode.SP_OUT_M)) < 0.015
    assert t_new < t_old - 0.8, (t_old, t_new)


def _turn(ApiNode, lift, err0_deg, act=None):
    import types
    n = _node(ApiNode, lift)
    n.NAV_WATCH_HZ = 20.0
    n.cdock = None
    n._depart_phase, n._depart_t = "turn", 0.0
    n._depart_turn_dir = 1.0
    n._depart_goal_xy = (2.0 + math.cos(math.radians(err0_deg)), math.sin(math.radians(err0_deg)))
    n._obstacle_pts = lambda: None
    n._sweep_hits = lambda pts, a: 0
    w_act = act
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=w_act or 0.0))))
    return n


def test_big_turn_keeps_min_speed_and_cuts_on_coast(ApiNode):
    n = _turn(ApiNode, "bottom", 170.0)
    n.DEPART_END_HOLD = True                                # 기본은 꺼 둠 (4차 시험) — 기능만 확인
    n._depart_turn_tick()                                   # 시작: 큰 회전
    n._sim["yaw"] = math.radians(170.0 - 8.0)                 # 8도 남음
    n._depart_t = 3.0
    n._depart_turn_tick()
    assert n._depart_phase == "turn" and abs(n._sim["w"]) >= ApiNode.DEPART_W_END - 1e-6
    n._sim["yaw"] = math.radians(170.0 - 3.0)                 # 3도 남음, 지령 0.30 x 0.20초 = 3.4도 더 돈다
    n.odom.twist.twist.angular.z = 0.30
    n._depart_turn_tick()
    assert n._depart_phase == "turn_settle"


def test_small_correction_uses_old_slow_finish(ApiNode):
    n = _turn(ApiNode, "bottom", 4.0)                        # 지나쳐서 4도만 되돌리기
    n._depart_turn_tick()
    assert n._depart_phase == "turn"                         # 바로 끊지 않고 돈다
    assert abs(n._sim["w"]) < ApiNode.DEPART_W_END
