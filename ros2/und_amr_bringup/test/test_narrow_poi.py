"""좁은 곳 POI 곧게 들어가기 · 후진으로 나오기 모의 (2026-11-16)."""
import math
import types

import pytest


@pytest.fixture
def ApiNode():
    from und_amr_bringup.api_node import ApiNode
    return ApiNode


def make(ApiNode, start, poi):
    n = ApiNode.__new__(ApiNode)
    n.estop, n.control_mode, n._scan_dead = False, "auto", False
    sim = {"x": start[0], "y": start[1], "yaw": start[2], "v": 0.0, "w": 0.0}
    n._sim = sim
    n.tf_pose = lambda target="map", source="base_footprint": (
        {"x": 0.0, "y": 0.0, "yaw": 0.0} if source == "odom"
        else {"x": sim["x"], "y": sim["y"], "yaw": sim["yaw"]})
    n.create_timer = lambda period, cb: object()
    n.destroy_timer = lambda t: None
    n.pub_twist_free = types.SimpleNamespace(
        publish=lambda t: sim.update(v=t.linear.x, w=t.angular.z))
    n._pub_zero = lambda: sim.update(v=0.0, w=0.0)
    n.limits = []
    n._pub_speed_limit = lambda v: n.limits.append(v)
    n._obstacle_pts = lambda: None
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.odom = None
    n.nav_seq = 1
    n.done = []
    n._finish_arrival = lambda note=None: n.done.append(note)
    n._np = {"state": "pre", "poi": dict(poi), "seq": 1}
    return n


def run(n, max_t=40.0):
    dt = 1.0 / n.NP_HZ
    t = 0.0
    while getattr(n, "_np_timer", None) is not None and t < max_t:
        n._np_tick()
        s = n._sim
        s["yaw"] += s["w"] * dt
        s["x"] += s["v"] * math.cos(s["yaw"]) * dt
        s["y"] += s["v"] * math.sin(s["yaw"]) * dt
        t += dt
    return t


def test_goes_straight_in_along_axis_from_offset_start(ApiNode):
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.0, 0.04, math.radians(3)), poi)   # 앞 자리에서 좌우 4cm, 3도 틀어짐
    assert n.np_on_arrival()
    run(n)
    s = n._sim
    assert n.done == ["좁은 곳 곧게 진입"]
    assert abs(s["x"] - 2.0) < 0.02 and abs(s["y"]) < 0.01 and abs(math.degrees(s["yaw"])) < 1.5
    assert n._np_last["name"] == "좁은"


def test_refuses_to_enter_when_heading_far_off(ApiNode):
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.0, 0.0, math.radians(35)), poi)
    assert n.np_on_arrival()
    assert n.nav_state == "failed" and getattr(n, "_np_timer", None) is None


def test_backs_out_along_axis_then_calls_after(ApiNode):
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.01, math.radians(-1)), poi)
    n._np_last = dict(poi)
    n._np_pre_pose = lambda p: (1.0, 0.0, 0.0, 1.0)
    assert n.np_back_out_needed()["name"] == "좁은"
    called = []
    n.np_back_out(poi, lambda: called.append(1))
    run(n)
    s = n._sim
    assert called == [1]
    assert abs(s["x"] - 1.0) < 0.02 and abs(s["y"]) < 0.01


def test_waits_when_front_blocked(ApiNode):
    import numpy as np
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.0, 0.0, 0.0), poi)
    n._obstacle_pts = lambda: np.array([[0.55, 0.0]] * 5)   # 차체 앞 11cm
    assert n.np_on_arrival()
    for _ in range(40):
        n._np_tick()
    assert n._sim["v"] == 0.0 and "막힘" in n.nav_msg and n.done == []


def test_picks_entry_side_nearest_robot(ApiNode):
    # 2026-11-16: 반대편에서 오면 POI 를 지나쳐 갔다 돌아왔다 -> 가까운 쪽 앞 자리
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": math.pi, "narrow": True}   # 도착 방향 = -x
    n = make(ApiNode, (0.0, 0.0, 0.0), poi)                  # 로봇은 서쪽(-x 쪽)에서 온다
    n._np_pre_pose = lambda q: (q["x"] - math.cos(q["yaw"]), q["y"] - math.sin(q["yaw"]), q["yaw"], 1.0)
    sent = []
    n.nav_goto = lambda x, y, yaw, **k: (sent.append((x, y, yaw)) or (True, ""))
    assert n.np_goto(poi)
    x, y, yaw = sent[0]
    assert abs(x - 1.0) < 1e-6 and abs(yaw) < 1e-6         # 서쪽 앞 자리, 동쪽을 보고 들어감


def test_reverse_entry_hands_over_to_arrival_turn(ApiNode):
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": math.pi}
    n = make(ApiNode, (1.0, 0.0, 0.0), poi)
    n._np["entry"] = dict(poi, yaw=0.0)                      # 반대로 들어간다
    n._np_lidar_pts = lambda: None                           # 사전 판단은 '모름' -> 들어간다
    turns = []
    n._final_turn = lambda: (turns.append(1) or True)
    assert n.np_on_arrival()
    run(n)
    assert turns == [1] and n.done == []                     # 도착 회전(공간 검사)으로 넘김
    assert n.nav_target == (2.0, 0.0, math.pi)


def test_goes_to_far_side_when_no_room_to_turn_inside(ApiNode):
    import numpy as np
    # 폭 0.92m 통로(벽 y=±0.46, 차체 외접 0.467 보다 좁음) - 180도 돌 공간 없음 -> 반대편 앞 자리로.
    poi = {"name": "좁은", "x": 2.0, "y": 0.0, "yaw": math.pi}
    n = make(ApiNode, (1.0, 0.0, 0.0), poi)
    n._np["entry"] = dict(poi, yaw=0.0)
    xs = np.linspace(-1.0, 1.4, 120)
    walls = np.r_[np.c_[xs, np.full_like(xs, 0.46)], np.c_[xs, np.full_like(xs, -0.46)]]
    n._obstacle_pts = lambda: walls          # 로봇(1,0,0) 기준 = 세계 기준 -1m
    n._np_lidar_pts = lambda: walls          # 입구 사전 판단은 라이다만 본다
    n._np_pre_pose = lambda q: (q["x"] - math.cos(q["yaw"]), q["y"] - math.sin(q["yaw"]), q["yaw"], 1.0)
    sent = []
    n.nav_goto = lambda x, y, yaw, **k: (sent.append((x, y, yaw)) or (True, ""))
    assert n.np_on_arrival()
    assert sent and abs(sent[0][0] - 3.0) < 1e-6 and abs(abs(sent[0][2]) - math.pi) < 1e-6
    assert n._np["entry"]["yaw"] == poi["yaw"]


# ---------------- 일반 POI 직선 도착 · 직선 출발 (2026-11-16 SP_*) ----------------

def test_open_poi_goes_straight_to_point_then_turns(ApiNode):
    """앞 자리에서 좌우 3cm · 2도 틀어져도 끝점을 바라보는 직선으로 들어가고, 방향만 제자리에서."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.4, 0.03, math.radians(2)), poi)
    n._np["mode"] = "open"
    turns = []
    n._final_turn = lambda: (turns.append(1) or False)      # 방향 오차가 작아 돌 것 없음
    yaws = []
    assert n.np_on_arrival()
    dt = 1.0 / n.NP_HZ
    t = 0.0
    while getattr(n, "_np_timer", None) is not None and t < 20:
        n._np_tick()
        s = n._sim
        s["yaw"] += s["w"] * dt
        s["x"] += s["v"] * math.cos(s["yaw"]) * dt
        s["y"] += s["v"] * math.sin(s["yaw"]) * dt
        yaws.append(math.degrees(s["yaw"]))
        t += dt
    s = n._sim
    assert turns == [1] and n.done == ["직선 도착"]
    assert abs(s["x"] - 2.0) < 0.02 and abs(s["y"]) < 0.005
    assert max(abs(y) for y in yaws) < 5.0                  # 휘청 없이 직선
    assert t < 6.0                                          # 0.6m 를 0.20 m/s
    assert n._np_last["_mode"] == "open"


@pytest.mark.parametrize("approach_deg,want_len,flip", [
    (0.0, 0.6, False),      # 경로가 축을 따라 들어온다 -> 0.6m 직선
    (180.0, 0.6, True),     # 반대쪽에서 축을 따라 -> 반대로 들어가 그 자리에서 돎
    (37.5, 0.3, False),     # 비스듬히 -> 반
    (90.0, 0.0, False),     # 옆에서 -> 직선 없이 POI 로 바로 (지나쳐 오지 않음)
])
def test_line_length_follows_path_approach_direction(ApiNode, approach_deg, want_len, flip):
    """2026-11-17 판정 없이: 경로가 POI 로 들어오는 방향으로 직선 길이를 정한다."""
    poi = {"name": "일반", "x": 3.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (-3.0, 0.0, 0.0), poi)
    n.SP_VIA_ON = False                     # 경유점은 test_sp_via.py
    n._np_pre_pose = lambda q, d_max=None, d_min=None: (
        q["x"] - d_max * math.cos(q["yaw"]), q["y"] - d_max * math.sin(q["yaw"]), q["yaw"], d_max)
    sent = []
    n.nav_goto = lambda x, y, yaw, **k: (sent.append((x, y, yaw, k.get("poi"))) or (True, ""))
    a = math.radians(approach_deg)
    path = [(3.0 - 2.0 * math.cos(a) * t, 0.0 - 2.0 * math.sin(a) * t) for t in (1.0, 0.75, 0.5, 0.25, 0.0)]
    ok, _ = n._sp_decide(poi, path)
    assert ok and len(sent) == 1
    x, y, yaw, direct = sent[0]
    if want_len == 0.0:
        assert direct is not None and (x, y) == (3.0, 0.0)          # POI 로 바로
    else:
        assert direct is None
        d = math.hypot(x - 3.0, y)
        assert abs(d - want_len) < 1e-6
        assert abs(abs(math.degrees(yaw)) - (180.0 if flip else 0.0)) < 1e-6
        assert n._np["mode"] == "open" and n._nav_precise is False


def test_no_auto_narrow_detection(ApiNode):
    """2026-11-17 넓은 곳이 지도 판정으로 좁은 곳이 돼 1m 앞으로 돌아갔다 -> 웹 표시만."""
    poi = {"name": "일반", "x": 3.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (0.0, 0.0, 0.0), poi)
    n._np_is_narrow = lambda p: True
    assert not n.np_goto(poi)


def test_open_poi_close_by_uses_normal_nav(ApiNode):
    poi = {"name": "일반", "x": 0.8, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (0.0, 0.0, 0.0), poi)
    n._np_is_narrow = lambda p: False
    assert not n.np_goto(poi)


@pytest.mark.parametrize("goal,want_dir", [((5.0, 0.5), +1), ((-5.0, 0.0), +1)])
def test_depart_from_open_poi_goes_straight_out_first(ApiNode, goal, want_dir):
    """목적지가 앞이든 뒤든 앞으로 SP_OUT_M_FWD 빠져나온 뒤 after (회전 -> nav2) — 2026-11-23 사용자 '뒤로는 별로'."""
    import threading
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.01, math.radians(1)), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [dict(poi, kind="point")]
    plan = n.sp_out_plan(*goal)
    assert plan is not None and plan[1] == want_dir
    called = []
    n.sp_out(plan, lambda: called.append(1))
    run(n)
    s = n._sim
    assert called == [1]
    assert abs(s["x"] - (2.0 + want_dir * n.SP_OUT_M_FWD)) < 0.02 and abs(s["y"] - 0.01) < 0.01
    assert n._np_last is None                               # 한 번만


def test_depart_straight_out_takes_other_way_if_blocked(ApiNode):
    import threading
    import numpy as np
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [dict(poi, kind="point")]
    n._obstacle_pts = lambda: np.array([[0.55, 0.0]] * 5)  # 바로 앞이 막힘
    plan = n.sp_out_plan(5.0, 0.0)
    assert plan is not None and plan[1] == -1


def test_no_straight_out_when_not_at_a_poi(ApiNode):
    import threading
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0, "kind": "point"}
    n = make(ApiNode, (2.5, 0.0, 0.0), poi)                 # POI 에서 50cm 벗어남
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [poi]
    assert n.sp_out_plan(5.0, 0.0) is None
    n._sim.update(x=2.0)
    assert n.sp_out_plan(5.0, 0.0) is not None               # 좁은 곳 표시와 상관없이 POI 위면 빠져나옴


def test_extend_straight_out_until_room(ApiNode):
    import threading
    n = make(ApiNode, (2.5, 0.0, 0.0), {"name": "x", "x": 2.0, "y": 0.0, "yaw": 0.0})
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n._sp_out_dir, n._sp_out_total = -1, 0.5
    p, d, step = n.sp_out_extend_plan()
    assert d == -1 and abs(step - 0.3) < 1e-9
    n._sp_out_total = 1.5
    assert n.sp_out_extend_plan() is None


def test_handoff_without_stopping_when_aligned(ApiNode):
    """nav2 가 앞 자리 근처에 축과 맞게 오면 서지 않고 그 속도로 직선 들어가기 (2026-11-16)."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.15, 0.02, math.radians(5)), poi)   # 앞 자리(1.4) 25cm 전
    n._np.update(mode="open", entry=dict(poi), pre=(1.4, 0.0, 0.0, 0.6))
    cancels = []
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: cancels.append(1))
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.15), angular=types.SimpleNamespace(z=0.0))))
    n._final_turn = lambda: False
    n._sim["v"] = 0.15
    assert n.np_handoff()
    assert cancels == [1] and n.nav_seq == 2 and n._np["seq"] == 2
    assert not n.np_handoff() and cancels == [1]          # 한 번만 (2026-11-17 18,235번 되풀이)
    n._np_tick()
    assert n._sim["v"] >= 0.15                              # 서지 않는다
    n.odom = None
    run(n)
    assert n.done == ["직선 도착"] and abs(n._sim["x"] - 2.0) < 0.02 and abs(n._sim["y"]) < 0.006


@pytest.mark.parametrize("start", [(1.15, 0.08, 0.0), (1.15, 0.0, math.radians(30)), (0.5, 0.0, 0.0),
                                   (1.15, 0.05, math.radians(22)), (1.15, -0.04, math.radians(-25))])
def test_no_handoff_when_not_aligned_or_far(ApiNode, start):
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, start, poi)
    n._np.update(mode="open", entry=dict(poi), pre=(1.4, 0.0, 0.0, 0.6))
    assert not n.np_handoff()


def test_straight_out_keeps_pending_entry_plan(ApiNode):
    """2026-11-17 빠져나오기가 다음 POI 의 직선 진입 계획(self._np)을 지워 앞 자리에서 멈췄다."""
    import threading
    poi = {"name": "1", "x": 2.0, "y": 0.0, "yaw": 0.0, "kind": "point"}
    n = make(ApiNode, (2.0, 0.0, 0.0), poi)
    n.lock = threading.Lock()
    n.map_png, n.map_meta = None, {}
    n.pois = [poi]
    plan = n.sp_out_plan(5.0, 0.0)
    pending = {"state": "pre", "poi": {"name": "2"}, "seq": 1, "mode": "open"}
    n.sp_out(plan, lambda: None)
    n._np = pending                                       # _sp_decide 가 nav_goto 뒤에 단다
    run(n)
    assert n._np is pending


@pytest.mark.parametrize("deg,lat", [(-15.0, 0.05), (16.0, -0.05), (-12.0, 0.03), (-8.0, 0.0)])
def test_handoff_with_large_heading_curves_in_smoothly(ApiNode, deg, lat):
    """2026-11-17 앞 자리에서 서서 20~31도 돌던 것 -> 25도까지 달리며 넘겨받아 완만하게 축으로."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.15, lat, math.radians(deg)), poi)
    n._np.update(mode="open", entry=dict(poi), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.15), angular=types.SimpleNamespace(z=0.0))))
    n._final_turn = lambda: False
    assert n.np_handoff()
    n.odom = None
    dt = 1.0 / n.NP_HZ
    w_act, t, yaws = 0.0, 0.0, []
    n._sim["v"] = 0.15
    while getattr(n, "_np_timer", None) is not None and t < 20:
        n._np_tick()
        s = n._sim
        w_act += (s["w"] - w_act) * dt / 0.3                 # 회전 반응 지연
        s["yaw"] += w_act * dt
        s["x"] += s["v"] * math.cos(s["yaw"]) * dt
        s["y"] += s["v"] * math.sin(s["yaw"]) * dt
        yaws.append(math.degrees(s["yaw"]))
        t += dt
    s = n._sim
    assert n.done == ["직선 도착"]
    assert abs(s["y"]) < 0.008 and abs(s["x"] - 2.0) < 0.02
    # 반대쪽으로 크게 넘어가지 않는다 (S자 없음)
    # 축으로 다가오던 쪽 반대로 크게 넘어가지 않는다 (S자 없음)
    over = max(-y for y in yaws) if deg > 0 else max(y for y in yaws)
    assert over < 6.0, over
    assert t < 8.0


def test_straight_entry_follows_map_when_slam_corrects(ApiNode):
    """2026-11-17 들어가는 동안 SLAM 보정(map->odom)이 옆으로 15mm 들어오면 예전에는 그만큼 비켜 섰다."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.4, 0.0, 0.0), poi)
    n._np["mode"] = "open"
    n._final_turn = lambda: False
    drift = {"y": 0.0}
    sim = n._sim

    def tf_pose(target="map", source="base_footprint"):
        if source == "odom":                                   # map -> odom
            return {"x": 0.0, "y": drift["y"], "yaw": 0.0}
        if target == "odom":                                   # odom 기준 로봇
            return {"x": sim["x"], "y": sim["y"], "yaw": sim["yaw"]}
        return {"x": sim["x"], "y": sim["y"] + drift["y"], "yaw": sim["yaw"]}
    n.tf_pose = tf_pose
    n._tf_nowait = lambda a, b, stamp=None: tf_pose(a, b) if b == "odom" else None
    assert n.np_on_arrival()
    dt = 1.0 / n.NP_HZ
    t = 0.0
    while getattr(n, "_np_timer", None) is not None and t < 20:
        drift["y"] = min(0.015, 0.015 * t / 4.0)              # 4초에 걸쳐 15mm
        n._np_tick()
        sim["yaw"] += sim["w"] * dt
        sim["x"] += sim["v"] * math.cos(sim["yaw"]) * dt
        sim["y"] += sim["v"] * math.sin(sim["yaw"]) * dt
        t += dt
    map_y = sim["y"] + drift["y"]
    assert n.done == ["직선 도착"]
    assert abs(map_y) < 0.008, map_y                          # 지도 기준으로 POI 에 선다 (다시 맞추기 없으면 15mm)


def test_handoff_reverse_entry_room_unknown_still_hands_off(ApiNode):
    """2026-11-17 반대 진입에서 회전 공간 '모름' 이면 서서 들어갈 때처럼 그대로 넘겨받는다 (돌 때 다시 검사)."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": math.pi}
    n = make(ApiNode, (1.15, 0.0, 0.0), poi)
    n._np.update(mode="open", entry=dict(poi, yaw=0.0), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    n._np_turn_room = lambda p, e: None
    assert n.np_handoff()
    n2 = make(ApiNode, (1.15, 0.0, 0.0), poi)
    n2._np.update(mode="open", entry=dict(poi, yaw=0.0), pre=(1.4, 0.0, 0.0, 0.6))
    n2.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    n2._np_turn_room = lambda p, e: False
    assert not n2.np_handoff()                                # 부족하면 서서 예전대로


def test_handoff_accepts_9cm_when_converging(ApiNode):
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.15, 0.09, math.radians(-12)), poi)   # 축 왼쪽 9cm, 축 쪽으로 다가옴
    n._np.update(mode="open", entry=dict(poi), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    assert n.np_handoff()


@pytest.mark.parametrize("deg,lat", [(0.0, 0.08), (5.0, 0.08), (-3.0, -0.07)])
def test_no_handoff_when_offset_and_not_converging(ApiNode, deg, lat):
    """옆으로 5cm 넘게 떨어져 나란히(또는 멀어지며) 오면 차선 바꾸기 S자 -> 서서 돌고 들어간다."""
    poi = {"name": "일반", "x": 2.0, "y": 0.0, "yaw": 0.0}
    n = make(ApiNode, (1.15, lat, math.radians(deg)), poi)
    n._np.update(mode="open", entry=dict(poi), pre=(1.4, 0.0, 0.0, 0.6))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    assert not n.np_handoff()
