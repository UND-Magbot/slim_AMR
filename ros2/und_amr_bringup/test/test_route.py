"""지점 왕복(대시보드 A/B) 시험 (2026-09-29).

nav2 는 없으므로 poi_goto 를 가짜로 바꿔, 보낼 때마다 nav_seq 를 올리고
nav_state 를 'active' 로 둔다. 도착은 시험이 nav_state='done' 으로 흉내 낸다.
리프트(2026-10-30)는 _route_lift_send 를 가짜로 바꿔 lift 를 'up'/'down' 으로 두고,
finish_lift 가 펌웨어처럼 'top'/'bottom' 으로 끝낸다.
"""
import math
import time

import pytest


@pytest.fixture
def rt(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.estop = False
    n.control_mode = "auto"
    n.job = {"state": "idle"}
    n.nav_goal = None
    n.nav_state = "idle"
    n.nav_msg = ""
    n.nav_seq = 0
    n.pois = [{"name": "A지점", "kind": "point", "x": 0, "y": 0, "yaw": 0},
              {"name": "B지점", "kind": "point", "x": 1, "y": 0, "yaw": 0},
              {"name": "렉2", "kind": "cart", "x": 2, "y": 0, "yaw": 0}]
    import threading
    n.lock = threading.Lock()
    n.sent = []
    n.cancels = 0

    def goto(name):
        n.sent.append(name)
        n.nav_seq += 1
        n.nav_state = "active"
        return True, "ok"

    def cancel():
        n.cancels += 1
        n.nav_state = "canceled"
        return True, "취소"

    n.lift = "bottom"
    n.odom = None
    n.lifts = []
    n.lift_stops = 0

    def lift_send(cmd):
        n.lifts.append(cmd)
        n.lift = cmd                # 'up' / 'down' = 움직이는 중
        return True

    def lift_stop():
        n.lift_stops += 1

    n.poi_goto = goto
    n.nav_cancel = cancel
    n._route_lift_send = lift_send
    n.lift_stop_async = lift_stop
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None,
                                          "warn": lambda *a: None})()
    n.route = {"state": "idle"}
    return n


def finish_lift(n):
    """펌웨어가 정해진 시간 뒤 올림/내림을 끝낸 것처럼."""
    if n.lift in ("up", "down"):
        n.lift = "top" if n.lift == "up" else "bottom"


def settle(n, k=12):
    """다음 구간을 보내거나 왕복이 끝날 때까지 tick (리프트·대기 포함)."""
    n0 = len(n.sent)
    for _ in range(k):
        n._route_tick()
        if n.route.get("lift", {}).get("seen"):
            finish_lift(n)
        if n.route["state"] == "waiting":
            n.route["until"] = 0.0  # 대기 시간을 건너뛴다
        if len(n.sent) > n0 or n.route["state"] not in ("lifting", "waiting"):
            return


def arrive(n):
    n.nav_state = "done"
    settle(n)


def test_one_lap_is_a_b_a(rt):
    ok, _ = rt.route_start("A지점", "B지점", 1)
    assert ok and rt.sent == ["A지점"]
    arrive(rt)                      # A 도착 -> B 로
    assert rt.sent[-1] == "B지점"
    arrive(rt)                      # B 도착 -> A 로
    assert rt.sent[-1] == "A지점" and rt.route["lap"] == 0
    arrive(rt)                      # A 로 돌아옴 = 1회, 내린 뒤 끝
    assert rt.route["state"] == "done" and rt.route["lap"] == 1
    assert rt.sent == ["A지점", "B지점", "A지점"]
    # 지점마다 도착하면 내리고 출발 전에 올린다. 끝나면 내린 채로.
    assert rt.lifts == ["down", "up", "down", "up", "down"]
    assert rt.lift == "bottom"


def test_infinite_keeps_going(rt):
    rt.route_start("A지점", "B지점", 0)
    for _ in range(9):
        arrive(rt)
    assert rt.route["state"] == "going" and rt.route["lap"] == 4


def test_dwell_waits_before_next_leg(rt):
    rt.route_start("A지점", "B지점", 2, dwell=30)
    rt.nav_state = "done"
    rt._route_tick()                # 내림 명령
    rt._route_tick()                # 내리는 중을 봄
    finish_lift(rt)
    rt._route_tick()                # 다 내림 -> 대기
    assert rt.route["state"] == "waiting" and rt.sent == ["A지점"]
    rt._route_tick()                # 아직 30초 안 지남
    assert rt.sent == ["A지점"]
    assert rt.route_status()["remain_s"] > 25


def test_failure_retries_once_then_stops(rt):
    rt.route_start("A지점", "B지점", 1)
    rt.nav_state = "failed"
    rt._route_tick()
    assert rt.route["state"] == "retry"
    rt.route["until"] = 0.0
    rt._route_tick()
    assert rt.sent == ["A지점", "A지점"]
    rt.nav_state = "failed"
    rt._route_tick()
    assert rt.route["state"] == "failed"


def test_other_goal_or_manual_stops_route(rt):
    rt.route_start("A지점", "B지점", 0)
    rt.nav_seq += 1                 # 누군가 다른 목표를 보냄
    rt._route_tick()
    assert rt.route["state"] == "canceled" and rt.cancels == 0
    rt.route_start("A지점", "B지점", 0)
    rt.control_mode = "manual"
    rt._route_tick()
    assert rt.route["state"] == "canceled" and rt.cancels == 1


def test_rejects_cart_poi_and_same_point(rt):
    assert not rt.route_start("A지점", "렉2", 1)[0]
    assert not rt.route_start("A지점", "A지점", 1)[0]
    assert not rt.route_start("A지점", "B지점", -1)[0]
    rt.control_mode = "manual"
    assert not rt.route_start("A지점", "B지점", 1)[0]


def test_nav_cut_before_rpp_spins_counts_as_arrival(ApiNode):
    # 2026-09-29: 목표 6cm 안이면 nav2 를 끊고, 그 취소 결과를 도착으로 친다.
    n = ApiNode.__new__(ApiNode)
    n.nav_state, n.nav_seq, n.nav_target = "active", 7, (1.0, 2.0, 0.0)
    n._turning = False
    cancels = []
    n.nav_goal = type("G", (), {"cancel_goal_async": lambda self: cancels.append(1)})()
    n.pub_twist = type("P", (), {"publish": lambda self, m: None})()
    n.pub_twist_free = n.pub_twist
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n.tf_pose = lambda *a: {"x": 1.10, "y": 2.0, "yaw": 0.0}
    n._nav_watch()
    assert not cancels                       # 10cm — 아직
    n.tf_pose = lambda *a: {"x": 1.04, "y": 2.0, "yaw": 0.0}
    n._nav_watch()
    n._nav_watch()
    assert cancels == [1]                    # 한 번만 끊는다
    done = []
    n._final_turn = lambda: False
    n._finish_arrival = lambda *a: done.append(1)
    n._nav_clear_path = lambda: None
    res = type("R", (), {"status": 5})()
    fut = type("F", (), {"result": lambda self: res})()
    n._nav_done(fut, 7)
    assert done == [1]
    # 사용자가 취소한 것(표시 없음)은 도착이 아니다
    n.nav_seq, done[:] = 8, []
    n._nav_done(fut, 8)
    assert done == [] and n.nav_state == "canceled"


def test_turn_near_180_does_not_mistake_wrap_for_overshoot(ApiNode):
    # 2026-09-29: -179.5도를 돌기 시작해 살짝 되튕겨 +179.6도가 되어도 계속 돈다.
    import math
    n = ApiNode.__new__(ApiNode)
    sent = []
    n.pub_twist = type("P", (), {"publish": lambda self, m: sent.append(m.angular.z)})()
    n.pub_twist_free = n.pub_twist
    n.estop, n.control_mode, n._scan_dead = False, "auto", False
    n._turn_phase, n._turn_dir, n._turn_t = "spin", -1.0, 0.0
    n._yaw_err = lambda: math.radians(179.6)
    n._turn_tick()
    assert n._turn_phase == "spin" and sent[-1] < 0
    # 진짜로 조금 지나친 것은 멈춘다
    n._yaw_err = lambda: math.radians(3.0)
    n._turn_tick()
    assert n._turn_phase == "settle" and sent[-1] == 0



def test_creep_goes_straight_to_goal_then_settles(ApiNode):
    # 2026-09-29: nav2 를 끊은 자리(앞 90mm)에서 목표점까지 곧게 기어 들어간다.
    n = ApiNode.__new__(ApiNode)
    out = []
    n.pub_twist = type("P", (), {"publish": lambda self, m: out.append(("n", m.linear.x))})()
    n.pub_twist_free = type("P", (), {"publish": lambda self, m: out.append(("f", m.linear.x))})()
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n._exit_front_blocked = lambda: False
    n.estop, n.control_mode, n._scan_dead = False, "auto", False
    n.nav_target = (1.0, 0.0, 0.0)
    pos = {"x": 0.91}
    n.tf_pose = lambda *a: {"x": pos["x"], "y": 0.002, "yaw": 0.0}
    n._turn_phase, n._turn_t = "creep", 0.0
    n._turn_tick()
    assert out[-1][0] == "f" and 0.03 <= out[-1][1] <= 0.06
    pos["x"] = 0.997                         # 3mm 남음 (0.03 m/s 로 3.6mm 더 감) -> 멈춤
    n._turn_tick()
    assert n._turn_phase == "settle" and out[-1][1] == 0
    # 앞에 뭔가 있으면 기지 않는다
    n._turn_phase, n._turn_t, pos["x"] = "creep", 0.0, 0.91
    n._exit_front_blocked = lambda: True
    n._turn_tick()
    assert n._turn_phase == "settle"


def test_settle_checks_position_before_heading(ApiNode):
    # 2026-09-29: 멈춘 뒤 목표점이 옆으로 어긋나 있으면 그쪽으로 돌고 들어간다.
    import math
    n = ApiNode.__new__(ApiNode)
    n.pub_twist = n.pub_twist_free = type("P", (), {"publish": lambda self, m: None})()
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n.nav_target = (1.0, 0.0, math.pi)
    n.tf_pose = lambda *a: {"x": 0.97, "y": 0.03, "yaw": 0.0}
    n._goal_offset = lambda: (0.03, -0.03)
    n.estop, n.control_mode, n._scan_dead, n._nav_precise = False, "auto", False, True
    n._turn_phase, n._turn_t, n._pos_pass, n._face_yaw = "settle", 5.0, 0, None
    n._turn_tick()
    assert n._turn_phase == "spin" and n._turn_dir < 0
    assert abs(n._face_yaw - math.atan2(-0.03, 0.03)) < 1e-9
    # 목표점 쪽으로 다 돌면 곧게 들어간다
    n._turn_phase, n._turn_t = "settle", 5.0
    n._turn_tick()
    assert n._turn_phase == "creep" and n._face_yaw is None
    # 좌우가 맞았으면 바로 곧게
    n._goal_offset = lambda: (0.05, 0.004)
    n._turn_phase, n._turn_t = "settle", 5.0
    n._turn_tick()
    assert n._turn_phase == "creep"


def test_depart_creeps_then_hands_to_nav2(ApiNode):
    # 2026-09-29: 멈춰 있다 출발하면 0.25m 를 0.04 m/s 로 방향 붙잡고 간 뒤 nav2 로.
    import math
    n = ApiNode.__new__(ApiNode)
    out, sent = [], []
    n.pub_twist = n.pub_twist_free = type("P", (), {"publish": lambda self, m: out.append((m.linear.x, m.angular.z))})()
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n._exit_front_blocked = lambda: False
    n.estop, n.control_mode = False, "auto"
    n._gyro_z, n._gyro_t = 0.0, 0.0
    n._depart_timer = object()
    n.destroy_timer = lambda t: None
    n._nav_send = lambda g, seq: sent.append(seq)
    n._depart_goal, n._depart_seq = "goal", 3
    n._depart_xy, n._depart_yaw, n._depart_t, n._depart_w_last = (0.0, 0.0), 0.0, 0.0, 0.0
    pos = {"x": ApiNode.DEPART_M / 2, "yaw": math.radians(-3)}
    n.tf_pose = lambda *a: {"x": pos["x"], "y": 0.0, "yaw": pos["yaw"]}
    n._depart_tick()
    assert out[-1][0] == n.DEPART_V and out[-1][1] > 0   # 오른쪽으로 밀렸으면 왼쪽으로 붙잡는다
    assert not sent
    pos["x"] = ApiNode.DEPART_M + 0.01
    n._depart_tick()
    assert sent == [3] and n._depart_timer is None


def test_depart_only_when_still_and_goal_ahead(ApiNode):
    import math, threading
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n._exit_front_blocked = lambda: False
    n.tf_pose = lambda *a: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    tw = type("T", (), {})()
    tw.linear = type("V", (), {"x": 0.0})()
    tw.angular = type("V", (), {"z": 0.0})()
    n.odom = type("O", (), {"twist": type("TT", (), {"twist": tw})()})()
    assert n._depart_ok(2.0, 0.3) is not None           # 앞 (8.5도)
    assert n._depart_ok(0.0, 2.0) is None               # 옆
    assert n._depart_ok(0.5, 0.0) is None               # 너무 가까움
    tw.linear.x = 0.2
    assert n._depart_ok(2.0, 0.0) is None               # 이미 움직임



def _mk_turn_node(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.out = []
    n.pub_twist = n.pub_twist_free = type("P", (), {"publish": lambda self, m: n.out.append(m.linear.x)})()
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None, "warn": lambda *a: None})()
    n.estop, n.control_mode, n._scan_dead = False, "auto", False
    n.destroy_timer = lambda t: None
    return n


def test_cancel_after_early_cut_is_not_arrival(ApiNode):
    # 2026-09-29 검토: 10cm 앞에서 끊은 직후 [정지] -> 뒤늦은 취소 결과가 '도착' 이 되면 안 된다.
    n = _mk_turn_node(ApiNode)
    n.nav_state, n.nav_seq, n._nav_early_seq = "active", 5, 5
    n.nav_goal = type("G", (), {"cancel_goal_async": lambda self: None})()
    n._turning, n._depart_timer, n._turn_timer = False, None, None
    n._nav_clear_path = lambda: None
    n.nav_cancel()
    assert n.nav_state == "canceled" and n._nav_early_seq is None and n.nav_seq == 6
    done = []
    n._final_turn = lambda: done.append("turn") or True
    res = type("R", (), {"status": 5})()
    n._nav_done(type("F", (), {"result": lambda self: res})(), 5)
    assert done == [] and n.nav_state == "canceled"


def test_arrival_motion_stops_on_estop_and_new_goal(ApiNode):
    n = _mk_turn_node(ApiNode)
    n._turn_timer, n._turning, n.nav_seq, n._turn_seq = None, True, 3, 3
    n._turn_phase, n._turn_t = "creep", 0.0
    n.nav_state = "active"
    n.estop = True
    n._turn_tick()
    assert n.nav_state == "canceled" and n.out[-1] == 0 and not n._turning
    # 새 목표가 들어왔으면 상태를 건드리지 않고 조용히 끝낸다
    n.estop, n._turning, n.nav_state, n.nav_seq = False, True, "sending", 4
    n._turn_tick()
    assert n.nav_state == "sending" and not n._turning


def test_small_offset_is_not_worth_a_big_turn(ApiNode):
    # 12mm 남았는데 66도 돌았다 돌아오지 않는다. 지나친 것도 180도 돌지 않는다.
    import math
    n = _mk_turn_node(ApiNode)
    n.nav_target = (1.0, 0.0, 0.0)
    n.tf_pose = lambda *a: {"x": 0.995, "y": 0.011, "yaw": 0.0}
    n._nav_precise, n._pos_pass, n._face_yaw = True, 0, None
    n._turn_ok_n, n._turn_max_n, n._turn_pass = 0.035, 4, 0
    n._yaw_err = lambda: 0.0
    n._finish_arrival = lambda *a: None
    for off in ((0.005, -0.011), (-0.03, 0.002)):
        n._goal_offset = lambda off=off: off
        n._turn_phase, n._turn_t, n._face_yaw, n._turning = "settle", 5.0, None, True
        n._turn_tick()
        assert n._face_yaw is None and n._turn_phase != "spin"



def _mk_turn_depart(ApiNode, yaw_deg):
    # 2026-10-02: 목표가 등 뒤면 먼저 제자리에서 돌고, 멈춘 뒤 기어간다.
    import math
    n = ApiNode.__new__(ApiNode)
    n.out, n.sent, n.zeros = [], [], 0
    n.pub_twist = n.pub_twist_free = type("P", (), {"publish": lambda self, m: n.out.append((m.linear.x, m.angular.z))})()
    n._pub_zero = lambda: setattr(n, "zeros", n.zeros + 1)
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n._exit_front_blocked = lambda: False
    # 2026-11-16 회전 공간은 쓸고 가는 영역 검사(_sweep_hits)로 본다. clear 가 False 면 막힘.
    n._clear = True
    n._obstacle_pts = lambda: None
    n._sweep_hits = lambda pts, ang, dx=0.0: 0 if (n._clear() if callable(n._clear) else n._clear) else 99
    n.estop, n.control_mode = False, "auto"
    n._depart_timer = object()
    n.destroy_timer = lambda t: None
    n._nav_send = lambda g, seq: n.sent.append(seq)
    n._depart_goal, n._depart_seq = "goal", 5
    n._depart_goal_xy = (3.0, 0.0)                  # 목표는 +x 쪽
    n._depart_xy, n._depart_yaw, n._depart_t, n._depart_w_last = (0.0, 0.0), 0.0, 0.0, 0.0
    n._depart_turn_dir = 0.0
    n._depart_phase = "turn"
    n._depart_blocked_t = 0.0
    n.depart_mode = "creep"
    n.pose = {"x": 0.0, "y": 0.0, "yaw": math.radians(yaw_deg)}
    n.tf_pose = lambda *a: dict(n.pose)
    return n


def test_depart_turns_toward_goal_then_creeps(ApiNode):
    import math
    n = _mk_turn_depart(ApiNode, 150)          # 목표가 왼쪽 뒤 -> 오른쪽(시계)으로 도는 게 짧다
    n._depart_tick()
    assert n.out[-1][0] == 0.0 and n.out[-1][1] < 0          # 제자리, 시계 방향
    n.pose["yaw"] = math.radians(1.0)                         # 거의 다 돌았다
    n._depart_tick()
    assert n._depart_phase == "turn_settle" and n.zeros == 1
    n._depart_t = n.DEPART_TURN_SETTLE_S
    n._depart_tick()
    assert n._depart_phase == "creep" and not n.sent          # 이제 기어간다
    assert abs(n._depart_yaw - math.radians(1.0)) < 1e-9


def test_depart_turn_waits_then_hands_to_nav2_when_blocked(ApiNode):
    # 물체가 잠깐 보이면 무시, 계속 보이면 멈춰 기다리고, 5초 넘게 막히면 nav2 에.
    n = _mk_turn_depart(ApiNode, 170)
    n.NAV_WATCH_HZ = 20.0
    n._clear = False
    n._depart_tick()
    assert not n.sent and n._depart_timer is not None       # 한 번은 무시
    for _ in range(10):
        n._depart_tick()
    assert not n.sent and n.zeros >= 1                       # 멈춰 기다린다
    for k in range(100):                                     # 타이머가 멈추면 더 안 불린다
        if n._depart_timer is None:
            break
        n._depart_tick()
    assert n.sent == [5] and n._depart_timer is None
    assert k * 0.05 >= n.DEPART_BLOCK_WAIT_S - 0.6           # 약 5초 기다린 뒤


def test_depart_turn_resumes_after_clear(ApiNode):
    n = _mk_turn_depart(ApiNode, 170)
    n.NAV_WATCH_HZ = 20.0
    blocked = {"v": True}
    n._clear = lambda: not blocked["v"]
    for _ in range(20):
        n._depart_tick()
    assert not n.sent
    blocked["v"] = False
    n._depart_tick()
    assert n.out and n.out[-1][1] != 0.0 and not n.sent       # 다시 돈다


def _mk_slow(ApiNode):
    # 2026-10-02: 출발 'nav2' 방식 — nav2 가 조향, 처음 DEPART_M 은 속도 제한만.
    n = ApiNode.__new__(ApiNode)
    n.limits = []
    n._pub_speed_limit = lambda v: n.limits.append(v)
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n.destroy_timer = lambda t: None
    n.NAV_WATCH_HZ = 20.0
    n.od = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    n.tf_pose = lambda *a: dict(n.od)
    n.create_timer = lambda p, cb: object()
    n._slow_timer = None
    n.nav_seq, n._slow_seq, n.nav_state = 7, 7, "active"
    n._slow_start()
    return n


def test_slow_start_limits_then_releases_after_depart_m(ApiNode):
    n = _mk_slow(ApiNode)
    n.DEPART_SMOOTH = False                # 예전 방식 (평평한 저속 -> 직선으로 올림)
    assert n.limits[-1] == n.DEPART_SLOW_V and n._slow_timer is not None
    n.od["x"] = n.DEPART_M / 2
    n._slow_tick()
    assert n.limits[-1] == n.DEPART_SLOW_V
    # 2026-10-08: 저속 구간 뒤에는 한 번에 풀지 않고 DEPART_RAMP_M 동안 올린다
    n.nav_speed = 0.40
    n.lift = None
    n.od["x"] = n.DEPART_M + 0.001
    n._slow_tick()
    assert n.DEPART_SLOW_V <= n.limits[-1] < 0.05 and n._slow_timer is not None
    n.od["x"] = n.DEPART_M + n.DEPART_RAMP_M / 2
    n._slow_tick()
    assert abs(n.limits[-1] - (0.04 + 0.40) / 2) < 1e-6
    n.od["x"] = n.DEPART_M + n.DEPART_RAMP_M
    n._slow_tick()
    assert n.limits[-1] == 0.0 and n._slow_timer is None


def test_slow_releases_on_cancel_or_new_goal(ApiNode):
    n = _mk_slow(ApiNode)
    n.nav_state = "canceled"
    n._slow_tick()
    assert n.limits[-1] == 0.0 and n._slow_timer is None
    n = _mk_slow(ApiNode)
    n.nav_seq = 8
    n._slow_tick()
    assert n.limits[-1] == 0.0 and n._slow_timer is None


def test_depart_mode_switch(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None})()
    n.depart_mode = "creep"
    assert n.set_depart_mode("nav2")[0] and n.depart_mode == "nav2"
    assert not n.set_depart_mode("x")[0] and n.depart_mode == "nav2"


def test_nav2_mode_aligns_then_hands_to_nav2_slow(ApiNode):
    # 2026-10-02: nav2 방식 — 등 뒤 목표면 정렬만 하고, 전진부터 nav2 (저속 제한).
    import math
    n = _mk_turn_depart(ApiNode, 170)
    n.depart_mode = "nav2"
    n.slow = []
    n._slow_start = lambda: n.slow.append(1)
    n._depart_tick()                                  # 돈다
    n.pose["yaw"] = math.radians(0.5)
    n._depart_tick()                                  # 다 돌았다
    n._depart_t = n.DEPART_TURN_SETTLE_S
    n._depart_tick()                                  # 멈춘 뒤
    assert n.sent == [5] and n.slow == [1] and n._depart_timer is None
    assert n._slow_seq == 5



def test_slow_tf_lost_cancels_instead_of_release_only(ApiNode):
    # 저속 출발 중 위치를 잃으면 제한만 풀지 않고 이동을 취소·정지한다.
    n = _mk_slow(ApiNode)
    n.cancels, n.zeros = 0, 0
    n.nav_cancel = lambda: setattr(n, "cancels", n.cancels + 1)
    n._pub_zero = lambda: setattr(n, "zeros", n.zeros + 1)
    n.get_logger = lambda: type("L", (), {"info": lambda *a: None, "error": lambda *a: None})()
    n.tf_pose = lambda *a: None
    # 2026-11-23 한 번 못 읽은 것으로는 취소하지 않는다 — SLOW_LOST_S 동안 계속 못 읽으면 취소
    n._slow_tick()
    assert n.cancels == 0 and n.nav_state != "failed"
    for _ in range(int(n.SLOW_LOST_S * n.NAV_WATCH_HZ) + 1):
        n._slow_tick()
    assert n.cancels == 1 and n.zeros >= 1 and n.nav_state == "failed"
    assert n._slow_timer is None


def test_depart_turn_needs_real_alignment_not_just_wait(ApiNode):
    # 30도는 시작 기준. 멈춘 뒤 3도 넘게 남았으면 다시 돈다(기다렸다고 출발하지 않음).
    import math
    n = _mk_turn_depart(ApiNode, 170)
    n._depart_passes = 0
    n._depart_phase, n._depart_t = "turn_settle", n.DEPART_TURN_SETTLE_S
    n.pose["yaw"] = math.radians(8.0)                 # 8도 남음
    n._depart_tick()
    assert n._depart_phase == "turn" and not n.sent and n._depart_passes == 1
    n._depart_passes = n.DEPART_TURN_PASSES - 1
    n._depart_phase, n._depart_t = "turn_settle", n.DEPART_TURN_SETTLE_S
    n._depart_tick()                                  # 마지막 회차도 안 맞으면 nav2 에
    assert n.sent == [5] and n._depart_timer is None


def test_small_offset_fixed_when_final_turn_goes_that_way(ApiNode):
    # 2026-10-08: 끝에 어차피 170도를 같은 쪽으로 돈다면 좌우 11mm 도 돌고 들어가 맞춘다.
    # 반대쪽으로 돌아야 하면(더 돌게 되면) 예전처럼 둔다.
    import math
    n = _mk_turn_node(ApiNode)
    n.nav_target = (1.0, 0.0, 0.0)
    n.tf_pose = lambda *a: {"x": 0.997, "y": 0.011, "yaw": 0.0}
    n._nav_precise, n._pos_pass, n._face_yaw = True, 0, None
    n._turn_ok_n, n._turn_max_n, n._turn_pass = 0.035, 4, 0
    n._finish_arrival = lambda *a: None
    n._goal_offset = lambda: (0.003, -0.011)
    n._yaw_err = lambda: math.radians(-170)
    n._turn_phase, n._turn_t, n._turning = "settle", 5.0, True
    n._turn_tick()
    assert n._turn_phase == "spin" and n._turn_dir < 0 and n._face_yaw is not None
    n._yaw_err = lambda: math.radians(170)
    n._pos_pass, n._face_yaw = 0, None
    n._turn_phase, n._turn_t, n._turning = "settle", 5.0, True
    n._turn_tick()
    assert n._face_yaw is None


def test_six_to_ten_mm_fixed_when_turn_agrees(ApiNode):
    # 2026-10-08 오후: 같은 쪽으로 돌 때 좌우 8mm(앞 2mm)도 '곧게' 가 아니라 돌고 들어간다.
    # 마지막 회전이 없으면 10mm 아래는 그대로 둔다.
    import math
    n = _mk_turn_node(ApiNode)
    n.nav_target = (1.0, 0.0, 0.0)
    n.tf_pose = lambda *a: {"x": 0.998, "y": 0.008, "yaw": 0.0}
    n._nav_precise, n._pos_pass, n._face_yaw = True, 0, None
    n._turn_ok_n, n._turn_max_n, n._turn_pass = 0.035, 4, 0
    n._finish_arrival = lambda *a: None
    n._goal_offset = lambda: (0.002, -0.008)
    n._yaw_err = lambda: math.radians(-170)
    n._turn_phase, n._turn_t, n._turning = "settle", 5.0, True
    n._turn_tick()
    assert n._turn_phase == "spin" and n._face_yaw is not None
    n._yaw_err = lambda: 0.0
    n._pos_pass, n._face_yaw, n._turn_pass = 0, None, 0
    n._turn_phase, n._turn_t, n._turning = "settle", 5.0, True
    n._turn_tick()
    assert n._face_yaw is None and n._pos_pass == 0


def test_depart_params_set_live(ApiNode):
    n = _mk_slow(ApiNode)
    ok, _ = n.set_depart_params({"slow_m": 0.05, "ramp_m": 0.30})
    assert ok and n.DEPART_M == 0.05 and n.DEPART_RAMP_M == 0.30
    assert n.depart_params() == {"slow_m": 0.05, "ramp_m": 0.30, "slow_v": n.DEPART_SLOW_V}
    # 범위 밖이 하나라도 있으면 아무것도 안 바꾼다
    ok, _ = n.set_depart_params({"slow_m": 0.08, "ramp_m": 5.0})
    assert not ok and n.DEPART_M == 0.05
    assert not n.set_depart_params({})[0]
    assert not n.set_depart_params({"slow_v": "x"})[0]
    # 올리기 0 이면 저속 끝에서 바로 푼다 (0 으로 나누지 않고)
    n.set_depart_params({"ramp_m": 0.0})
    n.nav_speed, n.lift = 0.40, None
    n.od["x"] = 0.06
    n._slow_tick()
    assert n.limits[-1] == 0.0 and n._slow_timer is None


def test_lift_waits_for_move_then_end_not_stale_state(rt):
    """명령 전 상태(bottom)가 다시 와도 끝으로 보지 않는다 — 움직임을 본 뒤의 끝만."""
    rt.route_start("A지점", "B지점", 0)
    rt.nav_state = "done"
    rt._route_tick()
    assert rt.lifts == ["down"]
    rt.lift = "bottom"              # 명령이 먹기 전 1초마다 오는 옛 상태
    rt._route_tick()
    assert rt.route["state"] == "lifting"
    rt.lift = "down"
    rt._route_tick()
    rt.lift = "bottom"
    rt._route_tick()
    rt.route["until"] = 0.0
    rt._route_tick()                # 대기 끝 -> 올림
    assert rt.lifts == ["down", "up"] and rt.sent == ["A지점"]
    rt._route_tick()
    rt.lift = "top"
    rt._route_tick()
    assert rt.sent == ["A지점", "B지점"]


def test_lift_only_when_still(rt):
    from types import SimpleNamespace as S
    rt.route_start("A지점", "B지점", 0)
    rt.odom = S(twist=S(twist=S(linear=S(x=0.05), angular=S(z=0.0))))
    rt.nav_state = "done"
    rt._route_tick()
    rt._route_tick()
    assert rt.lifts == []           # 움직이는 동안은 안 보낸다
    rt.route["lift"]["t0"] -= 10.0
    rt._route_tick()
    assert rt.route["state"] == "failed" and rt.lifts == []


def test_lift_stuck_or_timeout_stops_route(rt):
    rt.route_start("A지점", "B지점", 0)
    rt.nav_state = "done"
    rt._route_tick()
    rt._route_tick()
    rt.lift = "stopped"             # 중간에 l stop
    rt._route_tick()
    assert rt.route["state"] == "failed" and rt.lift_stops == 1
    rt.lift = "bottom"
    rt.route_start("A지점", "B지점", 0)
    rt.nav_state = "done"
    rt._route_tick()
    rt.route["lift"]["sent"] -= 60.0
    rt._route_tick()
    assert rt.route["state"] == "failed" and "끝나지 않았" in rt.route["message"]


def test_cancel_while_lifting_stops_lift(rt):
    rt.route_start("A지점", "B지점", 0)
    rt.nav_state = "done"
    rt._route_tick()
    rt.route_cancel("정지")
    assert rt.route["state"] == "canceled" and rt.lift_stops == 1


def test_sweep_ignores_close_front_object_that_rotation_does_not_touch(ApiNode):
    # 2026-11-16: 앞 0.55m 의 물체는 정지 상자(0.6m)엔 들지만 제자리 회전 영역(외접 0.47+여유) 밖.
    import math
    import numpy as np
    n = ApiNode.__new__(ApiNode)
    pts = np.array([[0.55, 0.0]] * 5)
    assert n._sweep_hits(pts, math.radians(90)) == 0
    # 오른쪽 앞 모서리 옆(0.40, -0.25)은 왼쪽으로 돌면 멀어지고, 오른쪽으로 돌면 쓸린다
    pts = np.array([[0.40, -0.25]] * 5)
    assert n._sweep_hits(pts, math.radians(60)) == 0
    assert n._sweep_hits(pts, math.radians(-60)) == 5


def test_spin_plan_turns_the_free_way_or_backs_off(ApiNode):
    import math
    import numpy as np
    n = ApiNode.__new__(ApiNode)
    # 오른쪽 앞 모서리 옆이 막힘: 오른쪽 90도도, 왼쪽 270도(결국 그 자리를 지남)도 막힘
    #   -> 뒤로 빠진 뒤 오른쪽 90도
    n._obstacle_pts = lambda: np.array([[0.40, -0.25]] * 5)
    p = n._spin_plan(math.radians(-90))
    assert p["dir"] == -1.0 and p["move"] < 0
    # (직사각형은 180도 대칭이라 180도 넘게 도는 반대쪽은 원 전체를 쓸어, 짧은 쪽이 막히면 반대쪽도 막힌다)
    # 앞 양쪽 모서리가 다 막히면 뒤로 빠졌다 돈다 (뒤는 비어 있음)
    n._obstacle_pts = lambda: np.array([[0.40, -0.25]] * 5 + [[0.40, 0.25]] * 5)
    p = n._spin_plan(math.radians(-90))
    assert p is not None and p["move"] < 0
    # 사방이 막히면 None
    ring = [[0.45 * math.cos(a), 0.45 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 72)] * 3
    n._obstacle_pts = lambda: np.array(ring)
    assert n._spin_plan(math.radians(-90)) is None


def test_depart_smooth_ramp_has_no_flat_part(ApiNode):
    """2026-11-17 출발 속도: 0.04 에서 곡선으로 (평평한 구간 없이) 올라간다."""
    n = ApiNode.__new__(ApiNode)
    n.DEPART_M, n.DEPART_RAMP_M, n.DEPART_SLOW_V = 0.10, 0.40, 0.04
    vs = [n._depart_smooth_v(g, 0.5)[0] for g in (0.0, 0.02, 0.05, 0.10, 0.25, 0.5)]
    assert abs(vs[0] - 0.04) < 1e-9
    assert all(b > a for a, b in zip(vs, vs[1:]))            # 계속 오른다
    assert vs[3] < 0.10                                      # 10cm 에서도 아직 천천히
    assert abs(vs[-1] - 0.5) < 1e-9


def test_smooth_slow_start_rises_without_flat_then_releases(ApiNode):
    n = _mk_slow(ApiNode)
    n.DEPART_SMOOTH = True
    n.nav_speed = 0.40
    n.lift = None
    seen = []
    for g in (0.01, 0.05, 0.10, 0.30):
        n.od["x"] = g
        n._slow_tick()
        seen.append(n.limits[-1])
    assert all(b > a for a, b in zip(seen, seen[1:])) and seen[0] < 0.05 and seen[2] < 0.10
    n.od["x"] = n.DEPART_M + n.DEPART_RAMP_M
    n._slow_tick()
    assert n.limits[-1] == 0.0 and n._slow_timer is None


def test_slow_start_not_skipped_after_forward_straight_out(ApiNode):
    """2026-11-17 생략했더니 틀어진 채 바로 가속해 휘청였다 -> 늘 천천히 시작 (DEPART_SKIP_AFTER_OUT False)."""
    n = _mk_slow(ApiNode)
    n._slow_release()
    n._sp_fwd_out = True
    n._slow_start()
    assert n.limits[-1] == n.DEPART_SLOW_V and n._slow_timer is not None and n._sp_fwd_out is False


def test_after_straight_out_waits_still_then_turns_at_10deg(ApiNode):
    """2026-11-17 충전소에서 나온 직후 감속 중이라 정렬 회전을 건너뛰고 36도 틀어진 채 nav2 로 갔다."""
    import threading
    import types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.estop = False
    n.nav_seq = 5
    n.NAV_WATCH_HZ = 20.0
    n.create_timer = lambda p, cb: object()
    n.destroy_timer = lambda t: None
    n._pub_zero = lambda: None
    tw = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.05), angular=types.SimpleNamespace(z=0.0))
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=tw))
    calls = []
    n._nav_dispatch = lambda *a: calls.append((a, n._depart_bearing_lim, n._sp_out_done))
    n._dispatch_after_out("g", 5, 1.0, 2.0, 0.0, True)
    n._out_wait_tick()
    assert calls == []                                     # 아직 움직인다 -> 기다림
    tw.linear.x = 0.0
    n._out_wait_tick()
    assert len(calls) == 1 and abs(calls[0][1] - n.DEPART_BEARING_OUT) < 1e-9 and calls[0][2] is True
    assert n._depart_bearing_lim == n.DEPART_BEARING and n._out_wait_timer is None


def _mk_out(ApiNode, v, yaw_deg, path=None):
    """곧게 빠져나온 직후 노드: 속도 v, 몸 방향 yaw_deg, 미리 받은 경로 path (2026-11-18)."""
    import math
    import threading
    import types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.estop = False
    n.nav_seq = 5
    n.NAV_WATCH_HZ = 20.0
    n.create_timer = lambda p, cb: object()
    n.destroy_timer = lambda t: None
    n.zeros = 0
    n._pub_zero = lambda: setattr(n, "zeros", n.zeros + 1)
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.tw = types.SimpleNamespace(linear=types.SimpleNamespace(x=v), angular=types.SimpleNamespace(z=0.0))
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=n.tw))
    n.tf_buffer = object()
    n.tf_pose = lambda *a: {"x": 0.0, "y": 0.0, "yaw": math.radians(yaw_deg)}
    n.requests = []
    n._depart_plan_request = lambda *a, **k: n.requests.append((a, k)) or {"done": True, "path": None}
    n.calls = []
    n._nav_dispatch = lambda *a: n.calls.append((a, n._depart_bearing_lim, n._depart_aim))
    if path is not None:
        n._out_pre = {"seq": 5, "pl": {"done": True, "path": path}}
    return n


def test_after_out_small_angle_goes_on_without_stopping(ApiNode):
    """2026-11-18 사용자: 나갈 때 직선 -> 제자리 회전 -> 출발이 부드럽지 않음. 경로가 20도면 서지 않고 곡선 출발."""
    import math
    path = [(0.0, 0.0)] + [(0.1 * k, -0.1 * k * math.tan(math.radians(20))) for k in range(1, 15)]
    n = _mk_out(ApiNode, 0.03, 0.0, path)            # 아직 앞으로 움직이는 중
    n._dispatch_after_out("g", 5, 3.0, -1.0, 0.0, True)
    assert len(n.calls) == 1 and n.requests == []     # 미리 받은 경로로 바로 (다시 묻지 않음)
    assert abs(n.calls[0][1] - n.DEPART_CURVE_OUT) < 1e-9
    assert n._depart_bearing_lim == n.DEPART_BEARING and n._out_wait_timer is None


def test_after_out_large_angle_still_waits_and_turns(ApiNode):
    path = [(0.0, 0.0)] + [(-0.1 * k, 0.0) for k in range(1, 15)]   # 등 뒤
    n = _mk_out(ApiNode, 0.03, 0.0, path)
    n._dispatch_after_out("g", 5, -3.0, 0.0, 0.0, True)
    assert n.calls == []                              # 움직이는 중 -> 설 때까지
    n.tw.linear.x = 0.0
    n._out_wait_tick()
    assert len(n.calls) == 1 and abs(n.calls[0][1] - n.DEPART_BEARING_OUT) < 1e-9


def test_after_out_reverse_waits_still_even_if_small_angle(ApiNode):
    path = [(0.0, 0.0)] + [(0.1 * k, 0.0) for k in range(1, 15)]
    n = _mk_out(ApiNode, -0.03, 0.0, path)            # 후진으로 나오는 중 -> 앞으로 곡선 출발은 서고 나서
    n._dispatch_after_out("g", 5, 3.0, 0.0, 0.0, True)
    assert n.calls == []
    n.tw.linear.x = 0.0
    n._out_wait_tick()
    assert len(n.calls) == 1 and abs(n.calls[0][1] - n.DEPART_CURVE_OUT) < 1e-9


def test_after_out_prefetched_path_from_elsewhere_is_replanned(ApiNode):
    path = [(0.5, 0.0)] + [(0.5 + 0.1 * k, 0.0) for k in range(1, 15)]   # 0.5m 떨어진 데서 시작
    n = _mk_out(ApiNode, 0.0, 0.0, path)
    n._dispatch_after_out("g", 5, 3.0, 0.0, 0.0, True)
    assert n.calls == [] and len(n.requests) == 1     # 지금 자리에서 다시 묻는다
    n._out_wait_tick()
    assert len(n.calls) == 1


def test_prefetch_asks_from_out_end_point(ApiNode):
    import math
    n = ApiNode.__new__(ApiNode)
    got = []
    n._depart_plan_request = lambda x, y, yaw, start=None: got.append(start) or "pl"
    n._out_prefetch(7, 3.0, 0.0, 0.0, {"x": 1.0, "y": 1.0, "yaw": math.pi / 2}, -1, 0.5)
    assert abs(got[0][0] - 1.0) < 1e-9 and abs(got[0][1] - 0.5) < 1e-9
    assert n._out_pre == {"seq": 7, "pl": "pl"}


def test_depart_settle_ends_early_when_wheels_stopped(ApiNode):
    """2026-11-18 회전 뒤 늘 0.5초 서 있던 것 -> 바퀴가 멈췄으면 0.15초부터 판정."""
    import types
    n = _mk_turn_depart(ApiNode, 0)
    n._depart_passes = 0
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.1))))
    n._depart_phase, n._depart_t = "turn_settle", n.DEPART_SETTLE_MIN_S
    n._depart_tick()
    assert n._depart_phase == "turn_settle"           # 아직 도는 중
    n.odom.twist.twist.angular.z = 0.0
    n._depart_tick()
    assert n._depart_phase == "creep"


def test_after_straight_out_dropped_when_goal_changes(ApiNode):
    import threading
    import types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.estop = False
    n.nav_seq = 5
    n.NAV_WATCH_HZ = 20.0
    n.create_timer = lambda p, cb: object()
    n.destroy_timer = lambda t: None
    n._pub_zero = lambda: None
    n.odom = None
    calls = []
    n._nav_dispatch = lambda *a: calls.append(a)
    n._dispatch_after_out("g", 5, 1.0, 2.0, 0.0, True)
    n.nav_seq = 6
    n._out_wait_tick()
    assert calls == [] and n._out_wait_timer is None


def test_route_follows_internal_reissue(ApiNode):
    """2026-11-17 직선 도착이 같은 이동 안에서 nav2 목표를 다시 걸어도 왕복이 멈추지 않는다."""
    n = ApiNode.__new__(ApiNode)
    n.route = {"state": "going", "seq": 4}
    n.nav_seq = 5
    n._seq_chain(4)
    assert n.route["seq"] == 5
    n.route = {"state": "going", "seq": 3}
    n._seq_chain(4)                                   # 다른 이동이면 손대지 않는다
    assert n.route["seq"] == 3


def test_depart_turn_uses_path_aim_and_3deg_after_out(ApiNode):
    """2026-11-17 빠져나온 뒤 6.8 · 8.7도 틀어진 채 회전 없이 출발해 휘청였다 -> 경로 점 기준 3도."""
    import threading
    import types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    tw = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0))
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=tw))
    n.tf_pose = lambda *a, **k: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    n._spin_plan = lambda b: {"dir": 1.0, "move": 0.0}
    n._exit_front_blocked = lambda: False
    # 목표 점은 정면(0도)이지만 경로는 처음에 6도 방향으로 나간다
    aim = (0.6 * math.cos(math.radians(6)), 0.6 * math.sin(math.radians(6)))
    n._depart_bearing_lim = n.DEPART_BEARING
    assert n._depart_ok(5.0, 0.0, turn=True, aim=aim) is not None
    assert not n._depart_ok(5.0, 0.0, turn=True, aim=aim).get("turn")          # 평소 30도 기준이면 안 돎
    n._depart_bearing_lim = n.DEPART_BEARING_OUT
    assert n._depart_ok(5.0, 0.0, turn=True, aim=aim).get("turn")              # 빠져나온 뒤 3도 기준이면 돎
    assert n._depart_ok(0.5, 0.0, turn=True, aim=aim) is None                  # 거리는 실제 목표로 판단


def test_path_aim_point(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.tf_pose = lambda *a, **k: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    path = [(0.1 * i, 0.02 * i) for i in range(20)]
    ax, ay = n._path_aim(path)
    assert math.hypot(ax, ay) >= n.DEPART_AIM_M and math.hypot(ax, ay) < n.DEPART_AIM_M + 0.11


def _mk_route_charger(ApiNode):
    import threading
    import types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.estop, n.control_mode = False, "auto"
    n.job, n.cdock = {"state": "idle"}, {"state": "idle"}
    n.nav_seq, n.nav_state, n.nav_msg = 1, "idle", ""
    n._lift_moving = lambda: False
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.pois = [{"name": "1", "kind": "point", "x": 0, "y": 0, "yaw": 0},
              {"name": "충전소1", "kind": "charger", "x": 3, "y": 0, "yaw": 0}]
    n.calls = []

    def poi_goto(nm):
        n.calls.append(("goto", nm)); n.nav_seq += 1; n.nav_state = "active"; return True, ""

    real_cdock_start = ApiNode.cdock_start

    def cdock_start(nm, from_route=False):
        # 진짜 cdock_start 의 '왕복 중' 거절 검사를 거친다 (2026-11-18 이것에 걸려 왕복이 멈췄는데 시험은 못 잡았다)
        if n.route.get("state") in n.ROUTE_BUSY and not from_route:
            return False, "왕복 중입니다. 왕복을 멈춘 뒤 하세요"
        n.calls.append(("dock", nm)); n.cdock = {"state": "moving"}; return True, ""
    n.poi_goto, n.cdock_start = poi_goto, cdock_start
    lifts = []
    n._route_lift = lambda r, cmd, after, why: (lifts.append(cmd), r.update(state="waiting", until=0.0))
    n.lifts = lifts
    return n


def test_route_with_charger_docks_and_skips_lift(ApiNode):
    """2026-11-18 POI 1 <-> 충전소 왕복: 충전소는 도킹으로 도착, 리프트 없이 기다렸다 출발."""
    n = _mk_route_charger(ApiNode)
    ok, _ = n.route_start("1", "충전소1", 2)
    assert ok and n.calls[-1] == ("goto", "1")
    n.nav_state = "done"; n._route_tick()                   # 1 도착 -> 리프트
    assert n.lifts == ["down"]
    n.route["target"] = "B"; n._route_send()                # (리프트 뒤) 충전소로
    assert n.calls[-1] == ("dock", "충전소1") and n.route["state"] == "docking"
    n.nav_seq += 5                                          # 도킹이 nav 목표를 끊어도 왕복은 그대로
    n._route_tick(); assert n.route["state"] == "docking"
    n.cdock = {"state": "done"}; n._route_tick()
    assert n.route["state"] == "waiting" and n.route["target"] == "A" and n.lifts == ["down"]
    n.route["until"] = 0.0; n._route_tick()
    assert n.calls[-1] == ("goto", "1")                     # 리프트 없이 바로 출발


def test_route_charger_dock_fail_retries(ApiNode):
    n = _mk_route_charger(ApiNode)
    n.route_start("충전소1", "1", 1)
    assert n.calls[-1] == ("dock", "충전소1")
    n.cdock = {"state": "failed", "message": "전류 없음"}; n._route_tick()
    assert n.route["state"] == "retry"
    n.route["until"] = 0.0; n._route_tick()
    assert n.calls[-1] == ("dock", "충전소1")


def test_real_cdock_start_accepts_route_call(ApiNode):
    """진짜 cdock_start: 왕복 중이어도 from_route 면 받는다, 아니면 거절."""
    import threading, types
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.estop, n.control_mode = False, "auto"
    n.job, n.cdock = {"state": "idle"}, {"state": "idle"}
    n.route = {"state": "lifting"}
    n._lift_moving = lambda: False
    n.pois = [{"name": "충전소1", "kind": "charger", "x": 3, "y": 0, "yaw": 0}]
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n._cdock_visible_now = lambda: False
    n._cdock_go = lambda p, pose=None: (True, "ok")
    ok, msg = n.cdock_start("충전소1")
    assert not ok and "왕복" in msg
    ok, msg = n.cdock_start("충전소1", from_route=True)
    assert ok, msg


def test_route_without_lift_never_lifts(ApiNode):
    """2026-11-18 리프트 중 축1 알람으로 시험이 끊겨 lift=False 왕복 (주행 · 도킹만)."""
    n = _mk_route_charger(ApiNode)
    ok, _ = n.route_start("1", "충전소1", 1, lift=False)
    assert ok and n.calls[-1] == ("goto", "1")
    n.nav_state = "done"; n._route_tick()                   # 1 도착 -> 리프트 없이 바로 다음
    assert n.lifts == [] and n.route["state"] == "waiting" and n.route["target"] == "B"
    n._route_tick()
    assert n.calls[-1] == ("dock", "충전소1")
    n.cdock = {"state": "done"}; n._route_tick()
    n.route["until"] = 0.0; n._route_tick()
    assert n.calls[-1] == ("goto", "1")
    n.nav_state = "done"; n._route_tick()                   # 1회 끝
    assert n.lifts == [] and n.route["state"] == "done"
