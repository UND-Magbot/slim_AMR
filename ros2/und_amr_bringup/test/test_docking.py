# 2026-11-13 차체 반폭 0.135 -> 0.1644 (10/26 차체 변경 반영). 시험이 보려는 여유(37mm 등)가
# 그대로 되게 clearance_width 를 +58.8mm (0.344->0.4028, 0.358->0.4168, 0.330->0.3888).
"""소형 렉 진입 회귀시험. ROS 노드·모터를 기동하지 않고 실제 메서드를 호출한다."""
import math
import json
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def dock(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.nav_speed = 0.15
    n.leg_radius = 0.020
    # 렉 축 계산 시험은 좌우 대칭 차체로 본다. 오른쪽 뒤 라이다 돌출은 따로 시험한다.
    n.JOB_REAR_R_PROTRUDE = 0.0
    n._base_limits = {"min_turn_radius_m": 1.5,
                      "max_angular_rps": 1.5, "max_linear_mps": 0.6}
    n._base_limits_t = time.monotonic()
    n._job_poi = {"standoff": 1.5, "yaw": 0.0}
    n._cart_spec = lambda _: {"width": 0.423, "depth": 0.566}
    n.job = {"cart": "ls_origin", "state": "entering", "poi": "렉2"}
    n._job_timer = None
    n._job_t = 0.05
    n._job_cart = (1.5, 0.0, 0.0)
    n._job_cart0 = n._job_cart
    n._job_cart_stamp = None
    n._job_cart_tf_cache = {}
    n._job_start_xy = (0.0, 0.0)
    n._job_budget = 2.0
    n._job_bad = 0
    n._job_int = 0.0
    n._job_steer = False
    n._job_reanchor = False
    n._job_pose = lambda frame="map": (0.0, 0.0, math.pi)
    n._job_pick = lambda *args: None
    n.sent = []
    n.pub_twist = SimpleNamespace(publish=n.sent.append)
    n.pub_twist_free = SimpleNamespace(publish=n.sent.append)
    n.get_logger = lambda: Mock()
    return n


@pytest.mark.parametrize("heading", [-8, 0, 8])
@pytest.mark.parametrize("world_yaw", [-2.1, 0, 1.3])
def test_axis_error_is_independent_of_robot_heading(dock, heading, world_yaw):
    # 렉 중심 (1.5,0), 로봇 중심 (0,0.025)을 함께 회전·평행이동한다.
    c, s = math.cos(world_yaw), math.sin(world_yaw)
    rp = (2.0 - 0.025*s, -3.0 + 0.025*c,
          world_yaw + math.pi + math.radians(heading))
    cart = (2.0 + 1.5*c, -3.0 + 1.5*s, world_yaw)
    dock._job_poi["yaw"] = world_yaw + math.pi
    r = dock._job_to_robot(rp, cart)
    distance, axis_lat, corner = dock._job_entry_geometry(r)
    assert distance == pytest.approx(1.5)
    assert axis_lat == pytest.approx(0.025)
    assert axis_lat == pytest.approx(dock._job_axis_off(rp, cart)[1])
    # 네 모서리를 직접 렉 축으로 투영한 실제 반폭을 과소평가하면 안 된다.
    projected = []
    for x in [-0.435, 0.435]:
        for y in [-0.135, 0.135]:
            a = math.pi + math.radians(heading)
            projected.append(abs(0.025 + x*math.sin(a) + y*math.cos(a)))
    assert corner + 0.135 >= max(projected) - 1e-12


def test_heading_can_hide_unsafe_axis_offset(dock):
    # 이전 식은 30mm로 통과. 실제 축 기준 보수 외곽은 75mm다.
    r = {"forward": -0.65, "lateral": 0.0, "yaw": math.radians(4)}
    _, axis, corner = dock._job_entry_geometry(r)
    assert axis == pytest.approx(0.0453417, abs=1e-7)
    assert corner > dock._job_free("ls_origin")
    assert not dock._job_entry_clear(r, 0.12)
    assert dock.job["state"] == "failed"


@pytest.mark.parametrize("prealign", [True, False])
def test_unsafe_entry_never_publishes_motion_first(dock, prealign):
    dock._job_cart = (0.65, 0.0, math.radians(4))
    dock._job_cart0 = dock._job_cart
    dock._job_start_xy = (0.0, 0.0) if prealign else (-0.3, 0.0)
    dock._job_reanchor = not prealign
    dock._job_entering()
    assert dock.job["state"] == "failed"
    assert dock.sent
    assert all(t.linear.x == 0 and t.angular.z == 0 for t in dock.sent)


def test_stop_margin_is_checked_before_reaching_legs(dock):
    r = {"forward": -0.79, "lateral": 0.08, "yaw": 0.0}
    assert not dock._job_entry_clear(r, 0.12)


def test_far_offset_is_not_mistaken_for_contact(dock):
    r = {"forward": -1.3, "lateral": 0.08, "yaw": 0.0}
    assert dock._job_entry_clear(r, 0.12)
    assert not dock.sent


@pytest.mark.parametrize("prealign", [True, False])
def test_live_unsafe_rack_stops_even_when_filtered_anchor_is_safe(dock, prealign):
    # 필터는 중앙으로 보지만 최신 인식은 80mm 벗어난 렉을 본다.
    # 연관 범위 120mm 안이므로 실제 진입에서도 선택될 수 있는 관측이다.
    dock._job_cart = (0.65, 0.0, 0.0)
    dock._job_cart0 = dock._job_cart
    dock._job_start_xy = (0, 0) if prealign else (-0.3, 0)
    dock._job_reanchor = not prealign
    q = {"x": 0.65, "y": 0.08, "yaw": 0.0, "score": 0.95,
         "err_mm": 5, "held": False, "legs": [[0, 0]] * 4,
         "stamp_ns": 100000000000}
    dock._job_pick = lambda *args: q
    dock._job_entering()
    assert dock.job["state"] == "failed"
    assert "라이다 기준" in dock.job["message"]
    assert dock.sent
    assert all(t.linear.x == 0 and t.angular.z == 0 for t in dock.sent)


def test_live_centered_rack_cannot_override_unsafe_anchor(dock):
    dock._job_cart = (0.65, 0.08, 0.0)
    dock._job_cart0 = dock._job_cart
    q = {"x": 0.65, "y": 0.0, "yaw": 0.0, "score": 0.95,
         "err_mm": 5, "held": False, "legs": [[0, 0]] * 4,
         "stamp_ns": 100000000000}
    dock._job_pick = lambda *args: q
    dock._job_entering()
    assert dock.job["state"] == "failed"
    assert "작업 기준" in dock.job["message"]
    assert all(t.linear.x == 0 and t.angular.z == 0 for t in dock.sent)


def test_reanchor_checks_rack_axis_not_bearing(dock):
    # 렉 축 위에 있으며 방향만 4도 어긋남. 로봇 기준 lateral은 약 98mm.
    dock._job_cart = (1.4, 0.0, 0.0)
    dock._job_cart0 = dock._job_cart
    dock._job_pose = lambda frame="map": (0, 0, math.pi + math.radians(4))
    dock._job_start_xy = (-0.26, 0)      # 캐스터 뒤집기 구간(0.25m)을 막 지남
    dock._job_entering()
    assert dock.job["state"] == "entering"
    assert dock._job_reanchor
    assert dock.sent[-1].linear.x < 0


def test_reverse_start_creeps_through_caster_flip(dock):
    # 2026-09-29: 후진 출발 0.25m 는 0.03 m/s 로 방향만 붙잡고, 그 뒤 가속한다.
    dock._job_cart = (1.4, 0.0, 0.0)
    dock._job_cart0 = dock._job_cart
    dock._job_pose = lambda frame="map": (0, 0, math.pi)
    dock._job_start_xy = (-0.20, 0)
    dock._job_entering()
    assert dock.sent[-1].linear.x == pytest.approx(-dock.JOB_FLIP_V)
    assert "캐스터 정렬" in dock.job["step"]
    dock._job_start_xy = (-0.26, 0)
    dock._job_entering()
    assert -dock.sent[-1].linear.x >= dock.JOB_CREEP_V - 1e-9


def test_missing_limits_stop_before_motion(dock):
    dock._base_limits_t = time.monotonic() - 7
    dock._job_entering()
    assert dock.job["state"] == "failed"
    assert all(t.linear.x == 0 and t.angular.z == 0 for t in dock.sent)


def test_correction_budget_uses_base_radius_and_remaining_distance(dock):
    # 2026-09-29: 방향만 붙잡는 구간이 0.10 -> 0.25m 로 늘어 보정 거리가 준다
    assert dock._job_lat_max("ls_origin") == pytest.approx(0.0351, abs=1e-4)
    assert dock._job_lat_max("ls_origin", distance=0.48) == pytest.approx(0.0288)
    assert dock._job_lat_max("ls_origin", distance=-0.1) == 0


def test_caster_clearance_is_separate_from_detection_width(dock):
    # 가상 실측값. 실제 렉의 측정 결과가 아니다.
    spec = {"width": 0.423, "depth": 0.566, "clearance_width": 0.3888}
    dock._cart_spec = lambda _: spec
    assert dock._job_free("ls_origin") == pytest.approx(0.030)
    dock.leg_radius = 0.040
    assert dock._job_free("ls_origin") == pytest.approx(0.030)
    # 이전 기둥 기준 56.5mm라면 통과했을 31mm 편차도 실측 폭으로 중단.
    assert not dock._job_entry_clear(
        {"forward": -0.736, "lateral": 0.031, "yaw": 0.0}, 0.12)
    assert dock.job["state"] == "failed"
    assert spec["width"] == 0.423


@pytest.mark.parametrize("value", [0, None, "미측정", float("nan"),
                                  float("inf"), True, -0.3, 0.25, 0.27])
def test_invalid_or_too_narrow_caster_clearance_blocks_start_and_entry(dock, value):
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                "clearance_width": value}
    dock.estop, dock.control_mode = False, "auto"
    dock.job["state"] = "idle"
    dock.poi_find = lambda _: {"kind": "cart", "cart": "ls_origin"}
    dock.nav_goto = Mock(side_effect=AssertionError("이동 금지"))
    dock._job_spin_start = Mock(side_effect=AssertionError("회전 금지"))
    ok, message = dock.job_start("렉2")
    assert not ok and "통과 폭" in message
    dock.job["state"] = "entering"
    dock._job_entering()
    assert dock.job["state"] == "failed"
    assert "통과 폭" in dock.job["message"]
    assert all(t.linear.x == 0 and t.angular.z == 0 for t in dock.sent)


def test_unknown_caster_clearance_cannot_report_done(dock):
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                "clearance_width": 0}
    dock._job_cart = (0.04, 0, 0)
    dock._job_done()
    assert dock.job["state"] == "failed"


def test_caster_clearance_survives_rack_reregistration(dock, tmp_path):
    import yaml
    text = ('cart_detector:\n  ros__parameters:\n    carts: [ls_origin]\n'
            '    ls_origin:\n      width: 0.423\n      depth: 0.566\n'
            '      clearance_width: 0.0\n      cam_legs: 0\n')
    updated = dock._yaml_put_cart(text, "ls_origin", 0.424, 0.565, 0.04, "재측정")
    spec = yaml.safe_load(updated)["cart_detector"]["ros__parameters"]["ls_origin"]
    assert spec["clearance_width"] == 0.0
    assert spec["width"] == 0.424
    path = tmp_path / "carts.yaml"
    path.write_text(updated)
    dock._carts_yaml_paths = lambda: [str(path)]
    assert dock.cart_specs_json()["carts"][0]["clearance_width"] == 0.0


def test_steering_limit_tracks_speed_and_base_angular_cap(dock):
    assert dock._job_w_limit(0.12, 0.24) == pytest.approx(0.08)
    assert dock._job_w_limit(0.05, 0.24) == pytest.approx(0.05 / 1.5)
    dock._base_limits["max_angular_rps"] = 0.02
    assert dock._job_w_limit(0.12, 0.24) == pytest.approx(0.02)
    dock._base_limits["max_linear_mps"] = 0.09
    assert dock._job_gains()["v"] == pytest.approx(0.09)


def test_integral_does_not_accumulate_into_saturated_output(dock):
    # 적분 구조 시험. 운용값은 0 이므로 예전 이득으로 고정한다(2026-09-28).
    dock.JOB_K_LAT, dock.JOB_K_YAW, dock.JOB_K_INT = 0.40, 0.50, 0.60
    dock._job_steer = True
    gn = dock._job_gains()
    for _ in range(100):
        w, _, _ = dock._job_steering(0.2, 0.1, 1.2, gn, 0.04, 1, 1)
        assert w == pytest.approx(0.04)
    assert dock._job_int == 0.0


def test_integral_can_unwind_while_output_is_saturated(dock):
    # 적분 구조 시험. 운용값은 0 이므로 예전 이득으로 고정한다(2026-09-28).
    dock.JOB_K_LAT, dock.JOB_K_YAW, dock.JOB_K_INT = 0.40, 0.50, 0.60
    dock._job_steer = True
    dock._job_int = 0.02
    w, _, _ = dock._job_steering(-0.01, 0.2, 1.2,
                                dock._job_gains(), 0.04, 1, 1)
    assert w == pytest.approx(0.04)
    assert 0 < dock._job_int < 0.02


def test_logged_integral_cannot_reverse_positive_correction(dock):
    # 적분 구조 시험. 운용값은 0 이므로 예전 이득으로 고정한다(2026-09-28).
    dock.JOB_V_MAX = 0.12                    # 기대값은 0.12 m/s 기준
    dock.JOB_V_FIXED = 0.12                  # 2026-11-13 진입 속도 고정값
    dock.JOB_K_LAT, dock.JOB_K_YAW, dock.JOB_K_INT = 0.40, 0.50, 0.60
    dock._job_steer = True
    dock._job_int = -0.065 / dock.JOB_K_INT
    cap = dock._job_w_limit(0.12, 0.24)
    w, p, raw = dock._job_steering(0.020, math.radians(3.6), 1.222,
                                  dock._job_gains(), cap, 1, 1)
    assert p == pytest.approx(0.0472991, abs=1e-7)
    assert w > 0 and raw > 0
    assert abs(dock.JOB_K_INT * dock._job_int) <= 0.4 * cap


def test_integral_bleeds_near_rack_and_resets_when_steering_off(dock):
    # 적분 구조 시험. 운용값은 0 이므로 예전 이득으로 고정한다(2026-09-28).
    dock.JOB_K_LAT, dock.JOB_K_YAW, dock.JOB_K_INT = 0.40, 0.50, 0.60
    dock._job_steer = True
    dock._job_int = 0.01
    dock._job_steering(0.02, 0, 0.5, dock._job_gains(), 0.06, 0.3, 0.8)
    assert dock._job_int == pytest.approx(0.009)
    dock._job_steer = False
    w, _, _ = dock._job_steering(0.02, 0, 1.0,
                                dock._job_gains(), 0.06, 1, 1)
    assert w == 0 and dock._job_int == 0


def set_detection(dock, now_ns=100400000000):
    dock.carts_t = time.time()
    dock.carts = {"frame": "laser", "stamp": {"sec": 100, "nanosec": 0},
                  "carts": [{"x": 1.5, "y": 0, "yaw": 0,
                             "cart": "ls_origin", "score": 0.95}]}
    dock.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=now_ns))


def test_detection_uses_scan_time_and_does_not_move_with_latest_tf(dock):
    set_detection(dock)
    dock.tf_pose_at = Mock(return_value={"x": 0.5, "y": 0.1, "yaw": 0.2})
    dock.tf_pose = Mock(side_effect=AssertionError("최신 TF를 쓰면 안 됨"))
    first = dock._job_carts_map("odom")
    assert first[0]["x"] == pytest.approx(0.5 + 1.5*math.cos(0.2))
    assert first[0]["y"] == pytest.approx(0.1 + 1.5*math.sin(0.2))
    assert dock.tf_pose_at.call_args[0][0].sec == 100
    assert dock.tf_pose_at.call_args[1] == {"target": "odom", "source": "laser"}
    dock.tf_pose_at.return_value = {"x": 9, "y": 9, "yaw": 2}
    assert dock._job_carts_map("odom") == first
    assert dock.tf_pose_at.call_count == 1


@pytest.mark.parametrize("stamp", [None, {}, {"sec": 100, "nanosec": -1},
                                    {"sec": 100, "nanosec": 1000000000},
                                    {"sec": 100.0, "nanosec": 0},
                                    {"sec": 0, "nanosec": 0}])
def test_missing_or_invalid_scan_time_is_rejected(dock, stamp):
    set_detection(dock)
    dock.carts["stamp"] = stamp
    assert dock._job_carts_map("odom") == []


@pytest.mark.parametrize("now", [99000000000, 101100000000])
def test_future_and_stale_scan_are_rejected_even_if_just_received(dock, now):
    set_detection(dock, now)
    assert dock._job_carts_map("odom") == []


def test_missing_historical_tf_never_falls_back_to_latest(dock):
    set_detection(dock)
    dock.tf_pose_at = Mock(return_value=None)
    dock.tf_pose = Mock(side_effect=AssertionError("최신 TF 대체 금지"))
    assert dock._job_carts_map("odom") == []


def test_same_detection_is_blended_only_once(dock):
    dock.JOB_V_MAX = 0.12                    # 기대값은 0.12 m/s 기준
    dock.JOB_V_FIXED = 0.12                  # 2026-11-13 진입 속도 고정값
    q = {"x": 1.5, "y": 0.05, "yaw": 0.0, "score": 0.95,
         "err_mm": 5, "held": False, "legs": [[0, 0]]*4,
         "stamp_ns": 100000000000}
    dock._job_pick = lambda *args: q
    dock._job_entering()
    first = dock._job_cart
    assert 0 < first[1] < 0.05
    for _ in range(3):
        dock._job_entering()
        assert dock._job_cart == first
    q["stamp_ns"] += 200000000
    dock._job_entering()
    expected = first[1] + (0.05-first[1]) * (1-math.exp(-0.12*0.2/0.15))
    assert dock._job_cart[1] == pytest.approx(expected)


def test_done_rejects_axis_envelope_outside_clearance(dock):
    # 로봇 기준 lateral=0이어도 축 기준 외곽은 여유를 초과한다.
    dock._job_cart = (0.65, 0, math.radians(4))
    dock._job_done()
    assert dock.job["state"] == "failed"


def test_done_accepts_centered_pose(dock):
    dock._job_cart = (0.04, 0, 0)
    dock._job_done()
    assert dock.job["state"] == "done"


def test_done_cannot_succeed_without_pose(dock):
    dock._job_pose = lambda frame="map": None
    dock._job_done()
    assert dock.job["state"] == "failed"


def test_old_detection_hits_cannot_start_docking_after_scan_expires(dock):
    dock._job_poi.update(x=1.5, y=0)
    dock._job_hits = [1] * dock.JOB_LOOK_N
    dock.carts_t = 100
    dock._job_seen_t = 100
    assert not dock._job_look_tick(cam=False)


def test_base_limits_are_read_asynchronously_and_invalidated(dock, api):
    future = Future()
    dock._base_limits_future = None
    dock._base_limits_cli = SimpleNamespace(
        service_is_ready=lambda: True, call_async=Mock(return_value=future))
    dock._job_refresh_base_limits()
    request = dock._base_limits_cli.call_async.call_args[0][0]
    assert request.names == ["min_turn_radius_m", "max_angular_rps",
                             "max_linear_mps", "max_decel_mps2"]
    # 모터 설정을 복사해 넣은 값이 아니라 응답값을 사용한다.
    values = [api.ParameterValue(type=api.ParameterType.PARAMETER_DOUBLE,
                                 double_value=v) for v in (2.0, 0.7, 0.4, 0.6)]
    future.set_result(SimpleNamespace(values=values))
    assert dock._job_w_limit(0.12, 0.24) == pytest.approx(0.06)
    bad = Future()
    dock._base_limits_cli.call_async.return_value = bad
    dock._job_refresh_base_limits()
    bad.set_result(SimpleNamespace(values=[]))
    assert dock._job_base_limits() is None


def test_pending_parameter_query_is_not_duplicated(dock):
    future = Future()
    dock._base_limits_future = None
    dock._base_limits_cli = SimpleNamespace(
        service_is_ready=lambda: True, call_async=Mock(return_value=future))
    dock._job_refresh_base_limits()
    dock._job_refresh_base_limits()
    assert dock._base_limits_cli.call_async.call_count == 1


def test_detector_publishes_original_scan_stamp_once(monkeypatch):
    from conftest import _load
    detector = _load("cart_detector")
    n = detector.CartDetector.__new__(detector.CartDetector)
    n.scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=123),
                               frame_id="laser"),
        ranges=[], angle_min=0.0, angle_increment=0.1)
    n.specs = ["ls_origin"]
    n.rmax, n.gap, n.wmin, n.wmax = 3.0, 0.05, 0.01, 0.1
    n.minpts, n.leg_r, n.min_score, n.hold_ratio = 2, 0.02, 0.25, 0.8
    n._ratio_hist = {}
    n._last_n = 0
    n._align = lambda *args: {}
    n._markers = lambda *args: None
    sent = []
    n.pub = SimpleNamespace(publish=sent.append)
    n.pub_al = SimpleNamespace(publish=lambda m: None)
    monkeypatch.setattr(detector, "cluster_scan", lambda *a, **k: ([], [], []))
    monkeypatch.setattr(detector, "leg_candidates", lambda *a, **k: [])
    monkeypatch.setattr(detector, "match_carts", lambda *a, **k: [])
    n._tick()
    assert json.loads(sent[0].data)["stamp"] == {"sec": 100, "nanosec": 123}
    n._tick()
    assert len(sent) == 1
    n.scan.header.stamp.nanosec += 1
    n._tick()
    assert len(sent) == 2


def test_approach_trace_logs_why_aim_was_skipped(dock):
    # 2026-09-28: 다가가는 동안 목표를 왜 안 옮겼는지 로그로 가린다.
    dock._job_poi.update(x=1.5, y=0.0)
    dock._job_goal = (0.0, 0.0, math.pi)
    dock._job_seen = 1
    dock._job_aim_n = 0
    dock._job_pick_near = lambda *a, **k: {
        "x": 1.55, "y": 0.07, "yaw": 0.0, "score": 0.9, "legs": [0] * 4}
    log = Mock()
    dock.get_logger = lambda: log
    dock._job_aim()
    assert dock._job_aim_why == "인식 1/%d" % dock.JOB_LOOK_N
    dock._job_approach_trace()
    msg = log.info.call_args[0][0]
    assert "보임 POI차 86mm" in msg and "다시걸기 인식 1/" in msg


def test_axis_following_gain_does_not_grow_with_distance(dock):
    # 2026-09-28: 예전 입력은 남은거리*방향이 섞여 멀수록 방향 이득이 커졌다.
    dock._job_steer = True
    gn = dock._job_gains()
    w1, p1, _ = dock._job_steering(0.0, math.radians(2), 1.5, gn, 1.0, 1, 1)
    w2, p2, _ = dock._job_steering(0.0, math.radians(2), 0.9, gn, 1.0, 1, 1)
    assert p1 == pytest.approx(p2)
    assert dock._job_int == 0.0


def _exit_setup(dock, pose):
    dock.job = {"cart": "ls_origin", "state": "exiting", "poi": "렉2"}
    dock._exit_c = (0.0, 0.0, 0.0)
    dock._exit_rack_end = 0.283 + dock.JOB_HALF_L
    dock._exit_target = dock._exit_rack_end + dock.JOB_EXIT_CLEAR
    dock._job_start_xy = (0.0, 0.0)
    dock._job_t = 1.0
    dock._job_pose = lambda frame="map": pose
    dock._exit_front_blocked = lambda: False


def test_exit_steers_back_toward_axis(dock):
    # 2026-09-29: 나오기는 좌우를 고치지 않고 방향만 한 번 편 뒤 곧게 간다.
    dock._run_dir, dock._run_m = 1, 1.0      # 출발 예측 보정 구간은 지났다
    _exit_setup(dock, (0.8, 0.03, 0.0))
    dock._exit_steer = False
    dock._job_exiting()
    assert dock.sent[-1].linear.x > 0 and dock.sent[-1].angular.z == 0
    # 왼쪽으로 틀어져 있으면 오른쪽으로 편다
    _exit_setup(dock, (0.8, 0.0, math.radians(3)))
    dock._job_exiting()
    assert dock.sent[-1].angular.z < 0
    # 사각지대(0.6도) 안이면 건드리지 않는다
    _exit_setup(dock, (0.8, 0.0, math.radians(0.2)))
    dock._exit_steer = False
    dock._job_exiting()
    assert dock.sent[-1].angular.z == 0

def test_exit_forward_start_feedforward(dock):
    # 2026-09-29: 후진 뒤 전진 출발 0.3m 동안 캐스터가 시계로 돌리는 만큼
    # 미리 반시계로 돈다. 그 뒤와 후진에는 넣지 않는다.
    # 렉에서는 캐스터가 반대로 차서 기본값은 꺼 두었다(0). 켰을 때 동작만 본다.
    dock.JOB_FF_FWD_DEG = -3.3
    dock._run_dir, dock._run_m = -1, 0.5     # 방금까지 후진
    assert dock._job_ff(0.1) > 0
    assert dock._job_ff(-0.1) == 0
    w = dock._job_ff(0.1)
    assert abs(w * dock.JOB_FF_M / 0.1
               - math.radians(-dock.JOB_FF_FWD_DEG)) < 1e-9
    dock._run_dir, dock._run_m = 1, 0.1
    assert dock._job_ff(0.1) == w
    dock._run_dir, dock._run_m = 1, 0.31
    assert dock._job_ff(0.1) == 0


def test_exit_between_legs_uses_near_limit_and_stops_on_envelope(dock):
    _exit_setup(dock, (0.3, 0.004, 0.0))
    dock._job_exiting()
    assert abs(dock.sent[-1].angular.z) <= dock._job_gains()["w_near"] + 1e-9
    _exit_setup(dock, (0.3, 0.065, 0.0))    # 여유 44mm 를 크게 넘는다
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4168}
    dock._job_exiting()
    assert dock.job["state"] == "failed"


def test_exit_finishes_past_rack_end(dock):
    _exit_setup(dock, (1.0, 0.0, 0.0))
    dock._job_exiting()
    assert dock.job["state"] == "done"
    assert dock.sent[-1].linear.x == 0


def test_stop_now_cancels_running_job(dock):
    dock.job = {"state": "exiting", "cart": "ls_origin", "poi": "렉2"}
    dock._jog = dock._jog_ref = None
    dock.srv = {}
    dock.stop_now()
    assert dock.job["state"] == "canceled"


def test_exit_envelope_uses_only_legs_beside_body(dock):
    # 2026-09-28 반복 3회차: 모서리 공식으로 50mm 였으나 닿지 않았다.
    # 뒤끝이 뒤 다리를 지난 뒤에는 앞 다리 옆의 짧은 지렛대만 본다.
    env = dock._exit_envelope(0.229, -0.020, 0.069, 0.566)
    assert env < 0.025
    # 한가운데서는 앞뒤 다리 모두 본다
    env0 = dock._exit_envelope(0.0, 0.0, math.radians(3), 0.566)
    assert env0 == pytest.approx((0.283 + 0.0325) * math.tan(math.radians(3)))


def test_exit_tolerates_brief_kick_but_stops_on_sustained_or_large(dock):
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4168}
    _exit_setup(dock, (0.3, 0.050, 0.0))      # 44mm 여유를 6mm 넘음
    dock._exit_jam = 0
    dock._exit_fix_n = dock.JOB_EXIT_FIX_MAX   # 방향 맞춤을 다 썼다
    dock._job_exiting()
    assert dock.job["state"] == "exiting"     # 한 주기는 봐준다
    dock._job_exiting()
    dock._job_exiting()
    assert dock.job["state"] == "failed"      # 이어지면 멈춘다
    _exit_setup(dock, (0.3, 0.065, 0.0))      # 21mm 넘음 — 즉시
    dock._exit_jam = 0
    dock._job_exiting()
    assert dock.job["state"] == "failed"


def test_center_offset_shifts_axis_toward_docked_left(dock):
    # 2026-09-28: 통로 중심이 라이다 축에서 도킹 자세 기준 왼쪽 7mm.
    # 도킹 좌표: 로봇 기준 lateral + 는 대차가 왼쪽 = 로봇이 축 오른쪽.
    r = {"forward": -0.5, "lateral": 0.0, "yaw": 0.0}
    _, lat0, _ = dock._job_entry_geometry(r)
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "center_offset": 0.007}
    _, lat1, _ = dock._job_entry_geometry(r)
    assert lat1 - lat0 == pytest.approx(0.007)   # 축 위 = 중심의 오른쪽 7mm
    # 나오기: + 가 왼쪽. 축 위에 있으면 중심의 오른쪽(-7mm)
    _exit_setup(dock, (0.8, 0.0, 0.0))
    _, e, _ = dock._exit_axis((0.8, 0.0, 0.0))
    assert e == pytest.approx(-0.007)


def test_retry_outside_rack_goes_back_to_stand_point(dock):
    # 2026-09-28: 렉 밖에서 멈추면 대기 자리로 돌아가 한 번 더 한다.
    dock._job_poi = {"x": 1.5, "y": 0.0, "yaw": 0.0, "standoff": 1.5}
    dock._job_pick = lambda *a, **k: None
    dock._job_stand_ok = lambda sp: True
    # 렉을 향하고 있다 — 축 따라 물러나기가 아니라 nav2 로 다시 선다
    dock._job_pose = lambda frame="map": (0.0, 0.0, 0.0)
    sent = []
    dock.nav_goto = lambda *a, **k: (sent.append(a) or (True, ""))
    dock._job_retry = 0
    dock._job_retry_or_fail("시험")
    assert dock.job["state"] == "moving" and len(sent) == 1
    assert sent[0][0] == pytest.approx(3.0)       # POI 앞 1.5m
    for _ in range(dock.JOB_RETRY_MAX - 1):         # 2026-11-13 재시도 1 -> 3
        dock._job_retry_or_fail("시험")
        assert dock.job["state"] == "moving"
    dock._job_retry_or_fail("시험")                # 횟수를 다 썼다
    assert dock.job["state"] == "failed"


def test_no_retry_once_between_legs(dock):
    dock._job_poi = {"x": 1.5, "y": 0.0, "yaw": 0.0, "standoff": 1.5}
    dock.nav_goto = lambda *a, **k: (True, "")
    dock._job_retry = 0
    dock._job_retry_or_fail("시험", inside=True)
    assert dock.job["state"] == "failed"


def test_creep_find_moves_toward_rack_and_stops_at_limits(dock):
    # 2026-09-28: 1.54m 에서 안 보이고 1.45m 에서 보였다. 곧장 조금 다가간다.
    dock._job_poi = {"x": 1.54, "y": 0.0, "yaw": 0.0, "standoff": 1.5}
    dock._job_pose = lambda frame="map": (0.0, 0.0, 0.0)   # 렉을 보고 있다
    dock._exit_front_blocked = lambda: False
    dock._job_find_xy = None
    dock._job_creeping = False
    assert dock._job_creep_find()
    assert dock.sent[-1].linear.x == pytest.approx(dock.JOB_FIND_V)
    # 40cm 다 왔으면 그만
    dock._job_pose = lambda frame="map": (0.41, 0.0, 0.0)
    assert not dock._job_creep_find()
    # 렉을 등지고 있으면 안 간다
    dock._job_find_xy = None
    dock._job_pose = lambda frame="map": (0.0, 0.0, math.pi)
    assert not dock._job_creep_find()


def test_exit_holds_heading_slowly_while_casters_flip(dock):
    # 2026-09-29: 후진 뒤 전진 0.25m 는 0.03 m/s 로 방향을 붙잡는다(사각지대 없이).
    dock._run_dir, dock._run_m = 1, 0.04     # 캐스터 미리 뒤집기만 끝남
    _exit_setup(dock, (0.25, 0.0, math.radians(0.3)))
    dock._job_start_xy = (0.0, 0.0)
    dock._job_exiting()
    assert dock.sent[-1].linear.x == pytest.approx(dock.JOB_FLIP_V)
    assert dock.sent[-1].angular.z < 0
    dock._run_m = 0.26
    _exit_setup(dock, (0.25, 0.0, 0.0))
    dock._job_start_xy = (0.0, 0.0)
    dock._job_exiting()
    assert dock.sent[-1].linear.x == pytest.approx(dock.JOB_CREEP_V)


def test_exit_creeps_until_casters_flip(dock):
    # 2026-09-28: 나오기 초반 속도를 올리면 캐스터가 뒤집히며 차체를 돌린다.
    dock._run_dir, dock._run_m = 1, 1.0      # 캐스터 뒤집기 구간은 지났다
    _exit_setup(dock, (0.25, 0.0, 0.0))
    dock._job_start_xy = (0.0, 0.0)              # 0.25m 나옴 — 아직 기어간다
    dock._job_exiting()
    assert dock.sent[-1].linear.x == pytest.approx(dock.JOB_CREEP_V)
    _exit_setup(dock, (0.6, 0.0, 0.0))
    dock._job_start_xy = (0.0, 0.0)
    dock._job_exiting()
    assert dock.sent[-1].linear.x > dock.JOB_CREEP_V


def test_exit_top_speed_is_capped(dock):
    # 2026-09-28: 나오기는 0.08 m/s 를 넘지 않는다.
    _exit_setup(dock, (0.6, 0.0, 0.0))
    dock._job_start_xy = (-0.5, 0.0)          # 1.1m 나온 셈 — 가속 끝
    dock._job_exiting()
    assert dock.sent[-1].linear.x <= dock.JOB_EXIT_V_MAX + 1e-9


def test_stop_distance_uses_base_decel(dock):
    # 2026-09-28: 0.072 m/s 에서 예전 식(0.72초)과 같고, 감속이 낮으면 늘어난다.
    dock._base_limits["max_decel_mps2"] = 0.9
    assert dock._job_stop_dist(0.072) == pytest.approx(0.072 * 0.72, abs=0.0005)
    dock._base_limits["max_decel_mps2"] = 0.6
    assert dock._job_stop_dist(0.072) > 0.072 * 0.72
    del dock._base_limits["max_decel_mps2"]
    assert dock._job_stop_dist(0.1) == pytest.approx(0.072)


def test_lidar_entry_check_needs_two_distinct_scans(dock):
    # 2026-09-28: 라이다 한 장이 2mm 넘은 것으로 다리 사이에서 멈췄다.
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4028}
    dock._job_retry_or_fail = lambda why, inside=False: dock.job.update(
        state="failed")
    dock._lidar_jam = (0, None)
    r = {"forward": -0.30, "lateral": -0.039, "yaw": 0.0}   # 모서리 39 / 여유 37
    # 2026-11-13: 라이다 기준은 측정 잡음 몫(JOB_LIDAR_NOISE 8mm)을 더 봐준다 — 2mm 넘김은 통과
    assert dock._job_entry_clear(r, 0.1, source="라이다 기준", scan=1)
    assert dock._job_entry_clear(r, 0.1, source="라이다 기준", scan=2)
    r2 = {"forward": -0.30, "lateral": -0.047, "yaw": 0.0}  # 모서리 47 / 여유 37+8
    dock._lidar_jam = (0, None)
    assert dock._job_entry_clear(r2, 0.1, source="라이다 기준", scan=3)
    assert dock._job_entry_clear(r2, 0.1, source="라이다 기준", scan=3)  # 같은 장
    assert not dock._job_entry_clear(r2, 0.1, source="라이다 기준", scan=4)
    assert dock.job["state"] == "failed"
    # 작업 기준(스캔 없음)은 잡음 몫 없이 즉시 멈춘다
    dock.job["state"] = "entering"
    assert not dock._job_entry_clear(r, 0.1)


def test_preflip_creeps_forward_then_finishes(dock):
    # 2026-09-28: 도킹 끝에 조금 앞으로 가서 캐스터를 미리 뒤집는다.
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4028}
    dock._job_cart = (-0.5, 0.0, 0.0)            # 로봇이 렉 중앙(뒤로 들어옴)
    dock._job_pose = lambda frame="map": (-0.5, 0.0, math.pi)
    dock._job_preflip_start()
    assert dock.job["state"] == "preflip"
    for _ in range(int(dock.JOB_PREFLIP_SETTLE_S * dock.JOB_HZ) + 1):
        dock._job_preflip()                  # 먼저 완전히 선다
    assert dock.sent[-1].linear.x == 0
    dock._job_preflip()
    assert dock.sent[-1].linear.x == pytest.approx(dock.JOB_PREFLIP_V)
    done = []
    dock._job_done = lambda: done.append(1)
    dock._job_pose = lambda frame="map": (-0.5 - dock.JOB_PREFLIP_M, 0.0, math.pi)
    dock._job_preflip()
    assert done and dock.sent[-1].linear.x == 0


def test_leg_envelope_at_center_uses_leg_lever(dock):
    # 2026-09-28: 한가운데 4도 튐 — 모서리 식 38mm, 다리 자리로는 약 29mm.
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566}
    env = dock._job_leg_envelope(0.0, 0.007, math.radians(4))
    assert env == pytest.approx(0.007 + 0.3155 * math.tan(math.radians(4)))
    assert env < 0.007 + 0.435 * math.sin(math.radians(4))


def test_preflip_holds_heading_through_radius_free_path(dock):
    # 2026-09-28: 뒤집기 중 캐스터가 돌리면 반대로 받아친다(반경 제한 없이).
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4028}
    dock._job_cart = (-0.5, 0.0, 0.0)
    dock._job_pose = lambda frame="map": (-0.5, 0.0, math.pi + math.radians(3))
    dock._job_preflip_start()
    for _ in range(int(dock.JOB_PREFLIP_SETTLE_S * dock.JOB_HZ) + 2):
        dock._job_preflip()
    w = dock.sent[-1].angular.z
    assert abs(w) > 0.03                     # 반경 제한(0.02)보다 세게
    # 틀어진 반대쪽으로: 로봇이 +3도 돌았으면(대차 기준 -3도) 음의 회전
    assert w < 0


def test_retry_in_front_backs_off_along_axis_then_reenters(dock):
    # 2026-09-28: 입구 앞(렉을 등짐)에서 멈추면 nav2 대신 축 따라 물러나 다시 진입.
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566}
    dock._job_poi = {"x": 0.0, "y": 0.0, "yaw": 0.0, "standoff": 1.5}
    dock._job_pick = lambda *a, **k: None
    dock.nav_goto = Mock(return_value=(True, ""))
    dock._job_retry = 0
    # 로봇은 렉 중심에서 +x 로 0.85m, 렉을 등지고(+x 를 봄) 서 있다
    dock._job_pose = lambda frame="map": (0.85, 0.0, 0.0)
    dock._job_retry_or_fail("시험")
    assert dock.job["state"] == "exiting" and not dock.nav_goto.called
    # 2026-11-13: 짧게 물러나기는 꺼 둠(JOB_BACKOFF_SHORT False) — 대기 거리 + 0.3
    assert dock._exit_target == pytest.approx(1.8)
    # 다 물러나면 돌지 않고 바로 후진 진입
    dock._job_pose = lambda frame="map": (dock._exit_target, 0.0, 0.0)
    dock._job_start_xy = (0.85, 0.0)
    dock._exit_front_blocked = lambda: False
    dock._job_exiting()
    assert dock.job["state"] == "entering"


def test_exit_kick_stops_straightens_and_resumes(dock):
    # 2026-09-28 B안: 나오다 튀면 멈추고 제자리에서 축에 맞춘 뒤 이어 간다.
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.4028}
    _exit_setup(dock, (0.0, 0.0, math.radians(8.0)))   # 한가운데 8도 튐
    dock._exit_jam = 0
    dock._exit_fix_n = 0
    for _ in range(dock.JOB_EXIT_JAM_N):
        dock._job_exiting()
    assert dock.job["state"] == "exit_fix"
    dock._exit_fix_t = 0.0
    dock._job_exit_fix()
    assert dock.sent[-1].linear.x == 0 and dock.sent[-1].angular.z < 0
    dock._job_pose = lambda frame="map": (0.0, 0.0, 0.0)  # 맞춰졌다
    dock._job_exit_fix()
    assert dock.job["state"] == "exiting"


def test_entry_start_holds_heading_on_radius_free_path(dock):
    # 2026-09-28: 진입 출발 0.1m 는 고치지 않고 출발 방향만 붙잡는다.
    free = []
    dock.pub_twist_free = SimpleNamespace(publish=free.append)
    dock._job_start_yaw = 0.0
    dock._job_start_xy = (0.0, 0.0)           # 렉 중심(1.5, 0)에서 1.5m 앞 출발
    # 5cm 후진한 자리, 캐스터 튐으로 2도 틀어짐 — 출발 방향(0)으로 되돌린다
    dock._job_pose = lambda frame="map": (0.05, 0.0, math.pi + math.radians(2))
    dock.job["state"] = "entering"
    dock._job_entering()
    assert free, "출발 구간은 반경 제한 없는 길로 보낸다"
    assert free[-1].linear.x < 0 and free[-1].angular.z < 0


def test_exit_uses_docking_reference_not_poi(dock):
    # 2026-09-29: 지도 위치가 틀어져 POI 로 기준을 잡아 렉 안에서 옆으로 틀었다.
    dock.estop, dock.control_mode = False, "auto"
    dock.job = {"state": "done", "cart": "ls_origin", "poi": "렉2"}
    dock.pois = [{"name": "렉2", "kind": "cart", "cart": "ls_origin",
                  "x": 0.6, "y": 0.3, "yaw": 0.0}]       # 0.6m 어긋난 POI
    dock._docked = {"poi": "렉2", "cart": "ls_origin",
                    "c": (0.0, 0.0, 0.0), "t": 0.0}      # 실제 도킹 기준(오돔)
    dock._job_pose = lambda frame="map": (0.0, 0.0, 0.0)
    dock._job_pick_near = lambda *a, **k: None
    dock.create_timer = lambda *a, **k: None
    ok, _ = dock.job_exit()
    assert ok
    _, e, yaw = dock._exit_axis((0.0, 0.0, 0.0))
    assert abs(e) < 1e-6 and abs(yaw) < 1e-6          # 렉 중심 기준, POI 아님


def test_exit_refuses_without_rack_reference(dock):
    dock.estop, dock.control_mode = False, "auto"
    dock.job = {"state": "idle", "cart": None, "poi": None}
    dock.pois = [{"name": "렉2", "kind": "cart", "cart": "ls_origin",
                  "x": 0.0, "y": 0.0, "yaw": 0.0}]
    dock._docked = None
    dock._job_pose = lambda frame="map": (0.0, 0.0, 0.0)
    dock._job_pick_near = lambda *a, **k: None
    ok, msg = dock.job_exit()
    assert not ok and "확인하지 못해" in msg


def test_gyro_damping_counters_unwanted_rotation(dock):
    # 2026-09-29: 캐스터가 돌리는 몫(자이로 - 명령)을 반대로 받아친다.
    dock._gyro_t = time.monotonic()
    dock._gyro_z = 0.05                  # 명령 0 인데 +0.05 rad/s 로 돈다
    dock._job_w_last = 0.0
    w = dock._job_damp(0.0, 0.15)
    assert w == pytest.approx(-dock.JOB_RATE_KD * 0.05)
    dock._gyro_t = 0.0                   # 자이로가 끊기면 건드리지 않는다
    assert dock._job_damp(0.02, 0.15) == pytest.approx(0.02)


def test_rear_right_protrusion_shifts_left_and_shrinks_margin(dock):
    # 2026-11-13: 오른쪽 뒤 라이다 돌출(가정 18mm) - 진입 줄을 왼쪽 9mm, 여유는 9mm 줄인다.
    dock.JOB_REAR_R_PROTRUDE = 0.018
    dock._cart_spec = lambda _: {"width": 0.423, "depth": 0.566,
                                 "clearance_width": 0.358, "center_offset": 0.0}
    assert dock._job_center_offset() == pytest.approx(0.009)
    assert dock._job_free("ls_origin") == pytest.approx(0.179 - 0.1644 - 0.009)
    # 렉 축 위에 정확히 있으면(라이다 축 좌우 0) 축 좌우는 +9mm = 오른쪽에 있다고 보고 왼쪽으로 간다
    r = {"forward": -1.0, "lateral": 0.0, "yaw": 0.0}
    _, axis_lat, _ = dock._job_entry_geometry(r)
    assert axis_lat == pytest.approx(0.009)
