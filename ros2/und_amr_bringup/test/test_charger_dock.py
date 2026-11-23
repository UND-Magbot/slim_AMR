"""충전소 후진 도킹 (2026-11-12) — 가상 로봇 · 가상 라이다로 처음부터 끝까지.

nav_goto 는 목표로 바로 옮겨 놓고 '도착' 으로 둔다. 후진은 /cmd_vel_noradius 지령을
적분한다. 스캔은 스테이션 앞면(평평한 어깨 + V 홈)을 로봇 기준으로 그린 360도 스캔이고,
차체 상자(+4.5cm) 안 점은 scan_merger 처럼 뺀다.
"""
import os
import math
import threading
import types

import numpy as np
import pytest

HALF_W, DEPTH = 0.137, 0.083


def station_points(face, yaw):
    """스테이션 앞면 윤곽 (지도 좌표). yaw = 바깥 방향."""
    t = np.linspace(0, 1, 60)
    pts = [np.c_[np.zeros(15), np.linspace(0.20, HALF_W, 15)],          # 왼쪽 어깨
           np.c_[-DEPTH * t, HALF_W * (1 - t)],                           # 왼 변
           np.c_[-DEPTH * (1 - t), -HALF_W * t],                          # 오른 변
           np.c_[np.zeros(15), np.linspace(-HALF_W, -0.20, 15)]]          # 오른쪽 어깨
    P = np.concatenate(pts)                    # 스테이션 기준: x = 바깥, y = 왼쪽
    c, s = math.cos(yaw), math.sin(yaw)
    return np.c_[face[0] + c * P[:, 0] - s * P[:, 1], face[1] + s * P[:, 0] + c * P[:, 1]]


class Sim:
    def __init__(self, n, face, syaw, robot):
        self.n, self.face, self.syaw, self.r = n, face, syaw, list(robot)
        self.k = 0
        self.tw = (0.0, 0.0)
        self.extra = None              # 지도 좌표 장애물 점 (N, 2)

    def scan(self):
        self.k += 1
        P = station_points(self.face, self.syaw)
        if self.extra is not None:
            P = np.r_[P, self.extra]
        c, s = math.cos(-self.r[2]), math.sin(-self.r[2])
        dx, dy = P[:, 0] - self.r[0], P[:, 1] - self.r[1]
        x, y = c * dx - s * dy, s * dx + c * dy
        keep = ~((np.abs(x) < 0.48175) & (np.abs(y) < 0.2094))
        x, y = x[keep], y[keep]
        n_ = 2400
        inc = 2 * math.pi / n_
        r = np.full(n_, np.inf)
        k = ((np.arctan2(y, x) + math.pi) / inc).astype(int) % n_
        np.minimum.at(r, k, np.hypot(x, y))
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=self.k, nanosec=0),
                                         frame_id="laser"),
            ranges=r, angle_min=-math.pi, angle_increment=inc)

    def step(self, dt):
        v, w = self.tw
        self.r[2] += w * dt
        self.r[0] += v * math.cos(self.r[2]) * dt
        self.r[1] += v * math.sin(self.r[2]) * dt


def make(ApiNode, face=(0.0, 0.0), syaw=math.pi / 2, poi_face=None, start_off=(0.0, 0.0, 0.0)):
    """스테이션 앞면 face, 바깥 방향 syaw. POI 는 poi_face(없으면 같은 자리) 기준 도킹 자세."""
    sim = {}

    class N(ApiNode):
        @property
        def scan(self):
            return sim["s"].scan()

    n = N.__new__(N)
    n.CD_BIAS_ON = False                       # 모의 라이다는 치우침이 없다 (2026-11-18 실측 보정은 따로 시험)
    n.lock = threading.Lock()
    n.estop = False
    n.control_mode = "auto"
    n.job = {"state": "idle"}
    n.route = {"state": "idle"}
    n.cdock = {"state": "idle"}
    n.nav_seq = 0
    n.nav_state = "idle"
    n.nav_msg = ""
    n._lift_moving = lambda: False
    n._poi_laser = lambda f: (0.0, 0.0, 0.0)
    n.bat, n.bat_t = None, 0.0                    # 기본: BMS 값 없음
    n.get_logger = lambda: types.SimpleNamespace(info=(lambda *a: print("  LOG", *a)) if os.environ.get("CD_LOG") else (lambda *a: None), warn=lambda *a: None)
    pf = poi_face or face
    hl = n.CHARGER_HALF_LEN
    n.pois = [{"name": "충전소", "kind": "charger", "standoff": 0.8,
               "x": pf[0] + hl * math.cos(syaw), "y": pf[1] + hl * math.sin(syaw), "yaw": syaw}]
    s = Sim(n, face, syaw, (0, 0, 0))
    sim["s"] = s
    n.tf_pose = lambda target="map", source="base_footprint": {"x": s.r[0], "y": s.r[1], "yaw": s.r[2]}
    n.sent = []

    def goto(x, y, yaw, **k):
        n.sent.append((x, y, yaw))
        s.r = [x + start_off[0], y + start_off[1], yaw + start_off[2]]
        n.nav_seq += 1
        n.nav_state = "done"
        return True, "ok"

    n.nav_goto = goto
    n.nav_cancel = lambda: None
    n._pub_zero = lambda: setattr(s, "tw", (0.0, 0.0))
    n.pub_twist_free = types.SimpleNamespace(
        publish=lambda t: setattr(s, "tw", (t.linear.x, t.angular.z)))
    return n, s


def run(n, s, max_t=60.0):
    dt = 1.0 / n.CD_HZ
    t = 0.0
    import time as _t
    real = _t.time
    clock = [real()]
    import und_amr_bringup.charger_dock as cdm
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    try:
        while t < max_t and n.cdock["state"] in n.CD_BUSY:
            n._cdock_tick()
            s.step(dt)
            t += dt
            clock[0] += dt
    finally:
        cdm.time = _t
    return t


def contact_err(n, s):
    gap, lat, dyaw = n._cdock_err(tuple(s.r), (s.face[0], s.face[1], s.syaw))
    return gap, lat, math.degrees(dyaw)


def test_aligned_backs_in_to_contact(ApiNode):
    n, s = make(ApiNode)
    assert n.cdock_start("충전소")[0]
    run(n, s)
    assert n.cdock["state"] == "done", n.cdock["message"]
    gap, lat, dyaw = contact_err(n, s)
    assert -0.003 < gap < 0.006 and abs(lat) < 0.005 and abs(dyaw) < 1.5


def test_small_offset_is_corrected_while_backing(ApiNode):
    # 대기 자리에서 좌우 2cm, 2도 어긋남 (허용 안) -> 후진하며 지움
    n, s = make(ApiNode, start_off=(0.02, 0.0, math.radians(2.0)))
    assert n.cdock_start("충전소")[0]
    run(n, s)
    assert n.cdock["state"] == "done", n.cdock["message"]
    gap, lat, dyaw = contact_err(n, s)
    assert abs(lat) < 0.01 and abs(dyaw) < 2.0 and gap < 0.006


def test_station_moved_a_bit_realigns_then_docks(ApiNode):
    # POI 는 실제보다 6cm 옆 · 4도 틀어진 데 저장돼 있다 -> 제자리에서 실제 축에 맞춘다 (nav2 다시 안 씀)
    syaw = math.pi / 2
    n, s = make(ApiNode, face=(0.06, 0.0), syaw=syaw + math.radians(4), poi_face=(0.0, 0.0))
    n.pois[0]["yaw"] = syaw
    assert n.cdock_start("충전소")[0]
    run(n, s)
    assert n.cdock["state"] == "done", n.cdock["message"]
    assert len(n.sent) == 1                       # 대기 자리만. 맞추기는 제자리에서
    gap, lat, dyaw = contact_err(n, s)
    assert abs(lat) < 0.008 and abs(dyaw) < 1.5


def test_station_far_from_poi_is_refused(ApiNode):
    # 대기 자리에서 스테이션이 보이도록 로봇을 실제 스테이션 앞에 둔다 (POI 와 1.3m 차이)
    n, s = make(ApiNode, face=(1.3, 0.0), poi_face=(0.0, 0.0), start_off=(1.3, 0.0, 0.0))
    assert n.cdock_start("충전소")[0]
    run(n, s)
    assert n.cdock["state"] == "failed" and "POI" in n.cdock["message"]
    assert s.tw == (0.0, 0.0)


def test_stop_cancels_backing(ApiNode):
    n, s = make(ApiNode)
    n.CD_TWO_LEG = False                           # 예전 흐름(대기 자리에서 바로 곡선 후진) 시험
    n.cdock_start("충전소")
    for _ in range(int(n.CD_HZ * 3)):              # 정렬 끝나고 후진 시작까지
        n._cdock_tick()
        s.step(1 / n.CD_HZ)
        if n.cdock["state"] == "backing" and s.tw[0] < 0:
            break
    assert n.cdock["state"] == "backing"
    n.cdock_cancel("정지")
    assert n.cdock["state"] == "canceled" and s.tw == (0.0, 0.0)


def test_refuses_non_charger_and_busy(ApiNode):
    n, s = make(ApiNode)
    n.pois.append({"name": "A", "kind": "point", "x": 1, "y": 1, "yaw": 0})
    assert not n.cdock_start("A")[0]
    n.control_mode = "manual"
    assert not n.cdock_start("충전소")[0]


def test_person_behind_pauses_then_continues(ApiNode):
    n, s = make(ApiNode)
    n.CD_TWO_LEG = False                           # 예전 흐름(대기 자리에서 바로 곡선 후진) 시험
    n.cdock_start("충전소")
    dt = 1 / n.CD_HZ
    for _ in range(int(n.CD_HZ * 10)):           # 정렬 + 후진 조금 (틈 0.5m 쯤에서 멈춤)
        n._cdock_tick(); s.step(dt)
        if n.cdock["state"] == "backing" and n.cdock.get("gap", 9) < 0.5:
            break
    assert n.cdock["state"] == "backing" and n.cdock["gap"] > 0.35
    gap0 = n.cdock["gap"]
    # 로봇과 스테이션 사이 한가운데에 다리 하나
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    cx, cy = s.face[0] + 0.5 * gap0 * ux, s.face[1] + 0.5 * gap0 * uy
    t = np.linspace(0, 2 * math.pi, 30)
    s.extra = np.c_[cx + 0.04 * np.cos(t), cy + 0.04 * np.sin(t)]
    for _ in range(int(n.CD_HZ * 3)):
        n._cdock_tick(); s.step(dt)
    assert n.cdock["state"] == "backing" and s.tw == (0.0, 0.0)
    assert "기다림" in n.cdock["message"] and abs(n.cdock["gap"] - gap0) < 0.05
    s.extra = None                                 # 치움 -> 이어서 들어감
    run(n, s)
    assert n.cdock["state"] == "done", n.cdock["message"]


def test_leaving_goes_straight_half_meter_then_dispatches(ApiNode):
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    run(n, s)
    assert n.cdock["state"] == "done"
    sent = []
    # 2026-11-17 나온 뒤에는 설 때까지 기다렸다 출발(_dispatch_after_out)
    n._dispatch_after_out = lambda *a: sent.append(a) or (True, "ok")
    n.cleave = {"state": "idle"}
    assert n._cdock_leave_needed()                 # 붙어 있다
    x0, y0, yaw0 = s.r
    n.nav_seq += 1
    n._cdock_leave_start("G", n.nav_seq, 5.0, 5.0, 0.0, True)
    dt = 1 / n.CD_HZ
    for _ in range(int(n.CD_HZ * 20)):
        n._cdock_tick()
        s.step(dt)
        if sent:
            break
    assert len(sent) == 1 and sent[0][0] == "G"
    gone = math.hypot(s.r[0] - x0, s.r[1] - y0)
    assert 0.49 < gone < 0.53
    # 곧게: 스테이션 축 방향으로 나왔고 방향도 그대로
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    assert abs((s.r[0] - x0) * ux + (s.r[1] - y0) * uy - gone) < 0.005
    assert abs(math.degrees(s.r[2] - yaw0)) < 1.0
    assert not n._cdock_leave_needed()             # 이제는 안 붙어 있다


def test_leaving_cancel_stops(ApiNode):
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    run(n, s)
    n._nav_dispatch = lambda *a: (_ for _ in ()).throw(AssertionError("보내면 안 됨"))
    n.cleave = {"state": "idle"}
    n.nav_seq += 1
    n._cdock_leave_start("G", n.nav_seq, 5.0, 5.0, 0.0, True)
    for _ in range(10):
        n._cdock_tick(); s.step(1 / n.CD_HZ)
    assert s.tw[0] > 0
    n.nav_seq += 1                                 # 정지 (nav_cancel 이 번호를 올린다)
    n._cdock_tick()
    assert n.cleave["state"] == "canceled" and s.tw == (0.0, 0.0)


def test_not_docked_far_from_charger(ApiNode):
    n, s = make(ApiNode)
    n.cleave = {"state": "idle"}
    s.r = [3.0, 3.0, 0.0]
    assert not n._cdock_leave_needed()


def test_docked_seen_by_lidar_even_if_slam_is_off(ApiNode):
    """SLAM 이 21cm 틀려도 라이다에 바로 뒤 V 가 보이면 붙어 있는 것 (2026-11-12)."""
    n, s = make(ApiNode)
    n.cleave = {"state": "idle"}
    n.cdock = {"state": "idle"}
    hl = n.CHARGER_HALF_LEN
    s.r = [s.face[0], s.face[1] + hl + 0.025, s.syaw]      # 뒤끝 틈 25mm, 스테이션 등짐
    real_tf = n.tf_pose
    n.tf_pose = lambda target="map", source="base_footprint": (
        {"x": s.r[0], "y": s.r[1] - 0.21, "yaw": s.r[2]} if target == "map" else real_tf(target))
    n.pois = [dict(n.pois[0], name="다른 데", x=9.0, y=9.0)]
    assert n._cdock_leave_needed()
    s.r = [s.face[0], s.face[1] + hl + 0.30, s.syaw]       # 30cm 떨어짐
    assert not n._cdock_leave_needed()


@pytest.mark.parametrize("lat,deg", [(0.10, 8.0), (-0.08, -6.0), (0.0, 175.0)])
def test_start_where_station_visible_aligns_locally(ApiNode, lat, deg):
    """POI 대기 자리가 아니어도 충전소가 보이면 nav2 없이 제자리에서 맞추고 들어간다.
    (0, 175도) = 충전소를 마주 보고 있음 -> 돌아선다."""
    n, s = make(ApiNode)
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    d = n.CHARGER_HALF_LEN + 0.9
    s.r = [s.face[0] + d * ux - lat * uy, s.face[1] + d * uy + lat * ux, s.syaw + math.radians(deg)]
    ok, msg = n.cdock_start("충전소")
    assert ok and "바로" in msg
    run(n, s, max_t=120)
    assert n.cdock["state"] == "done", n.cdock["message"]
    assert n.sent == []                            # nav2 를 한 번도 안 씀
    gap, l, dy = contact_err(n, s)
    assert gap < 0.006 and abs(l) < 0.01 and abs(dy) < 2.0


@pytest.mark.parametrize("lat,deg", [(0.15, -15.0), (-0.12, 12.0)])
def test_large_offset_corrected_while_backing_no_stop(ApiNode, lat, deg):
    """좌우 15cm · 방향 15도까지는 서서 맞추지 않고 후진하며 맞춘다 (2026-11-13)."""
    n, s = make(ApiNode)
    n.CD_TWO_LEG = False                           # 예전 흐름(대기 자리에서 바로 곡선 후진) 시험
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    d = n.CHARGER_HALF_LEN + 0.8
    s.r = [s.face[0] + d * ux - lat * uy, s.face[1] + d * uy + lat * ux, s.syaw + math.radians(deg)]
    n.cdock_start("충전소")
    t = run(n, s)
    assert n.cdock["state"] == "done" and n.cdock["realign"] == 0
    gap, l, dy = contact_err(n, s)
    assert abs(l) < 0.008 and abs(dy) < 1.0 and t < 15.0


def run_with_charger(n, s, touch_gap, max_t=60.0, on_a=5.0, off_a=-2.0):
    """뒤끝 틈이 touch_gap 이하가 되면 BMS 가 on_a 를 낸다 (실제 시각으로 신선하게)."""
    import time as _t
    dt = 1.0 / n.CD_HZ
    clock = [_t.time()]
    import und_amr_bringup.charger_dock as cdm
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    t = 0.0
    try:
        while t < max_t and n.cdock["state"] in n.CD_BUSY:
            gap, _, _ = n._cdock_err(tuple(s.r), (s.face[0], s.face[1], s.syaw))
            n.bat = {"ok": True, "current": on_a if gap <= touch_gap else off_a}
            n.bat_t = clock[0]
            n._cdock_tick()
            s.step(dt)
            t += dt
            clock[0] += dt
    finally:
        cdm.time = _t
    return t


@pytest.mark.parametrize("touch_gap,ok", [(0.004, True), (-0.010, True), (-0.050, False)])
def test_stops_on_charging_current(ApiNode, touch_gap, ok):
    """접점이 계산보다 앞 / 1cm 더 깊음 -> 전류에서 섬. 5cm 더 깊음(닿지 않음) -> 실패."""
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    run_with_charger(n, s, touch_gap)
    gap, lat, dy = contact_err(n, s)
    if ok:
        assert n.cdock["state"] == "done" and "충전 전류" in n.cdock["message"], n.cdock["message"]
        assert gap <= touch_gap + 0.003 and gap > touch_gap - 0.006
    else:
        assert n.cdock["state"] == "failed" and "충전 전류" in n.cdock["message"]
        assert gap > -0.025                       # 2cm 넘게 밀지 않는다
    assert s.tw == (0.0, 0.0)


def test_start_right_after_restart_without_nav_seq(ApiNode):
    """재시작 뒤 첫 명령이 도킹(충전소가 이미 보임)이어도 죽지 않는다 (2026-11-13 새로고침 버그)."""
    n, s = make(ApiNode)
    del n.nav_seq
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    d = n.CHARGER_HALF_LEN + 0.8
    s.r = [s.face[0] + d * ux, s.face[1] + d * uy, s.syaw]
    ok, msg = n.cdock_start("충전소")
    assert ok and n.cdock["state"] == "aligning"
    run(n, s)
    assert n.cdock["state"] == "done", n.cdock["message"]


def test_status_is_json_after_backing(ApiNode):
    """2026-11-16 scan_hdr(Time) 를 cdock 에 넣었더니 /charger/status 가 JSON 오류로 끊겼다."""
    import json
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    run(n, s, max_t=3.0)
    n.cdock["scan_hdr"] = types.SimpleNamespace(sec=1, nanosec=0)
    n.cdock["acc"] = [0.0] * 5
    json.dumps(n.cdock_status())


@pytest.mark.parametrize("lat,deg", [(0.05, 16.0), (-0.04, -12.0), (0.0, 0.0)])
def test_two_leg_docks_centered(ApiNode, lat, deg):
    """2026-11-18 두 단계: 축 위 0.3m 지점까지 곧게 -> 서서 다시 재고 축 방향 -> 곧게 접촉."""
    n, s = make(ApiNode)
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    d = n.CHARGER_HALF_LEN + 0.8
    s.r = [s.face[0] + d * ux - lat * uy, s.face[1] + d * uy + lat * ux, s.syaw + math.radians(deg)]
    n.cdock_start("충전소")
    states = []
    dt = 1 / n.CD_HZ
    import und_amr_bringup.charger_dock as cdm
    import time as _t
    clock = [_t.time()]
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    try:
        t = 0.0
        while t < 40 and n.cdock["state"] in n.CD_BUSY:
            if not states or states[-1] != n.cdock["state"]:
                states.append(n.cdock["state"])
            n._cdock_tick(); s.step(dt); t += dt; clock[0] += dt
    finally:
        cdm.time = _t
    assert n.cdock["state"] == "done", n.cdock["message"]
    assert n.cdock.get("mid_done") and "maneuver" in states
    gap, l, dy = contact_err(n, s)
    assert abs(l) < 0.005 and abs(dy) < 2.0 and t < 20.0


@pytest.mark.parametrize("on_a,ok", [(-0.3, True), (-1.5, False)])
def test_near_full_battery_charging_detected_when_still(ApiNode, on_a, ok):
    """2026-11-18 배터리 93%: 접촉 뒤 -0.3A(충전기 약하게 넣음)도 충전으로 본다. -1.5A(안 들어옴)는 실패."""
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    run_with_charger(n, s, 0.004, on_a=on_a, off_a=-1.5)
    assert n.cdock["state"] == ("done" if ok else "failed"), n.cdock["message"]


def test_full_battery_contact_counts_as_docked(ApiNode):
    """2026-11-18 배터리 90% 이상이면 전류가 약해도(-1.2A) 접촉으로 완료."""
    n, s = make(ApiNode)
    n.cdock_start("충전소")
    import time as _t
    dt = 1.0 / n.CD_HZ
    clock = [_t.time()]
    import und_amr_bringup.charger_dock as cdm
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    t = 0.0
    try:
        while t < 60 and n.cdock["state"] in n.CD_BUSY:
            n.bat = {"ok": True, "current": -1.2, "soc": 93}
            n.bat_t = clock[0]
            n._cdock_tick(); s.step(dt); t += dt; clock[0] += dt
    finally:
        cdm.time = _t
    assert n.cdock["state"] == "done" and "거의 차서" in n.cdock["message"]


def test_bias_correction_removes_measured_offset(ApiNode):
    """2026-11-18 실측: 정위치에서 라이다가 0.05m 틈일 때 좌우 -14.7mm · 방향 +1.22도로 봤다 -> 보정하면 0."""
    n = ApiNode.__new__(ApiNode)
    import und_amr_bringup.charger_dock as cdm
    real = cdm.detect_scan
    hl = n.CHARGER_HALF_LEN
    cdm.detect_scan = lambda *a: {"x": -(hl + 0.05), "y": -0.0147, "yaw": math.radians(1.22), "n": 100}
    try:
        c = n._cdock_detect(types.SimpleNamespace(ranges=[], angle_min=0, angle_increment=0), (0, 0, 0))
    finally:
        cdm.detect_scan = real
    assert abs(c["y"]) < 1e-6 and abs(c["yaw"]) < 1e-6


def test_other_v_shape_near_poi_is_ignored(ApiNode):
    """2026-11-18 POI 에서 0.9m 떨어진 V 모양을 잡아 나갔다 돌아왔다 -> 0.45m 넘게 먼 인식은 버린다."""
    n, s = make(ApiNode, face=(0.9, 0.0), poi_face=(0.0, 0.0), start_off=(0.9, 0.0, 0.0))
    n.cdock_start("충전소")
    run(n, s, max_t=10)
    assert n.cdock["state"] == "failed" and n.cdock.get("gated", 0) > 0
    assert "버림" in n.cdock["message"]


def test_approach_skips_precise_arrival_and_loose_heading(ApiNode):
    """2026-11-18 대기 자리 정밀 도착 + 2도 방향 맞추기로 약 15초 -> 자리 맞추기 없이, 방향 8도 한 번."""
    import math
    import threading
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.cdock = {"state": "starting"}
    n.nav_seq = 3
    got = []

    def goto(x, y, yaw, **k):
        got.append(k)
        n._nav_precise = bool(k.get("depart"))
        return True, "ok"
    n.nav_goto = goto
    p = {"name": "충전소1", "kind": "charger", "x": 0.0, "y": 0.0, "yaw": 0.0, "standoff": 0.8}
    ok, _ = n._cdock_go(p)
    assert ok and got[0]["turn_max"] == 1 and abs(got[0]["yaw_ok"] - math.radians(8.0)) < 1e-9
    assert n._nav_precise is False


@pytest.mark.parametrize("poi_off,deg", [(0.0, 160.0), (0.025, -155.0), (-0.03, 150.0)])
def test_poi_first_one_turn_from_arrival(ApiNode, poi_off, deg):
    """2026-11-18 대기 자리에서 -155도 방향 맞추기 중 25도 남기고 넘겨받아 서고 다시 -23도 돌던 것 ->
    도착해 방향 맞추기를 시작하면 POI 기준 중간 지점으로 한 번에 등 돌려 후진, 중간에서 재서 맞춘다."""
    n, s = make(ApiNode, poi_face=(poi_off, 0.0))             # POI 가 실제 스테이션과 poi_off 만큼 옆
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    d = n.CHARGER_HALF_LEN + 0.8

    def goto(x, y, yaw, **k):
        s.r = [x + 0.03, y - 0.02, yaw + math.radians(deg)]   # 대기 자리에 닿았는데 몸은 거의 반대로
        n.nav_seq += 1
        n.nav_state = "active"
        n._turning = True                                      # 도착 방향 맞추기 시작
        return True, "ok"
    n.nav_goto = goto
    s.r = [s.face[0] + 3.0 * ux, s.face[1] + 3.0 * uy, s.syaw + math.pi]   # 멀리서 (안 보이게)
    n._cdock_visible_now = lambda: False
    ok, _ = n.cdock_start("충전소")
    assert ok and n.cdock["state"] == "moving"
    import und_amr_bringup.charger_dock as cdm
    import time as _t
    clock = [_t.time()]
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    turns, dt, t = 0, 1 / n.CD_HZ, 0.0
    last = None
    try:
        while t < 60 and n.cdock["state"] in n.CD_BUSY:
            n._cdock_tick(); s.step(dt); t += dt; clock[0] += dt
            stp = (n.cdock.get("state"), n.cdock.get("step_i"))
            if stp != last and n.cdock.get("state") == "maneuver":
                st = n.cdock["steps"][n.cdock["step_i"]] if n.cdock["step_i"] < len(n.cdock["steps"]) else None
                if st and st[0] == "turn" and abs(_wrap_deg(st[1] - s.r[2])) > 60:
                    turns += 1
            last = stp
    finally:
        cdm.time = _t
    assert n.cdock["state"] == "done", n.cdock.get("message")
    assert turns == 1                                          # 큰 회전은 한 번
    gap, l, dy = contact_err(n, s)
    assert abs(l) < 0.006 and abs(dy) < 2.0, (l, dy)
    print("poi_off %+.0fmm deg %+.0f -> %.1f초, 끝 좌우 %+.1fmm 방향 %+.1f도, 중간 다시 돌기 %d번"
          % (poi_off * 1000, deg, t, l * 1000, dy, n.cdock.get("pivots", 0)))
    if n.CD_AXIS_BACK:
        # 2026-11-18 실기: 중간 지점에서 매번 1~2번 다시 돌았다 -> 축에 올라타 도착 (방향 1.5도 넘으면 한 번은 돌 수 있음)
        assert n.cdock.get("pivots", 0) <= 1
    assert t < 40.0, t


def _wrap_deg(a):
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))
