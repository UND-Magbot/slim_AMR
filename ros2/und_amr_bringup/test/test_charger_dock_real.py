"""충전소 후진 도킹 — 실기에 가까운 모의 (2026-11-16 S자).

실기 기록(11/16 15:23, 15:35): 후진 시작 때 방향 +16~18도 · 좌우 +25~50mm, 회전이 명령보다
0.2~0.3초 늦고, 라이다 좌우가 스캔마다 ±10mm 흔들렸다. 방향이 +18 -> -3 -> +5 -> -3 -> +1도로
흔들리며 들어갔다(S자). 여기서는
  - 명령 지연 0.15초 + 1차 지연(회전 0.30초, 직진 0.15초) — 실기 회전 반응 지연 0.4초
  - 명령과 상관없는 회전 흔들림 σ 0.04 rad/s (상관 시간 0.4초)
  - 스캔 10Hz, 0.15초 늦게 도착 (스캔 시각 자세는 TF 로 조회 가능)
  - 라이다 거리 잡음 σ 6mm
로 그린다. S자는 '방향 오차 부호가 바뀐 횟수(±1.5도 넘는 것만)' 로 센다.
"""
import collections
import math
import threading
import types

import numpy as np
import pytest

from test_charger_dock import station_points


class RealSim:
    DT = 0.05
    DIST = 0.04

    def __init__(self, robot, face=(0.0, 0.0), syaw=math.pi / 2, seed=1):
        self.face, self.syaw = face, syaw
        self.r = list(robot)
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.q = collections.deque([(0.0, 0.0)] * 3)     # 0.15초 지연
        self.dist = 0.0
        self.k = 0
        self.hist = {0: tuple(self.r)}
        self.rng = np.random.default_rng(seed)
        self.last_scan = None

    def step(self):
        self.q.append(self.cmd)
        vc, wc = self.q.popleft()
        self.v += (vc - self.v) * min(1.0, self.DT / 0.15)
        self.w += (wc - self.w) * min(1.0, self.DT / 0.30)
        # 명령과 상관없는 회전 흔들림 (실기: 실제-명령 표준편차 0.04 rad/s, 0.2~0.8Hz)
        a = math.exp(-self.DT / 0.4)
        self.dist = a * self.dist + math.sqrt(1 - a * a) * self.rng.normal(0, self.DIST)
        self.r[2] += (self.w + self.dist) * self.DT
        self.r[0] += self.v * math.cos(self.r[2]) * self.DT
        self.r[1] += self.v * math.sin(self.r[2]) * self.DT
        self.k += 1
        self.hist[self.k] = tuple(self.r)
        if self.k % 2 == 0 and self.k >= 3:
            self.last_scan = self._scan(self.k - 3)       # 0.15초 전 자세로 찍은 스캔

    def _scan(self, k):
        rx, ry, ryaw = self.hist[k]
        P = station_points(self.face, self.syaw)
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        dx, dy = P[:, 0] - rx, P[:, 1] - ry
        x, y = c * dx - s * dy, s * dx + c * dy
        keep = ~((np.abs(x) < 0.48175) & (np.abs(y) < 0.2094))
        x, y = x[keep], y[keep]
        n_ = 2400
        inc = 2 * math.pi / n_
        r = np.full(n_, np.inf)
        idx = ((np.arctan2(y, x) + math.pi) / inc).astype(int) % n_
        np.minimum.at(r, idx, np.hypot(x, y))
        fin = np.isfinite(r)
        r[fin] += self.rng.normal(0, 0.006, fin.sum())
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=k, nanosec=0),
                                         frame_id="laser"),
            ranges=r, angle_min=-math.pi, angle_increment=inc)


def make_real(ApiNode, lat, deg, gap=0.79, seed=1):
    syaw = math.pi / 2
    ux, uy = math.cos(syaw), math.sin(syaw)
    hl = ApiNode.CHARGER_HALF_LEN
    d = hl + gap
    s = RealSim((d * ux - lat * uy, d * uy + lat * ux, syaw + math.radians(deg)), seed=seed)
    for _ in range(4):
        s.step()

    class N(ApiNode):
        @property
        def scan(self):
            return s.last_scan

    n = N.__new__(N)
    n.CD_BIAS_ON = False
    n.lock = threading.Lock()
    n.estop = False
    n.control_mode = "auto"
    n.job = {"state": "idle"}
    n.route = {"state": "idle"}
    n.cdock = {"state": "idle"}
    n.nav_seq, n.nav_state, n.nav_msg = 0, "idle", ""
    n._lift_moving = lambda: False
    n._poi_laser = lambda f: (0.0, 0.0, 0.0)
    n.bat, n.bat_t = None, 0.0
    n.odom = None
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    n.pois = [{"name": "충전소", "kind": "charger", "standoff": 0.8,
               "x": hl * ux, "y": hl * uy, "yaw": syaw}]
    n.tf_pose = lambda target="map", source="base_footprint": {"x": s.r[0], "y": s.r[1], "yaw": s.r[2]}

    def at(target, source, stamp=None):
        p = s.hist.get(stamp.sec) if stamp is not None else tuple(s.r)
        return None if p is None else {"x": p[0], "y": p[1], "yaw": p[2]}

    n._tf_nowait = at
    n._pub_zero = lambda: setattr(s, "cmd", (0.0, 0.0))
    n.pub_twist_free = types.SimpleNamespace(
        publish=lambda t: setattr(s, "cmd", (t.linear.x, t.angular.z)))
    return n, s


def run_real(n, s, max_t=40.0):
    import time as _t
    import und_amr_bringup.charger_dock as cdm
    clock = [_t.time()]
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    yaws, t = [], 0.0
    try:
        ok, msg = n.cdock_start("충전소")
        assert ok, msg
        while t < max_t and n.cdock["state"] in n.CD_BUSY:
            n._cdock_tick()
            s.step()
            t += s.DT
            clock[0] += s.DT
            if n.cdock["state"] == "backing":
                yaws.append(math.degrees(math.atan2(math.sin(s.r[2] - s.syaw),
                                                    math.cos(s.r[2] - s.syaw))))
    finally:
        cdm.time = _t
    gap, lat, dyaw = n._cdock_err(tuple(s.r), (s.face[0], s.face[1], s.syaw))
    # 처음 크게 틀어진 방향을 지운 뒤의 흔들림(부호 바뀜)만 센다
    flips, sign = 0, 0
    for y in yaws:
        sg = 1 if y > 1.5 else -1 if y < -1.5 else 0
        if sg and sign and sg != sign:
            flips += 1
        if sg:
            sign = sg
    return dict(t=t, gap=gap, lat=lat, yaw=math.degrees(dyaw), flips=flips,
                state=n.cdock["state"], msg=n.cdock.get("message"))


CASES = [(0.050, 18.0), (0.025, 16.0), (-0.040, -12.0), (0.0, 0.0), (0.10, -8.0)]


@pytest.mark.parametrize("lat,deg", CASES)
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_real_dock_completes_within_limits(ApiNode, lat, deg, seed):
    """흔들림이 있어도 끝까지 들어가고, 시간 · 오차가 지금 수준보다 나빠지지 않는다.
    (2026-11-16 모의: 어떤 제어 변형도 끝 좌우 최대 6~10mm · 흔들림 1회 안팎 — 구동부 흔들림이 한계)"""
    n, s = make_real(ApiNode, lat, deg, seed=seed)
    r = run_real(n, s)
    print("좌우 %+.0fmm 방향 %+.0f도 seed %d -> %.1f초 흔들림 %d회 끝 좌우 %+.1fmm 방향 %+.1f도 (%s)"
          % (lat * 1000, deg, seed, r["t"], r["flips"], r["lat"] * 1000, r["yaw"], r["state"]))
    assert r["state"] == "done", r["msg"]
    # 2026-11-18 0.012 -> 0.016: seed 10개(47회)로 보니 제자리 회전 · 한쪽 바퀴 회전 둘 다 12mm 넘음 4번, 최대 14~16mm
    #   (모의 회전 흔들림 탓 - 실기 끝 좌우는 -1~-4mm). seed 1~3 만 보고 잡은 12mm 는 난수 순서만 바뀌어도 넘었다.
    assert abs(r["lat"]) < 0.016
    # 2026-11-18 4.5 -> 6.5: seed 10개(47회)로 보니 중간 다시 돌기 기준 1.5 · 2.0 · 3.0도 모두 끝 방향 최대 5.8도,
    #   평균 1.5~1.7도로 같다(모의 회전 흔들림 탓). seed 1~3 으로 잡은 4.5 는 예전 설정도 넘었다. 실기는 40회 모두 0.8도 안.
    assert abs(r["yaw"]) < (6.5 if ApiNode.CD_TWO_LEG else 3.5)
    assert r["t"] < (22.0 if ApiNode.CD_TWO_LEG else 13.0)   # 두 단계 + 한 가지 속도 0.06 m/s (2026-11-18 사용자 요청, 평균 18초)


class _Odom:
    def __init__(self, s):
        self.s = s

    @property
    def twist(self):
        s = self.s
        return types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=s.v), angular=types.SimpleNamespace(z=s.w + s.dist)))


def run_axis(ApiNode, lat, deg, est_off, seed):
    """대기 자리(틈 0.79m)에서 등을 돌린 뒤 '축 따라 중간 지점까지' 만 돌린다. 끝 자세 (좌우, 방향) 참값."""
    import time as _t
    import und_amr_bringup.charger_dock as cdm
    n, s = make_real(ApiNode, lat, deg, gap=0.79, seed=seed)
    n.odom = _Odom(s)
    ux, uy = math.cos(s.syaw), math.sin(s.syaw)
    est = (s.face[0] - est_off * uy, s.face[1] + est_off * ux, s.syaw)     # 바퀴 기록이 옆으로 est_off 틀림
    clock = [_t.time()]
    cdm.time = types.SimpleNamespace(time=lambda: clock[0])
    try:
        g0, lat0, _ = n._cdock_err(tuple(s.r), est)
        h0 = _w(est[2] + n._cdock_axis_head(lat0, g0 - n.CD_MID_GAP))       # 실기처럼 시작 방향으로 먼저 돈다
        n.cdock = {"state": "maneuver", "steps": [("turn", h0, n.CD_MID_TURN_TOL), ("axis", None)], "step_i": 0,
                   "t0": clock[0], "est_last": est, "mid_done": True}
        t = 0.0
        lats = []
        while t < 30 and n.cdock["step_i"] < 2:
            n._cdock_maneuver()
            s.step()
            t += s.DT
            clock[0] += s.DT
            if n.cdock["step_i"] == 1:
                lats.append(n._cdock_err(tuple(s.r), (s.face[0], s.face[1], s.syaw))[1])
        for _ in range(20):                      # 서는 동안
            n._pub_zero()
            s.step()
    finally:
        cdm.time = _t
    gap, l, dy = n._cdock_err(tuple(s.r), (s.face[0], s.face[1], s.syaw))
    # S자: 처음 좌우와 반대쪽으로 넘어간 최대 [m]
    sg = 1 if lats and lats[0] > 0 else -1
    over = max([0.0] + [-sg * x for x in lats])
    return t, gap, l, math.degrees(dy), over


def _w(a):
    return math.atan2(math.sin(a), math.cos(a))


@pytest.mark.parametrize("lat,deg,est_off", [(0.03, 3.0, 0.02), (-0.05, -3.0, -0.025), (0.0, 2.0, 0.025),
                                             (0.04, -3.0, 0.0)])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_real_axis_back_to_mid(ApiNode, lat, deg, est_off, seed):
    """2026-11-18 실기: 큰 회전 뒤 바퀴 기록만으로 점까지 후진 -> 중간 지점 좌우 -12~-29mm · 방향 -5~-6도, 매번 다시 돌기.
    등 뒤 V 홈을 재며 축에 올라타면 지연 · 흔들림이 있어도 좌우 · 방향이 작게 도착한다."""
    if not ApiNode.CD_AXIS_BACK:
        pytest.skip("CD_AXIS_BACK 꺼짐")
    t, gap, l, dy, over = run_axis(ApiNode, lat, deg, est_off, seed)
    print("시작 좌우 %+.0fmm 방향 %+.0f도 바퀴기록 틀림 %+.0fmm seed %d -> %.1f초 틈 %.0fmm 좌우 %+.1fmm 방향 %+.1f도 반대로 넘어감 %.0fmm"
          % (lat * 1000, deg, est_off * 1000, seed, t, gap * 1000, l * 1000, dy, over * 1000))
    assert abs(gap - ApiNode.CD_MID_GAP) < 0.03
    assert abs(l) < 0.012          # 2026-11-18 모의: 예전 '점까지 후진' 은 바퀴기록 틀림만큼 ±17~27mm, 새 방식 최대 10mm
    assert abs(dy) < 6.5           # 방향은 모의 회전 흔들림(σ 0.04 rad/s)이 좌우함 - 최대 5.7도
