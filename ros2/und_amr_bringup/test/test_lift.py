"""리프트 연동 시험 (2026-10-07).

- 리프트가 움직이는 동안(up/down) base_node 는 주행 지령을 0 으로 낸다
- 렉을 든 상태(top/stopped)도 속도 · 가속을 깎지 않는다 (2026-11-12, 사용자: "리프트해도 속도는 같아야해")
- 내린 상태(bottom)는 그대로
- [LIFT] 줄로 상태를 읽는다 (lift.c 의 printf 와 짝)
- api_node 는 움직이는 중에 리프트 올림·내림을 거절한다
"""
import importlib.util
import pathlib
import sys
import time
from types import SimpleNamespace

import pytest

SRC = pathlib.Path(__file__).resolve().parent.parent / "und_amr_bringup"


@pytest.fixture(scope="module")
def BaseNode():
    spec = importlib.util.spec_from_file_location("base_node", SRC / "base_node.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["base_node"] = mod
    spec.loader.exec_module(mod)
    return mod.BaseNode


@pytest.fixture(scope="module")
def ApiNode():
    spec = importlib.util.spec_from_file_location("api_node", SRC / "api_node.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["api_node"] = mod
    spec.loader.exec_module(mod)
    return mod.ApiNode


class _Tw:
    def __init__(self, v, w):
        self.linear = type("L", (), {"x": v})()
        self.angular = type("A", (), {"z": w})()


def _base(BaseNode, lift, acc=1e9):
    n = BaseNode.__new__(BaseNode)
    n.max_v, n.max_w = 0.6, 1.5
    n.acc_v = n.acc_w = n.dec_v = n.dec_w = acc
    n.ramp_v = n.ramp_w = 0.0
    n.ramp_t = time.monotonic() - 0.1
    n.min_radius = 0.0
    n.rate_kd = 0.0
    n.yaw_kp, n.yaw_tau, n.yaw_max = 0.0, 0.15, 0.15
    n._w_ref = 0.0
    n._gyro_sum, n._gyro_n, n._gyro_z = 0.0, 0, 0.0
    n._gyro_t = time.monotonic()
    n._w_sent = 0.0
    n._hold = None
    n.sep = 0.1789
    n.sent = []
    n.mps_to_rpm = lambda v, side: v * 100.0
    n.max_rpm = 1e9
    n.quantize = lambda a, b: (a, b)
    n.side_sign = {"l": 1.0, "r": 1.0}
    n.side_axis = {"l": 1, "r": 2}
    n.send_wheels = lambda a, b: n.sent.append((a, b))
    n.lift_state = lift
    n.lift_max_v, n.lift_acc_v = 0.20, 0.05
    return n


def _v_out(n):
    a, b = n.sent[-1]
    return (a + b) / 200.0


@pytest.mark.parametrize("lift", ["up", "down"])
def test_moving_lift_holds_robot(BaseNode, lift):
    n = _base(BaseNode, lift)
    n.on_cmd_vel(_Tw(0.3, 0.2))
    assert n.sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize("lift", ["top", "stopped"])
def test_loaded_is_not_capped(BaseNode, lift):
    """2026-11-12: 든 상태도 내린 상태와 같은 속도 · 가속."""
    a, b = _base(BaseNode, lift, acc=0.35), _base(BaseNode, "bottom", acc=0.35)
    for n in (a, b):
        n.ramp_t = time.monotonic() - 0.1
        n.on_cmd_vel(_Tw(0.5, 0.0))
    assert _v_out(a) == pytest.approx(_v_out(b), rel=0.2) and _v_out(a) > 0.02


def test_lowered_runs_normally(BaseNode):
    n = _base(BaseNode, "bottom")
    n.on_cmd_vel(_Tw(0.5, 0.0))
    assert _v_out(n) == pytest.approx(0.5)


@pytest.mark.parametrize("line,state", [
    ("[LIFT] up start (PC0 HIGH, 10000 ms)", "up"),
    ("> > [LIFT] up done (10000 ms) - PC0/PE1 LOW", "top"),
    ("[LIFT] down start (PE1 HIGH, 10000 ms)", "down"),
    ("[LIFT] down done (10000 ms) - PC0/PE1 LOW", "bottom"),
    ("[LIFT] stop after 3120 ms", "stopped"),
])
def test_state_from_firmware_line(BaseNode, tmp_path, line, state):
    n = _base(BaseNode, "unknown")
    n.LIFT_FILE = str(tmp_path / "lift")
    n._pub_lift = lambda: None
    n._on_lift_line(line)
    assert n.lift_state == state
    assert open(n.LIFT_FILE).read() == state


def test_status_line_keeps_state(BaseNode, tmp_path):
    n = _base(BaseNode, "top")
    n.LIFT_FILE = str(tmp_path / "lift")
    n._pub_lift = lambda: None
    n._on_lift_line("[LIFT] IDLE PC0=0 PE1=0 up=10000ms down=10000ms")
    assert n.lift_state == "top"


def _api(ApiNode, v=0.0, w=0.0, lift="bottom"):
    n = ApiNode.__new__(ApiNode)
    n.estop = False
    n.job = {"state": "idle"}
    n.route = {"state": "idle"}
    n.nav_goal = None
    n.nav_state = "idle"
    n.odom = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=v), angular=SimpleNamespace(z=w))))
    n.lift = lift
    n.nav_speed = 0.30
    return n


def test_lift_allowed_when_still(ApiNode):
    assert _api(ApiNode).lift_busy_error() is None


def test_lift_refused_while_moving(ApiNode):
    assert _api(ApiNode, v=0.2).lift_busy_error()
    assert _api(ApiNode, w=0.3).lift_busy_error()
    n = _api(ApiNode)
    n.nav_state = "active"
    assert n.lift_busy_error()


def test_nav_speed_same_when_loaded(ApiNode):
    """2026-11-12: 리프트를 올려도 nav2 속도는 설정 그대로."""
    assert _api(ApiNode, lift="top")._speed_eff() == pytest.approx(0.30)
    assert _api(ApiNode, lift="stopped")._speed_eff() == pytest.approx(0.30)
    assert _api(ApiNode, lift="bottom")._speed_eff() == pytest.approx(0.30)
