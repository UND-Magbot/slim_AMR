"""base_node 주행 중 차체 회전 잡기(yaw_hold) 시험 (2026-10-02).

- 꺼져 있으면(kp 0) 지령을 그대로 낸다
- 차체가 '지령을 바퀴 반응만큼 늦춘 회전' 보다 더 돌면 반대로 되잡는다
- 지령대로 따라 도는 중에는 거의 건드리지 않는다
- 서 있거나 /cmd_vel_noradius(api_node 가 직접 모는 통로)면 손대지 않는다
"""
import importlib.util
import math
import pathlib
import sys
import time

import pytest

SRC = pathlib.Path(__file__).resolve().parent.parent / "und_amr_bringup"


@pytest.fixture(scope="module")
def BaseNode():
    spec = importlib.util.spec_from_file_location("base_node", SRC / "base_node.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["base_node"] = mod
    spec.loader.exec_module(mod)
    return mod.BaseNode


class _Tw:
    def __init__(self, v, w):
        self.linear = type("L", (), {"x": v})()
        self.angular = type("A", (), {"z": w})()


def _node(BaseNode, kp):
    n = BaseNode.__new__(BaseNode)
    n.max_v, n.max_w = 0.6, 1.5
    n.acc_v = n.acc_w = n.dec_v = n.dec_w = 1e9      # 가속 제한 없이 계산만 본다
    n.ramp_v = n.ramp_w = 0.0
    n.ramp_t = time.monotonic() - 0.05
    n.min_radius = 0.0
    n.rate_kd = 0.0
    n.yaw_kp, n.yaw_tau, n.yaw_max = kp, 0.15, 0.15
    n._w_ref = 0.0
    n._gyro_sum, n._gyro_n, n._gyro_z = 0.0, 0, 0.0
    n._gyro_t = time.monotonic()
    n._w_sent = 0.0
    n._hold = None
    n.sep = 0.1789
    n.sent = []
    n.mps_to_rpm = lambda v, side: v * 100.0          # 계산 확인용 단순 변환
    n.max_rpm = 1e9
    n.quantize = lambda a, b: (a, b)
    n.side_sign = {"l": 1.0, "r": 1.0}
    n.side_axis = {"l": 1, "r": 2}
    n.send_wheels = lambda a, b: n.sent.append((a, b))
    return n


def _w_out(n):
    a, b = n.sent[-1]
    return (b - a) / 100.0 / n.sep


def _tick(n, v, w, gyro):
    """지령 한 번 받고, 50Hz 잡기 루프 한 번 돈다(20ms 뒤)."""
    n.ramp_t = time.monotonic() - 0.05
    n.on_cmd_vel(_Tw(v, w))
    n._gyro_sum, n._gyro_n = gyro, 1
    n._gyro_t = time.monotonic()
    n._hold_last = time.monotonic() - 0.02
    n._yaw_hold_tick()
    return _w_out(n)


def test_hold_stores_command_and_loop_sends(BaseNode):
    n = _node(BaseNode, 1.0)
    n.ramp_t = time.monotonic() - 0.05
    n.on_cmd_vel(_Tw(0.35, 0.0))
    assert n._hold is not None and not n.sent      # 달리는 중엔 루프가 낸다
    n._hold = (0.35, 0.0, time.monotonic() - 1.0)  # 지령이 끊겼으면 다시 내지 않는다
    n._yaw_hold_tick()
    assert not n.sent and n._hold is None


def test_off_passes_command(BaseNode):
    n = _node(BaseNode, 0.0)
    assert abs(_tick(n, 0.35, 0.0, math.radians(10)) - 0.0) < 1e-9


def test_counters_excess_body_rotation(BaseNode):
    n = _node(BaseNode, 1.0)
    for _ in range(10):                 # 곧게 가는데 차체가 왼쪽으로 5도/s 돈다
        out = _tick(n, 0.35, 0.0, math.radians(5))
    assert out < -math.radians(4)       # 오른쪽으로 되잡는다


def test_correction_is_capped(BaseNode):
    n = _node(BaseNode, 5.0)
    out = _tick(n, 0.35, 0.0, math.radians(30))
    assert abs(out) <= n.yaw_max + 1e-9


def test_following_command_is_left_alone(BaseNode):
    n = _node(BaseNode, 1.0)
    w = math.radians(8)
    for _ in range(60):                 # 지령대로 8도/s 로 돌고 있다 (50Hz, 1.2초)
        out = _tick(n, 0.35, w, w)
    assert abs(out - w) < math.radians(0.5)


def test_not_when_still_or_free_channel(BaseNode):
    n = _node(BaseNode, 1.0)
    assert abs(_tick(n, 0.0, 0.3, 0.0) - 0.3) < 1e-9          # 제자리 회전
    n.on_cmd_vel(_Tw(0.35, 0.0), radius=False)                   # api_node 직접 통로
    assert abs(_w_out(n)) < 1e-9
