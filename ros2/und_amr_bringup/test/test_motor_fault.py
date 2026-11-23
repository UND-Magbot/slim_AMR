"""모터 알람이면 바로 세우고 주행 지령을 막는다 (2026-11-17).

축1 알람 0x4000 이 걸린 채 왼쪽 바퀴만 돌아 로봇이 제자리에서 한 바퀴 반을 돌았다.
"""
import time
import types

from test_yaw_hold import BaseNode, _Tw, _node   # noqa: F401  (fixture 재사용)


def _fault_node(BaseNode):
    n = _node(BaseNode, 0.0)
    n.sent_w = []
    n.send_wheels = lambda l, r: n.sent_w.append((l, r))
    n.get_logger = lambda: types.SimpleNamespace(error=lambda *a: None, warn=lambda *a: None,
                                                 info=lambda *a: None)
    n._hold = None
    n.last_sent = (0, 0)
    n.diag = {1: {"online": True, "enabled": True, "error": 0},
              2: {"online": True, "enabled": True, "error": 0}}
    return n


def test_alarm_stops_and_blocks_commands(BaseNode):
    n = _fault_node(BaseNode)
    n.diag[1]["error"] = 0x4000
    n._check_fault(1)
    assert n.sent_w[-1] == (0, 0) and n._fault["axis"] == 1
    k = len(n.sent_w)
    n.on_cmd_vel(_Tw(0.3, 0.0))
    assert all(s == (0, 0) for s in n.sent_w[k:])      # 지령을 내지 않는다


def test_fault_clears_after_ok_for_a_while(BaseNode):
    n = _fault_node(BaseNode)
    n.diag[1]["error"] = 0x4000
    n._check_fault(1)
    n.diag[1]["error"] = 0
    n._check_fault(1)
    assert n._fault is not None                          # 바로 풀지 않는다
    n._fault_ok_t -= 2.0
    n._check_fault(1)
    assert n._fault is None


def test_offline_axis_is_a_fault(BaseNode):
    n = _fault_node(BaseNode)
    n.diag[2]["online"] = False
    n._check_fault(2)
    assert n._fault is not None and n.sent_w[-1] == (0, 0)
