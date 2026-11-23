"""켤 때 드라이브 자동 켜기 (2026-11-19).

아침에 로봇을 켠 뒤 수동 enable 을 잊어 첫 왕복이 '충전소에서 나오지 못함(시간 초과)' 으로 실패했다.
"""
import time
import types

from test_yaw_hold import BaseNode, _node   # noqa: F401  (fixture 재사용)


def _ae_node(BaseNode, enabled=False, error=0, online=True):
    n = _node(BaseNode, 0.0)
    n.sent_w, n.lines, n.logs = [], [], []
    n.send_wheels = lambda l, r: n.sent_w.append((l, r))
    n.ser = types.SimpleNamespace(write_line=lambda s: n.lines.append(s))
    lg = lambda *a: n.logs.append(a[0])                   # noqa: E731
    n.get_logger = lambda: types.SimpleNamespace(error=lg, warn=lg, info=lg)
    n._fault = None
    now = time.monotonic()
    n.diag = {ax: {"online": online, "enabled": enabled, "error": error, "t": now}
              for ax in (1, 2)}
    n._ae_until = now + 60.0
    n._ae_tries, n._ae_last, n._ae_warned = 0, -1e9, False
    n._ae_timer = types.SimpleNamespace(cancel=lambda: n.logs.append("cancel"))
    return n


def test_enables_once_then_stops_when_enabled(BaseNode):
    n = _ae_node(BaseNode)
    n._auto_enable_tick()
    assert n.lines == ["i"] and n.sent_w[-1] == (0, 0)
    n._auto_enable_tick()
    assert n.lines == ["i"]                                # 3초 안에는 다시 안 보낸다
    for d in n.diag.values():
        d["enabled"] = True
    n._auto_enable_tick()
    assert n._ae_timer is None and n.lines == ["i"]


def test_already_enabled_does_nothing(BaseNode):
    n = _ae_node(BaseNode, enabled=True)
    n._auto_enable_tick()
    assert n.lines == [] and n._ae_timer is None


def test_alarm_blocks_auto_enable(BaseNode):
    n = _ae_node(BaseNode, error=0x4000)
    n._auto_enable_tick()
    n._auto_enable_tick()
    assert n.lines == [] and n._ae_timer is not None
    n._ae_until = time.monotonic() - 1.0                   # 창이 끝나면 포기
    n._auto_enable_tick()
    assert n.lines == [] and n._ae_timer is None


def test_offline_or_stale_waits(BaseNode):
    n = _ae_node(BaseNode, online=False)
    n._auto_enable_tick()
    assert n.lines == []
    n = _ae_node(BaseNode)
    for d in n.diag.values():
        d["t"] -= 5.0                                      # 피드백이 끊김
    n._auto_enable_tick()
    assert n.lines == []


def test_gives_up_after_three_tries(BaseNode):
    n = _ae_node(BaseNode)
    for _ in range(3):
        n._ae_last -= 10.0
        n._auto_enable_tick()
    assert n.lines == ["i", "i", "i"]
    n._ae_last -= 10.0
    n._auto_enable_tick()
    assert n.lines == ["i", "i", "i"] and n._ae_timer is None
